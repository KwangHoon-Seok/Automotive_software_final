#include "driving_way_node.hpp"

DrivingWayNode::DrivingWayNode(const std::string& node_name, const double& loop_rate, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    RCLCPP_INFO(this->get_logger(), "Initializing DrivingWayNode...");
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "/ego/lane_points", qos_profile,
        std::bind(&DrivingWayNode::CallbackLanePoints, this, std::placeholders::_1));
    
    // Publishers
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>("/ego/driving_way", qos_profile);
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>("/ego/poly_lanes", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

    lane_point_LEFT.id = "1";
    lane_point_RIGHT.id = "2";
    inliers_LEFT.id = "1";
    inliers_RIGHT.id = "2";
}

DrivingWayNode::~DrivingWayNode() {}

void DrivingWayNode::Run(const rclcpp::Time& current_time) {
    ad_msgs::msg::LanePointData lane_points;
    {
        std::lock_guard<std::mutex> lock(mutex_lane_points_);
        lane_points = i_lane_points_;
    }

    if (lane_points.point.empty()) {
        RCLCPP_WARN(this->get_logger(), "No lane points received. Skipping processing...");
        return;
    }

    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    ad_msgs::msg::PolyfitLaneData driving_way;

    lane_point_LEFT.point.clear();
    lane_point_RIGHT.point.clear();
    inliers_LEFT.point.clear();
    inliers_RIGHT.point.clear();

    splitLanePoints(lane_points);
    process_lanes();
    populatePolyLanes(poly_lanes);
    populateCenterLane(driving_way);
    prev_driving_way_ = driving_way;
    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;

    PublishDrivingWay(current_time);
}

// Functions
float DrivingWayNode::distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}



void DrivingWayNode::splitLanePoints(const ad_msgs::msg::LanePointData& lane_points){
    if(!is_init){
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto& point : lane_points.point) {
            // geometry_msgs::msg::Point 객체 생성
            geometry_msgs::msg::Point new_point;
            new_point.x = static_cast<float>(point.x);  // x 좌표 설정
            new_point.y = static_cast<float>(point.y);  // y 좌표 설정
            new_point.z = static_cast<float>(point.z);  // z 좌표 설정

            // 생성된 new_point를 points에 추가
            points.push_back(new_point);
        }
        
        // 차량 좌표계 x축을 기준으로 sort
        std::sort(points.begin(), points.end(), [](const Point& p1, const Point& p2){
            return p1.x < p2.x;
        });

        // lane_point 초기화
        lane_point_LEFT.point.clear();
        lane_point_RIGHT.point.clear();

        for(const auto & point : points){
            if(point.y < 0){
                lane_point_RIGHT.point.push_back(point);
            }
            else if(point.y > 0){
                lane_point_LEFT.point.push_back(point);
            }
        }
        is_init = true;
        // RCLCPP_INFO(this->get_logger(), "[--1st--] Split Lane Points");
        
    }

    else if (is_init) {
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto& point : lane_points.point) {
            // geometry_msgs::msg::Point 객체 생성
            geometry_msgs::msg::Point new_point;
            new_point.x = static_cast<float>(point.x);  // x 좌표 설정
            new_point.y = static_cast<float>(point.y);  // y 좌표 설정
            new_point.z = static_cast<float>(point.z);  // z 좌표 설정

            // 생성된 new_point를 points에 추가
            points.push_back(new_point);
        }
        
        // 차량 좌표계 x축을 기준으로 sort
        std::sort(points.begin(), points.end(), [](const Point& p1, const Point& p2){
            return p1.x < p2.x;
        });

        // lane_point 초기화
        lane_point_LEFT.point.clear();
        lane_point_RIGHT.point.clear();

        // prev_driving_way_를 기반으로 차선 분류
        float distance;
        for (const auto& point : points) {
            float x = point.x;
            float y = point.y;

            // prev_driving_way_의 계수를 사용하여 도로 경로의 Y 값을 계산
            float Y_poly = prev_driving_way_.a3 * std::pow(x, 3) +
                           prev_driving_way_.a2 * std::pow(x, 2) +
                           prev_driving_way_.a1 * x +
                           prev_driving_way_.a0;

            // 도로 경로에 대한 Y 값에 따라 좌측과 우측 차선 분류
            distance = abs(Y_poly - y);
            if (distance < 2.2) {
                if (y < Y_poly) {
                    // y가 도로 경로보다 크면 우측 차선
                    lane_point_RIGHT.point.push_back(point);
                }
                else if (y > Y_poly) {
                    // y가 도로 경로보다 작으면 좌측 차선
                    lane_point_LEFT.point.push_back(point);
                }
            }
        }

        // RCLCPP_INFO(this->get_logger(), "[--2st--] Split Lane Points with prev_driving_way_");
    }


}

void DrivingWayNode::process_lanes() {
    
    num_points_LEFT = lane_point_LEFT.point.size();
    num_points_RIGHT = lane_point_RIGHT.point.size();
    X_LEFT.resize(num_points_LEFT);
    Y_LEFT.resize(num_points_LEFT);
    X_RIGHT.resize(num_points_RIGHT);
    Y_RIGHT.resize(num_points_RIGHT);

    for (size_t i = 0; i < num_points_LEFT; ++i) {
        X_LEFT(i) = lane_point_LEFT.point[i].x;
        Y_LEFT(i) = lane_point_LEFT.point[i].y;
    }

    for (size_t i = 0; i < num_points_RIGHT; ++i) {
        X_RIGHT(i) = lane_point_RIGHT.point[i].x;
        Y_RIGHT(i) = lane_point_RIGHT.point[i].y;
    }
    if (X_LEFT.size() >= 4 && X_RIGHT.size() >= 4){
        A_LEFT = calculateA(X_LEFT, Y_LEFT);
        A_RIGHT = calculateA(X_RIGHT, Y_RIGHT);
    }
    // RCLCPP_WARN(this->get_logger(), "Number of points in LEFT lane: %zu", num_points_LEFT);
    // RCLCPP_WARN(this->get_logger(), "Number of points in RIGHT lane: %zu", num_points_RIGHT);

}

std::tuple<double, double, double, double> DrivingWayNode::computeCubicModel(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2,
    const geometry_msgs::msg::Point& p3,
    const geometry_msgs::msg::Point& p4) {

    Eigen::Matrix4d X;
    Eigen::Vector4d Y;

    X << std::pow(p1.x, 3), std::pow(p1.x, 2), p1.x, 1,
         std::pow(p2.x, 3), std::pow(p2.x, 2), p2.x, 1,
         std::pow(p3.x, 3), std::pow(p3.x, 2), p3.x, 1,
         std::pow(p4.x, 3), std::pow(p4.x, 2), p4.x, 1;

    Y << p1.y, p2.y, p3.y, p4.y;

    Eigen::Vector4d coeffs = X.colPivHouseholderQr().solve(Y);
    return {coeffs(0), coeffs(1), coeffs(2), coeffs(3)};
}

double DrivingWayNode::pointToCubicDistance(
    const geometry_msgs::msg::Point& point,
    double a, double b, double c, double d) {

    double y_estimated = a * std::pow(point.x, 3) + b * std::pow(point.x, 2) + c * point.x + d;
    return std::abs(point.y - y_estimated);
}

Eigen::Vector4d DrivingWayNode::calculateA(const Eigen::VectorXd& X, const Eigen::VectorXd& Y) {

    Eigen::MatrixXd X_matrix(X.size(), 4); // 3차 다항식의 경우 4개의 열
    for (int i = 0; i < X.size(); ++i) {
        X_matrix(i, 0) = std::pow(X(i), 3);  // x^3
        X_matrix(i, 1) = std::pow(X(i), 2);  // x^2
        X_matrix(i, 2) = X(i);               // x
        X_matrix(i, 3) = 1;                  // 상수항
    }
    Eigen::Vector4d A = X_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);

    return A; // Vector4d로 반환
}

std::string DrivingWayNode::Vector4dToString(const Eigen::Vector4d& vec) {
    std::ostringstream oss;
    for (int i = 0; i < 4; ++i) {  // Vector4d는 항상 크기가 4입니다.
        if (i > 0) {
            oss << ", ";
        }
        oss << vec(i);
    }
    return oss.str();
}

void DrivingWayNode::populatePolyLanes(ad_msgs::msg::PolyfitLaneDataArray& poly_lanes) {

    // poly_lanes 초기화
    poly_lanes.polyfitlanes.clear(); // 이전 데이터 초기화

    // 오른쪽 차선 (id = "1", A_RIGHT)
    ad_msgs::msg::PolyfitLaneData lane_right;
    lane_right.id = "1";  // id 설정
    lane_right.a0 = A_RIGHT(3);  // a0 계수
    lane_right.a1 = A_RIGHT(2);  // a1 계수
    lane_right.a2 = A_RIGHT(1);  // a2 계수
    lane_right.a3 = A_RIGHT(0);  // a3 계수
    lane_right.frame_id = "ego/body";  // frame_id 설정
    poly_lanes.polyfitlanes.push_back(lane_right);  // 오른쪽 차선을 polyfitlanes에 추가

    // 왼쪽 차선 (id = "2", A_LEFT)
    ad_msgs::msg::PolyfitLaneData lane_left;
    lane_left.id = "2";  // id 설정
    lane_left.a0 = A_LEFT(3);  // a0 계수
    lane_left.a1 = A_LEFT(2);  // a1 계수
    lane_left.a2 = A_LEFT(1);  // a2 계수
    lane_left.a3 = A_LEFT(0);  // a3 계수
    lane_left.frame_id = "ego/body";  // frame_id 설정
    poly_lanes.polyfitlanes.push_back(lane_left);  // 왼쪽 차선을 polyfitlanes에 추가

}

void DrivingWayNode::populateCenterLane(ad_msgs::msg::PolyfitLaneData& driving_way) {
    driving_way.frame_id = "ego/body";
    driving_way.id = "0"; // id 설정 아직 중앙이 0인줄 모름
    A_MID(0) = (A_RIGHT(3) + A_LEFT(3)) / 2.0;  // a0 계수
    A_MID(1) = (A_RIGHT(2) + A_LEFT(2)) / 2.0;  // a1 계수
    A_MID(2) = (A_RIGHT(1) + A_LEFT(1)) / 2.0;  // a2 계수
    A_MID(3) = (A_RIGHT(0) + A_LEFT(0)) / 2.0;  // a3 계수

    driving_way.a0 = A_MID(0);  // a0 계수
    driving_way.a1 = A_MID(1);  // a1 계수
    driving_way.a2 = A_MID(2);  // a2 계수
    driving_way.a3 = A_MID(3);  // a3 계수
    // 석광훈 
}

//-------------------------------------------------------------------------------------------------------------------//

void DrivingWayNode::PublishDrivingWay(const rclcpp::Time& current_time) {
    // RCLCPP_INFO(this->get_logger(), "Publishing at time: %f", current_time.seconds());
    p_driving_way_->publish(o_driving_way_);
    p_poly_lanes_->publish(o_poly_lanes_);
}
int main(int argc, char **argv) {
    std::string node_name = "driving_way_node";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivingWayNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}

