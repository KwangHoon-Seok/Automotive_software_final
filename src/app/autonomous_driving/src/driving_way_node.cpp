#include "driving_way_node.hpp"

DrivingWayNode::DrivingWayNode(const std::string& node_name, const double& loop_rate, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    RCLCPP_INFO(this->get_logger(), "Initializing DrivingWayNode...");
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "/ego/lane_points", qos_profile,
        std::bind(&DrivingWayNode::CallbackLanePoints, this, std::placeholders::_1));

    s_behavior_state_ = this->create_subscription<std_msgs::msg::Float32>(
        "/behavior_state", qos_profile, 
        std::bind(&DrivingWayNode::CallbackBehaviorState, this, std::placeholders::_1));
    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", qos_profile,
        std::bind(&DrivingWayNode::CallbackMissionState, this, std::placeholders::_1));

    // Publishers
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>("/ego/driving_way", qos_profile);
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>("/ego/poly_lanes", qos_profile);
    p_left_lane_ = this->create_publisher<ad_msgs::msg::LanePointData>("/ego/left_lane", qos_profile);
    p_right_lane_ = this->create_publisher<ad_msgs::msg::LanePointData>("/ego/right_lane", qos_profile);
    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

    lane_point_LEFT.id = "LEFT";
    lane_point_LEFT.frame_id = "ego/body";
    lane_point_RIGHT.id = "RIGHT";
    lane_point_RIGHT.frame_id = "ego/body";
    sub_lane_point_LEFT_.id = "SUB_LEFT";
    sub_lane_point_LEFT_.frame_id = "ego/body";
    sub_lane_point_RIGHT_.id = "SUB_RIGHT";
    sub_lane_point_RIGHT_.frame_id = "ego/body";

}

DrivingWayNode::~DrivingWayNode() {}

void DrivingWayNode::Run(const rclcpp::Time& current_time) {
    
    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointData lane_points = i_lane_points_;
    mutex_lane_points_.unlock();

    mutex_behavior_.lock();
    std_msgs::msg::Float32 behavior_state = i_behavior_state_;
    mutex_behavior_.unlock();
    
    mutex_mission_state_.lock();
    ad_msgs::msg::Mission mission_state = i_mission_state_;
    mutex_mission_state_.unlock();
    
    if (lane_points.point.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No lane points received. Skipping processing...");
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(), " lane point size: %zu", lane_points.point.size());
    }

    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    poly_lanes.frame_id = "ego/body";
    ad_msgs::msg::PolyfitLaneData driving_way;
    driving_way.frame_id = "ego/body";
    ad_msgs::msg::LanePointData left_lane;
    left_lane.frame_id = "ego/body";
    ad_msgs::msg::LanePointData right_lane;
    right_lane.frame_id = "ego/body";

    
    RCLCPP_INFO(this->get_logger(), "PARKING : %s", mission_state.parking ? "true" : "false");

    
    if(start_parking || mission_state.parking == true){
        RCLCPP_INFO(this->get_logger(), "START PARKING");
        getMeanPoints(lane_points, left_lane, right_lane);
        getParkingWay(driving_way);
    }
    else if (!start_parking){
        splitLanePoints(lane_points);
        process_lanes(left_lane, right_lane);
        populatePolyLanes(poly_lanes);
        populateCenterLane(driving_way);
        lane_condition(driving_way, behavior_state, lane_points);
    }
    
    
    
    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;
    o_left_lane_ = left_lane;
    o_right_lane_ = right_lane;
    

    PublishDrivingWay(current_time);
}

void DrivingWayNode::getMeanPoints(const ad_msgs::msg::LanePointData& lane_points, ad_msgs::msg::LanePointData& left_lane, ad_msgs::msg::LanePointData& right_lane){

    left_lane.point.clear();
    right_lane.point.clear();

    for(const auto & point : lane_points.point){
        left_lane.point.push_back(point);
    }

    mean_point.x = 0.0;
    mean_point.y = 0.0;

    for(const auto& point : lane_points.point){
        mean_point.x += point.x;
        mean_point.y += point.y;
    }

    mean_point.x /= lane_points.point.size();
    mean_point.y /= lane_points.point.size();

    right_lane.point.push_back(mean_point);



    if(mean_point.x < 0.0){
        is_completed = true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Mean Point (X: %.4f , Y: %.4f)", mean_point.x, mean_point.y);
    RCLCPP_INFO(this->get_logger(), "Complete : %s" , is_completed ? "True" : "False");

}

void DrivingWayNode::getParkingWay(ad_msgs::msg::PolyfitLaneData& driving_way){
    driving_way.frame_id = "ego/body";
    double x0 = 0.0;
    double y0 = 0.0;
    double slope_start = 0.0;
    double slope_end = 0.0; 

    // Eigen 행렬 정의
    Eigen::Matrix4d A;
    Eigen::Vector4d b;

    // 연립 방정식 행렬 정의
    A << std::pow(x0, 3), std::pow(x0, 2), x0, 1,
         std::pow(mean_point.x, 3), std::pow(mean_point.x, 2), mean_point.x, 1,
         3 * std::pow(x0, 2), 2 * x0, 1, 0,
         3 * std::pow(mean_point.x, 2), 2 * mean_point.x, 1, 0;

    // 결과 벡터 정의
    b << y0, mean_point.y, slope_start, slope_end;

    // 연립 방정식 풀이
    Eigen::Vector4d coeffs = A.colPivHouseholderQr().solve(b);

    driving_way.a3 = coeffs(0);
    driving_way.a2 = coeffs(1);
    driving_way.a1 = coeffs(2);
    driving_way.a0 = coeffs(3);

}

void DrivingWayNode::lane_condition(ad_msgs::msg::PolyfitLaneData& driving_way, std_msgs::msg::Float32& behavior_state, const ad_msgs::msg::LanePointData& lane_points){

    // is_error를 판별하는 조건문 -> merge때 나 아예 경로 이탈을 해결
    // if((driving_way.a0 > 0.1 || driving_way.a3 < -8.1)){
    //     is_error = true;
    //     // RCLCPP_INFO(this->get_logger(), " ERROR ");
    // }
    if(driving_way.a3 > 0.1 || driving_way.a3 < -8.1){
        is_error = true;
        RCLCPP_INFO(this->get_logger(), " ERROR ");
        
    }
    else if (will_parking && !start_parking){
        driving_way = prev_driving_way_;
        if (max_x > 15){
            start_parking = true;
        }
    }
    // PARKING 전
    else if((max_x < 10 && lane_points.point.size() < 40)){
        will_parking = true;
    }
    else{
        prev_driving_way_ = driving_way;
        is_error = false;
        // RCLCPP_INFO(this->get_logger(), " Normal driving mode ");
    }

    
    lane_point_LEFT.point.clear();
    lane_point_RIGHT.point.clear();
}



// Functions
float DrivingWayNode::distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// regionQuery 함수: 특정 점을 기준으로 epsilon 거리 내의 이웃 포인트들을 찾음
std::vector<int> DrivingWayNode::regionQuery(const std::vector<Point>& points, int pointIdx, float epsilon) {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (distance(points[pointIdx], points[i]) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// expandCluster 함수: DBSCAN에서 클러스터 확장 수행
void DrivingWayNode::expandCluster(std::vector<Point>& points, int pointIdx, int clusterID, float epsilon, size_t min_samples) {
    std::vector<int> seeds = regionQuery(points, pointIdx, epsilon);
    if (seeds.size() < min_samples) {
        points[pointIdx].cluster = -1; // 노이즈로 처리
        return;
    }

    // 초기 포인트 설정
    points[pointIdx].cluster = clusterID;

    // 클러스터 확장
    for (size_t i = 0; i < seeds.size(); ++i) {
        int idx = seeds[i];
        if (!points[idx].visited) {
            points[idx].visited = true;
            std::vector<int> result = regionQuery(points, idx, epsilon);
            if (result.size() >= min_samples) {
                seeds.insert(seeds.end(), result.begin(), result.end());
            }
        }
        if (points[idx].cluster == -1) {
            points[idx].cluster = clusterID;
        }
    }
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

        // // lane_point 초기화
        // lane_point_LEFT.point.clear();
        // lane_point_RIGHT.point.clear();

        for(const auto & point : points){
            if(point.y < 0){
                lane_point_RIGHT.point.push_back(point);
            }
            else if(point.y > 0){
                lane_point_LEFT.point.push_back(point);
            }
        }
        is_init = true;
        
    }

    else if (is_init && !is_error) {
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

        // // lane_point 초기화
        // lane_point_LEFT.point.clear();
        // lane_point_RIGHT.point.clear();

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


    }

    else if(is_init && is_error){
        float epsilon = 3.5;  // 밀도 거리
        size_t min_samples = 3;  // 최소 포인트 개수

        // 1. lane_points를 vector<Point>로 변환
        std::vector<Point> points;
        for (const auto& point : lane_points.point) {
            points.push_back({static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)});
        }

        // 2. y > 0과 y < 0에 따라 가장 가까운 포인트(p0, p1)를 선택
        Point p0, p1;
        bool p0_found = false, p1_found = false;
        float min_distance_p0 = std::numeric_limits<float>::max();
        float min_distance_p1 = std::numeric_limits<float>::max();

        for (const auto& point : points) {
            float distance = std::hypot(point.x, point.y);
            if (point.y > 0 && distance < min_distance_p0) {
                min_distance_p0 = distance;
                p0 = point;
                p0_found = true;
            } else if (point.y < 0 && distance < min_distance_p1) {
                min_distance_p1 = distance;
                p1 = point;
                p1_found = true;
            }
        }

        // 3. 초기 좌/우 차선에 대해 클러스터링을 수행할 준비
        // lane_point_LEFT.point.clear();
        // lane_point_RIGHT.point.clear();

        // 4. 좌측 차선 DBSCAN 시작 (p0 기준으로 expandCluster 호출)
        if (p0_found) {
            size_t p0_index = std::distance(points.begin(), std::find(points.begin(), points.end(), p0));
            expandCluster(points, p0_index, 0, epsilon, min_samples); // 좌측 클러스터 ID는 0
            geometry_msgs::msg::Point new_point;

            for (const auto& point : points) {
                if (point.cluster == 0) {  // 좌측 차선에 속하는 포인트
                    new_point.x = point.x;
                    new_point.y = point.y;
                    new_point.z = point.z;
                    lane_point_LEFT.point.push_back(new_point);
                }
            }
        }

        // 5. 우측 차선 DBSCAN 시작 (p1 기준으로 expandCluster 호출)
        if (p1_found) {
            size_t p1_index = std::distance(points.begin(), std::find(points.begin(), points.end(), p1));
            expandCluster(points, p1_index, 1, epsilon, min_samples); // 우측 클러스터 ID는 1
            geometry_msgs::msg::Point new_point;

            for (const auto& point : points) {
                if (point.cluster == 1) {  // 우측 차선에 속하는 포인트
                    
                    new_point.x = point.x;
                    new_point.y = point.y;
                    new_point.z = point.z;
                    lane_point_RIGHT.point.push_back(new_point);
                }
            }
        }


    }
    


}


void DrivingWayNode::process_lanes(ad_msgs::msg::LanePointData& left_lane, ad_msgs::msg::LanePointData& right_lane) {
    
    left_lane.point.clear();
    right_lane.point.clear();
    sub_lane_point_LEFT_.point.clear();
    sub_lane_point_RIGHT_.point.clear();
    num_points_LEFT = lane_point_LEFT.point.size();
    num_points_RIGHT = lane_point_RIGHT.point.size();
    max_x = 0.0;
    
    // float lookahead = 10.0;
    // float interval = 50.0;
    // geometry_msgs::msg::Point sub_point;
    geometry_msgs::msg::Point point;

    if (lane_point_LEFT.point.size() <= lane_point_RIGHT.point.size()){
        
        X_LEFT.resize(num_points_LEFT + num_points_RIGHT);
        Y_LEFT.resize(num_points_LEFT + num_points_RIGHT);
        X_RIGHT.resize(num_points_RIGHT);
        Y_RIGHT.resize(num_points_RIGHT);

        for (size_t i = 0; i < num_points_LEFT; ++i) {
            if (max_x < lane_point_LEFT.point[i].x){
                max_x = lane_point_LEFT.point[i].x;
            }
            X_LEFT(i) = lane_point_LEFT.point[i].x;
            Y_LEFT(i) = lane_point_LEFT.point[i].y;
            left_lane.point.push_back(lane_point_LEFT.point[i]);
        }

        for (size_t i = 0; i < num_points_RIGHT; ++i) {
            if (max_x < lane_point_RIGHT.point[i].x){
                max_x = lane_point_RIGHT.point[i].x;
            }
            X_RIGHT(i) = lane_point_RIGHT.point[i].x;
            Y_RIGHT(i) = lane_point_RIGHT.point[i].y;
            right_lane.point.push_back(lane_point_RIGHT.point[i]);
        }

        for (size_t i = 0; i < num_points_RIGHT; ++i) {
            point.x = lane_point_RIGHT.point[i].x;
            point.y = lane_point_RIGHT.point[i].y + 4.0;
            X_LEFT(i + num_points_LEFT) = point.x;
            Y_LEFT(i + num_points_LEFT) = point.y;
            left_lane.point.push_back(point);
        }

    }
    else if (lane_point_LEFT.point.size() > lane_point_RIGHT.point.size()){

        X_LEFT.resize(num_points_LEFT);
        Y_LEFT.resize(num_points_LEFT);
        X_RIGHT.resize(num_points_RIGHT + num_points_LEFT);
        Y_RIGHT.resize(num_points_RIGHT + num_points_LEFT);

        for (size_t i = 0; i < num_points_LEFT; ++i) {
            if (max_x < lane_point_LEFT.point[i].x){
                max_x = lane_point_LEFT.point[i].x;
            }
            X_LEFT(i) = lane_point_LEFT.point[i].x;
            Y_LEFT(i) = lane_point_LEFT.point[i].y;
            left_lane.point.push_back(lane_point_LEFT.point[i]);
        }

        for (size_t i = 0; i < num_points_RIGHT; ++i) {
            if (max_x < lane_point_RIGHT.point[i].x){
                max_x = lane_point_RIGHT.point[i].x;
            }
            X_RIGHT(i) = lane_point_RIGHT.point[i].x;
            Y_RIGHT(i) = lane_point_RIGHT.point[i].y;
            right_lane.point.push_back(lane_point_RIGHT.point[i]);
        }

        for (size_t i = 0; i < num_points_LEFT; ++i) {
            point.x = lane_point_LEFT.point[i].x;
            point.y = lane_point_LEFT.point[i].y - 4.0;
            X_RIGHT(i + num_points_RIGHT) = point.x;
            Y_RIGHT(i + num_points_RIGHT) = point.y;
            right_lane.point.push_back(point);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "LANE MAX.X : %.3f", max_x);
    
    if (X_LEFT.size() >= 4 && X_RIGHT.size() >= 4){
        A_LEFT = calculateA(X_LEFT, Y_LEFT);
        A_RIGHT = calculateA(X_RIGHT, Y_RIGHT);
    }
   
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
    if (X_LEFT.size() >= 4 && X_RIGHT.size() >= 4){
        // poly_lanes 초기화
        poly_lanes.polyfitlanes.clear(); // 이전 데이터 초기화

        // 오른쪽 차선 (id = "1", A_RIGHT)
        ad_msgs::msg::PolyfitLaneData lane_right;
        lane_right.id = "RIGHT";  // id 설정
        lane_right.a0 = A_RIGHT(3);  // a0 계수
        lane_right.a1 = A_RIGHT(2);  // a1 계수
        lane_right.a2 = A_RIGHT(1);  // a2 계수
        lane_right.a3 = A_RIGHT(0);  // a3 계수
        lane_right.frame_id = "ego/body";  // frame_id 설정
        poly_lanes.polyfitlanes.push_back(lane_right);  // 오른쪽 차선을 polyfitlanes에 추가

        // 왼쪽 차선 (id = "2", A_LEFT)
        ad_msgs::msg::PolyfitLaneData lane_left;
        lane_left.id = "LEFT";  // id 설정
        lane_left.a0 = A_LEFT(3);  // a0 계수
        lane_left.a1 = A_LEFT(2);  // a1 계수
        lane_left.a2 = A_LEFT(1);  // a2 계수
        lane_left.a3 = A_LEFT(0);  // a3 계수
        lane_left.frame_id = "ego/body";  // frame_id 설정
        poly_lanes.polyfitlanes.push_back(lane_left);  // 왼쪽 차선을 polyfitlanes에 추가
    }
}

void DrivingWayNode::populateCenterLane(ad_msgs::msg::PolyfitLaneData& driving_way) {
    if (X_LEFT.size() >= 4 && X_RIGHT.size() >= 4){
        driving_way.frame_id = "ego/body";
        driving_way.id = "MID"; // id 설정 아직 중앙이 0인줄 모름
        A_MID(0) = (A_RIGHT(3) + A_LEFT(3)) / 2.0;  // a0 계수
        A_MID(1) = (A_RIGHT(2) + A_LEFT(2)) / 2.0;  // a1 계수
        A_MID(2) = (A_RIGHT(1) + A_LEFT(1)) / 2.0;  // a2 계수
        A_MID(3) = (A_RIGHT(0) + A_LEFT(0)) / 2.0;  // a3 계수

        driving_way.a0 = A_MID(0);  // a0 계수
        driving_way.a1 = A_MID(1);  // a1 계수
        driving_way.a2 = A_MID(2);  // a2 계수
        driving_way.a3 = A_MID(3);  // a3 계수
    }

}

//-------------------------------------------------------------------------------------------------------------------//

void DrivingWayNode::PublishDrivingWay(const rclcpp::Time& current_time) {
    
    p_driving_way_->publish(o_driving_way_);
    p_poly_lanes_->publish(o_poly_lanes_);
    p_left_lane_->publish(o_left_lane_);
    p_right_lane_->publish(o_right_lane_);
    
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