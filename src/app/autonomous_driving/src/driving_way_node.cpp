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
    
    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointData lane_points = i_lane_points_;
    mutex_lane_points_.unlock();

    mutex_behavior_.lock();
    std_msgs::msg::Float32 behavior_state = i_behavior_state_;
    mutex_behavior_.unlock();
    
    
    if (lane_points.point.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No lane points received. Skipping processing...");
        return;
    }
    else{
        // RCLCPP_INFO(this->get_logger(), " lane point size: %zu", lane_points.point.size());
    }

    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    poly_lanes.frame_id = "ego/body";
    ad_msgs::msg::PolyfitLaneData driving_way;
    driving_way.frame_id = "ego/body";
    
    
    
    splitLanePoints(lane_points);
    process_lanes();
    populatePolyLanes(poly_lanes);
    populateCenterLane(driving_way);
    lane_condition(driving_way, behavior_state);
    
    
    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;


    PublishDrivingWay(current_time);
}
// driving_way가 너무 꺾이면 error라고 생각해서 -> DBSCAN으로 분류


void DrivingWayNode::lane_condition(ad_msgs::msg::PolyfitLaneData& driving_way, const std_msgs::msg::Float32& behavior_state){

    // is_error를 판별하는 조건문 -> merge때 나 아예 경로 이탈을 해결
    if((driving_way.a3 > 0.1 || driving_way.a3 < -8.1)){
        is_error = true;
        // RCLCPP_INFO(this->get_logger(), " ERROR ");
    }
    else{
        prev_driving_way_ = driving_way;
        is_error = false;
        // RCLCPP_INFO(this->get_logger(), " Normal driving mode ");
        
    }

    // if(behavior_state.data != 3.0){
    //     if((lane_point_LEFT.point.size() < 10 && lane_point_RIGHT.point.size() > 10) || lane_point_LEFT.point.size() > 10 && lane_point_RIGHT.point.size() < 10){
    //         is_pass = true;
    //     }
    // }

    // RCLCPP_INFO(this->get_logger(), "is_error = %s", is_error ? "true" : "false");
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
        // RCLCPP_INFO(this->get_logger(), "[--1st--] Split Lane Points");
        
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

        // RCLCPP_INFO(this->get_logger(), "[--2st--] Split Lane Points with prev_driving_way_");
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

        // RCLCPP_INFO(this->get_logger(), "Lane points split using DBSCAN with initial points p0 and p1.");
    }
    


}


// 이전 Polyfit Lane LEFT와 RIGHT를 기준으로 3만큼의 Sub point를 생성해 보조 주행 수행 추가
void DrivingWayNode::process_lanes() {
    
    num_points_LEFT = lane_point_LEFT.point.size();
    num_points_RIGHT = lane_point_RIGHT.point.size();
    if(lane_point_LEFT.point.size() > 5){
        sub_lane_point_LEFT_.point.clear();
    }
    if(lane_point_LEFT.point.size() > 5){
        sub_lane_point_RIGHT_.point.clear();
    }
    float offset = 2;
    geometry_msgs::msg::Point sub_point;
    if (lane_point_LEFT.point.size() + lane_point_RIGHT.point.size() < 25){
        RCLCPP_INFO(this->get_logger(), " Sub LANE ");
        for (int i = 0; i <= 10; ++i) { // 0부터 4까지 총 5개 점 생성
            float x = i * (3.0 / 10.0); // 0, 0.75, 1.5, 2.25, 3의 값 생성
            float y = A_LEFT(0) * std::pow(x, 3) +
                    A_LEFT(1) * std::pow(x, 2) +
                    A_LEFT(2) * x +
                    A_LEFT(3);

            sub_point.x = x;
            sub_point.y = y;
            sub_point.z = 0.0;

            sub_lane_point_LEFT_.point.push_back(sub_point);
        }

        for (int i = 0; i <= 10; ++i) { // 0부터 4까지 총 5개 점 생성
            float x = i * (3.0 / 10.0); // 0, 0.75, 1.5, 2.25, 3의 값 생성
            float y = A_RIGHT(0) * std::pow(x, 3) +
                    A_RIGHT(1) * std::pow(x, 2) +
                    A_RIGHT(2) * x +
                    A_RIGHT(3);

            sub_point.x = x;
            sub_point.y = y;
            sub_point.z = 0.0;

            sub_lane_point_RIGHT_.point.push_back(sub_point);
        }
        num_sub_points_LEFT = sub_lane_point_LEFT_.point.size();
        num_sub_points_RIGHT = sub_lane_point_RIGHT_.point.size();

        X_LEFT.resize(num_points_LEFT + num_sub_points_LEFT);
        Y_LEFT.resize(num_points_LEFT + num_sub_points_LEFT);
        X_RIGHT.resize(num_points_RIGHT + num_sub_points_RIGHT);
        Y_RIGHT.resize(num_points_RIGHT + num_sub_points_RIGHT);

        for (size_t i = 0; i < num_points_LEFT; ++i) {
            X_LEFT(i) = lane_point_LEFT.point[i].x;
            Y_LEFT(i) = lane_point_LEFT.point[i].y;
        }

        for (size_t i = 0; i < num_points_RIGHT; ++i) {
            X_RIGHT(i) = lane_point_RIGHT.point[i].x;
            Y_RIGHT(i) = lane_point_RIGHT.point[i].y;
        }
        for(size_t i = 0; i < num_sub_points_LEFT; ++i){
            X_LEFT(num_points_LEFT + i) = sub_lane_point_LEFT_.point[i].x;
            Y_LEFT(num_points_LEFT + i) = sub_lane_point_LEFT_.point[i].y;
        }

        for(size_t i = 0; i < num_sub_points_RIGHT; ++i){
            X_RIGHT(num_points_RIGHT + i) = sub_lane_point_RIGHT_.point[i].x;
            Y_RIGHT(num_points_RIGHT + i) = sub_lane_point_RIGHT_.point[i].y;
        }

    }
    else{
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
    }
    // // lane_point_복사
    // if (lane_point_LEFT.point.size() + lane_point_RIGHT.point.size() < 20){
    //     if(lane_point_LEFT.point.size() < 4 && lane_point_RIGHT.point.size() < 4){
    //         RCLCPP_INFO(this->get_logger(), " [1] Sample copy for ALL");
    //         X_LEFT.resize(num_points_LEFT + num_points_RIGHT);
    //         Y_LEFT.resize(num_points_LEFT + num_points_RIGHT);
    //         X_RIGHT.resize(num_points_LEFT + num_points_RIGHT);
    //         Y_RIGHT.resize(num_points_LEFT + num_points_RIGHT);
    //         for (size_t i = 0; i < num_points_LEFT; ++i) {
    //             X_LEFT(i) = lane_point_LEFT.point[i].x;
    //             Y_LEFT(i) = lane_point_LEFT.point[i].y;
    //         }
    //         for (size_t i = 0; i < num_points_RIGHT; ++i) {
    //             X_RIGHT(i) = lane_point_RIGHT.point[i].x;
    //             Y_RIGHT(i) = lane_point_RIGHT.point[i].y;
    //         }

    //         for(size_t i = 0; i < num_points_RIGHT; ++i){
    //             X_LEFT(num_points_LEFT + i) = lane_point_RIGHT.point[i].x;
    //             Y_LEFT(num_points_LEFT + i) = lane_point_RIGHT.point[i].y - offset;
    //         }

    //         for(size_t i = 0; i < num_points_LEFT; ++i){
    //             X_RIGHT(num_points_RIGHT + i) = lane_point_LEFT.point[i].x;
    //             Y_RIGHT(num_points_RIGHT + i) = lane_point_LEFT.point[i].y + offset;
    //         }
    //     }

    //     else if(lane_point_LEFT.point.size() < lane_point_RIGHT.point.size()){
    //         RCLCPP_INFO(this->get_logger(), " [2] Sample copy for Left");
    //         X_RIGHT.resize(num_points_RIGHT);
    //         Y_RIGHT.resize(num_points_RIGHT);
    //         X_LEFT.resize(num_points_LEFT + num_points_RIGHT);
    //         Y_LEFT.resize(num_points_LEFT + num_points_RIGHT);
    //         for (size_t i = 0; i < num_points_RIGHT; ++i) {
    //             X_RIGHT(i) = lane_point_RIGHT.point[i].x;
    //             Y_RIGHT(i) = lane_point_RIGHT.point[i].y;
    //         }

    //         for(size_t i = 0; i < num_points_RIGHT; ++i){
    //             X_LEFT(num_points_LEFT + i) = lane_point_RIGHT.point[i].x;
    //             Y_LEFT(num_points_LEFT + i) = lane_point_RIGHT.point[i].y - offset;
    //         }
            
    //     }
    //     else if(lane_point_LEFT.point.size() >= lane_point_RIGHT.point.size()){
    //         RCLCPP_INFO(this->get_logger(), " [3] Sample copy for Right ");
    //         X_LEFT.resize(num_points_LEFT);
    //         Y_LEFT.resize(num_points_LEFT);
    //         X_RIGHT.resize(num_points_LEFT + num_points_RIGHT);
    //         Y_RIGHT.resize(num_points_LEFT + num_points_RIGHT);

    //         for (size_t i = 0; i < num_points_LEFT; ++i) {
    //             X_LEFT(i) = lane_point_LEFT.point[i].x;
    //             Y_LEFT(i) = lane_point_LEFT.point[i].y;
    //         }

    //         for(size_t i = 0; i < num_points_LEFT; ++i){
    //             X_RIGHT(num_points_RIGHT + i) = lane_point_LEFT.point[i].x;
    //             Y_RIGHT(num_points_RIGHT + i) = lane_point_LEFT.point[i].y + offset;
    //         }
    //     }
    // }
    
   
    
    if (X_LEFT.size() >= 4 && X_RIGHT.size() >= 4){
        A_LEFT = calculateA(X_LEFT, Y_LEFT);
        A_RIGHT = calculateA(X_RIGHT, Y_RIGHT);
    }
    // RCLCPP_INFO(this->get_logger(), "Number of points in LEFT lane: %zu", lane_point_LEFT.point.size());
    // RCLCPP_WARN(this->get_logger(), "Number of points in RIGHT lane: %zu", lane_point_RIGHT.point.size());

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
}

void DrivingWayNode::populateCenterLane(ad_msgs::msg::PolyfitLaneData& driving_way) {
    if (X_LEFT.size() >= 4 && X_RIGHT.size() >= 4){
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
    }
    // RCLCPP_INFO(this->get_logger(), "Current Driving way A3 Coefficient: %f", driving_way.a3);
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

