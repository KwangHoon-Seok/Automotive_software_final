# include "parking_node.hpp"

ParkingNode::ParkingNode(const std::string& node_name, const double& loop_rate, const rclcpp::NodeOptions& options)
    : Node(node_name, options){
    RCLCPP_INFO(this->get_logger(), "Initializing ParkingNode..." );

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", qos_profile,
        std::bind(&ParkingNode::CallbackVehicleState, this, std::placeholders::_1));

    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", qos_profile,
        std::bind(&ParkingNode::CallbackMissionState, this, std::placeholders::_1));

    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "/ego/mission", qos_profile,
        std::bind(&ParkingNode::CallbackLanePoints, this, std::placeholders::_1));


    // Publishers
    p_lane_parking_ = this->create_publisher<ad_msgs::msg::LanePointData>("/ego/lane_parking", qos_profile);
    p_parking_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>("/ego/parking_way", qos_profile);
    

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

}
ParkingNode::~ParkingNode() {};

void ParkingNode::Run(const rclcpp::Time& current_time){
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleState vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_mission_state_.lock();
    ad_msgs::msg::Mission mission_state = i_mission_state_;
    mutex_mission_state_.unlock();

    mutex_mission_state_.lock();
    ad_msgs::msg::LanePointData lane_point = i_lane_points_;
    mutex_mission_state_.unlock();

    while(vehicle_state.yaw> M_PI){
            vehicle_state.yaw -= 2 * M_PI;
    }
    while(vehicle_state.yaw < - M_PI){
            vehicle_state.yaw += 2 * M_PI;
    }

    // Point 개수가 너무작으면 의미없는 clustering일 수 있음.
    if(i_lane_point.point.size() > 5){
        dbscan(lane_point, vehicle_state);
    }

    o_lane_parking_ = lane_parking_;

    PublishParking(current_time);
   
    
}


float ParkingNode::distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// regionQuery 함수: 특정 점을 기준으로 epsilon 거리 내의 이웃 포인트들을 찾음
std::vector<int> ParkingNode::regionQuery(const std::vector<Point>& points, int pointIdx, float epsilon) {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (distance(points[pointIdx], points[i]) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

void ParkingNode::expandCluster(std::vector<Point>& points, int pointIdx, int clusterID, float epsilon, size_t min_samples) {
    std::vector<int> seeds = regionQuery(points, pointIdx, epsilon);
    points[pointIdx].cluster = clusterID;

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


void ParkingNode::dbscan(const ad_msgs::msg::LanePointData& lane_points, const ad_msgs::msg::VehicleState& vehicle_state) {
    float epsilon = 2.0;  // 밀도 거리
    size_t min_samples = 3;  // 최소 포인트 개수

    // 1. lane_points를 vector<Point>로 변환
    std::vector<Point> points;
    int count_x = 0;
    for (const auto& point : lane_points.point) {
        points.push_back({static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)});
        if (lane_points.point.x < 0 && !can_start){
            count_x ++;
        }

    }
    if(count_x < 1){
        can_start  = true;
    }
    else{
        can_start = false;
    }

    // 2. 모든 포인트에 대해 DBSCAN 클러스터링 수행
    int cluster_id = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].visited) {
            continue;  // 이미 처리된 포인트는 건너뜀
        }

        points[i].visited = true;
        std::vector<int> neighbors = regionQuery(points, i, epsilon);

        if (neighbors.size() < min_samples) {
            points[i].cluster = -1;  // 노이즈로 처리
            continue;
        }

        // 클러스터 확장
        expandCluster(points, i, cluster_id, epsilon, min_samples);
        cluster_id++;
    }
    geometry_msgs::msg::Point new_point;
    for(size_t i = 0; i < points.size(); ++i){
        if(points.cluster != -1){
            new_point.x = point.x;
            new_point.y = point.y;
            new_point.z = point.z;
            lane_parking_.point.push_back(new_point); // lane_parking_에 추가
        }
    }

    // // 1. taget_global_parking_space 정하기

    // if(is_init && can_start){
    //     float min_y = 100;
    //     int target_cluster = 100;
    //     for(size_t i = 0; i < points.size(); ++i){
    //         if(min_y > points[i].y){
    //             min_y = points[i].y;
    //             target_cluster = points[i].cluster
    //         }
    //     }

    //     if(target_cluster != -1){
    //         float sum_x = 0.0, sum_y = 0.0;
    //         int count = 0;
            
    //         for(size_t i = 0; i < points.size(); ++i){
    //             if(points.cluster == target_cluster){
    //                 sum_x += point.x;
    //                 sum_y += point.y;
    //                 count ++;
    //             }
    //         }

    //         if(count > 0){
    //             float mean_x = sum_x / count;
    //             float mean_y = sum_y / count;
    //             Eigen::Vector3d global = Local2Global(mean_x, mean_y);
    //             target_global_space.x = global(0);
    //             target_global_space.y = global(1);
    //             is_init = true;
    //         }
    //     }
    // }
    // else if(!is_init){

    // }

    
}

Eigen::Vector3d ParkingNode::Local2Global(float x, float y, const ad_msgs::msg::VehicleState& vehicle_state){
    Eigen::Vector3d Global;
    Eigen::Vector3d Local;
    Eigen::Matrix3d G2L;
    
    Local(0) = x;
    Local(1) = y;
    Local(2) = 1;
    
    L2G << std::cos(vehicle_state.yaw), -std::sin(vehicle_state.yaw), vehicle_state.x,
           -std::sin(vehicle_state.yaw),  std::cos(vehicle_state.yaw), vehicle_state.y,
           0,               0,               1;

    Global = L2G * Local;
    return Global;

}

Eigen::Vector3d ParkingNode::Global2Local(float x, float y, const ad_msgs::msg::VehicleState& vehicle_state) {
    
    Eigen::Vector3d Global;
    Eigen::Vector3d Local;
    Eigen::Matrix3d G2L;

    Global(0) = x;
    Global(1) = y;
    Global(2) = 1;

   
    Eigen::Matrix3d G2L;
    float yaw = vehicle_state.yaw; // 차량의 방향 (라디안)
    float x_g = vehicle_state.x;   // 차량의 전역 x 좌표
    float y_g = vehicle_state.y;   // 차량의 전역 y 좌표

    G2L << std::cos(vehicle_state.yaw),  std::sin(vehicle_state.yaw), -(vehicle_state.x * std::cos(vehicle_state.yaw) + vehicle_state.y * std::sin(vehicle_state.yaw)),
           -std::sin(vehicle_state.yaw), std::cos(vehicle_state.yaw),  (vehicle_state.x * std::sin(vehicle_state.yaw) - vehicle_state.y * std::cos(vehicle_state.yaw)),
           0,              0,             1;

    // Local 좌표 계산
    Local = G2L * Global;

    // 결과 반환
    return Local;
}


void ParkingNode::PublishParking(const rclcpp::Time& current_time) {
    // RCLCPP_INFO(this->get_logger(), "Publishing at time: %f", current_time.seconds());
    p_lane_parking_->publish(o_lane_parking_);
    RCLCPP_INFO(this->get_logger(), "Parking lane size : %zu ", o_lane_parking_.point.size());
}




int main(int argc, char **argv) {
    std::string node_name = "parking_node";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParkingNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}