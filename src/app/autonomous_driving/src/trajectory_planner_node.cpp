#include "trajectory_planner_node.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <stdexcept>

TrajectoryNode::TrajectoryNode(const std::string &node_name, const double &loop_rate, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {
    // Timer
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / loop_rate)),
        std::bind(&TrajectoryNode::Run, this));
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", 10,
        std::bind(&TrajectoryNode::CallbackVehicleState, this, std::placeholders::_1));

    s_ego_prediction_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/ego_prediction", 10,
        std::bind(&TrajectoryNode::CallbackEgoPrediction, this, std::placeholders::_1));

    s_object_prediction_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/object_prediction", 10,
        std::bind(&TrajectoryNode::CallbackObjectPrediction, this, std::placeholders::_1));
    
    s_behavior_state_ = this->create_subscription<std_msgs::msg::Float32>(
        "/ego/behavior_state", qos_profile,
        std::bind(&TrajectoryNode::CallbackBehaviorState, this, std::placeholders::_1));

    // Publishers
    p_trajectory_candidates_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "/ego/local_path_array", rclcpp::QoS(10));

    p_best_trajectory_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "/ego/merge_path", qos_profile);

    RCLCPP_INFO(this->get_logger(), "TrajectoryNode has been initialized.");
}

TrajectoryNode::~TrajectoryNode() {}

Point TrajectoryNode::TargetPoint(const ad_msgs::msg::PolyfitLaneData& driving_way, double x,double lateral_offset){
    double a0 = driving_way.a0;
    double a1 = driving_way.a1;
    double a2 = driving_way.a2;
    double a3 = driving_way.a3;

    double y_base = a3 * std::pow(x, 3) + a2 * std::pow(x, 2) + a1 * x + a0;
    
    double slope = 3 * a3 * std::pow(x, 2) + 2 * a2 * x + a1;

    double magnitude = std::sqrt(1 + slope * slope);
    double normal_dx = -slope / magnitude;
    double normal_dy = 1 / magnitude;
    
    // lateral offset 적용
    double x_offset = x + lateral_offset * normal_dx;
    double y_offset = y_base + lateral_offset * normal_dy;


    return {x_offset, y_offset};
}

void TrajectoryNode::Run() {
    // Lock the mutex for thread safety
    std::lock_guard<std::mutex> lock_vehicle_state(mutex_vehicle_state_);
    std::lock_guard<std::mutex> lock_behavior_state(mutex_behavior_state_);

    std_msgs::msg::Float32 behavior_state_float = i_behavior_state_;
    ad_msgs::msg::PolyfitLaneData driving_way = i_driving_way_;
    ad_msgs::msg::VehicleState vehicle_state = i_vehicle_state_;
    ad_msgs::msg::Mission object_prediction = i_object_prediction_;
    std::vector<Point> target_points;
    double drive_mode = static_cast<double>(behavior_state_float.data);

    Point current_position = {0.0 , 0.0};
    yaw = normalize(static_cast<double>(vehicle_state.yaw));

    double x = 40.0;
    double lateral_offset_min = -5.0;
    double lateral_offset_max = 5.0;
    double lateral_offset_step = 2.0;

    target_points.clear();

    // Generate target points
    target_points.push_back(current_position);

    for (double lateral_offset = lateral_offset_min; lateral_offset <= lateral_offset_max; lateral_offset += lateral_offset_step) {
        Point target_point = TargetPoint(driving_way, x, lateral_offset);
        target_points.push_back(target_point);
    }

    ad_msgs::msg::PolyfitLaneDataArray spline_array_msg;
    spline_array_msg.frame_id = "ego/body";

    for (size_t i = 1; i < target_points.size(); ++i) {
        double slope_start = 0.0;
        double slope_end = 0.0;
        std::vector<Point> segment_points = {target_points[0], target_points[i]};
        std::vector<double> coeffs = ComputeCubicSpline(segment_points, slope_start, slope_end);
        
        ad_msgs::msg::PolyfitLaneData spline_msg;
        spline_msg.frame_id = "ego/body";
        spline_msg.id = std::to_string(i);
        spline_msg.a0 = coeffs[3];
        spline_msg.a1 = coeffs[2];
        spline_msg.a2 = coeffs[1];
        spline_msg.a3 = coeffs[0];

        spline_array_msg.polyfitlanes.push_back(spline_msg);
    }
    // Publish trajectory candidates (always updated)
    o_trajectories_ = spline_array_msg;
    p_trajectory_candidates_->publish(o_trajectories_);

    // Evaluate costs and select the best trajectory
    static ad_msgs::msg::PolyfitLaneData best_path;
    static bool is_best_path = 0;
    static int is_flag = 0;

    if (!is_best_path){
        best_path.frame_id = "ego/body";
        best_path.id = "0";
        best_path.a0 = 0.0;
        best_path.a1 = 0.0;
        best_path.a2 = 0.0;
        best_path.a3 = 0.0;
        is_best_path = true;
    }
    

    if (drive_mode == MERGE) {
        double min_ttc = 10000;
        for (const auto& path : spline_array_msg.polyfitlanes) {
            double ttc = CalculateTTC(path, vehicle_state, object_prediction, 3.0, 0.1, 4);
            // Update best path only if conditions are met
            if (path.id == "5" && ttc > 2.0 && ttc < 3.0 && is_flag == 0) {
                best_path = path;
                is_flag = 1;
            }
        }
    }
    // Publish the best trajectory
    p_best_trajectory_->publish(best_path);
}


//--------------------------------------Local path functions------------------------------//
double TrajectoryNode::CalculateTTC(
    const ad_msgs::msg::PolyfitLaneData& path,
    const ad_msgs::msg::VehicleState& ego_state,
    const ad_msgs::msg::Mission& object_prediction,
    double time_horizon,
    double interval,
    double collision_threshold) {
    
    double ttc = 20000;

    // Ego 경로 샘플링
    std::vector<prediction_points> ego_path_points = SampleEgoPath(path, ego_state, time_horizon, interval);

    for (const auto& object : object_prediction.objects) {
        if (object.object_type != "Dynamic") continue;

        // Object prediction에서 시간과 위치를 가져옴
        for (const auto& ego_point : ego_path_points) {
            // 시간 일치 확인
            if (std::abs(object.time - ego_point.time) > interval) continue;

            // 충돌 거리 계산
            double distance = std::hypot(object.x - ego_point.x, object.y - ego_point.y);
            // RCLCPP_INFO(this->get_logger(),"충돌 거리 : %.2f", distance);
            if (distance < collision_threshold) {
                ttc = std::min(ttc, ego_point.time); // TTC 업데이트
                break; // 해당 객체에 대해 더 이상 확인하지 않음
            }
        }
    }

    return ttc;
}


std::vector<prediction_points> TrajectoryNode::SampleEgoPath(const ad_msgs::msg::PolyfitLaneData& path, const ad_msgs::msg::VehicleState& ego_state, double time_horizon, double interval) {
    std::vector<prediction_points> sampled_path;
    double velocity = 0.0;
    if (ego_state.velocity < 5) {
        velocity = 15;
    }else {
        velocity = ego_state.velocity;
    }
    for (double t = 0; t <= time_horizon; t += interval) {
        double x = velocity * t;
        double y = path.a0 + path.a1 * x + path.a2 * x * x + path.a3 * x * x * x;

        Point local_point = {x, y};
        Point global_point = LocalToGlobal(local_point, ego_state);

        prediction_points point_with_time = {global_point.x, global_point.y, t};
        //RCLCPP_INFO(this->get_logger(),"sampled point - X : %.2f, Y : %.2f, Time : %.2f", point_with_time.x, point_with_time.y, point_with_time.time);
        sampled_path.push_back(point_with_time);
    }
    
    return sampled_path;
}




//----------------------------------------------------------------------------------------//

Point TrajectoryNode::LocalToGlobal(const Point& local_point, const ad_msgs::msg::VehicleState& vehicle_state) {
    double x_global = vehicle_state.x + local_point.x * std::cos(yaw) - local_point.y * std::sin(yaw);
    double y_global = vehicle_state.y + local_point.x * std::sin(yaw) + local_point.y * std::cos(yaw); 
    return {x_global, y_global};
}

double TrajectoryNode::normalize(double yaw){
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < M_PI) yaw += 2.0 * M_PI;
    return yaw;
}


std::vector<double> TrajectoryNode::ComputeCubicSpline(const std::vector<Point>& points, double slope_start, double slope_end) {
    if (points.size() != 2) {
        throw std::invalid_argument("ComputeCubicSpline requires exactly 4 points.");
    }

    // 시작점과 끝점의 좌표
    double x0 = points[0].x, y0 = points[0].y;
    double x1 = points[1].x, y1 = points[1].y;

    // Eigen 행렬을 사용해 연립방정식 풀기
    Eigen::Matrix4d A;
    Eigen::Vector4d b;

    A << std::pow(x0, 3), std::pow(x0, 2), x0, 1,
         std::pow(x1, 3), std::pow(x1, 2), x1, 1,
         3 * std::pow(x0, 2), 2 * x0, 1, 0,
         3 * std::pow(x1, 2), 2 * x1, 1, 0;

    b << y0, y1, slope_start, slope_end;

    // 연립 방정식 풀이
    Eigen::Vector4d coeffs = A.colPivHouseholderQr().solve(b);

    // 반환: [a3, a2, a1, a0]
    return {coeffs(0), coeffs(1), coeffs(2), coeffs(3)};
}



int main(int argc, char **argv) {
    std::string node_name = "trajectory_planner_node";
    double loop_rate = 100.0;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
