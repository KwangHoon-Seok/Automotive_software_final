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

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", 10,
        std::bind(&TrajectoryNode::CallbackVehicleState, this, std::placeholders::_1));

    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", 10,
        std::bind(&TrajectoryNode::CallbackMissionState, this, std::placeholders::_1));

    // Publishers
    p_trajectory_candidates_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "/ego/local_path_array", rclcpp::QoS(10));

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
    std::lock_guard<std::mutex> lock_vehicle_state(mutex_vehicle_state_);
    ad_msgs::msg::PolyfitLaneData driving_way = i_driving_way_;
    ad_msgs::msg::VehicleState vehicle_state = i_vehicle_state_;
    std::vector<Point> target_points;
    
    Point current_position = {0.0 , 0.0};
    yaw = normalize(static_cast<double>(vehicle_state.yaw));
    
    double x = 40.0;
    double lateral_offset_min = -5.0;
    double lateral_offset_max = 5.0;
    double lateral_offset_step = 2.0;

    target_points.clear();

    // Target points 계산
    target_points.push_back(current_position);

    for (double lateral_offset = lateral_offset_min; lateral_offset <= lateral_offset_max; lateral_offset += lateral_offset_step) {
        Point target_point = TargetPoint(driving_way, x, lateral_offset);
        // RCLCPP_INFO(this->get_logger(),"--------------------------------------------------------------");
        // RCLCPP_INFO(this->get_logger(), "Generated target_point: x=%.4f, y=%.4f", target_point.x, target_point.y);
        target_points.push_back(target_point);
    }

    ad_msgs::msg::PolyfitLaneDataArray spline_array_msg;
    spline_array_msg.frame_id = "ego/body";

    for (size_t i = 1; i < target_points.size(); ++i) {
        // bodundary condition (기울기)
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
    // Evaluate costs and select the best trajectory
    // double min_ttc = std::numeric_limits<double>::max();
    // ad_msgs::msg::PolyfitLaneData best_path;

    // for (const auto& path : spline_array_msg.polyfitlanes) {
    //     double ttc = CalculateTTC(path, vehicle_state, i_mission_state_, 5.0, 0.1, 1.5);
    //     // RCLCPP_INFO(this->get_logger(), "Path ID: %s, TTC: %.2f seconds", path.id.c_str(), ttc);
    //     if (ttc < min_ttc) {
    //         min_ttc = ttc;
    //         best_path = path;
    //     }
    // }

    // if (min_ttc == std::numeric_limits<double>::max()) {
    //     RCLCPP_WARN(this->get_logger(), "No safe found. Using default path.");
    //     best_path = spline_array_msg.polyfitlanes[0];
    // }

    o_trajectories_ = spline_array_msg;
    p_trajectory_candidates_->publish(o_trajectories_);
}



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
