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
    s_static_position_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/static/position", qos_profile,
        std::bind(&TrajectoryNode::CallbackStaticPosition, this, std::placeholders::_1));

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
    geometry_msgs::msg::Point static_position = i_static_position_;
    // std::vector<Point> target_points;
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
    if(drive_mode == MERGE)
    {
        
        Point left_point = LeftTargetPoint(static_position, 4.0);
        target_points.push_back(left_point);
        Point right_point = RightTargetPoint(static_position, 4.0);
        target_points.push_back(right_point);
        Point back_point = BackTargetPoint(static_position, 20.0);
        target_points.push_back(back_point);
            
        slope_second_right = (back_point.y - right_point.y) / (back_point.x - right_point.x);
        slope_second_left = (back_point.y - left_point.y) / (back_point.x - left_point.x);
        slope_first_right = (right_point.y - 0.0) / (right_point.x - 0.0);
        slope_first_left = (left_point.y - 0.0) / (left_point.x - 0.0);

    }else
    {
        for (double lateral_offset = lateral_offset_min; lateral_offset <= lateral_offset_max; lateral_offset += lateral_offset_step) {
            Point target_point = TargetPoint(driving_way, x, lateral_offset);
            target_points.push_back(target_point);
        }
    }

    ad_msgs::msg::PolyfitLaneDataArray spline_array_msg;
    spline_array_msg.frame_id = "ego/body";

    static ad_msgs::msg::PolyfitLaneData best_path;
    static bool is_best_path = 0;
    static int is_flag = 0;
    if (drive_mode == MERGE)
    {
        if (!is_best_path){
            best_path.frame_id = "ego/body";
            best_path.id = "0";
            best_path.a0 = 0.0;
            best_path.a1 = 0.0;
            best_path.a2 = 0.0;
            best_path.a3 = 0.0;
            is_best_path = true;
        }
        for (size_t i = 1; i < 3; ++i){
            std::vector<Point> temp_points = {target_points[0],target_points[i], target_points[3]};
            std::vector<double> coeffs = ComputeQuinticSpline(temp_points, slope_start, slope_first_right, slope_first_left, slope_second_right, slope_second_left, slope_end, vehicle_state);
            ad_msgs::msg::PolyfitLaneData spline_msg;
            spline_msg.frame_id = "ego/body";
            spline_msg.id = std::to_string(i);
            spline_msg.a0 = coeffs[5];
            spline_msg.a1 = coeffs[4];
            spline_msg.a2 = coeffs[3];
            spline_msg.a3 = coeffs[2];
            spline_msg.a4 = coeffs[1];
            spline_msg.a5 = coeffs[0];
            spline_msg.merge = 1.0;
            spline_array_msg.polyfitlanes.push_back(spline_msg);
        }
        o_trajectories_ = spline_array_msg;
        p_trajectory_candidates_->publish(o_trajectories_);
    }
    else
    {
        for (size_t i = 1; i < target_points.size(); ++i) {
            std::vector<Point> segment_points = {target_points[0], target_points[i]};
            std::vector<double> coeffs = ComputeCubicSpline(segment_points, slope_start, slope_end);
            
            ad_msgs::msg::PolyfitLaneData spline_msg;
            spline_msg.frame_id = "ego/body";
            spline_msg.id = std::to_string(i);
            spline_msg.a0 = coeffs[3];
            spline_msg.a1 = coeffs[2];
            spline_msg.a2 = coeffs[1];
            spline_msg.a3 = coeffs[0];
            spline_msg.merge = 0.0;

            spline_array_msg.polyfitlanes.push_back(spline_msg);
        }
        // Publish trajectory candidates (always updated)
        o_trajectories_ = spline_array_msg;
        p_trajectory_candidates_->publish(o_trajectories_);
    }

    // if (drive_mode == MERGE) {
    //     double min_ttc = 10000;
    //     for (const auto& path : spline_array_msg.polyfitlanes) {
    //         double ttc = CalculateTTC(path, vehicle_state, object_prediction, 3.0, 0.1, 3.5);
    //         // Update best path only if conditions are met
    //         RCLCPP_INFO(this->get_logger(), "path id %s ttc: %.2f", path.id.c_str(), ttc);
    //         if ((path.id == "4" || path.id == "5") && ttc > 1.8 && ttc < 3.0 && is_flag == 0) {
    //             best_path = path;
    //             is_flag = 1;
    //         }
    //     }
    // }
    // Publish the best trajectory
    
    p_best_trajectory_->publish(best_path);
}
//--------------------------------------------------------temp------------------------------------------------------//
Point TrajectoryNode::RightTargetPoint(const geometry_msgs::msg::Point& static_position, double lateral_offset) {
    // Calculate the relative position in the global frame
    double dx = static_position.x - i_vehicle_state_.x;
    double dy = static_position.y - i_vehicle_state_.y;

    // Convert to the local frame using the vehicle's yaw
    double local_x = std::cos(-i_vehicle_state_.yaw) * dx - std::sin(-i_vehicle_state_.yaw) * dy;
    double local_y = std::sin(-i_vehicle_state_.yaw) * dx + std::cos(-i_vehicle_state_.yaw) * dy;

    // Apply lateral offset in the local frame (right side is positive lateral offset)
    double right_x = local_x;
    double right_y = local_y - lateral_offset;

    RCLCPP_INFO(this->get_logger(), "Right Point in Local Frame - X: %.2f, Y: %.2f", right_x, right_y);

    return {right_x, right_y};
}


Point TrajectoryNode::LeftTargetPoint(const geometry_msgs::msg::Point& static_position, double lateral_offset) {
    // Calculate the relative position in the global frame
    double dx = static_position.x - i_vehicle_state_.x;
    double dy = static_position.y - i_vehicle_state_.y;

    // Convert to the local frame using the vehicle's yaw
    double local_x = std::cos(-i_vehicle_state_.yaw) * dx - std::sin(-i_vehicle_state_.yaw) * dy;
    double local_y = std::sin(-i_vehicle_state_.yaw) * dx + std::cos(-i_vehicle_state_.yaw) * dy;

    // Apply lateral offset in the local frame (left side is negative lateral offset)
    double left_x = local_x;
    double left_y = local_y + lateral_offset;

    RCLCPP_INFO(this->get_logger(), "Left Point in Local Frame - X: %.2f, Y: %.2f", left_x, left_y);

    return {left_x, left_y};
}


Point TrajectoryNode::BackTargetPoint(const geometry_msgs::msg::Point& static_position, double back_distance) {
    // Calculate the relative position in the global frame
    double dx = static_position.x - i_vehicle_state_.x;
    double dy = static_position.y - i_vehicle_state_.y;

    // Convert to the local frame using the vehicle's yaw
    double local_x = std::cos(-i_vehicle_state_.yaw) * dx - std::sin(-i_vehicle_state_.yaw) * dy;
    double local_y = std::sin(-i_vehicle_state_.yaw) * dx + std::cos(-i_vehicle_state_.yaw) * dy;

    // Apply backward offset in the local frame
    double back_x = local_x + back_distance; // Moving backward along the local x-axis
    double back_y = local_y + 7.0;                 // No change in the y-axis

    RCLCPP_INFO(this->get_logger(), "Back Point in Local Frame - X: %.2f, Y: %.2f", back_x, back_y);

    return {back_x, back_y};
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

Point TrajectoryNode::GlobalToLocal(const Point& global_point, const ad_msgs::msg::VehicleState& vehicle_state) {
    double dx = global_point.x - vehicle_state.x;
    double dy = global_point.y - vehicle_state.y;
    double yaw = vehicle_state.yaw;

    // 로컬 좌표로 변환
    double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    return {local_x, local_y};
}

double TrajectoryNode::normalize(double yaw){
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

// 3차 다항식으로 (waypoint) (경계조건 로컬 벡터 확인)
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

// 5차 다항식
std::vector<double> TrajectoryNode::ComputeQuinticSpline(const std::vector<Point>& points, double slope_start, double slope_first_right, double slope_first_left, double slope_second_right, double slope_second_left, double slope_end, const ad_msgs::msg::VehicleState vehicle_state) {
    if (points.size() != 3) {
        throw std::invalid_argument("ComputeCubicSpline requires exactly 4 points.");
    }

    // 시작점과 중간점 끝점의 좌표
    double x0 = points[0].x, y0 = points[0].y;
    double x1 = points[1].x, y1 = points[1].y;
    double x2 = points[2].x, y2 = points[2].y;

    // Eigen 행렬을 사용해 연립방정식 풀기
    Eigen::MatrixXd A(6,6);
    Eigen::VectorXd b(6);

    A << std::pow(x0, 5), std::pow(x0, 4), std::pow(x0, 3), std::pow(x0, 2), x0, 1,
         std::pow(x1, 5), std::pow(x1, 4), std::pow(x1, 3), std::pow(x1, 2), x1, 1,
         std::pow(x2, 5), std::pow(x2, 4), std::pow(x2, 3), std::pow(x2, 2), x2, 1,
         5 * std::pow(x0, 4), 4 * std::pow(x0, 3), 3 * std::pow(x0, 2), 2 * x0, 1, 0,
         5 * std::pow(x1, 4), 4 * std::pow(x1, 3), 3 * std::pow(x1, 2), 2 * x1, 1, 0,
         5 * std::pow(x2, 4), 4 * std::pow(x2, 3), 3 * std::pow(x2, 2), 2 * x2, 1, 0;
    // 오른쪽 라인
    if (y1 < 0)
    {
        b << y0, y1, y2, slope_start, slope_first_right, slope_second_right;
    }
    else
    {
        b << y0, y1, y2, slope_start, slope_first_left, slope_second_left;
    }
    

    // 연립 방정식 풀이
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

    // 반환: [a3, a2, a1, a0]
    return {coeffs(0), coeffs(1), coeffs(2), coeffs(3), coeffs(4), coeffs(5)};
}



int main(int argc, char **argv) {
    std::string node_name = "trajectory_planner_node";
    double loop_rate = 100.0;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
