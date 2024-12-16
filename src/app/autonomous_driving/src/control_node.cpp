#include "control_node.hpp"

ControlNode::ControlNode(const std::string& node_name, const double& loop_rate,
                         const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", qos_profile,
        std::bind(&ControlNode::CallbackVehicleState, this, std::placeholders::_1));
    s_driving_way_ = this->create_subscription<ad_msgs::msg::PolyfitLaneData>(
        "/ego/driving_way", qos_profile,
        std::bind(&ControlNode::CallbackDrivingWay, this, std::placeholders::_1));
    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", qos_profile,
        std::bind(&ControlNode::CallbackMissionState, this, std::placeholders::_1));
    s_lead_distance_ = this->create_subscription<std_msgs::msg::Float32>(
        "/ego/lead_distance", qos_profile,
        std::bind(&ControlNode::CallbackLeadDistance, this, std::placeholders::_1));
    s_behavior_state_ = this->create_subscription<std_msgs::msg::Float32>(
        "/ego/behavior_state", qos_profile,
        std::bind(&ControlNode::CallbackBehaviorState, this, std::placeholders::_1));
    s_ref_vel_ = this->create_subscription<std_msgs::msg::Float32>(
        "/ego/ref_vel", qos_profile,
        std::bind(&ControlNode::CallbackRefVel, this, std::placeholders::_1));
    s_merge_path_ = this->create_subscription<ad_msgs::msg::PolyfitLaneData>(
        "/ego/merge_path", qos_profile,
        std::bind(&ControlNode::CallbackMergePath, this, std::placeholders::_1));
    s_merge_target_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/merge_target", qos_profile,
        std::bind(&ControlNode::CallbackMergeTarget, this, std::placeholders::_1));

    // Publisher
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleCommand>("/ego/vehicle_command", qos_profile);
    p_global_waypoint_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/global_points", qos_profile);

    // Timer
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / loop_rate)),
        std::bind(&ControlNode::Run, this));

    RCLCPP_INFO(this->get_logger(), "ControlNode initialized.");
}

ControlNode::~ControlNode() {}

void ControlNode::Run() {
    // Lock necessary mutexes
    std::lock_guard<std::mutex> lock_vehicle_state(mutex_vehicle_state_);
    std::lock_guard<std::mutex> lock_ref_vel(mutex_ref_vel_);
    std::lock_guard<std::mutex> lock_behavior_state(mutex_behavior_state_);
    std::lock_guard<std::mutex> lock_merge_path(mutex_merge_path_);

    ad_msgs::msg::VehicleState vehicle_state = i_vehicle_state_;
    std_msgs::msg::Float32 behavior_state_float = i_behavior_state_;
    double drive_mode = static_cast<double>(behavior_state_float.data);
    std_msgs::msg::Float32 ref_vel_float = i_ref_vel_;
    double target_speed = static_cast<double>(ref_vel_float.data);
    std_msgs::msg::Float32 lead_distance_float = i_lead_distance_;
    ad_msgs::msg::PolyfitLaneData merge_path = i_merge_path_;
    double lead_distance = static_cast<double>(lead_distance_float.data);
    double target_distance = 15;
    double control_signal;

    double target_x = i_merge_target_.x;
//---------------------------------REF_VEL_TRACKING---------------------------------------//
    if (drive_mode == REF_VEL_TRACKING)
    {
        control_signal = computePID(target_speed, vehicle_state.velocity, 0.6);
        if (control_signal > 0)
        {
            o_vehicle_command_.accel = std::min(control_signal, 0.5);
            o_vehicle_command_.brake = 0.0;
        }else{
            o_vehicle_command_.accel = 0.0;
            o_vehicle_command_.brake = std::min(-control_signal, max_brake);
        }
        yaw = PurePursuit(i_driving_way_);
    }
//--------------------------------------ACC--------------------------------------------------//
    if (drive_mode == ACC)
    {
        control_signal = computePID_ACC(target_distance, lead_distance, 0.25);
        if (control_signal > 0)
        {
            o_vehicle_command_.accel = std::min(control_signal, 0.5);
            o_vehicle_command_.brake = 0.0;
        }else{
            o_vehicle_command_.accel = 0.0;
            o_vehicle_command_.brake = std::min(-control_signal, max_brake);
        }
        yaw = PurePursuit(i_driving_way_);
    }
//------------------------------------AEB-------------------------------------------//
    if (drive_mode == AEB)
    {
        o_vehicle_command_.accel = 0.0;
        o_vehicle_command_.brake = 0.0;
    }

//------------------------------------MERGE------------------------------------------//
    if (drive_mode == MERGE)
    {
        merge_flag = true;
    }

    if (merge_flag == true) {

        static ad_msgs::msg::PolyfitLaneData tracked_path; // 유효한 경로를 저장

        // 유효한 경로가 올 때까지 정지
        if (path_initialized == false) {
            if (merge_path.merge == 1) {
                tracked_path = merge_path;
                path_initialized = true;
                merge_target_x = target_x;
                std::array<double, 6> coeffs = {merge_path.a0, merge_path.a1, merge_path.a2,
                                        merge_path.a3, merge_path.a4, merge_path.a5};
                global_waypoints = SampleGlobalPath(coeffs, vehicle_state, 0.0, merge_target_x, 0.1);
                for (const auto& point : global_waypoints) {
                    RCLCPP_INFO(this->get_logger(), "Global Point - X: %.2f, Y: %.2f", point.x, point.y);
                }
                RCLCPP_INFO(this->get_logger(), "Valid path received. Starting merge with path ID: %s", tracked_path.id.c_str());
            } else {
                o_vehicle_command_.accel = 0.0;
                o_vehicle_command_.brake = 1.0;
                o_vehicle_command_.steering = 0.0;
                p_vehicle_command_->publish(o_vehicle_command_);  
                RCLCPP_WARN(this->get_logger(), "Waiting for a valid path...");
            }
        }

        // 병합 동작
        if (path_initialized == true)
        {
            if (!merge_completed) 
            {
                double current_x = vehicle_state.x;

                if (goal_reached_flag) { // 목표 지점 도달 확인
                    merge_completed = true;
                    RCLCPP_INFO(this->get_logger(), "Merge completed. Switching to next mode.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Keep Merge");
                }

                // 속도 제어 및 경로 추적
                double control_signal = computePID(6, vehicle_state.velocity, 0.6);
                if (control_signal > 0) {
                    o_vehicle_command_.accel = std::min(control_signal, 0.5);
                    o_vehicle_command_.brake = 0.0;
                } else {
                    o_vehicle_command_.accel = 0.0;
                    o_vehicle_command_.brake = std::min(-control_signal, max_brake);
                }

                yaw = GlobalPurePursuit(vehicle_state, 5.0, 0.55);
                o_vehicle_command_.steering = yaw;

                p_vehicle_command_->publish(o_vehicle_command_);
            }
        }

        // 병합 완료 후 제동
        if (merge_completed) {
            RCLCPP_INFO(this->get_logger(), "Merge Completed");
            o_vehicle_command_.accel = 0.00;
            o_vehicle_command_.brake = 1.0;
            o_vehicle_command_.steering = 0.0;
            p_vehicle_command_->publish(o_vehicle_command_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            merge_flag = false;
        }
    }

    // global_waypoint publish
    std_msgs::msg::Float64MultiArray waypoint_msg;
    for (const auto& point : global_waypoints) {
        waypoint_msg.data.push_back(point.x);
        waypoint_msg.data.push_back(point.y);
        waypoint_msg.data.push_back(point.z);
    }
    p_global_waypoint_->publish(waypoint_msg);

    //o_vehicle_command_.accel = 0.0;
    o_vehicle_command_.steering = yaw;  

    // Publish Vehicle 


    p_vehicle_command_->publish(o_vehicle_command_);
}

double ControlNode::computePID(double target_speed, double current_speed, double kp)
{
    double speed_error = target_speed - current_speed;
    double dt = 0.1;

    double p_term = kp * speed_error;
    speed_error_integral_ += ki * speed_error * dt;
    speed_error_integral_ = std::clamp(speed_error_integral_, -max_brake, max_accel);

    double control_signal = p_term + speed_error_integral_;

    speed_error_prev_ = speed_error;
    return control_signal;
}

double ControlNode::computePID_ACC(double target_distance, double obstacle_distance, double kp)
{
    double distance_error = obstacle_distance - target_distance;
    double dt = 0.1;

    double p_term = kp * distance_error;
    distance_error_integral_ += ki * distance_error * dt;
    speed_error_integral_ = std::clamp(distance_error_integral_, -max_brake, max_accel);

    double control_signal = p_term + distance_error_integral_;

    distance_error_prev_ = distance_error;
    return control_signal;
}

double ControlNode::PurePursuit(const ad_msgs::msg::PolyfitLaneData &driving_way){
    double a0 = driving_way.a0;
    double a1 = driving_way.a1;
    double a2 = driving_way.a2;
    double a3 = driving_way.a3;

    g_y = a3 * std::pow(g_x, 3) + a2 * std::pow(g_x, 2) + a1 * g_x + a0;
    l_d = std::sqrt(std::pow(g_x, 2) + std::pow(g_y, 2));
    e_ld = g_y;
    yaw = std::atan2(2 * L * g_y, std::pow(l_d, 2));
    
    return yaw;
}

double ControlNode::GlobalPurePursuit(const ad_msgs::msg::VehicleState &vehicle_state, 
                                double lookahead_distance, double max_steering_angle) {
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point target_point;

    // Find the closest point ahead of the vehicle at the specified look-ahead distance
    for (const auto& point : global_waypoints) {
        double dx = point.x - vehicle_state.x;
        double dy = point.y - vehicle_state.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 거리가 가장 가까운 point 찾음 ->  target_point
        if (distance >= lookahead_distance && distance < min_distance) {
            min_distance = distance;
            target_point = point;
        }
    }

    // If no valid target point is found, return 0 (vehicle remains straight)
    if (min_distance == std::numeric_limits<double>::max()) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found for Pure Pursuit.");
        return 0.0;
    }
    
    geometry_msgs::msg::Point final_point = global_waypoints.back();
    double dx_to_goal = final_point.x - vehicle_state.x;
    double dy_to_goal = final_point.y - vehicle_state.y;
    double distance_to_goal = std::sqrt(dx_to_goal * dx_to_goal + dy_to_goal * dy_to_goal);

    if (distance_to_goal < 4.0) { 
        goal_reached_flag = true;
        RCLCPP_INFO(this->get_logger(), "Goal reached! Distance to goal: %.2f", distance_to_goal);
        return 0.0; // 조향값 0 반환
    } else {
        goal_reached_flag = false;
    }
    RCLCPP_INFO(this->get_logger(), "Final Point - X: %.2f, Y: %.2f", final_point.x, final_point.y);
    RCLCPP_INFO(this->get_logger(), "Current Point - X: %.2f, Y: %.2f", vehicle_state.x, vehicle_state.y);

    // Global frame
    double dx = target_point.x - vehicle_state.x;
    double dy = target_point.y - vehicle_state.y;

    double theta = std::atan2(dy, dx) - vehicle_state.yaw;

    while(theta > M_PI) theta -= 2 * M_PI;
    while(theta < -M_PI) theta += 2 * M_PI;

    // Local frame
    // double local_x = std::cos(-vehicle_state.yaw) * dx - std::sin(-vehicle_state.yaw) * dy;
    // double local_y = std::sin(-vehicle_state.yaw) * dx + std::cos(-vehicle_state.yaw) * dy;
    // double curvature = 2 * local_y / (lookahead_distance * lookahead_distance);

    // Pure Pursuit steering angle calculation
    double curvature = 2 * std::sin(theta) / lookahead_distance;
    double steering_angle = std::atan2(curvature * vehicle_state.length, 1.0);
    double steering_value = std::max(-1.0, std::min(1.0, steering_angle / max_steering_angle));
    // Log target point and steering angle
    // RCLCPP_INFO(this->get_logger(), "Target Point - X: %.2f, Y: %.2f", target_point.x, target_point.y);
    RCLCPP_INFO(this->get_logger(), "Steering Angle: %.2f ", steering_value);

    return steering_value;
}




std::vector<geometry_msgs::msg::Point> ControlNode::SampleGlobalPath(
    const std::array<double, 6>& coeffs,  // 5차 다항식 계수: [a0, a1, a2, a3, a4, a5]
    const ad_msgs::msg::VehicleState& vehicle_state,  // 차량 상태 (x, y, yaw - 글로벌 yaw)
    double start_x,  // 시작 x 좌표 (로컬 기준)
    double end_x,    // 끝 x 좌표 (로컬 기준)
    double step      // 샘플링 간격
) {
    std::vector<geometry_msgs::msg::Point> global_points;

    for (double local_x = start_x; local_x <= end_x; local_x += step) {
        // 로컬 y 계산 (5차 다항식)
        double local_y = coeffs[0] + coeffs[1] * local_x + coeffs[2] * std::pow(local_x, 2) +
                         coeffs[3] * std::pow(local_x, 3) + coeffs[4] * std::pow(local_x, 4) +
                         coeffs[5] * std::pow(local_x, 5);

        // 글로벌 yaw을 고려한 글로벌 좌표 변환
        double global_x = std::cos(vehicle_state.yaw) * local_x - std::sin(vehicle_state.yaw) * local_y + vehicle_state.x;
        double global_y = std::sin(vehicle_state.yaw) * local_x + std::cos(vehicle_state.yaw) * local_y + vehicle_state.y;

        // 글로벌 좌표를 저장
        geometry_msgs::msg::Point point;
        point.x = global_x;
        point.y = global_y;
        point.z = 0.0;  // 2D 평면이므로 z는 0
        global_points.push_back(point);
    }
    
    return global_points;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ControlNode>("control_node", 100.0);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}