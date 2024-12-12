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


    // Publisher
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleCommand>("/ego/vehicle_command", qos_profile);

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

        // 반복문 외부에서 vehicle_state와 target_x 설정
        

        // 유효한 경로가 올 때까지 정지
        if (path_initialized == false) {
            if (merge_path.id == "4" || merge_path.id == "5") {
                tracked_path = merge_path;
                path_initialized = true;
                target_x = vehicle_state.x + 10;
                RCLCPP_INFO(this->get_logger(), "Valid path received. Starting merge with path ID: %s", tracked_path.id.c_str());
            } else {
                o_vehicle_command_.accel = 0.0;
                o_vehicle_command_.brake = 1.0;
                o_vehicle_command_.steering = 0.0;
                p_vehicle_command_->publish(o_vehicle_command_);  
                RCLCPP_WARN(this->get_logger(), "Waiting for a valid path...");
                // rclcpp::Rate(1).sleep();
            }
        }

        // 병합 동작
        if (path_initialized == true)
        {
            if (!merge_completed) 
            {
                double current_x = vehicle_state.x;

                if (current_x >= target_x) { // 목표 지점 도달 확인
                    merge_completed = true;
                    RCLCPP_INFO(this->get_logger(), "Merge completed. Switching to next mode.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Keep Merge");
                    RCLCPP_INFO(this->get_logger(), "Current x: %.2f Target x: %.2f", current_x, target_x);
                }

                // 속도 제어 및 경로 추적
                double control_signal = computePID(10, vehicle_state.velocity, 0.6);
                if (control_signal > 0) {
                    o_vehicle_command_.accel = std::min(control_signal, 0.5);
                    o_vehicle_command_.brake = 0.0;
                } else {
                    o_vehicle_command_.accel = 0.0;
                    o_vehicle_command_.brake = std::min(-control_signal, max_brake);
                }

                yaw = PurePursuit(tracked_path);
                o_vehicle_command_.steering = yaw;

                // // 병합 도중 behavior_state 변경 방지
                // if (drive_mode != MERGE) {
                //     RCLCPP_WARN(this->get_logger(), "Behavior state changed during merging. Forcing MERGE mode.");
                //     drive_mode = MERGE; // 병합 모드 유지
                // }

                p_vehicle_command_->publish(o_vehicle_command_);
                // rclcpp::Rate(1).sleep(); // 루프 속도 조절
            }
        }

        // 병합 완료 후 제동
        if (merge_completed) {
            RCLCPP_INFO(this->get_logger(), "Merge Completed");
            o_vehicle_command_.accel = 0.04;
            o_vehicle_command_.brake = 0.0;
            o_vehicle_command_.steering = 0.5;
            p_vehicle_command_->publish(o_vehicle_command_);
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            merge_flag = false;
        }
    }


    o_vehicle_command_.accel = 0.0;
    o_vehicle_command_.steering = yaw;  

    // Publish Vehicle Command
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ControlNode>("control_node", 100.0);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
