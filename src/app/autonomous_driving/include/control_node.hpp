#ifndef __CONTROL_NODE_HPP__
#define __CONTROL_NODE_HPP__

#define AEB 1
#define ACC 2
#define MERGE 3
#define REF_VEL_TRACKING 4

// STD Header
#include <mutex>

// ROS Header
#include <rclcpp/rclcpp.hpp>
    // control 정보
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/vehicle_command.hpp>
    // 미션 정보
#include <ad_msgs/msg/mission.hpp>
    // driving way 정보
#include <ad_msgs/msg/polyfit_lane_data.hpp>
    // behavior planner 정보
#include <std_msgs/msg/float32.hpp>

class ControlNode : public rclcpp::Node {
    public:
        ControlNode(const std::string& node_name, const double& loop_rate,
                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ControlNode();
    private:
        // Algorithm Function
        void Run();
        double computePID(double target_speed, double current_speed, double kp);
        double computePID_ACC(double target_distance, double obstacle_distance, double kp);
        double PurePursuit(const ad_msgs::msg::PolyfitLaneData &driving_way);
        // Variables for Algorithm
        double l_d;
        double g_x = 5.0;
        double g_y;
        double e_ld;
        double yaw;
        double accel;
        double brake;
        double max_brake = 1.0;
        double max_accel = 1.0;
        double L = 2.7;
        double ki = 0.01;

        double speed_error_integral_ = 0.0;
        double speed_error_prev_ = 0.0;
        double distance_error_integral_ = 0.0;
        double distance_error_prev_ = 0.0;

        double behavior_state = 0.0;

        // Publishers
        rclcpp::Publisher<ad_msgs::msg::VehicleCommand>::SharedPtr p_vehicle_command_;

        // Subscribers
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_driving_way_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_state_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_lead_distance_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_behavior_state_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_ref_vel_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_merge_path_;

        // Callback Functions
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = *msg;
        }
        inline void CallbackDrivingWay(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_driving_way_);
            i_driving_way_ = *msg;
        }
        inline void CallbackMissionState(const ad_msgs::msg::Mission::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_mission_state_);
            i_mission_state_ = *msg;
        }
        inline void CallbackLeadDistance(const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_lead_distance_);
            i_lead_distance_ = *msg;
            // RCLCPP_INFO(this->get_logger(), "그렇다면 나는?");
        }
        inline void CallbackBehaviorState(const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_behavior_state_);
            i_behavior_state_ = *msg;
        }
        inline void CallbackRefVel(const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_ref_vel_);
            i_ref_vel_ = *msg;
        }
        inline void CallbackMergePath(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_merge_path_);
            i_merge_path_ = *msg;
            RCLCPP_INFO(this->get_logger(), "제발 나 좀 받아와라");
        }
        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        ad_msgs::msg::VehicleState i_vehicle_state_;
        ad_msgs::msg::PolyfitLaneData i_driving_way_;
        ad_msgs::msg::Mission i_mission_state_;
        std_msgs::msg::Float32 i_lead_distance_;
        std_msgs::msg::Float32 i_behavior_state_;
        std_msgs::msg::Float32 i_ref_vel_;
        ad_msgs::msg::PolyfitLaneData i_merge_path_;

        // Outputs 
        ad_msgs::msg::VehicleCommand o_vehicle_command_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_driving_way_;
        std::mutex mutex_mission_state_;
        std::mutex mutex_lead_distance_;
        std::mutex mutex_behavior_state_;
        std::mutex mutex_ref_vel_;
        std::mutex mutex_merge_path_;
};  
#endif