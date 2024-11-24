#ifndef __BEHAVIOR_PLANNER_NODE_HPP__
#define __BEHAVIOR_PLANNER_NODE_HPP__

#define AEB 1
#define ACC 2
#define MERGE 3
#define REF_VEL_TRACKING 4


#define BEHAVIOR_STATE_TO_STRING(state) \
    (state == AEB ? "AEB" : \
     state == ACC ? "ACC" : \
     state == MERGE ? "MERGE" : \
     state == REF_VEL_TRACKING ? "REFERENCE_VELOCITY_TRACKING" : "UNKNOWN")

// STD Header
#include <mutex>

// Ros Header
#include <rclcpp/rclcpp.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/mission_object.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <ad_msgs/msg/polyfit_lane_data.hpp>
#include <ad_msgs/msg/mission.hpp>

#include <std_msgs/msg/float32.hpp>


// Algorithm Header


class BehaviorPlannerNode : public rclcpp::Node {
    public:
        BehaviorPlannerNode(const std::string& node_name, const double& loop_rate,
                        const rclcpp::NodeOptions& optios = rclcpp::NodeOptions());
        ~BehaviorPlannerNode();

        
    private:
        // Algorithm Functiosn
        void Run();
        void updatePlannerState();
        void velocity_planner();

        // Variables for Algorithm
        double current_velocity_;
        double ref_vel_;
        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_behavior_state_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_ref_velocity_;

        // Subscribers
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_driving_way_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr s_obstacle_way_; // motion prediction결과
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_state_;

        // Callback Functions 
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        i_vehicle_state_ = *msg;
        }

        inline void CallbackDrivingWay(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_driving_way_);
        i_driving_way_ = *msg;
        }

        inline void CallbackObstacleWay(const ad_msgs::msg::PolyfitLaneDataArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_obstacle_way_);
        i_obstacle_way_ = *msg;
        }

        inline void CallbackMissionState(const ad_msgs::msg::Mission::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_mission_state_);
        i_mission_state_ = *msg;
        }


        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        ad_msgs::msg::VehicleState i_vehicle_state_;
        ad_msgs::msg::PolyfitLaneData i_driving_way_;
        ad_msgs::msg::PolyfitLaneDataArray i_obstacle_way_;
        ad_msgs::msg::Mission i_mission_state_;

        // Outputs 
        float o_behavior_state_;  // 1: ACC, 2: AEB, 3: Merge, 4: Reference Velocity Tracking
        float o_ref_velocity_;
        
        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_driving_way_;
        std::mutex mutex_obstacle_way_;
        std::mutex mutex_mission_state_;
}; 
#endif // __BEHAVIOR_PLANNER_NODE_HPP__
