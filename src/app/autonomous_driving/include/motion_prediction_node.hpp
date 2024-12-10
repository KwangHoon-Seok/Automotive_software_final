#ifndef __MOTION_PREDICTION_NODE_HPP__
#define __MOTION_PREDICTION_NODE_HPP__


#include <cmath>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <ad_msgs/msg/mission.hpp>
#include <ad_msgs/msg/mission_object.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>


class MotionPredictionNode : public rclcpp::Node {
    public:
        MotionPredictionNode(const std::string& node_name, const double& loop_rate,
                        const rclcpp::NodeOptions& optios = rclcpp::NodeOptions());
        ~MotionPredictionNode();


    private:
        void Run();
        void prediction(const ad_msgs::msg::VehicleState& vehicle_state, const ad_ms::msg::Mission& mission_state);

        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_state_;

        // Callback Functions 
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        i_vehicle_state_ = *msg;
        }
        inline void CallbackMissionState(const ad_msgs::msg::Mission::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_mission_state_);
        i_mission_state_ = *msg;
        }

        ad_msgs::msg::VehicleState i_vehicle_state_;
        ad_msgs::msg::Mission i_mission_state_;
        
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_mission_state_;

        std::vector<geometry_msgs::msg::Point> motion_;
}
#endif //__MOTION_PREDICTION_NODE_HPP__