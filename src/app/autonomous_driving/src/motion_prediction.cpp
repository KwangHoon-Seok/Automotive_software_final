#ifndef __MOTION_PREDICTION_NODE_HPP__
#define __MOTION_PREDICTION_NODE_HPP__


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

        // Callback Functions 
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        i_vehicle_state_ = *msg;
        }

        

}