#ifndef __TRAJECTORY_NODE_HPP__
#define __TRAJECTORY_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <cmath>
#include <chrono>
#include <string>
#include <unordered_map>
#include <queue>


// Ros Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>


// Bridge Header
#include "ros2_bridge_vehicle.hpp"
#include "ros2_bridge_lane.hpp"
#include "ros2_bridge_mission.hpp"

// Parameter Header
#include "autonomous_driving_config.hpp"

// lattice state
struct State {
    double x;
    double y;
    double theta; // Heading angle in radians [0, 2π)

    bool operator==(const State& other) const {
        return std::fabs(x - other.x) < 1e-4 &&
               std::fabs(y - other.y) < 1e-4 &&
               std::fabs(theta - other.theta) < 1e-4;
    }
};

// unordered map hash
namespace std {
    template <>
    struct hash<State> {
        std::size_t operator()(const State& s) const {
            return ((std::hash<double>()(s.x) ^
                    (std::hash<double>()(s.y) << 1)) >> 1) ^
                   (std::hash<double>()(s.theta) << 1);
        }
    };
}

// lattice node - A*
struct Node {
    State state;
    double g; // Cost from start to current node
    double h; // Heuristic cost to goal
    double f; // Total cost
    std::shared_ptr<Node> parent;

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};


class TrajectoryNode : public rclcpp::Node
{
    public:
        TrajectoryNode(const std::string& node_name, const double& loop_rate,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~TrajectoryNode();
    private:
        // Algorithm Functions
        void GenerateSearchSpace(); // state space 생성 함수
        void DefineMotionPrimitives();
        void PlanPath();

        // Variabls for Algorithm
        double a_step_distance_;
        int a_num_layers_;
        double a_heading_increment_;
        double a_min_turn_radius_;
        std::vector<State>

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr p_state_space_;
        
        // Subscribers
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;

        // Callback Functions
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = *msg;
        }
       
        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        ad_msgs::msg::VehicleState i_vehicle_state_;

        // Outputs
        geometry_msgs::msg::PoseArray o_state_space;

        // Mutex
        std::mutex mutex_vehicle_state_;
};

#endif 