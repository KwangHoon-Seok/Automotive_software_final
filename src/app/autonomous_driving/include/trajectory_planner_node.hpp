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
#include <functional>
#include <Eigen/Dense>

#include <fstream>
#include <sstream>

// Ros Header
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/polyfit_lane_data.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>



// Parameter Header
#include "autonomous_driving_config.hpp"

// My type
struct Point {
    double x;
    double y;
};

class TrajectoryNode : public rclcpp::Node
{
    public:
        TrajectoryNode(const std::string& node_name, const double& loop_rate,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~TrajectoryNode();
        
        void Run();
    private:
        //------------------------------Algorithm Functions-----------------------------//
        Point TargetPoint(const ad_msgs::msg::PolyfitLaneData& driving_way, double x,double lateral_offset);
        Point LocalToGlobal(const Point& local_point, const ad_msgs::msg::VehicleState& vehicle_state, double yaw);
        double normalize(double yaw);
        std::vector<double> ComputeCubicSpline(const std::vector<Point>& points, double slope_start, double slope_end);
        // 최종 후보 경로 퍼블리시 함수
        void PublishSplineCoefficients(const std::vector<double>& coeffs);

        // Variabls for Algorithm
        // std::vector<Point> target_points;
        // Publishers
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_trajectory_candidates_;
        
        // Subscribers
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_driving_way_;
        
        // Callback Functions
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = *msg;
        }
       
        inline void CallbackDrivingWay(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_driving_way_);
            i_driving_way_ = *msg;
        }

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        ad_msgs::msg::VehicleState i_vehicle_state_;
        ad_msgs::msg::PolyfitLaneData i_driving_way_;

        // Outputs
        ad_msgs::msg::PolyfitLaneDataArray o_trajectories_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_driving_way_;

};

#endif 