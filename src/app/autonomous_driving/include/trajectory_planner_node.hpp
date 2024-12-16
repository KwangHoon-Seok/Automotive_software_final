#ifndef __TRAJECTORY_NODE_HPP__
#define __TRAJECTORY_NODE_HPP__
#pragma once

#define AEB 1
#define ACC 2
#define MERGE 3
#define REF_VEL_TRACKING 4

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
#include <ad_msgs/msg/mission_object.hpp>
#include <ad_msgs/msg/mission.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>



// Parameter Header
#include "autonomous_driving_config.hpp"

// My type
struct Point {
    double x;
    double y;
};

struct prediction_points {
    double x;
    double y;
    double time;
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
        Point LocalToGlobal(const Point& local_point, const ad_msgs::msg::VehicleState& vehicle_state);
        double normalize(double yaw);
        std::vector<double> ComputeCubicSpline(const std::vector<Point>& points, double slope_start, double slope_end);
            // motion prediction
        double CalculateTTC(
                            const ad_msgs::msg::PolyfitLaneData& path,
                            const ad_msgs::msg::VehicleState& ego_state,
                            const ad_msgs::msg::Mission& object_prediction,
                            double time_horizon,
                            double interval,
                            double collision_threshold);
        std::vector<prediction_points> SampleEgoPath(const ad_msgs::msg::PolyfitLaneData& path, const ad_msgs::msg::VehicleState& ego_state, double time_horizon, double interval);
        void PublishSplineCoefficients(const std::vector<double>& coeffs);
        Point GlobalToLocal(const Point& global_point, const ad_msgs::msg::VehicleState& vehicle_state);
        //temp fucntion
        Point RightTargetPoint(const geometry_msgs::msg::Point& static_position, double lateral_offset);
        Point LeftTargetPoint(const geometry_msgs::msg::Point& static_position, double lateral_offset);
        Point BackTargetPoint(const geometry_msgs::msg::Point& static_position, double back_distance);
        std::vector<double> ComputeQuinticSpline(const std::vector<Point>& points, double slope_start, double slope_first_right, double slope_first_list, double slope_second_right, double slope_second_left, double slope_end, const ad_msgs::msg::VehicleState vehicle_state);
        // Variabls for Algorithm
        double yaw = 0.0;
        int path_flag = 0;
        double slope_start = 0.0;
        double slope_end_right = 0.0;
        double slope_end_left = 0.0;

        double slope_second_right = 0.0;
        double slope_second_left = 0.0;
        double slope_first_right = 0.0;
        double slope_first_left = 0.0;
        

        double slope_end = 0.0;
        Point merge_target = {0.0, 0.0};

        int target_flag = 0;
        std::vector<Point> target_points;
        
        // Publishers
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_trajectory_candidates_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_best_trajectory_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr p_target_point_;
        
        // Subscribers
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_driving_way_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_ego_prediction_;
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_object_prediction_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_behavior_state_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr s_static_position_;

        // Callback Functions
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = *msg;
        }
       
        inline void CallbackDrivingWay(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_driving_way_);
            i_driving_way_ = *msg;
        }

        inline void CallbackEgoPrediction(const ad_msgs::msg::Mission::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_ego_prediction_);
            i_ego_prediction_ = *msg;
        }

        inline void CallbackObjectPrediction(const ad_msgs::msg::Mission::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_obeject_prediction_);
            i_object_prediction_ = *msg;
        }

        inline void CallbackBehaviorState(const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_behavior_state_);
            i_behavior_state_ = *msg;
        }

        inline void CallbackStaticPosition(const geometry_msgs::msg::Point::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_static_position_);
            i_static_position_ = *msg;
        }

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        ad_msgs::msg::VehicleState i_vehicle_state_;
        ad_msgs::msg::PolyfitLaneData i_driving_way_;
        ad_msgs::msg::Mission i_mission_state_;
        ad_msgs::msg::Mission i_ego_prediction_;
        ad_msgs::msg::Mission i_object_prediction_;
        std_msgs::msg::Float32 i_behavior_state_;
        geometry_msgs::msg::Point i_static_position_;

        // Outputs
        ad_msgs::msg::PolyfitLaneDataArray o_trajectories_;
        ad_msgs::msg::PolyfitLaneData o_best_trajectory_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_driving_way_;
        std::mutex mutex_ego_prediction_;
        std::mutex mutex_obeject_prediction_;
        std::mutex mutex_behavior_state_;
        std::mutex mutex_static_position_;
};

#endif 