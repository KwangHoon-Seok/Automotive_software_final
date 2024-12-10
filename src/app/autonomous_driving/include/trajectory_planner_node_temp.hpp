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


// Ros Header
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/polyfit_lane_data.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>



// Parameter Header
#include "autonomous_driving_config.hpp"

// My type
struct CubicSpiralCoefficients{
    double a;
    double b;
    double c;
    double d;
    double sf;
};

struct State{
    double x;
    double y;
    double theta;
    double kappa;
    double s; // 종방향 거리
    double l; // 횡방향 거리
    bool operator==(const State &other) const {
        return std::fabs(x - other.x) < 1 &&
               std::fabs(y - other.y) < 1 &&
               std::fabs(theta - other.theta) < 1e-4 &&
               std::fabs(kappa - other.kappa) < 1e-4;
    }
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
        // Reference Path Sampling fucntion
        void ReferencePathSampling(const ad_msgs::msg::PolyfitLaneData &driving_way);
        // (S, l) grid 생성 function
        std::vector<State> GenerateSLGrid(const std::vector<State> &reference_path, double l_min, double l_max, double l_step);

        // Cubic Spiral functions 
        std::vector<State> SelectTargetPoints(const std::vector<State> &sl_grid);
        std::vector<CubicSpiralCoefficients> GenerateCubicSpiralCoefficients(const std::vector<State> &sl_grid);
        Eigen::MatrixXd ComputeJacobian(const std::vector<double> &p, const State &start);
        State ComputeEndpoint(const std::vector<double> &p, const State &start);
        double SimpsonIntegration(double a, double b, const std::function<double(double)> &f, int n = 1000);
        double PartialKappa(double s, const std::vector<double> &p, int i);
        double PartialTheta(double s, const std::vector<double> &p, int i);

        // 최종 후보 경로 퍼블리시 함수
        void PublishTrajectoryCoefficients(const std::vector<CubicSpiralCoefficients> &coefficients);

        // Variabls for Algorithm
        bool is_vehicle_state_initialized_ = false;
        std::vector<State> reference_path_; // Reference Path
        State start_;
        // Publishers
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_trajectory_candidates_;
        std::vector<double> p = {0.0, 0.0, 0.0, 1.0};

        // Subscribers
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_driving_way_;
        
        // Callback Functions
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = *msg;
            is_vehicle_state_initialized_ = true;
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
        geometry_msgs::msg::PoseArray o_state_space;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_driving_way_;

};

#endif 