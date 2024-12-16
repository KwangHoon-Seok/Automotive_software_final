#ifndef __PARKING_NODE_HPP__
#define __PARKING_NODE_HPP__


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
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/lane_point_data.hpp>
#include <ad_msgs/msg/polyfit_lane_data.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <eigen3/Eigen/Dense>

// Point 구조체 정의
struct Point {
    float x, y, z;
    bool visited = false;
    int cluster = -1;

    Point(const geometry_msgs::msg::Point& msg_point)
        : x(msg_point.x), y(msg_point.y), z(msg_point.z) {}

    Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Point() = default;
    Point(const Point&) = default;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};


class ParkingNode : public rclcpp::Node {
    public:
        ParkingNode(const std::string& node_name, const double& loop_rate,
                        const rclcpp::NodeOptions& optios = rclcpp::NodeOptions());
        ~ParkingNode();


    private:
        void Run(const rclcpp::Time& current_time);
        float distance(const Point& p1, const Point& p2);
        std::vector<int> regionQuery(const std::vector<Point>& points, int pointIdx, float epsilon);
        void expandCluster(std::vector<Point>& points, int pointIdx, int clusterID, float epsilon, size_t min_samples);
        void dbscan(const ad_msgs::msg::LanePointData& lane_points);
        void PublishParking(const rclcpp::Time& current_time);
        
        Eigen::Vector3d Local2Global(float x, float y, const ad_msgs::msg::VehicleState& vehicle_state);
        Eigen::Vector3d Global2Local(float x, float y, const ad_msgs::msg::VehicleState& vehicle_state);


        rclcpp::TimerBase::SharedPtr t_run_node_;
        // Publisher
        rclcpp::Publisher<ad_msgs::msg::LanePointData>::SharedPtr p_lane_parking_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_parking_way_;

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;  // trajectory generate
        rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_state_;       // bool parking 
        rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr s_lane_points_;   // parking lane


        // Callback Functions 
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        i_vehicle_state_ = *msg;
        }
        inline void CallbackMissionState(const ad_msgs::msg::Mission::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_mission_state_);
        i_mission_state_ = *msg;
        }
        inline void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {
        mutex_lane_points_.lock();
        i_lane_points_ = *msg;
        mutex_lane_points_.unlock();
        }

        ad_msgs::msg::VehicleState i_vehicle_state_;
        ad_msgs::msg::Mission i_mission_state_;
        ad_msgs::msg::LanePointData i_lane_points_;
        
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_mission_state_;
        std::mutex mutex_lane_points_;

        // Custom Variables

        ad_msgs::msg::LanePointData lane_parking_;
        ad_msgs::msg::PolyfitLaneData parking_way_;
        bool is_init = false;
        bool can_start = false;

        ad_msgs::msg::PolyfitLaneData o_parking_way_;
        ad_msgs::msg::LanePointData o_lane_parking_;
        

        

        geometry_msgs::msg::Point target_global_space;
        
};
#endif //__PARKING_Node_NODE_HPP__