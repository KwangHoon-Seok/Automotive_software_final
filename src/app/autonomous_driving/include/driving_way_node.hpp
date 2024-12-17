#ifndef __DRIVING_WAY_NODE_HPP__
#define __DRIVING_WAY_NODE_HPP__

#define AEB 1
#define ACC 2
#define MERGE 3
#define REF_VEL_TRACKING 4


// STD Header
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <utility>
#include <cmath>
#include <mutex>

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <ad_msgs/msg/vehicle_command.hpp>
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/lane_point_data.hpp>
#include <ad_msgs/msg/polyfit_lane_data.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <ad_msgs/msg/mission.hpp>
#include <ad_msgs/msg/mission_object.hpp>

// Algorithm Header
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

class DrivingWayNode : public rclcpp::Node {
public:
    // Constructor & Destructor
    DrivingWayNode(const std::string& node_name, const double& loop_rate,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DrivingWayNode();

    // Main Execution Functions
    void Run(const rclcpp::Time& current_time);
    void PublishDrivingWay(const rclcpp::Time& current_time);

private:
    // Algorithm Functions
    void getMeanPoints(const ad_msgs::msg::LanePointData& lane_points, ad_msgs::msg::LanePointData& left_lane, ad_msgs::msg::LanePointData& right_lane);
    void getParkingWay(ad_msgs::msg::PolyfitLaneData& driving_way);
    void lane_condition(ad_msgs::msg::PolyfitLaneData& driving_way, const ad_msgs::msg::LanePointData& lane_points);
    std::vector<int> regionQuery(const std::vector<Point>& points, int pointIdx, float epsilon);
    void expandCluster(std::vector<Point>& points, int pointIdx, int clusterID, float epsilon, size_t min_samples);
    void populatePolyLanes(ad_msgs::msg::PolyfitLaneDataArray& poly_lanes);
    void populateCenterLane(ad_msgs::msg::PolyfitLaneData& driving_way);
    void splitLanePoints(const ad_msgs::msg::LanePointData& lane_points);
    void process_lanes(ad_msgs::msg::LanePointData& left_lane, ad_msgs::msg::LanePointData& right_lane);
    
    std::tuple<double, double, double, double> computeCubicModel(const geometry_msgs::msg::Point& p1,
                                                                 const geometry_msgs::msg::Point& p2,
                                                                 const geometry_msgs::msg::Point& p3,
                                                                 const geometry_msgs::msg::Point& p4);
    double pointToCubicDistance(const geometry_msgs::msg::Point& point, double a, double b, double c, double d);
    Eigen::Vector4d calculateA(const Eigen::VectorXd& X, const Eigen::VectorXd& Y);
    std::string Vector4dToString(const Eigen::Vector4d& vec);
    float distance(const Point& a, const Point& b);
    
    // Variables for Algorithm
    bool is_init = false;
    bool is_error = false;
    bool will_parking = false;
    bool start_parking = false;
    bool is_completed = false;
    bool stop_lane_detection = false;
    bool merge_completed = false;
    float max_x;
    float is_completed_;
    float merge_state_;
    double merge_mode;
    double drive_mode;

    std_msgs::msg::Float32 is_completed_msg;
    std_msgs::msg::Float32 merge_state_msg;
    
    
    geometry_msgs::msg::Point mean_point;


    
    ad_msgs::msg::PolyfitLaneData prev_driving_way_;
    ad_msgs::msg::PolyfitLaneData prev_lane_LEFT_;
    ad_msgs::msg::PolyfitLaneData prev_lane_RIGHT_;

    ad_msgs::msg::LanePointData sub_lane_point_LEFT_;
    ad_msgs::msg::LanePointData sub_lane_point_RIGHT_;
    ad_msgs::msg::LanePointData lane_point_LEFT;
    ad_msgs::msg::LanePointData lane_point_RIGHT;
    ad_msgs::msg::LanePointData inliers_LEFT;
    ad_msgs::msg::LanePointData inliers_RIGHT;

    size_t num_points_LEFT;
    size_t num_points_RIGHT;
    size_t num_sub_points_LEFT;
    size_t num_sub_points_RIGHT;
    size_t number_point;

    Eigen::VectorXd X_LEFT;
    Eigen::VectorXd Y_LEFT;
    Eigen::Vector4d A_LEFT;
    Eigen::VectorXd X_RIGHT;
    Eigen::VectorXd Y_RIGHT;
    Eigen::Vector4d A_RIGHT;
    Eigen::Vector4d A_MID;

    rclcpp::Time last_time_;

    // Publishers
    rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_driving_way_;
    rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_poly_lanes_;
    rclcpp::Publisher<ad_msgs::msg::LanePointData>::SharedPtr p_left_lane_;
    rclcpp::Publisher<ad_msgs::msg::LanePointData>::SharedPtr p_right_lane_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_is_completed_;


    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_behavior_state_;
    rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr s_lane_points_;
    rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr s_vehicle_state_;
    rclcpp::Subscription<ad_msgs::msg::Mission>::SharedPtr s_mission_state_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_merge_state_;

    // // Callback Functions
    // inline void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {
    //     std::lock_guard<std::mutex> lock(mutex_lane_points_);
    //     i_lane_points_ = *msg;
    // }

    // inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
    //     std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
    //     i_vehicle_state_ = *msg;
    // }
        // Callback Functions
    inline void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {
        mutex_lane_points_.lock();
        i_lane_points_ = *msg;
        mutex_lane_points_.unlock();
    }

    inline void CallbackVehicleState(const ad_msgs::msg::VehicleState::SharedPtr msg) {
        mutex_vehicle_state_.lock();
        i_vehicle_state_ = *msg;
        mutex_vehicle_state_.unlock();
    }

    inline void CallbackBehaviorState(const std_msgs::msg::Float32::SharedPtr msg) {
        mutex_behavior_.lock();
        i_behavior_state_ = *msg;
        mutex_behavior_.unlock();
    }
    inline void CallbackMissionState(const ad_msgs::msg::Mission::SharedPtr msg) {
        mutex_mission_state_.lock();
        i_mission_state_ = *msg;
        mutex_mission_state_.unlock();
    }
    inline void CallbackMergeState(const std_msgs::msg::Float32::SharedPtr msg) {
        mutex_merge_state_.lock();
        i_merge_state_ = *msg;
        mutex_merge_state_.unlock();
    }

    // Timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // Inputs
    ad_msgs::msg::LanePointData i_lane_points_;
    ad_msgs::msg::VehicleState i_vehicle_state_;
    std_msgs::msg::Float32 i_behavior_state_;
    ad_msgs::msg::Mission i_mission_state_;
    std_msgs::msg::Float32 i_merge_state_;

    // Outputs
    ad_msgs::msg::PolyfitLaneData o_driving_way_;
    ad_msgs::msg::PolyfitLaneDataArray o_poly_lanes_;
    ad_msgs::msg::LanePointData o_left_lane_;
    ad_msgs::msg::LanePointData o_right_lane_;

    // Mutex
    std::mutex mutex_lane_points_;
    std::mutex mutex_vehicle_state_;
    std::mutex mutex_behavior_;
    std::mutex mutex_mission_state_;
    std::mutex mutex_merge_state_;
};

#endif // __DRIVING_WAY_NODE_HPP__