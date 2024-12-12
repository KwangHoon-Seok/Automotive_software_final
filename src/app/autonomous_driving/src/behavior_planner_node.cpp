#include "behavior_planner_node.hpp"
#include <cmath>
#include <limits>

BehaviorPlannerNode::BehaviorPlannerNode(const std::string& node_name, const double& loop_rate,
                                         const rclcpp::NodeOptions& options)
    : Node(node_name, options), current_velocity_(0.0), ref_vel_(40.0), o_behavior_state_(0.0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", qos_profile,
        std::bind(&BehaviorPlannerNode::CallbackVehicleState, this, std::placeholders::_1));

    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", qos_profile,
        std::bind(&BehaviorPlannerNode::CallbackMissionState, this, std::placeholders::_1));


    // Publishers
    p_behavior_state_ = this->create_publisher<std_msgs::msg::Float32>("/ego/behavior_state", qos_profile);
    p_ref_velocity_ = this->create_publisher<std_msgs::msg::Float32>("/ego/ref_vel", qos_profile);
    p_lead_distance_ = this->create_publisher<std_msgs::msg::Float32>("/ego/lead_distance", qos_profile);
    p_static_object_position_ = this->create_publisher<geometry_msgs::msg::Point>("/static/position", qos_profile);

    // Timer
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / loop_rate)),
        std::bind(&BehaviorPlannerNode::Run, this));

    RCLCPP_INFO(this->get_logger(), "BehaviorPlannerNode initialized.");
}

BehaviorPlannerNode::~BehaviorPlannerNode() {}

void BehaviorPlannerNode::Run()
{
    std::lock_guard<std::mutex> lock_vehicle(mutex_vehicle_state_);
    std::lock_guard<std::mutex> lock_mission(mutex_mission_state_);

    // Update behavior state based on the latest vehicle and mission data
    updatePlannerState();
    velocity_planner();

    // Publish the behavior state
    std_msgs::msg::Float32 state_msg;
    state_msg.data = o_behavior_state_;
    p_behavior_state_->publish(state_msg);

    std_msgs::msg::Float32 ref_velocity_msg;
    ref_velocity_msg.data = o_ref_velocity_;
    p_ref_velocity_->publish(ref_velocity_msg);

    std_msgs::msg::Float32 lead_distance_msg;
    lead_distance_msg.data = o_lead_distance_;
    p_lead_distance_->publish(lead_distance_msg);

    // RCLCPP_INFO(this->get_logger(),
    //              "Behavior State: %.1f", o_behavior_state_);
}

void BehaviorPlannerNode::updatePlannerState()
{
    // Variables to store the closest distances
    double closest_static_distance = 10000; 
    double closest_dynamic_distance = 10000;
    const double lane_width_threshold = 0.0;
    double yaw = i_vehicle_state_.yaw;
    
    bool static_object_found = false;


    // Iterate through all objects in the /ego/mission message
    for (const auto &object : i_mission_state_.objects) {
        // Calculate the distance between ego vehicle and the mission object
        double dx = object.x - i_vehicle_state_.x;
        double dy = object.y - i_vehicle_state_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // global -> local coordinate
        double dx_local = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double dy_local = -std::sin(yaw) * dx + std::cos(yaw) * dy;

        //RCLCPP_INFO(this->get_logger()," x: %.2f, y: %.2f", dx_local, dy_local);

        // Update the closest distances based on object type
        if(dx_local > 0 && dy_local >= lane_width_threshold){
            if (object.object_type == "Static") {
                if (dx > 0){
                    if (distance < closest_static_distance) {
                        closest_static_distance = distance; // Update closest static distance
                        static_object_position_.x = object.x;
                        static_object_position_.y = object.y;
                        static_object_position_.z = 0.0;
                        static_object_found = true;
                    }
                }  
            } else if (object.object_type == "Dynamic") {
                if (dx > 0){
                    if (distance < closest_dynamic_distance) {
                    closest_dynamic_distance = distance; // Update closest dynamic distance
                    }
                }
            }
        }
    }

    o_lead_distance_ = closest_dynamic_distance;

    // Decide behavior state based on the closest distances
        // Decide behavior state with priority: Static > Dynamic
    if (closest_static_distance < 50.0) {
        o_behavior_state_ = MERGE; // Static obstacle within 20m
    } else if (closest_dynamic_distance < 20.0) {
        o_behavior_state_ = ACC; // Dynamic obstacle within 20m
        if (closest_dynamic_distance < 10.0) {
            o_behavior_state_ = AEB; // Dynamic obstacle within 10m
        }
    } else {
        o_behavior_state_ = REF_VEL_TRACKING; // No obstacles in significant range
    }

    // Static object 좌표 퍼블리시
    if (static_object_found == true) {
        p_static_object_position_ -> publish(static_object_position_);
    }

    // Log the results
    // RCLCPP_INFO(this->get_logger(),
    //             "Closest Static Distance: %.2f, Closest Dynamic Distance: %.2f, Behavior State: %s",
    //             closest_static_distance, closest_dynamic_distance, BEHAVIOR_STATE_TO_STRING(o_behavior_state_));
}

void BehaviorPlannerNode::velocity_planner() {
    if (i_mission_state_.road_condition == "None") {
        bool has_static_object = false;

        for (const auto& object : i_mission_state_.objects) {
            if (object.object_type == "Static") {
                has_static_object = true;
                break;
            }
        }
        if (has_static_object) {
            o_ref_velocity_ = 5.0;
        } else {
            o_ref_velocity_ = 15.0;
        }
    } else {
        o_ref_velocity_ = 7.5;
    }
}



int main(int argc, char **argv) {
    std::string node_name = "behavior_planner_node";
    double loop_rate = 100.0;
    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorPlannerNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
