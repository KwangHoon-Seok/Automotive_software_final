# include "motion_prediction_node.hpp"


MotionPredictionNode::MotionPredictionNode(const std::string& node_name, const double& loop_rate,
                        const rclcpp::NodeOptions& optios = rclcpp::NodeOptions())
    Node(node_name, options){
    RCLCPP_INFO(this->get_logger(), "Initializing MotionPredictionNode..." )

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", qos_profile,
        std::bind(&BehaviorPlannerNode::CallbackVehicleState, this, std::placeholders::_1));

    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", qos_profile,
        std::bind(&BehaviorPlannerNode::CallbackMissionState, this, std::placeholders::_1));
    
    // Publishers
    p_motion_ = this->create_publisher<std::vector<geometry_msgs::msg::Point>>("/ego/motion", qos_profile);
    

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

}
MotionPredictionNode::~MotionPredictionNode() {};

void MotionPredictionNode::Run(){
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleState vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_mission_state_.lock();
    ad_ms::msg::Mission mission_state = i_mission_state_;
    mutex_mission_state_.unlock();

    prediction();


}

void MotionPredictionNode::prediction(const ad_msgs::msg::VehicleState& vehicle_state, const ad_ms::msg::Mission& mission_state){
    geometry_msgs::msg::Point position;
    motion_.clear();

    if(mission_state.objects.size() > 0){
        for(const auto& object : mission_state_.objects){
            if(object.is_reach_ned == false && object.object_type == "Dynamic"){
                for (int i = 1; i < 10; i++){
                    position.x = object.x + object.velocity * cos(object.yaw) * i * (5/10);
                    position.y = object.y + object.velocity * sin(object.yaw) * i * (5/10);
                    motion_.push_back(postion);
                }

            }

        }

        PublishMotion(current_time);
    }

}

// publish type 어떻게 해야할지 알아보기
// display node에서 할 수 있도록
void MotionPredictionNode::PublishMotion(const rclcpp::Time& current_time) {
    // RCLCPP_INFO(this->get_logger(), "Publishing at time: %f", current_time.seconds());
    p_motion_->publish(motion_);
    
}





int main(int argc, char **argv) {
    std::string node_name = "motion_prediction_node";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivingWayNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}