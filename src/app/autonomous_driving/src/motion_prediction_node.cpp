# include "motion_prediction_node.hpp"
/* psuedo code
input : vehicle_state , Mission
output : ad_msgs::msg::Mission motion_;
         -> motion_.objects의 x,y,time이 각각 predictioned 위치(x,y), prediction time을 의미
         ad_msgs::msg::Mission ego_motion_; 
         -> ego_motion_.objects의 x,y,time이 각각 predictioned 위치(x,y), prediction time을 의미
function : prediction();
         -> motion_ : CV Model (yaw_rate의 부재)
         -> ego_motion_ : CTRV Model (yaw_rate가 존재)
*/
MotionPredictionNode::MotionPredictionNode(const std::string& node_name, const double& loop_rate, const rclcpp::NodeOptions& options)
    : Node(node_name, options){
    RCLCPP_INFO(this->get_logger(), "Initializing MotionPredictionNode..." );

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
        "/ego/vehicle_state", qos_profile,
        std::bind(&MotionPredictionNode::CallbackVehicleState, this, std::placeholders::_1));

    s_mission_state_ = this->create_subscription<ad_msgs::msg::Mission>(
        "/ego/mission", qos_profile,
        std::bind(&MotionPredictionNode::CallbackMissionState, this, std::placeholders::_1));
    
    // Publishers
    p_motion_ = this->create_publisher<ad_msgs::msg::Mission>("/ego/object_prediction", qos_profile);
    p_ego_motion_ = this->create_publisher<ad_msgs::msg::Mission>("/ego/ego_prediction", qos_profile);
    

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

}
MotionPredictionNode::~MotionPredictionNode() {};

void MotionPredictionNode::Run(const rclcpp::Time& current_time){
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleState vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_mission_state_.lock();
    ad_msgs::msg::Mission mission_state = i_mission_state_;
    mutex_mission_state_.unlock();

    motion_.objects.clear();
    ego_motion_.objects.clear();
    prediction(vehicle_state, mission_state);
    PublishMotion(current_time);    
}

void MotionPredictionNode::prediction(const ad_msgs::msg::VehicleState& vehicle_state, const ad_msgs::msg::Mission& mission_state){
    ad_msgs::msg::MissionObject position;
    float prediction_time = 3;
    float interval = 30;
    // 이전 yaw_rate를 저장하기 위한 static 변수 추가
    static double previous_yaw_rate = 0.0;

    if (mission_state.objects.size() > 0) {
        for (const auto& object : mission_state.objects) {
            if (object.is_reach_end == false && object.object_type == "Dynamic") {
                double yaw_rate_to_use = 0.0;

                // 자차의 yaw_rate를 사용하되, 멈춰 있을 경우 이전 yaw_rate 사용
                if (std::abs(vehicle_state.velocity) < 0.5) { // 자차가 정지 상태
                    yaw_rate_to_use = previous_yaw_rate;
                } else {
                    yaw_rate_to_use = vehicle_state.yaw_rate;
                    previous_yaw_rate = yaw_rate_to_use; // yaw_rate 갱신
                }

                // CTRV 모델 적용
                if (std::abs(yaw_rate_to_use) > 1e-6) {
                    for (int i = 1; i < interval; i++) {
                        position.x = object.x + (object.velocity / yaw_rate_to_use) *
                                    (sin(object.yaw + yaw_rate_to_use * i * (prediction_time / interval)) 
                                    - sin(object.yaw));
                        position.y = object.y + (object.velocity / yaw_rate_to_use) *
                                    (-cos(object.yaw + yaw_rate_to_use * i * (prediction_time / interval)) 
                                    + cos(object.yaw));
                        position.time = i * (prediction_time / interval);
                        position.object_type = "Dynamic";
                        motion_.objects.push_back(position);
                        // RCLCPP_INFO(this->get_logger(), "CTRV Dynamic - TIME : %f X: %f Y: %f", position.time, position.x, position.y);
                    }
                } else { // Straight-line motion (yaw_rate ~ 0)
                    for (int i = 1; i < interval; i++) {
                        position.x = object.x + object.velocity * cos(object.yaw) * i * (prediction_time / interval);
                        position.y = object.y + object.velocity * sin(object.yaw) * i * (prediction_time / interval);
                        position.time = i * (prediction_time / interval);
                        position.object_type = "Dynamic";
                        motion_.objects.push_back(position);
                        // RCLCPP_INFO(this->get_logger(), "Straight-line Dynamic - TIME : %f X: %f Y: %f", position.time, position.x, position.y);
                    }
                }
            }
        }
    }



    // if(mission_state.objects.size() > 0){
    //     for(const auto& object : mission_state.objects){
    //         if(object.is_reach_end == false && object.object_type == "Dynamic"){
    //             for (int i = 1; i < interval; i++){
    //                 position.x = object.x + object.velocity * cos(object.yaw) * i * (prediction_time /interval);
    //                 position.y = object.y + object.velocity * sin(object.yaw) * i * (prediction_time /interval);
    //                 position.time = i * (prediction_time / interval);
    //                 position.object_type = "Dynamic";
    //                 motion_.objects.push_back(position);
    //                 // RCLCPP_INFO(this->get_logger(), "TIME : %f X: %f Y: %f", position.time, position.x, position.y);
    //             }
    //         }
    //     }
    // }

    // CTRV Model
    ad_msgs::msg::MissionObject ego_position;
    if(std::abs(vehicle_state.yaw_rate) > 1e-6){
        for(int i = 1; i < interval; i ++){
            ego_position.x = vehicle_state.x + (vehicle_state.velocity / vehicle_state.yaw_rate) * 
                        (sin (vehicle_state.yaw + vehicle_state.yaw_rate * i * (prediction_time /interval)) 
                        - sin(vehicle_state.yaw));
            ego_position.y = vehicle_state.y + (vehicle_state.velocity / vehicle_state.yaw_rate) * 
                        (-cos (vehicle_state.yaw + vehicle_state.yaw_rate * i * (prediction_time /interval)) 
                        + cos(vehicle_state.yaw));
            ego_position.time = i * (prediction_time / interval);
            ego_motion_.objects.push_back(ego_position);
            // RCLCPP_INFO(this->get_logger(), "TIME : %f X: %f Y: %f", ego_position.time, ego_position.x, ego_position.y);
        }
    }
    else{
        for(int i = 1; i < interval; i ++){
            ego_position.x = vehicle_state.x + vehicle_state.velocity * cos(vehicle_state.yaw) * i * (prediction_time /interval);
            ego_position.y = vehicle_state.y + vehicle_state.velocity * sin(vehicle_state.yaw) * i * (prediction_time /interval);
            ego_position.time = i * (prediction_time / interval);
            ego_motion_.objects.push_back(ego_position);
            // RCLCPP_INFO(this->get_logger(), "TIME : %f X: %f Y: %f", ego_position.time, ego_position.x, ego_position.y);
        }
    }
    
    

}

void MotionPredictionNode::PublishMotion(const rclcpp::Time& current_time) {
    // RCLCPP_INFO(this->get_logger(), "Publishing at time: %f", current_time.seconds());
    p_motion_->publish(motion_);
    p_ego_motion_->publish(ego_motion_);
    
}

int main(int argc, char **argv) {
    std::string node_name = "motion_prediction_node";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPredictionNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}