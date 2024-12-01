// #include "trajectory_planner.hpp"

// TrajectoryPlanner::TrajectoryPlanner(const std::string& node_name, const double& loop_rate,
//                                      const rclcpp::NodeOptions& options)
//     : Node(node_name, options),
//       a_step_distance_(2.0),
//       a_num_layers_(3),
//       a_heading_increment_(M_PI / 4), // 45도씩 증가
//       a_min_turn_radius_(5.0)
// {
//     auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

//     // Subscriber
//     s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
//         "/ego/vehicle_state", qos_profile,
//         std::bind(&TrajectoryPlanner::CallbackVehicleState, this, std::placeholders::_1));

//     // Publisher:
//     p_planned_path_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", qos_profile);

//     // Timer: 경로 계획 주기 설정
//     t_run_node_ = this->create_wall_timer(
//         std::chrono::milliseconds(static_cast<int>(1000 / loop_rate)),
//         std::bind(&TrajectoryPlanner::PlanPath, this));

//     // Motion Primitives 정의
//     DefineMotionPrimitives();

//     RCLCPP_INFO(this->get_logger(), "TrajectoryPlanner initialized.");
// }

// TrajectoryPlanner::~TrajectoryPlanner()
// {
//     RCLCPP_INFO(this->get_logger(), "TrajectoryPlanner terminated.");
// }


// void TrajectoryPlanner::DefineMotionPrimitives()
// {
//     // 최소 회전 반경을 기반으로 가능한 움직임 정의
//     a_motion_primitives_.clear();
//     double delta_theta = a_heading_increment_;

//     // 직진
//     a_motion_primitives_.push_back({a_step_distance_, 0.0, 0.0});

//     // 좌회전
//     a_motion_primitives_.push_back({a_step_distance_, a_step_distance_ / a_min_turn_radius_, delta_theta});

//     // 우회전
//     a_motion_primitives_.push_back({a_step_distance_, -a_step_distance_ / a_min_turn_radius_, -delta_theta});
// }

// void TrajectoryPlanner::GenerateSearchSpace()
// {
//     std::lock_guard<std::mutex> lock(mutex_vehicle_state_);

//     a_search_space_.clear();
//     a_state_lattice_.clear();

//     State start_state = {i_vehicle_state_.x, i_vehicle_state_.y, i_vehicle_state_.yaw};

//     std::vector<State> current_states = {start_state};

//     for (int layer = 0; layer < a_num_layers_; ++layer)
//     {
//         std::vector<State> next_states;
//         for (const auto& state : current_states)
//         {
//             for (const auto& primitive : a_motion_primitives_)
//             {
//                 // 새로운 상태 계산
//                 State new_state;
//                 new_state.x = state.x + primitive.x * std::cos(state.theta);
//                 new_state.y = state.y + primitive.x * std::sin(state.theta);
//                 new_state.theta = std::fmod(state.theta + primitive.theta, 2 * M_PI);
//                 if (new_state.theta < 0) new_state.theta += 2 * M_PI;

//                 a_search_space_.push_back(new_state);
//                 a_state_lattice_[state].push_back(new_state);
//                 next_states.push_back(new_state);
//             }
//         }
//         current_states = next_states;
//     }
// }

// double TrajectoryPlanner::Heuristic(const State& state, const State& goal)
// {
//     // Control Set을 고려한 휴리스틱 함수
//     double dx = goal.x - state.x;
//     double dy = goal.y - state.y;
//     double distance = std::sqrt(dx * dx + dy * dy);

//     // 방향 차이 고려
//     double dtheta = std::fabs(goal.theta - state.theta);
//     if (dtheta > M_PI)
//         dtheta = 2 * M_PI - dtheta;

//     double heading_cost = a_min_turn_radius_ * dtheta;

//     return distance + heading_cost;
// }

// void TrajectoryPlanner::PlanPath()
// {
//     GenerateSearchSpace();

//     State start_state = {i_vehicle_state_.x, i_vehicle_state_.y, i_vehicle_state_.yaw};
//     State goal_state = {i_vehicle_state_.x + 10.0, i_vehicle_state_.y + 10.0, 0.0}; // 임의의 목표 상태 설정

//     // A* 알고리즘 초기화
//     std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
//     std::unordered_map<State, double> g_score;
//     std::unordered_map<State, std::shared_ptr<Node>> all_nodes;

//     auto start_node = std::make_shared<Node>();
//     start_node->state = start_state;
//     start_node->g = 0.0;
//     start_node->h = Heuristic(start_state, goal_state);
//     start_node->f = start_node->g + start_node->h;
//     start_node->parent = nullptr;

//     open_set.push(*start_node);
//     g_score[start_state] = 0.0;
//     all_nodes[start_state] = start_node;

//     bool path_found = false;

//     while (!open_set.empty())
//     {
//         Node current = open_set.top();
//         open_set.pop();

//         if (Heuristic(current.state, goal_state) < 1.0) // 목표 상태에 도달
//         {
//             path_found = true;
//             // 경로 복원
//             nav_msgs::msg::Path planned_path;
//             planned_path.header.frame_id = "map";
//             planned_path.header.stamp = this->get_clock()->now();

//             auto node_ptr = std::make_shared<Node>(current);
//             while (node_ptr != nullptr)
//             {
//                 geometry_msgs::msg::PoseStamped pose_stamped;
//                 pose_stamped.pose.position.x = node_ptr->state.x;
//                 pose_stamped.pose.position.y = node_ptr->state.y;
//                 pose_stamped.pose.position.z = 0.0;

//                 pose_stamped.pose.orientation.z = std::sin(node_ptr->state.theta / 2);
//                 pose_stamped.pose.orientation.w = std::cos(node_ptr->state.theta / 2);

//                 planned_path.poses.push_back(pose_stamped);

//                 node_ptr = node_ptr->parent;
//             }

//             std::reverse(planned_path.poses.begin(), planned_path.poses.end());
//             o_planned_path_ = planned_path;

//             // 경로 퍼블리시
//             p_planned_path_->publish(o_planned_path_);

//             RCLCPP_INFO(this->get_logger(), "Path found and published.");
//             break;
//         }

//         for (const auto& neighbor_state : a_state_lattice_[current.state])
//         {
//             double tentative_g_score = current.g + Heuristic(current.state, neighbor_state);

//             if (g_score.find(neighbor_state) == g_score.end() || tentative_g_score < g_score[neighbor_state])
//             {
//                 g_score[neighbor_state] = tentative_g_score;

//                 auto neighbor_node = std::make_shared<Node>();
//                 neighbor_node->state = neighbor_state;
//                 neighbor_node->g = tentative_g_score;
//                 neighbor_node->h = Heuristic(neighbor_state, goal_state);
//                 neighbor_node->f = neighbor_node->g + neighbor_node->h;
//                 neighbor_node->parent = all_nodes[current.state];

//                 open_set.push(*neighbor_node);
//                 all_nodes[neighbor_state] = neighbor_node;
//             }
//         }
//     }

//     if (!path_found)
//     {
//         RCLCPP_WARN(this->get_logger(), "No path found.");
//     }
// }
