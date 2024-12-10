#include "trajectory_planner_node.hpp"

TrajectoryNode::TrajectoryNode(const std::string &node_name, const double &loop_rate, const rclcpp::NodeOptions & options)
    : Node(node_name, options){
        t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / loop_rate)),
        std::bind(&TrajectoryNode::Run, this)
        );
        // Initialize Subscribers
        s_driving_way_ = this->create_subscription<ad_msgs::msg::PolyfitLaneData>(
            "/ego/driving_way", 10,
            std::bind(&TrajectoryNode::CallbackDrivingWay, this, std::placeholders::_1));
        s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleState>(
            "/ego/vehicle_state", 10,
            std::bind(&TrajectoryNode::CallbackVehicleState, this, std::placeholders::_1));

        // Initialize Publishers
        p_trajectory_candidates_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
            "/trajectory_candidates", rclcpp::QoS(10));

        // Timer
        RCLCPP_INFO(this->get_logger(), "TrajectoryNode has been initialized.");
    }

TrajectoryNode::~TrajectoryNode() {}


void TrajectoryNode::Run() {
    std::scoped_lock lock(mutex_vehicle_state_, mutex_driving_way_);
    

    RCLCPP_INFO(this->get_logger(), "Running the Trajectory Planner Node ...");
    if (!is_vehicle_state_initialized_) {
        RCLCPP_WARN(this->get_logger(), "Vehicle state not initialized. Waiting for vehicle_state");
    }

    start_.x = i_vehicle_state_.x;
    start_.y = i_vehicle_state_.y;
    start_.theta = i_vehicle_state_.yaw;
    if (i_vehicle_state_.velocity > 2){
        start_.kappa = i_vehicle_state_.yaw_rate / i_vehicle_state_.velocity;
    }else{
        start_.kappa = 0.0;
    }

    ReferencePathSampling(i_driving_way_);

    // (s, l) 격자 생성
    auto sl_grid = GenerateSLGrid(reference_path_, -3.0, 3.0, 0.5);

    // Cubic Spiral 경로 생성
    auto cubic_spiral_coeffs = GenerateCubicSpiralCoefficients(sl_grid);

    // Cubic Spiral 계수 퍼블리시
    PublishTrajectoryCoefficients(cubic_spiral_coeffs);
}


void TrajectoryNode::ReferencePathSampling(const ad_msgs::msg::PolyfitLaneData &driving_way) {
    reference_path_.clear();

    // 다항식 계수 추출
    double a0 = driving_way.a0;
    double a1 = driving_way.a1;
    double a2 = driving_way.a2;
    double a3 = driving_way.a3;

    // 도로 길이와 샘플링 간격
    double x_range = 20.0; // 도로 길이
    double s_step = 1.0;   // 아크 길이 기준 샘플링 간격

    // 초기값
    double prev_x = 0.0;
    double prev_y = a3 * std::pow(prev_x, 3) + a2 * std::pow(prev_x, 2) + a1 * prev_x + a0;

    double accumulated_s = 0.0;

    // 참조 경로 생성
    for (double x = 0.0; x <= x_range; x += 0.01) { // 세밀하게 샘플링
        double y = a3 * std::pow(x, 3) + a2 * std::pow(x, 2) + a1 * x + a0;

        // 아크 길이 계산 (두 점 사이의 거리)
        double dx = x - prev_x;
        double dy = y - prev_y;
        double ds = std::sqrt(dx * dx + dy * dy);

        accumulated_s += ds;

        // 곡률 계산
        double dy_dx = 3 * a3 * std::pow(x, 2) + 2 * a2 * x + a1;
        double d2y_dx2 = 6 * a3 * x + 2 * a2;
        double kappa = std::fabs(d2y_dx2) / std::pow(1 + std::pow(dy_dx, 2), 1.5);

        // s_step마다 샘플링
        if (accumulated_s >= s_step) {
            double theta = std::atan2(dy, dx); // 방향 계산
            reference_path_.emplace_back(State{x, y, theta, kappa});
            accumulated_s = 0.0; // 누적 거리 초기화
        }

        prev_x = x;
        prev_y = y;
    }

    for (const auto &state : reference_path_) 
    {
        RCLCPP_INFO(this->get_logger(), "State: x=%f, y=%f, theta=%f, kappa=%f", state.x, state.y, state.theta, state.kappa);
    }
}

std::vector<State> TrajectoryNode::GenerateSLGrid(const std::vector<State> &reference_path, double l_min, double l_max, double l_step) {
    std::vector<State> sl_grid;

    for (size_t i = 0; i < reference_path.size(); ++i) {
        const auto &ref_point = reference_path[i];
        double x_s = ref_point.x;
        double y_s = ref_point.y;
        double theta_s = ref_point.theta;
        double kappa_s = ref_point.kappa;

        // s 값은 참조 경로 상에서의 길이, i가 인덱스이므로 이를 누적 거리로 계산
        double s = i * 1.0; // 참조 경로 샘플링 간격(1.0)을 기준으로 종방향 거리 계산

        for (double l = l_min; l <= l_max; l += l_step) {
            double x_sl = x_s + l * std::cos(theta_s);
            double y_sl = y_s + l * std::sin(theta_s);
            double kappa_sl = (std::fabs(kappa_s) > 1e-6) ? 1.0 / ((1.0 / kappa_s) + l) : 0.0;

            sl_grid.emplace_back(State{x_sl, y_sl, theta_s, kappa_sl, s, l});
        }
    }

    return sl_grid;
}


//-------------------Cubic Spiral Function Module----------------------//
std::vector<State> TrajectoryNode::SelectTargetPoints(const std::vector<State> &sl_grid) {
    std::unordered_map<double, State> max_s_points; // l 위치별 최대 s값 저장

    for (const auto &point : sl_grid) {
        double l = point.l;
        if (max_s_points.find(l) == max_s_points.end() || point.s > max_s_points[l].s) {
            max_s_points[l] = point;
        }
    }

    std::vector<State> target_points;
    for (const auto &[l, point] : max_s_points) {
        target_points.push_back(point);
    }

    return target_points;
}


double TrajectoryNode::SimpsonIntegration(double a, double b, const std::function<double(double)> &f, int n) {
    
    if (n % 2 != 0) n++; // n은 짝수여야 함
    double h = (b - a) / n; // step size
    double integral = f(a) + f(b);
    // 홀수 인덱스 합
    for(int i = 1; i < n; i += 2){
        integral += 4 * f(a + i*h);
    }
    // 짝수 인덱스 합
    for(int i = 2; i < n; i += 2){
        integral += 2 * f(a + i*h);
    }
    integral = integral * h / 3.0;


    return integral;
}



State TrajectoryNode::ComputeEndpoint(const std::vector<double> &p, const State &start) {
    double sf = p[4]; // 끝점 아크 길이

    // 끝점에서의 곡률 계산
    double kappa = p[0] + p[1] * sf + p[2] * sf * sf + p[3] * sf * sf * sf;

    // 끝점에서의 방향각 계산
    double theta = start.theta + SimpsonIntegration(0, sf, [&](double s) {
        return p[0] + p[1] * s + p[2] * s * s + p[3] * s * s * s;
    });

    // 끝점에서의 좌표 계산
    double x = start.x + SimpsonIntegration(0, sf, [&](double s) {
        double local_theta = start.theta + SimpsonIntegration(0, s, [&](double u) {
            return p[0] + p[1] * u + p[2] * u * u + p[3] * u * u * u;
        });
        return cos(local_theta);
    });

    double y = start.y + SimpsonIntegration(0, sf, [&](double s) {
        double local_theta = start.theta + SimpsonIntegration(0, s, [&](double u) {
            return p[0] + p[1] * u + p[2] * u * u + p[3] * u * u * u;
        });
        return sin(local_theta);
    });

    return State{x, y, theta, kappa};
}


Eigen::MatrixXd TrajectoryNode::ComputeJacobian(const std::vector<double> &p, const State &start) {
    const double sf = p[3]; // 끝점 아크 길이
    Eigen::MatrixXd J(4, 4); // 4x4 자코비안 행렬

    // 파라미터별 편미분 계산
    for (int i = 0; i < 4; ++i) {
        // x_p(sf)에 대한 편미분
        J(0, i) = SimpsonIntegration(0, sf, [&](double s) {
            double theta = start.theta + SimpsonIntegration(0, s, [&](double u) {
                return PartialKappa(u, p, i); // 곡률에 대한 편미분
            });
            return -std::sin(theta) * PartialTheta(s, p, i); // x에 대한 편미분
        });

        // y_p(sf)에 대한 편미분
        J(1, i) = SimpsonIntegration(0, sf, [&](double s) {
            double theta = start.theta + SimpsonIntegration(0, s, [&](double u) {
                return PartialKappa(u, p, i);
            });
            return std::cos(theta) * PartialTheta(s, p, i); // y에 대한 편미분
        });

        // \theta_p(sf)에 대한 편미분
        J(2, i) = SimpsonIntegration(0, sf, [&](double s) {
            return PartialKappa(s, p, i); // \kappa에 대한 편미분
        });

        // \kappa_p(sf)에 대한 편미분
        J(3, i) = PartialKappa(sf, p, i); // 끝점에서의 \kappa 편미분
    }

    return J;
}


// 곡률의 편미분 계산
double TrajectoryNode::PartialKappa(double s, const std::vector<double> &p, int i) {
    if (i == 0) return 1.0;       // p0에 대한 편미분
    if (i == 1) return s;         // p1에 대한 편미분
    if (i == 2) return s * s;     // p2에 대한 편미분
    if (i == 3) return s * s * s; // p3에 대한 편미분
    return 0.0;
}


// 방향의 편미분 계산
double TrajectoryNode::PartialTheta(double s, const std::vector<double> &p, int i) {
    return SimpsonIntegration(0, s, [&](double u) {
        return PartialKappa(u, p, i); // 곡률의 편미분을 적분
    });
}

std::vector<CubicSpiralCoefficients> TrajectoryNode::GenerateCubicSpiralCoefficients(const std::vector<State> &sl_grid)
{
    std::vector<CubicSpiralCoefficients> coefficients;

    // Step 1: 각 l 축에서 가장 큰 s 점 선택
    auto target_points = SelectTargetPoints(sl_grid);

    // Step 2: 각 타겟 점에 대해 Cubic Spiral 계산
    for (const auto &target : target_points)
    {   
        // p0 = 초기 곡률값
        double p0 = 0.0;
        // p => p1, p2 , p3 , sf
        
        int max_iter = 1000;
        double tol = 2.0;

        for (int iter = 0; iter < max_iter; ++iter)
        {
            State endpoint = ComputeEndpoint(p, start_);

            // 오차 계산
            Eigen::VectorXd error(4);
            error << target.x - endpoint.x, target.y - endpoint.y, target.theta - endpoint.theta, target.kappa - endpoint.kappa;
            
            RCLCPP_INFO(this->get_logger(), "수치적분 쪽 문제");
            RCLCPP_INFO(this->get_logger(), "Iteration %d, Error norm: %f", iter, error.norm());

            if (error.norm() < tol)
                break;

            // Jacobian 계산
            Eigen::MatrixXd J = ComputeJacobian(p, start_);

            // Newton's Method 업데이트
            Eigen::VectorXd delta_p = J.inverse() * error;
            for (std::size_t i = 0; i < p.size(); ++i)
            {
                p[i] += delta_p(i);
            }
        }
        p = {p[0],p[1],p[2],p[3]};

        std::vector<double> p_v = {p0, p[0], p[1], p[2], p[3]}; // 완전한 P벡터

        double sf = p[3];
        double b = (-11 * p0 - 18 * p[0] + 9 * p[1] - 2 * p[2]) / (2 * sf);
        double c = (9 * (2 * p0 - 5 * p[0] + 4 * p[1] - p[2])) / (2 * sf * sf);
        double d = (-9 * (p0 - 3 * p[0] + 3 * p[1] - p[2])) / (2 * sf * sf * sf);
        // 최종 Cubic Spiral 계수 저장
        coefficients.push_back({p0, b, c, d, sf});
    }

    return coefficients;
}

void TrajectoryNode::PublishTrajectoryCoefficients(const std::vector<CubicSpiralCoefficients> &coefficients) {
    // Create a message to hold the coefficients
    ad_msgs::msg::PolyfitLaneDataArray msg;
    msg.frame_id = "/ego/body";

    // Fill the message with cubic spiral coefficients
    for (size_t i = 0; i < coefficients.size(); ++i) {
        ad_msgs::msg::PolyfitLaneData lane_data;
        lane_data.frame_id = msg.frame_id;
        lane_data.id = std::to_string(i + 1);
        lane_data.a0 = coefficients[i].a;
        lane_data.a1 = coefficients[i].b;
        lane_data.a2 = coefficients[i].c;
        lane_data.a3 = coefficients[i].d;
        
        // Add to the array
        msg.polyfitlanes.push_back(lane_data);
    }

    // Publish the message
    p_trajectory_candidates_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published %zu cubic spiral candidates", coefficients.size());
}

int main(int argc, char **argv) {
    std::string node_name = "trajectory_planner_node";
    double loop_rate = 100.0;
    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
