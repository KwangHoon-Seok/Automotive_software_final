#include "driving_way_node.hpp"

DrivingWayNode::DrivingWayNode(const std::string& node_name, const double& loop_rate, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    RCLCPP_INFO(this->get_logger(), "Initializing DrivingWayNode...");
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribers
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "/ego/lane_points", qos_profile,
        std::bind(&DrivingWayNode::CallbackLanePoints, this, std::placeholders::_1));
    
    // Publishers
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>("/ego/driving_way", qos_profile);
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>("/ego/poly_lanes", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

    lane_point_LEFT.id = "1";
    lane_point_RIGHT.id = "2";
    inliers_LEFT.id = "1";
    inliers_RIGHT.id = "2";
}

DrivingWayNode::~DrivingWayNode() {}

void DrivingWayNode::Run(const rclcpp::Time& current_time) {
    ad_msgs::msg::LanePointData lane_points;
    {
        std::lock_guard<std::mutex> lock(mutex_lane_points_);
        lane_points = i_lane_points_;
    }

    if (lane_points.point.empty()) {
        RCLCPP_WARN(this->get_logger(), "No lane points received. Skipping processing...");
        return;
    }

    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    ad_msgs::msg::PolyfitLaneData driving_way;

    lane_point_LEFT.point.clear();
    lane_point_RIGHT.point.clear();
    inliers_LEFT.point.clear();
    inliers_RIGHT.point.clear();

    splitLanePoints(lane_points);
    number_point = 4;
    process_lanes();
    populatePolyLanes(poly_lanes);
    populateCenterLane(driving_way);

    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;

    PublishDrivingWay(current_time);
}

// Functions
std::vector<int> DrivingWayNode::regionQuery(const std::vector<Point>& points, int pointIdx, float epsilon) {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (distance(points[pointIdx], points[i]) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

void DrivingWayNode::expandCluster(std::vector<Point>& points, int pointIdx, int clusterID, float epsilon, size_t min_samples) {
    std::vector<int> seeds = regionQuery(points, pointIdx, epsilon);
    if (seeds.size() < min_samples) {
        points[pointIdx].cluster = -1; // 노이즈로 처리
        return;
    }

    // 초기 포인트 설정
    points[pointIdx].cluster = clusterID;

    // 클러스터 확장
    for (size_t i = 0; i < seeds.size(); ++i) {
        int idx = seeds[i];
        if (!points[idx].visited) {
            points[idx].visited = true;
            std::vector<int> result = regionQuery(points, idx, epsilon);
            if (result.size() >= min_samples) {
                seeds.insert(seeds.end(), result.begin(), result.end());
            }
        }
        if (points[idx].cluster == -1) {
            points[idx].cluster = clusterID;
        }
    }
}

void DrivingWayNode::splitLanePoints(const ad_msgs::msg::LanePointData& lane_points) {
    float epsilon = 3.5;  // 밀도 거리
    size_t min_samples = 3;  // 최소 포인트 개수

    // 1. lane_points를 vector<Point>로 변환
    std::vector<Point> points;
    for (const auto& point : lane_points.point) {
        points.push_back({static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)});
    }

    // 2. y > 0과 y < 0에 따라 가장 가까운 포인트(p0, p1)를 선택
    Point p0, p1;
    bool p0_found = false, p1_found = false;
    float min_distance_p0 = std::numeric_limits<float>::max();
    float min_distance_p1 = std::numeric_limits<float>::max();

    for (const auto& point : points) {
        float distance = std::hypot(point.x, point.y);
        if (point.y > 0 && distance < min_distance_p0) {
            min_distance_p0 = distance;
            p0 = point;
            p0_found = true;
        } else if (point.y < 0 && distance < min_distance_p1) {
            min_distance_p1 = distance;
            p1 = point;
            p1_found = true;
        }
    }

    // 3. 초기 좌/우 차선에 대해 클러스터링을 수행할 준비
    lane_point_LEFT.point.clear();
    lane_point_RIGHT.point.clear();

    // 4. 좌측 차선 DBSCAN 시작 (p0 기준으로 expandCluster 호출)
    if (p0_found) {
        size_t p0_index = std::distance(points.begin(), std::find(points.begin(), points.end(), p0));
        expandCluster(points, p0_index, 0, epsilon, min_samples); // 좌측 클러스터 ID는 0
        geometry_msgs::msg::Point new_point;

        for (const auto& point : points) {
            if (point.cluster == 0) {  // 좌측 차선에 속하는 포인트
                new_point.x = point.x;
                new_point.y = point.y;
                new_point.z = point.z;
                lane_point_LEFT.point.push_back(new_point);
            }
        }
    }

    // 5. 우측 차선 DBSCAN 시작 (p1 기준으로 expandCluster 호출)
    if (p1_found) {
        size_t p1_index = std::distance(points.begin(), std::find(points.begin(), points.end(), p1));
        expandCluster(points, p1_index, 1, epsilon, min_samples); // 우측 클러스터 ID는 1
        geometry_msgs::msg::Point new_point;

        for (const auto& point : points) {
            if (point.cluster == 1) {  // 우측 차선에 속하는 포인트
                
                new_point.x = point.x;
                new_point.y = point.y;
                new_point.z = point.z;
                lane_point_RIGHT.point.push_back(new_point);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Lane points split using DBSCAN with initial points p0 and p1.");
}

void DrivingWayNode::process_lanes() {
    // RCLCPP_INFO(this->get_logger(), "[1-3] Processing Lanes");

    // RANSAC을 통해 각 차선의 inliers를 계산합니다.
   
    applyRANSAC(lane_point_LEFT, inliers_LEFT);
    applyRANSAC(lane_point_RIGHT, inliers_RIGHT);


    // 오른쪽 차선의 inliers로부터 A_RIGHT 계산
    if (inliers_RIGHT.point.size() > 0) {
        // x 좌표 기준으로 정렬
        std::sort(inliers_RIGHT.point.begin(), inliers_RIGHT.point.end(),
                  [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
                      return a.x < b.x;
                  });

        // 일정 간격으로 포인트를 추출
        size_t interval = inliers_RIGHT.point.size() / number_point;
        num_points_RIGHT = number_point;
        X_RIGHT.resize(num_points_RIGHT);
        Y_RIGHT.resize(num_points_RIGHT);

        for (size_t i = 0; i < number_point; ++i) {
            X_RIGHT(i) = inliers_RIGHT.point[i * interval].x;
            Y_RIGHT(i) = inliers_RIGHT.point[i * interval].y;
        }
        
        A_RIGHT = calculateA(X_RIGHT, Y_RIGHT);
        // RCLCPP_INFO(this->get_logger(), "[1-5] RIGHT LANE A: [%s]", this->Vector4dToString(A_RIGHT).c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "No inliers found for RIGHT lane.");
    }

    // 왼쪽 차선의 inliers로부터 A_LEFT 계산
    if (inliers_LEFT.point.size() > 0) {
        // x 좌표 기준으로 정렬
        std::sort(inliers_LEFT.point.begin(), inliers_LEFT.point.end(),
                  [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
                      return a.x < b.x;
                  });

        // 일정 간격으로 포인트를 추출
        size_t interval = inliers_LEFT.point.size() / number_point;
        num_points_LEFT = number_point;
        X_LEFT.resize(num_points_LEFT);
        Y_LEFT.resize(num_points_LEFT);

        for (size_t i = 0; i < number_point; ++i) {
            X_LEFT(i) = inliers_LEFT.point[i * interval].x;
            Y_LEFT(i) = inliers_LEFT.point[i * interval].y;
        }
        
        A_LEFT = calculateA(X_LEFT, Y_LEFT);
        
    } else {
        RCLCPP_WARN(this->get_logger(), "No inliers found for LEFT lane.");
    }
}

void DrivingWayNode::applyRANSAC(ad_msgs::msg::LanePointData& lane_points, ad_msgs::msg::LanePointData& inliers, int maxIterations, double distanceThreshold) {
    if (lane_points.point.size() < 4) {  // 최소 네 개의 포인트가 필요
        RCLCPP_WARN(this->get_logger(), "Not enough points for cubic RANSAC.");
        return;
    }

    std::vector<geometry_msgs::msg::Point>& points = lane_points.point;
    std::vector<geometry_msgs::msg::Point> bestInliers;
    srand(static_cast<unsigned>(time(0)));  // 시드 초기화

    for (int i = 0; i < maxIterations; ++i) {
        // 포인트 중에서 랜덤하게 4개 선택
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();
        int idx3 = rand() % points.size();
        int idx4 = rand() % points.size();
        while (idx2 == idx1) idx2 = rand() % points.size();
        while (idx3 == idx1 || idx3 == idx2) idx3 = rand() % points.size();
        while (idx4 == idx1 || idx4 == idx2 || idx4 == idx3) idx4 = rand() % points.size();

        // 3차 함수 모델을 계산
        auto [a, b, c, d] = computeCubicModel(points[idx1], points[idx2], points[idx3], points[idx4]);

        // 인라이어를 찾음
        std::vector<geometry_msgs::msg::Point> currentInliers;
        for (const auto& point : points) {
            if (pointToCubicDistance(point, a, b, c, d) < distanceThreshold) {
                currentInliers.push_back(point);
            }
        }

        // 더 많은 인라이어를 가진 모델을 저장
        if (currentInliers.size() > bestInliers.size()) {
            bestInliers = currentInliers;
        }
    }

    inliers.point = bestInliers;

    // 디버깅 로그
    RCLCPP_INFO(this->get_logger(), "RANSAC with cubic model: Number of inliers = %zu", inliers.point.size());
}

std::tuple<double, double, double, double> DrivingWayNode::computeCubicModel(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2,
    const geometry_msgs::msg::Point& p3,
    const geometry_msgs::msg::Point& p4) {

    Eigen::Matrix4d X;
    Eigen::Vector4d Y;

    X << std::pow(p1.x, 3), std::pow(p1.x, 2), p1.x, 1,
         std::pow(p2.x, 3), std::pow(p2.x, 2), p2.x, 1,
         std::pow(p3.x, 3), std::pow(p3.x, 2), p3.x, 1,
         std::pow(p4.x, 3), std::pow(p4.x, 2), p4.x, 1;

    Y << p1.y, p2.y, p3.y, p4.y;

    Eigen::Vector4d coeffs = X.colPivHouseholderQr().solve(Y);
    return {coeffs(0), coeffs(1), coeffs(2), coeffs(3)};
}

double DrivingWayNode::pointToCubicDistance(
    const geometry_msgs::msg::Point& point,
    double a, double b, double c, double d) {

    double y_estimated = a * std::pow(point.x, 3) + b * std::pow(point.x, 2) + c * point.x + d;
    return std::abs(point.y - y_estimated);
}

Eigen::Vector4d DrivingWayNode::calculateA(const Eigen::VectorXd& X, const Eigen::VectorXd& Y) {

    Eigen::MatrixXd X_matrix(X.size(), 4); // 3차 다항식의 경우 4개의 열
    for (int i = 0; i < X.size(); ++i) {
        X_matrix(i, 0) = std::pow(X(i), 3);  // x^3
        X_matrix(i, 1) = std::pow(X(i), 2);  // x^2
        X_matrix(i, 2) = X(i);               // x
        X_matrix(i, 3) = 1;                  // 상수항
    }
    Eigen::Vector4d A = X_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);

    return A; // Vector4d로 반환
}

std::string DrivingWayNode::Vector4dToString(const Eigen::Vector4d& vec) {
    std::ostringstream oss;
    for (int i = 0; i < 4; ++i) {  // Vector4d는 항상 크기가 4입니다.
        if (i > 0) {
            oss << ", ";
        }
        oss << vec(i);
    }
    return oss.str();
}

void DrivingWayNode::populatePolyLanes(ad_msgs::msg::PolyfitLaneDataArray& poly_lanes) {

    // poly_lanes 초기화
    poly_lanes.polyfitlanes.clear(); // 이전 데이터 초기화

    // 오른쪽 차선 (id = "1", A_RIGHT)
    ad_msgs::msg::PolyfitLaneData lane_right;
    lane_right.id = "1";  // id 설정
    lane_right.a0 = A_RIGHT(3);  // a0 계수
    lane_right.a1 = A_RIGHT(2);  // a1 계수
    lane_right.a2 = A_RIGHT(1);  // a2 계수
    lane_right.a3 = A_RIGHT(0);  // a3 계수
    lane_right.frame_id = "ego/body";  // frame_id 설정
    poly_lanes.polyfitlanes.push_back(lane_right);  // 오른쪽 차선을 polyfitlanes에 추가

    // 왼쪽 차선 (id = "2", A_LEFT)
    ad_msgs::msg::PolyfitLaneData lane_left;
    lane_left.id = "2";  // id 설정
    lane_left.a0 = A_LEFT(3);  // a0 계수
    lane_left.a1 = A_LEFT(2);  // a1 계수
    lane_left.a2 = A_LEFT(1);  // a2 계수
    lane_left.a3 = A_LEFT(0);  // a3 계수
    lane_left.frame_id = "ego/body";  // frame_id 설정
    poly_lanes.polyfitlanes.push_back(lane_left);  // 왼쪽 차선을 polyfitlanes에 추가

}

void DrivingWayNode::populateCenterLane(ad_msgs::msg::PolyfitLaneData& driving_way) {

    driving_way.id = "0"; // id 설정 아직 중앙이 0인줄 모름
    A_MID(0) = (A_RIGHT(3) + A_LEFT(3)) / 2.0;  // a0 계수
    A_MID(1) = (A_RIGHT(2) + A_LEFT(2)) / 2.0;  // a1 계수
    A_MID(2) = (A_RIGHT(1) + A_LEFT(1)) / 2.0;  // a2 계수
    A_MID(3) = (A_RIGHT(0) + A_LEFT(0)) / 2.0;  // a3 계수

    driving_way.a0 = A_MID(0);  // a0 계수
    driving_way.a1 = A_MID(1);  // a1 계수
    driving_way.a2 = A_MID(2);  // a2 계수
    driving_way.a3 = A_MID(3);  // a3 계수

}

//-------------------------------------------------------------------------------------------------------------------//

void DrivingWayNode::PublishDrivingWay(const rclcpp::Time& current_time) {
    p_driving_way_->publish(o_driving_way_);
    p_poly_lanes_->publish(o_poly_lanes_);
}
int main(int argc, char **argv) {
    std::string node_name = "driving_way_node";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivingWayNode>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}

