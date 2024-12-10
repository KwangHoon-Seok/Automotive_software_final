#include <rclcpp/rclcpp.hpp>
#include <ad_msgs/msg/vehicle_state.hpp>
#include <fstream>
#include <cmath>
#include <mutex>

class EgoVelocityStateListener : public rclcpp::Node {
public:
    EgoVelocityStateListener() : Node("ego_velocity_state_listener"), last_x_(std::numeric_limits<double>::quiet_NaN()),
                                  last_y_(std::numeric_limits<double>::quiet_NaN()) {
        // CSV 파일 초기화
        csv_file_ = "ego_position.csv";
        csv_stream_.open(csv_file_, std::ios::out | std::ios::trunc);
        if (csv_stream_.is_open()) {
            csv_stream_ << "x,y\n"; // 헤더 추가
        }

        // Subscriber 초기화
        subscription_ = this->create_subscription<ad_msgs::msg::VehicleState>(
            "/ego/vehicle_state", 10,
            std::bind(&EgoVelocityStateListener::listenerCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Listening to /ego/vehicle_state...");
    }

    ~EgoVelocityStateListener() {
        if (csv_stream_.is_open()) {
            csv_stream_.close();
        }
    }

private:
    void listenerCallback(const ad_msgs::msg::VehicleState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        double x = msg->x;
        double y = msg->y;

        // 처음이거나 10cm 이상의 간격일 때 저장
        if (std::isnan(last_x_) || std::isnan(last_y_) || std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2)) >= 0.1) {
            if (csv_stream_.is_open()) {
                csv_stream_ << x << "," << y << "\n";
                RCLCPP_INFO(this->get_logger(), "Saved position: x=%.3f, y=%.3f", x, y);
            }

            // 마지막 좌표 업데이트
            last_x_ = x;
            last_y_ = y;
        }
    }

    std::ofstream csv_stream_;  // CSV 파일 스트림
    std::string csv_file_;      // CSV 파일 이름
    double last_x_;             // 마지막 x 좌표
    double last_y_;             // 마지막 y 좌표
    std::mutex mutex_;          // 동기화를 위한 mutex

    rclcpp::Subscription<ad_msgs::msg::VehicleState>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EgoVelocityStateListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
