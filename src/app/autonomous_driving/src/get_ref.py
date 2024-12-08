import rclpy
from rclpy.node import Node
from ad_msgs.msg import VehicleState
import csv
import math

class EgoVelocityStateListener(Node):
    def __init__(self):
        super().__init__('ego_velocity_state_listener')

        # 마지막 저장된 좌표
        self.last_x = None
        self.last_y = None

        # CSV 파일 초기화
        self.csv_file = 'ego_position.csv'
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y'])  # 헤더 추가

        # Subscriber 초기화
        self.subscription = self.create_subscription(
            VehicleState,
            '/ego/vehicle_state',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Listening to /ego/vehicle_state...')

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y

        # 처음이거나 10cm 이상의 간격일 때 저장
        if self.last_x is None or self.last_y is None or math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2) >= 0.1:
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([x, y])
            self.get_logger().info(f"Saved position: x={x}, y={y}")

            # 마지막 좌표 업데이트
            self.last_x = x
            self.last_y = y

def main(args=None):
    rclpy.init(args=args)
    node = EgoVelocityStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
