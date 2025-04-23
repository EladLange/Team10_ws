#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DrivePublisher(Node):
    def __init__(self):
        super().__init__('drive_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_steering_controller/cmd', 10)
        self.timer = self.create_timer(0.1, self.send_cmd)

    def send_cmd(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.5  # m/s
        msg.drive.steering_angle = 0.3  # radians
        self.publisher.publish(msg)
        self.get_logger().info(f'Published speed: {msg.drive.speed}, steering: {msg.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = DrivePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
