#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.msg import String

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.string_publisher = self.create_publisher(String, "robot_news", 10)
        self.string_timer_ = self.create_timer(1, self.publish_news)

        self.cmd_vel_pub_ = self.create_publisher(msg_type=Twist, topic="/turtle1/cmd_vel", qos_profile=10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def publish_news(self):
        msg = String()
        msg.data = "Hello, robot news"
        self.string_publisher.publish(msg)

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()