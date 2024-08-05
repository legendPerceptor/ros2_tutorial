#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("Hello")
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f"Timer callback triggered with counter: {self.counter}")
        self.counter += 1



def main(args=None):
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()