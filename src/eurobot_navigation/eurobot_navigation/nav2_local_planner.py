#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Nav2LocalPlanner(Node):
    def __init__(self):
        super().__init__('nav2_local_planner')
        self.get_logger().info("Nav2 Local Planner started - wrapper for controller_server")
        # Wrap controller_server, can remap topics or add logging

def main(args=None):
    rclpy.init(args=args)
    node = Nav2LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()

