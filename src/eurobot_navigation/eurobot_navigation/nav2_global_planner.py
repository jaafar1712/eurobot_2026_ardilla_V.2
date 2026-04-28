#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Nav2GlobalPlanner(Node):
    def __init__(self):
        super().__init__('nav2_global_planner')
        self.get_logger().info("Nav2 Global Planner started - wrapper for planner_server")
        # Wrap planner_server, can remap topics or add logging

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()

