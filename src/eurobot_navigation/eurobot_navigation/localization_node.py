#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.get_logger().info("Localization Node started - wrapper for AMCL")
        # You can configure map frame, odom frame, scan topic, etc. via params

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()

