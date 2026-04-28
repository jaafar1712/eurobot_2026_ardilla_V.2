#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.get_logger().info("SLAM Node started - wrapper for slam_toolbox")
        # You can add parameters if needed
        # Parameters can include scan topic, map frame, mode, etc.

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
