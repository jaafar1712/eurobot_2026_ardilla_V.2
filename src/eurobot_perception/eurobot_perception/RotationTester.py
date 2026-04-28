#!/usr/bin/env python3
"""
Test script to measure actual rotation detection limits
"""
import rclpy
from rclpy.node import Node
from eurobot_interfaces.msg import CrateDetectionArray
import time

class RotationTester(Node):
    def __init__(self):
        super().__init__('rotation_tester')
        
        self.detections = []
        self.sub = self.create_subscription(
            CrateDetectionArray,
            '/crate/detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info("=== ROTATION TEST ===")
        self.get_logger().info("Manually rotate crate and observe detection")
        
    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            det = msg.detections[0]
            self.get_logger().info(
                f"✓ DETECTED: AR={det.aspect_ratio:.2f}, "
                f"Conf={det.confidence:.2f}, "
                f"Corners={det.corner_count}"
            )
        else:
            self.get_logger().warn("✗ NO DETECTION")

def main():
    rclpy.init()
    node = RotationTester()
    
    print("\n" + "="*60)
    print("ROTATION TEST PROCEDURE:")
    print("="*60)
    print("1. Place crate in front of camera (perpendicular)")
    print("2. Slowly rotate crate while watching terminal")
    print("3. Note the angle where detection STOPS")
    print("4. Press Ctrl+C when done")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()