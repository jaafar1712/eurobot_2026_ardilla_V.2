#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
                
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        
        self.get_logger().info('Fixing /scan frame_id to lidar_link -> /scan_fixed')
    
    def callback(self, msg):
        msg.header.frame_id = 'lidar_link'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()