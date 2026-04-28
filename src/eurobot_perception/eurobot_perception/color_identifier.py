#!/usr/bin/env python3
# color_identifier.py
# ROS2 Humble node that subscribes to crate detection image or bounding boxes,
# identifies the crate color, and publishes it on /crate/color.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ColorIdentifier(Node):
    def __init__(self):
        super().__init__('color_identifier')

        # Parameters
        self.declare_parameter('crate_topic', '/crate_detection/image')
        self.declare_parameter('visualize', True)
        self.declare_parameter('debug_topic', '/crate_color_debug')

        self.crate_topic = self.get_parameter('crate_topic').get_parameter_value().string_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.bridge = CvBridge()

        # Subscriber & publisher
        self.sub = self.create_subscription(Image, self.crate_topic, self.image_cb, qos)
        self.pub = self.create_publisher(String, '/crate/color', 10)
        if self.visualize:
            self.debug_pub = self.create_publisher(Image, self.debug_topic, 1)

        self.get_logger().info(f'Subscribed to: {self.crate_topic}')
        self.get_logger().info('ColorIdentifier node started')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        detected_color = "unknown"

        # For simplicity, take the **largest blue/yellow contour** in the image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for team colors
        color_ranges = {
            'blue': ([95, 80, 40], [135, 255, 255]),
            'yellow': ([20, 100, 100], [35, 255, 255])
        }

        mask_color_area = {}
        for color, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            mask_area = cv2.countNonZero(mask)
            mask_color_area[color] = mask_area

        # Pick the color with largest detected area
        if mask_color_area:
            detected_color = max(mask_color_area, key=mask_color_area.get)
            if mask_color_area[detected_color] < 50:  # small threshold
                detected_color = "unknown"

        # Publish color as string
        color_msg = String()
        color_msg.data = detected_color
        self.pub.publish(color_msg)

        # Optional visualization
        if self.visualize:
            debug_frame = frame.copy()
            cv2.putText(debug_frame, f'Color: {detected_color}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            try:
                img_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                img_msg.header = msg.header
                self.debug_pub.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge publish error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorIdentifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

