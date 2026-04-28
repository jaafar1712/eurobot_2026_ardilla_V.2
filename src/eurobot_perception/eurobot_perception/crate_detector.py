#!/usr/bin/env python3
# crate_detection.py
# ROS2 Humble. Python node that subscribes to /camera/image_raw, detects blue crates,
# and publishes a boolean gripper command on /gripper_up (True => raise gripper).

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CrateDetector(Node):
    def __init__(self):
        super().__init__('crate_detector')

        # Parameters (can override on launch)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('min_area', 2000)   # minimum contour area to consider
        self.declare_parameter('team_color', 'blue')  # team color
        self.declare_parameter('visualize', True)  # publish debug image
        self.declare_parameter('debug_topic', '/crate_detection/image')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.team_color = self.get_parameter('team_color').get_parameter_value().string_value.lower()
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.bridge = CvBridge()

        # Subscribers & publishers
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, qos)

        self.gripper_pub = self.create_publisher(Bool, '/gripper_up', 10)
        if self.visualize:
            self.debug_pub = self.create_publisher(Image, self.debug_topic, 1)

        self.get_logger().info('CrateDetector node started')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error('CvBridge error: %s' % str(e))
            return

        # 1) Preprocess
        blurred = cv2.GaussianBlur(frame, (7,7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 2) Color threshold for blue (team color); you can tune ranges
        # Note: two ranges to be safe across HSV wrap
        lower_blue = np.array([95, 80, 40])   # adjust as needed
        upper_blue = np.array([135, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3) Morphology to clean up mask
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # 4) find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_team_color = False
        chosen_contour = None
        largest_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            # Optional: approximate rectangle and aspect ratio checks to ensure crate-like shape
            x,y,w,h = cv2.boundingRect(cnt)
            ar = w / float(h) if h>0 else 0
            # simple area + aspect ratio filter
            if area > largest_area:
                largest_area = area
                chosen_contour = (x,y,w,h)
                # For this implementation, any sufficiently large "blue" blob counts
                found_team_color = True

        # Decide gripper action:
        gripper_msg = Bool()
        if self.team_color == 'blue':
            # if we detect blue crate -> raise gripper (True)
            gripper_msg.data = bool(found_team_color)
        else:
            # For other colors, default logic (you can add red/yellow detection)
            gripper_msg.data = bool(found_team_color)

        # Publish gripper command
        self.gripper_pub.publish(gripper_msg)

        # Publish debug image with overlays (optional)
        if self.visualize:
            debug = frame.copy()
            if chosen_contour is not None:
                x,y,w,h = chosen_contour
                cv2.rectangle(debug, (x,y), (x+w, y+h), (0,255,0), 2)
                cv2.putText(debug, f'Area:{int(largest_area)}', (x,y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(debug, 'TEAM COLOR' if found_team_color else 'OTHER', (x,y+h+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)

            # publish as sensor_msgs/Image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                img_msg.header = msg.header
                self.debug_pub.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error('CvBridge publish error: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CrateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

