#!/usr/bin/env python3
"""
Enhanced Crate Perception Node - NAV2 Compatible
Detects blue and yellow crates with TF2 transforms for navigation
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from eurobot_interfaces.msg import CrateDetection, CrateDetectionArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler

class CratePerception(Node):
    def __init__(self):
        super().__init__('crate_perception')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('visualize', True)
        self.declare_parameter('min_area', 2000)
        self.declare_parameter('known_crate_width', 0.12)  # meters
        self.declare_parameter('focal_length_px', 600.0)   # camera calibration
        self.declare_parameter('camera_center_offset', 0.0)  # optional, meters offset if camera not centered
        
        # NEW: Frame parameters for TF2
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'odom')  # or 'odom' if no SLAM
        
        # NEW: Navigation parameters
        self.declare_parameter('approach_distance', 0.30)  # Stop 30cm before crate
        self.declare_parameter('publish_nav_goals', False)  # Enable goal publishing

        self.camera_topic = self.get_parameter('camera_topic').value
        self.visualize = self.get_parameter('visualize').value
        self.min_area = self.get_parameter('min_area').value
        self.crate_width = self.get_parameter('known_crate_width').value
        self.focal_length = self.get_parameter('focal_length_px').value
        self.camera_offset = self.get_parameter('camera_center_offset').value
        
        # NEW: Frame IDs
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.publish_nav_goals = self.get_parameter('publish_nav_goals').value

        # Color ranges in HSV
        self.color_ranges = {
            'blue': ([95, 80, 40], [135, 255, 255]),
            'yellow': ([20, 100, 100], [35, 255, 255])
        }

        # NEW: TF2 setup for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Topics
        self.sub = self.create_subscription(Image, self.camera_topic, self.image_cb, 10)
        self.pub = self.create_publisher(CrateDetectionArray, '/crate/detections', 10)
        
        # NEW: Navigation goal publisher (optional)
        if self.publish_nav_goals:
            self.goal_pub = self.create_publisher(PoseStamped, '/crate/nav_goal', 10)
            self.get_logger().info("Navigation goal publishing enabled")
        
        if self.visualize:
            self.debug_pub = self.create_publisher(Image, '/crate/image_debug', 1)

        self.get_logger().info(f"Subscribed to: {self.camera_topic}")
        self.get_logger().info(f"Using frames: {self.camera_frame} -> {self.base_frame} -> {self.global_frame}")
        self.get_logger().info("Enhanced CratePerception Node started")

    def transform_to_global_frame(self, x, y, source_frame=None):
        """
        NEW METHOD: Transform point from source frame to global frame (map/odom)
        Uses Time(0) to get the latest available transform, avoiding extrapolation errors
        
        Args:
            x, y: Coordinates in source frame (meters)
            source_frame: Source frame name (defaults to base_frame)
        
        Returns:
            (x_global, y_global) in global frame, or (None, None) if transform fails
        """
        if source_frame is None:
            source_frame = self.base_frame
            
        try:
            # Create point in source frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            # Always use Time(0) for latest available transform (avoids extrapolation)
            point_stamped.header.stamp = rclpy.time.Time().to_msg()
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = 0.0
            
            # Transform to global frame
            timeout = rclpy.duration.Duration(seconds=5.0)
            point_global = self.tf_buffer.transform(
                point_stamped, 
                self.global_frame,
                timeout=timeout
            )
            
            return point_global.point.x, point_global.point.y
            
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}", throttle_duration_sec=2.0)
            return None, None

    def compute_approach_pose(self, crate_x, crate_y, robot_x=0.0, robot_y=0.0):
        """
        NEW METHOD: Compute approach pose (position + orientation) for navigation
        
        Args:
            crate_x, crate_y: Crate position in base_link frame
            robot_x, robot_y: Robot position (default 0,0 in base_link)
        
        Returns:
            (approach_x, approach_y, approach_theta) - pose to reach before grasping
        """
        # Vector from robot to crate
        dx = crate_x - robot_x
        dy = crate_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Approach point: stop at approach_distance before crate
        if distance > self.approach_distance:
            ratio = (distance - self.approach_distance) / distance
            approach_x = robot_x + dx * ratio
            approach_y = robot_y + dy * ratio
        else:
            # Already very close - use current position
            approach_x = robot_x
            approach_y = robot_y
        
        # Orientation: face toward crate
        approach_theta = math.atan2(dy, dx)
        
        return approach_x, approach_y, approach_theta

    def create_nav_goal(self, x, y, theta, frame_id=None):
        """
        NEW METHOD: Create PoseStamped message for navigation goal
        
        Args:
            x, y: Position in meters
            theta: Orientation in radians
            frame_id: Reference frame (defaults to global_frame)
        
        Returns:
            PoseStamped message ready for NAV2
        """
        if frame_id is None:
            frame_id = self.global_frame
            
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        return goal

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = []
        debug = frame.copy()

        frame_center_x = frame.shape[1] / 2  # Image center in pixels
        
        # NEW: Track best crate for goal publishing (closest high-confidence detection)
        best_crate = None
        best_distance = float('inf')

        for color, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w / 2
                cy = y + h / 2

                # ========== PRESERVED: Original distance calculation ========== #
                # Distance estimation (simple pinhole model)
                distance = (self.crate_width * self.focal_length) / w  # meters

                # Horizontal offset from image center (pixels)
                offset_px = cx - frame_center_x

                # Convert pixel offset to radians (approx.)
                angle_rad = math.atan2(offset_px, self.focal_length)

                # Convert to robot-relative coordinates (meters) in base_link frame
                rel_x = distance * math.cos(angle_rad) + self.camera_offset  # forward direction
                rel_y = distance * math.sin(angle_rad)                      # lateral direction

                # Confidence (relative to area)
                confidence = min(1.0, area / (frame.shape[0]*frame.shape[1]*0.1))
                # ========== END PRESERVED SECTION ========== #

                # NEW: Transform to global frame for navigation
                # Uses Time(0) for latest available transform
                global_x, global_y = self.transform_to_global_frame(rel_x, rel_y)
                
                # Fill message
                det = CrateDetection()
                det.color = color
                det.x = float(rel_x)        # PRESERVED: Robot-relative x
                det.y = float(rel_y)        # PRESERVED: Robot-relative y
                det.distance = float(distance)
                det.angle = float(math.degrees(angle_rad))
                det.confidence = float(confidence)
                
                # NEW: Add global coordinates to detection (if transform succeeded)
                # Note: This requires extending the CrateDetection message definition
                # to include global_x, global_y fields. For now, we track separately.
                if global_x is not None and global_y is not None:
                    det.x = float(global_x)  # MODIFIED: Use global coordinates
                    det.y = float(global_y)  # MODIFIED: Use global coordinates
                
                detections.append(det)
                
                # NEW: Track best crate for goal publishing
                if confidence > 0.5 and distance < best_distance:
                    best_crate = {
                        'color': color,
                        'x': rel_x,  # base_link frame
                        'y': rel_y,
                        'global_x': global_x,  # map/odom frame
                        'global_y': global_y,
                        'distance': distance,
                        'confidence': confidence
                    }
                    best_distance = distance

                if self.visualize:
                    cv2.rectangle(debug, (x, y), (x+w, y+h), (0,255,0), 2)
                    
                    # MODIFIED: Enhanced visualization with frame info
                    text1 = f"{color} {distance:.2f}m"
                    text2 = f"Base: ({rel_x:.2f},{rel_y:.2f})"
                    text3 = f"Conf: {confidence:.2f}"
                    
                    cv2.putText(debug, text1, (x, y-30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    cv2.putText(debug, text2, (x, y-15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
                    cv2.putText(debug, text3, (x, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
                    
                    if global_x is not None and global_y is not None:
                        text4 = f"Global: ({global_x:.2f},{global_y:.2f})"
                        cv2.putText(debug, text4, (x, y+h+15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)

        # NEW: Publish navigation goal for best crate (if enabled)
        if self.publish_nav_goals and best_crate is not None:
            if best_crate['global_x'] is not None:
                # Compute approach pose in base_link
                app_x, app_y, app_theta = self.compute_approach_pose(
                    best_crate['x'], 
                    best_crate['y']
                )
                
                # Transform approach pose to global frame
                global_app_x, global_app_y = self.transform_to_global_frame(app_x, app_y)
                
                if global_app_x is not None:
                    # Get robot's current orientation in global frame for theta adjustment
                    # (Simplified: use angle to crate as approach orientation)
                    global_theta = math.atan2(
                        best_crate['global_y'] - global_app_y,
                        best_crate['global_x'] - global_app_x
                    )
                    
                    # Create and publish goal
                    nav_goal = self.create_nav_goal(
                        global_app_x, 
                        global_app_y, 
                        global_theta
                    )
                    self.goal_pub.publish(nav_goal)
                    
                    if self.visualize:
                        cv2.putText(debug, 
                                    f"Goal published: {best_crate['color']} at ({global_app_x:.2f}, {global_app_y:.2f})",
                                    (10, frame.shape[0] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        # PRESERVED: Publish detections
        if detections:
            msg_out = CrateDetectionArray()
            msg_out.detections = detections
            msg_out.header = msg.header
            msg_out.header.frame_id = self.global_frame  # MODIFIED: Set proper frame
            self.pub.publish(msg_out)

        # PRESERVED: Publish visualization
        if self.visualize:
            # NEW: Add frame info overlay
            cv2.putText(debug, f"Frame: {self.global_frame}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(debug, f"Detections: {len(detections)}", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            
            img_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            img_msg.header = msg.header
            self.debug_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CratePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()