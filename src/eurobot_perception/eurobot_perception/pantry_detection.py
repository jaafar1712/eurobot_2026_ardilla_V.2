#!/usr/bin/env python3
"""
Enhanced Pantry Detection Node - NAV2 Compatible
Detects pantries by green rectangle outlines with global frame tracking
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from eurobot_interfaces.msg import PantryDetection, PantryDetectionArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler

class PantryPerception(Node):
    def __init__(self):
        super().__init__('pantry_perception')
        self.bridge = CvBridge()

        # Parameters - PRESERVED from original
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('visualize', True)
        self.declare_parameter('min_contour_area', 200)
        self.declare_parameter('min_rect_area', 1000)
        self.declare_parameter('known_pantry_width', 0.20)
        self.declare_parameter('focal_length_px', 600.0)
        self.declare_parameter('camera_center_offset', 0.0)
        self.declare_parameter('home_detection_timeout', 5.0)
        
        # NEW: Frame parameters for TF2
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')  # or 'odom' if no SLAM
        
        # NEW: Navigation parameters
        self.declare_parameter('publish_nav_goals', False)  # Enable goal publishing
        self.declare_parameter('pantry_approach_distance', 0.50)  # Stop 50cm from pantry

        # PRESERVED: Original parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.visualize = self.get_parameter('visualize').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.min_rect_area = self.get_parameter('min_rect_area').value
        self.pantry_width = self.get_parameter('known_pantry_width').value
        self.focal_length = self.get_parameter('focal_length_px').value
        self.camera_offset = self.get_parameter('camera_center_offset').value
        self.home_timeout = self.get_parameter('home_detection_timeout').value
        
        # NEW: Frame IDs
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.publish_nav_goals = self.get_parameter('publish_nav_goals').value
        self.approach_distance = self.get_parameter('pantry_approach_distance').value

        # PRESERVED: Home position tracking
        self.home_position = None  # Will store in global frame now
        self.home_captured = False
        self.start_time = self.get_clock().now()

        # PRESERVED: Green color range for pantry markers
        self.green_lower = np.array([35, 60, 60])
        self.green_upper = np.array([85, 255, 255])
        
        # PRESERVED: Confidence and filtering parameters
        self.declare_parameter('min_detection_confidence', 0.3)
        self.declare_parameter('clustering_distance_ratio', 0.35)
        self.min_confidence = self.get_parameter('min_detection_confidence').value
        self.cluster_ratio = self.get_parameter('clustering_distance_ratio').value

        # NEW: TF2 setup for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Topics - PRESERVED
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, 10
        )
        self.img_sub = self.create_subscription(
            Image, self.camera_topic, self.image_cb, 10
        )
        self.pub = self.create_publisher(PantryDetectionArray, '/pantry/detections', 10)
        
        # NEW: Navigation goal publisher (optional)
        if self.publish_nav_goals:
            self.goal_pub = self.create_publisher(PoseStamped, '/pantry/nav_goal', 10)
            self.get_logger().info("Navigation goal publishing enabled")
        
        if self.visualize:
            self.debug_pub = self.create_publisher(Image, '/pantry/image_debug', 1)

        # PRESERVED: Persistent pantry tracking
        # MODIFIED: Now stores global coordinates instead of robot-relative
        self.pantry_id_counter = 0
        self.detected_pantries = []  # List of {id, global_x, global_y}
        self.last_detection_time = {}
        
        self.get_logger().info(f"Subscribed to camera: {self.camera_topic}")
        self.get_logger().info(f"Subscribed to odom: {self.odom_topic}")
        self.get_logger().info(f"Using frames: {self.camera_frame} -> {self.base_frame} -> {self.global_frame}")
        self.get_logger().info("Waiting to capture home position...")

    def get_robot_pose_in_global_frame(self):
        """
        NEW METHOD: Get current robot pose in global frame via TF
        
        Returns:
            (x, y, theta) in global frame, or (None, None, None) if transform fails
        """
        try:
            # Get transform from base_link to global frame
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            quat = transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, theta
            
        except Exception as e:
            self.get_logger().warn(f"Could not get robot pose: {e}", throttle_duration_sec=2.0)
            return None, None, None

    def odom_cb(self, msg):
        """
        MODIFIED: Capture home position in global frame using TF
        """
        if not self.home_captured:
            # NEW: Get robot pose in global frame via TF instead of raw odometry
            home_x, home_y, home_theta = self.get_robot_pose_in_global_frame()
            
            if home_x is not None:
                self.home_position = {
                    'x': home_x,
                    'y': home_y,
                    'theta': home_theta,
                    'frame_id': self.global_frame  # NEW: Track which frame
                }
                self.home_captured = True
                
                self.get_logger().info(
                    f"Home position captured in {self.global_frame}: "
                    f"x={self.home_position['x']:.2f}, y={self.home_position['y']:.2f}, "
                    f"theta={math.degrees(self.home_position['theta']):.1f}°"
                )
            else:
                # Fallback to odometry if TF not available yet
                self.home_position = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'theta': 0.0,  # Could extract from quaternion if needed
                    'frame_id': self.odom_topic.split('/')[-1]  # 'odom' usually
                }
                self.home_captured = True
                
                self.get_logger().warn(
                    f"Home position captured from odometry (TF unavailable): "
                    f"x={self.home_position['x']:.2f}, y={self.home_position['y']:.2f}"
                )

    def transform_to_global_frame(self, x, y, source_frame=None):
        """
        NEW METHOD: Transform point from source frame to global frame
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
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            # Always use Time(0) for latest available transform (avoids extrapolation)
            point_stamped.header.stamp = rclpy.time.Time().to_msg()
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = 0.0
            
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

    def find_or_create_pantry_id(self, global_x, global_y, distance):
        """
        MODIFIED: Find existing pantry ID or create new one using GLOBAL coordinates
        
        Args:
            global_x, global_y: Pantry position in global frame
            distance: Distance from robot (for filtering)
        
        Returns:
            (id, is_new) tuple
        """
        if global_x is None or global_y is None:
            return None, False
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        match_threshold = 0.7  # 70cm matching threshold in global frame
        
        # Check if this matches an existing pantry
        best_match = None
        best_dist = float('inf')
        
        for pantry in self.detected_pantries:
            dx = pantry['global_x'] - global_x
            dy = pantry['global_y'] - global_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < match_threshold and dist < best_dist:
                best_match = pantry
                best_dist = dist
        
        if best_match is not None:
            # Update the position with weighted average (favor stability)
            weight = 0.3  # 30% new, 70% old position
            best_match['global_x'] = best_match['global_x'] * (1 - weight) + global_x * weight
            best_match['global_y'] = best_match['global_y'] * (1 - weight) + global_y * weight
            self.last_detection_time[best_match['id']] = current_time
            return best_match['id'], False
        
        # New pantry - create ID
        if distance > 0.3:  # Only add if not too close to robot
            new_id = self.pantry_id_counter
            self.pantry_id_counter += 1
            
            self.detected_pantries.append({
                'id': new_id,
                'global_x': global_x,  # MODIFIED: Store global coordinates
                'global_y': global_y
            })
            self.last_detection_time[new_id] = current_time
            
            self.get_logger().info(
                f"✓ New pantry ID={new_id} at global ({global_x:.2f}, {global_y:.2f}) - "
                f"distance: {distance:.2f}m"
            )
            return new_id, True
        
        return None, False

    # PRESERVED: group_nearby_rectangles and merge_rectangles methods unchanged
    def group_nearby_rectangles(self, rectangles, img_shape):
        """
        PRESERVED: Group rectangles that are close together (belong to same pantry)
        """
        if not rectangles:
            return []
        
        cluster_threshold = img_shape[1] * self.cluster_ratio
        grouped = []
        used = set()
        
        for i, rect1 in enumerate(rectangles):
            if i in used:
                continue
            
            group = [rect1]
            used.add(i)
            
            adaptive_threshold = max(cluster_threshold, 
                                    (rect1['w'] + rect1['h']) * 0.8)
            
            for j, rect2 in enumerate(rectangles):
                if j in used or i == j:
                    continue
                
                dx = rect1['cx'] - rect2['cx']
                dy = rect1['cy'] - rect2['cy']
                dist = math.sqrt(dx*dx + dy*dy)
                
                y_diff = abs(rect1['cy'] - rect2['cy'])
                are_aligned = y_diff < img_shape[0] * 0.15
                
                if dist < adaptive_threshold or (are_aligned and dist < cluster_threshold * 1.5):
                    group.append(rect2)
                    used.add(j)
            
            if len(group) > 1:
                merged = self.merge_rectangles(group)
                grouped.append(merged)
            else:
                grouped.append(rect1)
        
        return grouped
    
    def merge_rectangles(self, rectangles):
        """PRESERVED: Merge multiple rectangles into one by finding bounding box"""
        min_x = min(r['x'] for r in rectangles)
        min_y = min(r['y'] for r in rectangles)
        max_x = max(r['x'] + r['w'] for r in rectangles)
        max_y = max(r['y'] + r['h'] for r in rectangles)
        
        w = max_x - min_x
        h = max_y - min_y
        
        return {
            'x': min_x,
            'y': min_y,
            'w': w,
            'h': h,
            'cx': min_x + w / 2,
            'cy': min_y + h / 2,
            'area': w * h,
            'contour': None
        }

    def detect_green_rectangles(self, frame, hsv):
        """PRESERVED: Detect green rectangular outlines on the floor and group them"""
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        
        kernel_large = np.ones((7, 7), np.uint8)
        kernel_small = np.ones((3, 3), np.uint8)
        
        mask = cv2.dilate(mask, kernel_large, iterations=4)
        mask = cv2.erode(mask, kernel_small, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        raw_rectangles = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            rect_area = w * h
            
            if rect_area < self.min_rect_area:
                continue
            
            raw_rectangles.append({
                'x': x,
                'y': y,
                'w': w,
                'h': h,
                'cx': x + w / 2,
                'cy': y + h / 2,
                'area': rect_area,
                'contour': cnt
            })
        
        rectangles = self.group_nearby_rectangles(raw_rectangles, frame.shape)
        
        return rectangles, mask

    def create_nav_goal(self, x, y, theta, frame_id=None):
        """
        NEW METHOD: Create PoseStamped message for navigation goal
        """
        if frame_id is None:
            frame_id = self.global_frame
            
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
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
        frame_center_x = frame.shape[1] / 2
        
        # NEW: Track nearest pantry for goal publishing
        nearest_pantry = None
        nearest_distance = float('inf')

        # PRESERVED: Always include home pantry if captured
        if self.home_captured and self.home_position is not None:
            home_det = PantryDetection()
            home_det.id = -1
            home_det.x = float(self.home_position['x'])
            home_det.y = float(self.home_position['y'])
            home_det.distance = 0.0
            home_det.angle = 0.0
            home_det.confidence = 1.0
            detections.append(home_det)

        # PRESERVED: Detect green rectangles
        rectangles, mask = self.detect_green_rectangles(frame, hsv)

        for rect in rectangles:
            x, y, w, h = rect['x'], rect['y'], rect['w'], rect['h']
            cx, cy = rect['cx'], rect['cy']

            # ========== PRESERVED: Original distance calculation ========== #
            distance = (self.pantry_width * self.focal_length) / w if w > 0 else 0
            offset_px = cx - frame_center_x
            angle_rad = math.atan2(offset_px, self.focal_length)
            
            # Robot-relative coordinates in base_link frame
            rel_x = distance * math.cos(angle_rad) + self.camera_offset
            rel_y = distance * math.sin(angle_rad)
            
            confidence = min(1.0, rect['area'] / (frame.shape[0] * frame.shape[1] * 0.1))
            # ========== END PRESERVED SECTION ========== #

            # Skip low confidence detections
            if confidence < self.min_confidence:
                continue

            # NEW: Transform to global frame
            # Uses Time(0) for latest available transform
            global_x, global_y = self.transform_to_global_frame(rel_x, rel_y)
            
            # Find or create pantry ID using global coordinates
            pantry_id, is_new = self.find_or_create_pantry_id(global_x, global_y, distance)
            
            if pantry_id is None:
                continue

            # Fill message
            det = PantryDetection()
            det.id = pantry_id
            
            # MODIFIED: Use global coordinates if available, fallback to relative
            if global_x is not None and global_y is not None:
                det.x = float(global_x)
                det.y = float(global_y)
            else:
                det.x = float(rel_x)
                det.y = float(rel_y)
            
            det.distance = float(distance)
            det.angle = float(math.degrees(angle_rad))
            det.confidence = float(confidence)
            detections.append(det)
            
            # NEW: Track nearest pantry
            if global_x is not None and distance < nearest_distance:
                nearest_pantry = {
                    'id': pantry_id,
                    'global_x': global_x,
                    'global_y': global_y,
                    'distance': distance
                }
                nearest_distance = distance

            if self.visualize:
                cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 255), 3)
                cv2.circle(debug, (int(cx), int(cy)), 8, (255, 0, 255), -1)
                
                text1 = f"Pantry ID={pantry_id}: {distance:.2f}m"
                text2 = f"Base: ({rel_x:.2f}, {rel_y:.2f})"
                
                (w1, h1), _ = cv2.getTextSize(text1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(debug, (x, y-h1-25), (x+w1+5, y-5), (0, 0, 0), -1)
                
                cv2.putText(debug, text1, (x, y-15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(debug, text2, (x, y+h+20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
                if global_x is not None and global_y is not None:
                    text3 = f"Global: ({global_x:.2f}, {global_y:.2f})"
                    cv2.putText(debug, text3, (x, y+h+35), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # NEW: Publish navigation goal to nearest pantry (if enabled)
        if self.publish_nav_goals and nearest_pantry is not None:
            # Get robot's current pose for approach calculation
            robot_x, robot_y, robot_theta = self.get_robot_pose_in_global_frame()
            
            if robot_x is not None:
                # Compute approach pose (stop before pantry boundary)
                dx = nearest_pantry['global_x'] - robot_x
                dy = nearest_pantry['global_y'] - robot_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist > self.approach_distance:
                    # Approach point
                    ratio = (dist - self.approach_distance) / dist
                    approach_x = robot_x + dx * ratio
                    approach_y = robot_y + dy * ratio
                else:
                    approach_x = robot_x
                    approach_y = robot_y
                
                # Face toward pantry
                approach_theta = math.atan2(dy, dx)
                
                nav_goal = self.create_nav_goal(approach_x, approach_y, approach_theta)
                self.goal_pub.publish(nav_goal)

        # PRESERVED: Add home indicator and stats
        if self.visualize:
            if self.home_captured:
                cv2.putText(debug, 
                            f"HOME ({self.global_frame}): ({self.home_position['x']:.2f}, {self.home_position['y']:.2f})",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug, f"Unique Pantries: {len(self.detected_pantries)}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug, f"Current frame: {len(rectangles)} detections",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # PRESERVED: Publish detections
        if detections:
            msg_out = PantryDetectionArray()
            msg_out.detections = detections
            msg_out.header = msg.header
            msg_out.header.frame_id = self.global_frame  # MODIFIED: Set proper frame
            self.pub.publish(msg_out)

        # PRESERVED: Publish visualization
        if self.visualize:
            mask_small = cv2.resize(mask, (160, 120))
            mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
            debug[0:120, 0:160] = mask_bgr
            
            img_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            img_msg.header = msg.header
            self.debug_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PantryPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()