#!/usr/bin/env python3
"""
Enhanced Crate Perception Node - Eurobot 2026 "Winter is Coming"
Rotation-robust detection with orientation estimation for grasp planning
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


class EnhancedCratePerception(Node):
    """
    Enhanced crate perception with rotation robustness and orientation estimation
    """
    
    def __init__(self):
        super().__init__('enhanced_crate_perception')
        self.bridge = CvBridge()

        # ==================== PARAMETERS ====================
        
        # Camera parameters
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('visualize', True)
        
        # CORRECTED: Eurobot 2026 crate dimensions (150mm x 50mm x 30mm)
        self.declare_parameter('crate_length', 0.15)   # 150mm - longest dimension
        self.declare_parameter('crate_width', 0.05)    # 50mm - medium dimension
        self.declare_parameter('crate_height', 0.03)   # 30mm - shortest dimension
        
        # Camera calibration
        self.declare_parameter('focal_length_px', 600.0)
        self.declare_parameter('camera_center_offset', 0.0)
        
        # Detection thresholds
        self.declare_parameter('min_area', 1500)               # Minimum contour area (pixels²) 2000
        self.declare_parameter('min_confidence', 0.5)          # Minimum confidence to publish
        self.declare_parameter('aspect_ratio_min', 1.5)        # 150/50 = 3.0, allow ±0.5 , 2.5
        self.declare_parameter('aspect_ratio_max', 4.5)          #3.5
        
        # Frame parameters for TF2
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'odom')
        
        # Navigation parameters
        self.declare_parameter('approach_distance', 0.30)      # Stop 30cm before crate
        self.declare_parameter('grasp_distance', 0.15)         # Final grasp distance
        self.declare_parameter('publish_nav_goals', True)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.visualize = self.get_parameter('visualize').value
        self.crate_length = self.get_parameter('crate_length').value
        self.crate_width = self.get_parameter('crate_width').value
        self.crate_height = self.get_parameter('crate_height').value
        self.focal_length = self.get_parameter('focal_length_px').value
        self.camera_offset = self.get_parameter('camera_center_offset').value
        self.min_area = self.get_parameter('min_area').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.aspect_ratio_min = self.get_parameter('aspect_ratio_min').value
        self.aspect_ratio_max = self.get_parameter('aspect_ratio_max').value
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.grasp_distance = self.get_parameter('grasp_distance').value
        self.publish_nav_goals = self.get_parameter('publish_nav_goals').value

        # Color ranges in HSV (blue and yellow only - no black/rotten crates)
        self.color_ranges = {
            'blue': ([95, 80, 40], [135, 255, 255]),
            'yellow': ([20, 100, 100], [35, 255, 255])
        }

        # TF2 setup for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and Subscribers
        self.sub = self.create_subscription(
            Image, 
            self.camera_topic, 
            self.image_callback, 
            10
        )
        self.pub = self.create_publisher(
            CrateDetectionArray, 
            '/crate/detections', 
            10
        )
        
        if self.publish_nav_goals:
            self.goal_pub = self.create_publisher(
                PoseStamped, 
                '/crate/nav_goal', 
                10
            )
            self.get_logger().info("Navigation goal publishing enabled")
        
        if self.visualize:
            self.debug_pub = self.create_publisher(
                Image, 
                '/crate/image_debug', 
                1
            )

        self.get_logger().info(f"Enhanced Crate Perception Node started")
        self.get_logger().info(f"Crate dimensions: {self.crate_length*1000:.0f}mm x {self.crate_width*1000:.0f}mm x {self.crate_height*1000:.0f}mm")
        self.get_logger().info(f"Frames: {self.camera_frame} -> {self.base_frame} -> {self.global_frame}")

    # ==================== HELPER FUNCTIONS ====================

    def angle_diff(self, angle1, angle2):
        """
        Compute smallest difference between two angles
        Returns value in range [-π, π]
        """
        diff = angle1 - angle2
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def transform_to_global_frame(self, x, y, source_frame=None):
        """
        Transform point from source frame to global frame (map/odom)
        
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
            point_stamped.header.stamp = rclpy.time.Time().to_msg()
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = 0.0
            
            timeout = rclpy.duration.Duration(seconds=1.0)
            point_global = self.tf_buffer.transform(
                point_stamped, 
                self.global_frame,
                timeout=timeout
            )
            
            return point_global.point.x, point_global.point.y
            
        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed: {e}", 
                throttle_duration_sec=2.0
            )
            return None, None

    # ==================== DETECTION FUNCTIONS ====================

    def validate_crate_shape(self, contour):
        """
        Verify contour is rectangular with 4 corners and ~90° angles
        
        Returns:
            (is_valid, corner_count)
        """
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        corner_count = len(approx)
        
        # Must have 4 corners for rectangle
        if corner_count != 4:
            return False, corner_count
        
        # Check angles are approximately 90°
        angles = []
        for i in range(4):
            p1 = approx[i][0]
            p2 = approx[(i+1)%4][0]
            p3 = approx[(i+2)%4][0]
            
            v1 = p1 - p2
            v2 = p3 - p2
            
            # Avoid division by zero
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            if norm1 == 0 or norm2 == 0:
                return False, corner_count
            
            cos_angle = np.dot(v1, v2) / (norm1 * norm2)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.degrees(np.arccos(cos_angle))
            angles.append(angle)
        
        # All angles should be 70-110° (allow perspective distortion)
        is_valid = all(55 < a < 125 for a in angles)      #was 70-110
        
        return is_valid, corner_count

    def estimate_crate_orientation(self, contour, frame_center_x, crate_cx, crate_cy):
        """
        Estimate crate's orientation and recommended approach angle
        
        Returns:
            (crate_yaw, approach_yaw)
            - crate_yaw: orientation of crate's long axis in image plane (radians)
            - approach_yaw: recommended approach direction (perpendicular to long face)
        """
        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h), angle = rect
        
        # Ensure w is the longer dimension (150mm face)
        if h > w:
            w, h = h, w
            angle = (angle + 90) % 180
        
        # Convert angle to radians (OpenCV uses degrees)
        # Normalize to [-π, π]
        crate_yaw = math.radians(angle)
        if crate_yaw > math.pi:
            crate_yaw -= 2 * math.pi
        
        # Compute two possible approach angles (perpendicular to long axis)
        approach_option_1 = crate_yaw + math.pi/2
        approach_option_2 = crate_yaw - math.pi/2
        
        # Choose the approach angle that points toward the robot
        # Robot is typically below the crate in image coordinates
        robot_angle = math.atan2(frame_center_x - crate_cx, crate_cy - (self.image_height / 2))
        
        diff1 = abs(self.angle_diff(approach_option_1, robot_angle))
        diff2 = abs(self.angle_diff(approach_option_2, robot_angle))
        
        approach_yaw = approach_option_1 if diff1 < diff2 else approach_option_2
        
        # Normalize to [-π, π]
        while approach_yaw > math.pi:
            approach_yaw -= 2 * math.pi
        while approach_yaw < -math.pi:
            approach_yaw += 2 * math.pi
        
        return crate_yaw, approach_yaw

    def compute_confidence(self, area, aspect_ratio, corner_count, image_shape):
        """
        Multi-factor confidence score for detection quality
        
        Returns:
            confidence: float in [0.0, 1.0]
        """
        # Area confidence (larger = better, up to a point)
        max_expected_area = image_shape[0] * image_shape[1] * 0.15
        area_score = min(1.0, area / max_expected_area)
        
        # Aspect ratio confidence (closer to 3.0 = better)
        ideal_aspect = 3.0  # 150mm / 50mm
        aspect_error = abs(aspect_ratio - ideal_aspect) / ideal_aspect
        aspect_score = max(0.0, 1.0 - aspect_error)
        
        # Shape confidence (4 corners = perfect)
        shape_score = 1.0 if corner_count == 4 else 0.5
        
        # Combined confidence (weighted average)
        confidence = (
            area_score * 0.3 +
            aspect_score * 0.5 +
            shape_score * 0.2
        )
        
        return confidence

    def compute_grasp_score(self, distance, confidence, angle_to_camera):
        """
        Combined score for selecting best grasp candidate
        Balances distance (closer = better) with detection quality
        
        Returns:
            grasp_score: float in [0.0, 1.0]
        """
        # Distance score (closer is better, but not too close)
        ideal_distance = 0.50  # 50cm is ideal
        distance_error = abs(distance - ideal_distance) / ideal_distance
        distance_score = max(0.0, 1.0 - distance_error)
        
        # Angle score (centered in view = better)
        angle_score = max(0.0, 1.0 - abs(angle_to_camera) / 45.0)  # 45° = 0 score
        
        # Combined score
        grasp_score = (
            distance_score * 0.4 +
            confidence * 0.4 +
            angle_score * 0.2
        )
        
        return grasp_score

    # ==================== MAIN DETECTION PIPELINE ====================

    def detect_crates_with_rotation(self, frame, color, lower, upper):
        """
        Rotation-robust crate detection pipeline
        
        Returns:
            detections: list of CrateDetection messages
        """
        detections = []
        
        # Convert to HSV and apply color mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        
        # Morphological operations to clean up mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        frame_center_x = frame.shape[1] / 2
        frame_center_y = frame.shape[0] / 2
        self.image_height = frame.shape[0]
        self.image_width = frame.shape[1]
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filter by minimum area
            if area < self.min_area:
                continue
            
            # Validate shape (4 corners, ~90° angles)
            is_valid_shape, corner_count = self.validate_crate_shape(cnt)
            if not is_valid_shape:
                continue
            
            # Get rotated bounding rectangle
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w, h), angle = rect
            
            # Ensure w is the longer dimension
            if h > w:
                w, h = h, w
                angle = (angle + 90) % 180
            
            # Validate aspect ratio (150mm : 50mm ≈ 3:1)
            aspect_ratio = w / h if h > 0 else 0
            if not (self.aspect_ratio_min < aspect_ratio < self.aspect_ratio_max):
                continue
            
            # === DISTANCE ESTIMATION (using rotated width) ===
            # Use the LONGER dimension (150mm) for accurate distance
            distance = (self.crate_length * self.focal_length) / w
            
            # Horizontal offset from image center (pixels)
            offset_px = cx - frame_center_x
            
            # Convert pixel offset to angle (radians)
            angle_rad = math.atan2(offset_px, self.focal_length)
            
            # Convert to robot-relative coordinates (base_link frame)
            rel_x = distance * math.cos(angle_rad) + self.camera_offset
            rel_y = distance * math.sin(angle_rad)
            
            # === ORIENTATION ESTIMATION ===
            crate_yaw, approach_yaw = self.estimate_crate_orientation(
                cnt, frame_center_x, cx, cy
            )
            
            # === CONFIDENCE SCORING ===
            confidence = self.compute_confidence(
                area, aspect_ratio, corner_count, frame.shape
            )
            
            # Filter by minimum confidence threshold
            if confidence < self.min_confidence:
                continue
            
            # === GRASP SCORE (for selecting best target) ===
            grasp_score = self.compute_grasp_score(
                distance, confidence, math.degrees(angle_rad)
            )
            
            # === TRANSFORM TO GLOBAL FRAME ===
            global_x, global_y = self.transform_to_global_frame(rel_x, rel_y)
            
            if global_x is None or global_y is None:
                # TF failed, use base_link coordinates
                global_x, global_y = rel_x, rel_y
            
            # === CREATE DETECTION MESSAGE ===
            det = CrateDetection()
            det.color = color
            det.confidence = float(confidence)
            
            # Position
            det.x = float(global_x)
            det.y = float(global_y)
            det.distance = float(distance)
            det.angle = float(math.degrees(angle_rad))
            
            # Orientation
            det.crate_yaw = float(crate_yaw)
            det.approach_yaw = float(approach_yaw)
            det.grasp_score = float(grasp_score)
            
            # Geometric validation
            det.corner_count = int(corner_count)
            det.aspect_ratio = float(aspect_ratio)
            
            # Image coordinates (for visualization/debugging)
            det.pixel_x = float(cx)
            det.pixel_y = float(cy)
            det.bbox_width = float(w)
            det.bbox_height = float(h)
            
            detections.append((det, rect, cnt))
        
        return detections

    # ==================== VISUALIZATION ====================

    def draw_detection(self, debug_frame, det, rect, cnt):
        """
        Draw detection visualization on debug frame
        """
        # Get rotated box points
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        # Color based on detection color
        if det.color == 'blue':
            vis_color = (255, 0, 0)  # BGR
        elif det.color == 'yellow':
            vis_color = (0, 255, 255)
        else:
            vis_color = (128, 128, 128)
        
        # Draw rotated bounding box
        cv2.drawContours(debug_frame, [box], 0, vis_color, 2)
        
        # Draw center point
        cx, cy = int(det.pixel_x), int(det.pixel_y)
        cv2.circle(debug_frame, (cx, cy), 5, vis_color, -1)
        
        # Draw orientation arrow (crate's long axis)
        arrow_len = 50
        arrow_end_x = int(cx + arrow_len * math.cos(det.crate_yaw))
        arrow_end_y = int(cy + arrow_len * math.sin(det.crate_yaw))
        cv2.arrowedLine(
            debug_frame, 
            (cx, cy), 
            (arrow_end_x, arrow_end_y),
            (0, 255, 0), 2, tipLength=0.3
        )
        
        # Draw approach direction arrow (perpendicular)
        approach_end_x = int(cx + arrow_len * math.cos(det.approach_yaw))
        approach_end_y = int(cy + arrow_len * math.sin(det.approach_yaw))
        cv2.arrowedLine(
            debug_frame,
            (cx, cy),
            (approach_end_x, approach_end_y),
            (255, 0, 255), 2, tipLength=0.3
        )
        
        # Text overlay
        text_y = cy - 40
        cv2.putText(
            debug_frame,
            f"{det.color} {det.distance:.2f}m",
            (cx - 40, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, vis_color, 2
        )
        
        text_y += 15
        cv2.putText(
            debug_frame,
            f"Conf: {det.confidence:.2f}",
            (cx - 40, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, vis_color, 1
        )
        
        text_y += 15
        cv2.putText(
            debug_frame,
            f"Grasp: {det.grasp_score:.2f}",
            (cx - 40, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1
        )
        
        text_y += 15
        cv2.putText(
            debug_frame,
            f"AR: {det.aspect_ratio:.2f}",
            (cx - 40, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, vis_color, 1
        )

    # ==================== NAVIGATION GOAL ====================

    def compute_approach_pose(self, crate_x, crate_y, approach_yaw):
        """
        Compute approach pose for navigation
        
        Returns:
            (approach_x, approach_y, approach_theta)
        """
        # Position 30cm away along approach direction
        approach_x = crate_x - self.approach_distance * math.cos(approach_yaw)
        approach_y = crate_y - self.approach_distance * math.sin(approach_yaw)
        
        # Face toward crate (approach direction)
        approach_theta = approach_yaw
        
        return approach_x, approach_y, approach_theta

    def create_nav_goal(self, x, y, theta, frame_id=None):
        """
        Create PoseStamped message for navigation goal
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

    # ==================== MAIN CALLBACK ====================

    def image_callback(self, msg):
        """
        Main image processing callback
        """
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug = frame.copy() if self.visualize else None
        
        all_detections = []
        best_grasp_candidate = None
        best_grasp_score = -1.0
        
        # Detect crates for each color
        for color, (lower, upper) in self.color_ranges.items():
            detections = self.detect_crates_with_rotation(frame, color, lower, upper)
            
            for det, rect, cnt in detections:
                all_detections.append(det)
                
                # Track best candidate for navigation
                if det.grasp_score > best_grasp_score:
                    best_grasp_score = det.grasp_score
                    best_grasp_candidate = det
                
                # Visualize
                if self.visualize:
                    self.draw_detection(debug, det, rect, cnt)
        
        # Publish detections
        if all_detections:
            msg_out = CrateDetectionArray()
            msg_out.header = msg.header
            msg_out.header.frame_id = self.global_frame
            msg_out.detections = all_detections
            msg_out.total_detections = len(all_detections)
            self.pub.publish(msg_out)
            
            self.get_logger().info(
                f"Detected {len(all_detections)} crate(s)",
                throttle_duration_sec=1.0
            )
        
        # Publish navigation goal for best candidate
        if self.publish_nav_goals and best_grasp_candidate is not None:
            # Only publish if high confidence
            if best_grasp_candidate.grasp_score > 0.6:
                app_x, app_y, app_theta = self.compute_approach_pose(
                    best_grasp_candidate.x,
                    best_grasp_candidate.y,
                    best_grasp_candidate.approach_yaw
                )
                
                nav_goal = self.create_nav_goal(app_x, app_y, app_theta)
                self.goal_pub.publish(nav_goal)
                
                if self.visualize and debug is not None:
                    cv2.putText(
                        debug,
                        f"Target: {best_grasp_candidate.color} (score: {best_grasp_score:.2f})",
                        (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                    )
        
        # Publish visualization
        if self.visualize and debug is not None:
            # Add info overlay
            cv2.putText(
                debug,
                f"Frame: {self.global_frame}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )
            cv2.putText(
                debug,
                f"Detections: {len(all_detections)}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )
            
            # Legend
            cv2.putText(debug, "Green arrow: crate orientation", 
                       (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(debug, "Magenta arrow: approach direction", 
                       (10, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            img_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            img_msg.header = msg.header
            self.debug_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedCratePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()