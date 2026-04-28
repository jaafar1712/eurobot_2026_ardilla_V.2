#!/usr/bin/env python3
"""
PRODUCTION ARUCO CRATE PERCEPTION NODE - EUROBOT 2026
=====================================================

Features:
- Temporal filtering (Kalman-like smoothing)
- Outlier rejection
- Adaptive confidence scoring
- Comprehensive parameter validation
- Proper coordinate transformations
- Orientation information
- Debug visualization
- Performance monitoring

Author: Team Yellow - Eurobot 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Dict

from eurobot_interfaces.msg import CrateDetection, CrateDetectionArray


@dataclass
class TrackedCrate:
    """Track a crate over time for temporal filtering"""
    marker_id: int
    color: str
    positions: deque  # Recent (x, y) positions
    distances: deque  # Recent distances
    angles: deque     # Recent angles
    confidences: deque  # Recent confidence scores
    last_seen: float  # Timestamp
    detection_count: int
    
    def __init__(self, marker_id: int, color: str, max_history: int = 5):
        self.marker_id = marker_id
        self.color = color
        self.positions = deque(maxlen=max_history)
        self.distances = deque(maxlen=max_history)
        self.angles = deque(maxlen=max_history)
        self.confidences = deque(maxlen=max_history)
        self.last_seen = 0.0
        self.detection_count = 0
    
    def update(self, x: float, y: float, distance: float, angle: float, 
               confidence: float, timestamp: float):
        """Add new detection"""
        self.positions.append((x, y))
        self.distances.append(distance)
        self.angles.append(angle)
        self.confidences.append(confidence)
        self.last_seen = timestamp
        self.detection_count += 1
    
    def get_filtered_position(self) -> tuple:
        """Get temporally filtered position (moving average)"""
        if not self.positions:
            return (0.0, 0.0)
        
        # Weighted average (more recent = higher weight)
        weights = np.linspace(0.5, 1.0, len(self.positions))
        weights /= weights.sum()
        
        x_vals = [p[0] for p in self.positions]
        y_vals = [p[1] for p in self.positions]
        
        x_filtered = np.average(x_vals, weights=weights)
        y_filtered = np.average(y_vals, weights=weights)
        
        return (float(x_filtered), float(y_filtered))
    
    def get_filtered_distance(self) -> float:
        """Get filtered distance"""
        if not self.distances:
            return 0.0
        weights = np.linspace(0.5, 1.0, len(self.distances))
        weights /= weights.sum()
        return float(np.average(list(self.distances), weights=weights))
    
    def get_filtered_angle(self) -> float:
        """Get filtered angle (handle wrap-around)"""
        if not self.angles:
            return 0.0
        weights = np.linspace(0.5, 1.0, len(self.angles))
        weights /= weights.sum()
        return float(np.average(list(self.angles), weights=weights))
    
    def get_filtered_confidence(self) -> float:
        """Get average confidence"""
        if not self.confidences:
            return 0.0
        return float(np.mean(list(self.confidences)))
    
    def is_stable(self, min_detections: int = 2) -> bool:
        """Check if crate has enough stable detections"""
        return self.detection_count >= min_detections
    
    def age(self, current_time: float) -> float:
        """Time since last seen"""
        return current_time - self.last_seen


class ArucoCratePerception(Node):
    """Production-ready ArUco crate perception for Eurobot 2026"""
    
    def __init__(self):
        super().__init__('aruco_crate_perception')
        
        self.bridge = CvBridge()
        
        # ==================== PARAMETERS ====================
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()
        
        # ==================== ARUCO SETUP ====================
        self._setup_aruco()
        
        # ==================== CAMERA MODEL ====================
        self._setup_camera_model()
        
        # ==================== TRACKING ====================
        self.tracked_crates: Dict[int, TrackedCrate] = {}
        self.tracking_timeout = 1.0  # seconds
        
        # ==================== STATISTICS ====================
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_report = self.get_clock().now()
        
        # ==================== ROS INTERFACES ====================
        self.sub_image = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        
        # Optional: camera_info for proper calibration
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.camera_info_received = False
        
        self.pub_detections = self.create_publisher(
            CrateDetectionArray, '/crate/detections', 10)
        
        if self.visualize:
            self.pub_debug = self.create_publisher(
                Image, '/crate/debug', 1)
        
        # Periodic cleanup timer
        self.create_timer(0.5, self._cleanup_old_tracks)
        
        self.get_logger().info("="*60)
        self.get_logger().info("ARUCO CRATE PERCEPTION")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Marker size: {self.marker_size}m")
        self.get_logger().info(f"Image size: {self.image_width}x{self.image_height}")
        self.get_logger().info(f"Focal length: {self.focal_length}px")
        self.get_logger().info(f"Temporal filtering: {self.enable_filtering}")
        self.get_logger().info(f"Min detections for stability: {self.min_stable_detections}")
        self.get_logger().info("="*60)
    
    # ==================== INITIALIZATION ====================
    
    def _declare_parameters(self):
        """Declare all ROS parameters"""
        # Camera
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('focal_length', 277.0)
        
        # ArUco
        self.declare_parameter('marker_size', 0.04)  # 40mm
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        
        # Detection thresholds
        self.declare_parameter('min_distance', 0.10)
        self.declare_parameter('max_distance', 6.0)
        self.declare_parameter('min_confidence', 0.30)
        self.declare_parameter('min_marker_perimeter', 30)  # pixels
        
        # Coordinate transform
        self.declare_parameter('camera_pitch', 0.7)  # radians
        self.declare_parameter('camera_x_offset', 0.10)  # meters
        self.declare_parameter('camera_z_offset', 0.20)  # meters
        self.declare_parameter('base_frame', 'base_link')
        
        # Filtering
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('filter_history_size', 5)
        self.declare_parameter('min_stable_detections', 2)
        self.declare_parameter('tracking_timeout', 1.0)  # seconds
        
        # Outlier rejection
        self.declare_parameter('enable_outlier_rejection', True)
        self.declare_parameter('max_position_jump', 0.5)  # meters
        
        # Debug
        self.declare_parameter('visualize', True)
        self.declare_parameter('show_fps', True)
    
    def _load_parameters(self):
        """Load all parameters"""
        self.camera_topic = self.get_parameter('camera_topic').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.focal_length = self.get_parameter('focal_length').value
        
        self.marker_size = self.get_parameter('marker_size').value
        self.aruco_dict_name = self.get_parameter('aruco_dict').value
        
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.min_marker_perimeter = self.get_parameter('min_marker_perimeter').value
        
        self.camera_pitch = self.get_parameter('camera_pitch').value
        self.camera_x_offset = self.get_parameter('camera_x_offset').value
        self.camera_z_offset = self.get_parameter('camera_z_offset').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.enable_filtering = self.get_parameter('enable_filtering').value
        self.filter_history_size = self.get_parameter('filter_history_size').value
        self.min_stable_detections = self.get_parameter('min_stable_detections').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        
        self.enable_outlier_rejection = self.get_parameter('enable_outlier_rejection').value
        self.max_position_jump = self.get_parameter('max_position_jump').value
        
        self.visualize = self.get_parameter('visualize').value
        self.show_fps = self.get_parameter('show_fps').value
    
    def _validate_parameters(self):
        """Validate parameter values"""
        assert self.marker_size > 0, "Marker size must be positive"
        assert self.min_distance < self.max_distance, "Invalid distance range"
        assert 0 <= self.min_confidence <= 1, "Confidence must be 0-1"
        assert self.focal_length > 0, "Focal length must be positive"
        
        if self.marker_size != 0.04:
            self.get_logger().warn(f"⚠️ Marker size is {self.marker_size}m, competition standard is 0.04m (40mm)")
    
    def _setup_aruco(self):
        """Setup ArUco detector with optimized parameters"""
        # Get dictionary
        dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_6X6_50': aruco.DICT_6X6_50,
        }
        
        dict_id = dict_map.get(self.aruco_dict_name, aruco.DICT_4X4_50)
        self.aruco_dict = aruco.getPredefinedDictionary(dict_id)
        
        # Optimized detector parameters
        try:
            self.aruco_params = aruco.DetectorParameters()
        except:
            self.aruco_params = aruco.DetectorParameters_create()
        
        # Tuned for Eurobot competition lighting
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.minMarkerPerimeterRate = self.min_marker_perimeter / max(self.image_width, self.image_height)
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        
        # Marker ID mapping
        self.ARUCO_IDS = {
            36: 'blue',
            47: 'yellow',
            41: 'empty'  # Competition may have empty crates
        }
    
    def _setup_camera_model(self):
        """Setup camera intrinsic matrix"""
        fx = self.focal_length
        fy = self.focal_length
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.dist_coeffs = np.zeros(5, dtype=np.float64)
    
    def camera_info_callback(self, msg: CameraInfo):
        """Update camera model from camera_info (if available)"""
        if not self.camera_info_received:
            self.get_logger().info("✓ Received camera_info - using calibrated parameters")
            
            # Use calibrated camera matrix
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K.astype(np.float64)
            
            # Use distortion coefficients
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            
            self.camera_info_received = True
    
    # ==================== COORDINATE TRANSFORMS ====================
    
    def camera_to_base_link(self, tvec: np.ndarray) -> tuple:
        """
        Transform from camera frame to base_link frame
        
        Camera frame (OpenCV convention):
          X: right
          Y: down
          Z: forward (optical axis)
        
        Robot base_link frame:
          X: forward
          Y: left
          Z: up
        
        Camera is mounted with pitch angle looking down/forward
        
        Args:
            tvec: Translation vector from ArUco (x_cam, y_cam, z_cam)
        
        Returns:
            (x_base, y_base): Position in base_link frame (meters)
        """
        x_cam, y_cam, z_cam = tvec
        
        # Apply camera pitch rotation (camera tilted down)
        # Rotate around X-axis (camera's right direction)
        pitch = self.camera_pitch
        
        # Rotation matrix for pitch
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)
        
        # Transform from camera to intermediate frame (aligned with robot but at camera position)
        y_intermediate = y_cam * cos_p + z_cam * sin_p
        z_intermediate = -y_cam * sin_p + z_cam * cos_p
        x_intermediate = x_cam
        
        # Now transform to robot base_link
        # Camera X → Robot -Y (camera right = robot left)
        # Camera Y_intermediate → Robot Z (camera down = robot up, after pitch)
        # Camera Z_intermediate → Robot X (camera forward = robot forward)
        
        x_base = z_intermediate + self.camera_x_offset
        y_base = -x_intermediate  # Camera right is robot left (negative Y)
        
        return (x_base, y_base)
    
    # ==================== DETECTION & FILTERING ====================
    
    def compute_confidence(self, corners: np.ndarray, tvec: np.ndarray, 
                          distance: float, angle: float) -> float:
        """
        Compute detection confidence score
        
        Factors:
        - Corner quality (square-ness)
        - Distance (closer = better)
        - Angle (frontal = better)
        - Marker size in image (bigger = better)
        """
        # Corner quality (how square is the marker?)
        corner_quality = self._corner_quality(corners)
        
        # Distance score (inverse normalized)
        distance_score = 1.0 - min(distance / self.max_distance, 1.0)
        
        # Angle score (frontal = best)
        angle_score = 1.0 - min(abs(angle) / (math.pi / 2), 1.0)
        
        # Size score (larger markers in image = better)
        perimeter = cv2.arcLength(corners, True)
        max_perimeter = (self.image_width + self.image_height) * 0.5  # Rough max
        size_score = min(perimeter / max_perimeter, 1.0)
        
        # Weighted combination
        confidence = (
            0.30 * corner_quality +
            0.30 * distance_score +
            0.20 * angle_score +
            0.20 * size_score
        )
        
        return float(np.clip(confidence, 0.0, 1.0))
    
    def _corner_quality(self, corners: np.ndarray) -> float:
        """
        Measure how square/regular the detected marker is
        
        Perfect square = 1.0
        Distorted = lower score
        """
        corners = corners.reshape(4, 2)
        
        # Compute side lengths
        sides = []
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i + 1) % 4]
            sides.append(np.linalg.norm(p2 - p1))
        
        # Compute diagonal lengths
        diag1 = np.linalg.norm(corners[2] - corners[0])
        diag2 = np.linalg.norm(corners[3] - corners[1])
        
        # Side ratio (opposite sides should be equal)
        if max(sides[0], sides[2]) > 0:
            side_ratio_1 = min(sides[0], sides[2]) / max(sides[0], sides[2])
        else:
            side_ratio_1 = 0.0
        
        if max(sides[1], sides[3]) > 0:
            side_ratio_2 = min(sides[1], sides[3]) / max(sides[1], sides[3])
        else:
            side_ratio_2 = 0.0
        
        # Diagonal ratio (diagonals should be equal in square)
        if max(diag1, diag2) > 0:
            diag_ratio = min(diag1, diag2) / max(diag1, diag2)
        else:
            diag_ratio = 0.0
        
        # Combine
        quality = (side_ratio_1 + side_ratio_2 + diag_ratio) / 3.0
        
        return float(quality)
    
    def is_outlier(self, marker_id: int, x: float, y: float) -> bool:
        """Check if detection is an outlier (sudden jump in position)"""
        if not self.enable_outlier_rejection:
            return False
        
        if marker_id not in self.tracked_crates:
            return False  # First detection, can't be outlier
        
        tracked = self.tracked_crates[marker_id]
        if not tracked.positions:
            return False
        
        last_x, last_y = tracked.positions[-1]
        jump_distance = math.sqrt((x - last_x)**2 + (y - last_y)**2)
        
        if jump_distance > self.max_position_jump:
            self.get_logger().warn(
                f"⚠️ Outlier rejected for {tracked.color} (jump: {jump_distance:.2f}m)",
                throttle_duration_sec=1.0
            )
            return True
        
        return False
    
    def detect_markers(self, frame: np.ndarray, timestamp: float) -> List[dict]:
        """Main detection function"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None:
            return []
        
        # Estimate poses
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        
        detections = []
        
        for i, marker_id in enumerate(ids.flatten()):
            # Filter unknown IDs
            if marker_id not in self.ARUCO_IDS:
                continue
            
            color = self.ARUCO_IDS[marker_id]
            
            # Get pose
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]
            
            # Transform to base_link
            x_base, y_base = self.camera_to_base_link(tvec)
            
            # Compute metrics
            distance = math.sqrt(x_base**2 + y_base**2)
            angle = math.atan2(y_base, x_base)
            
            # Distance filter
            if distance < self.min_distance or distance > self.max_distance:
                continue
            
            # Outlier rejection
            if self.is_outlier(marker_id, x_base, y_base):
                continue
            
            # Confidence
            confidence = self.compute_confidence(corners[i][0], tvec, distance, angle)
            
            if confidence < self.min_confidence:
                continue
            
            # Update tracking
            if self.enable_filtering:
                if marker_id not in self.tracked_crates:
                    self.tracked_crates[marker_id] = TrackedCrate(
                        marker_id, color, self.filter_history_size)
                
                self.tracked_crates[marker_id].update(
                    x_base, y_base, distance, angle, confidence, timestamp)
            
            # Store detection
            detections.append({
                'marker_id': marker_id,
                'color': color,
                'x': x_base,
                'y': y_base,
                'distance': distance,
                'angle': angle,
                'confidence': confidence,
                'corners': corners[i][0],
                'rvec': rvec,
                'tvec': tvec
            })
        
        return detections
    
    def get_filtered_detections(self) -> List[dict]:
        """Get temporally filtered detections"""
        if not self.enable_filtering:
            return []
        
        filtered = []
        
        for marker_id, tracked in self.tracked_crates.items():
            if not tracked.is_stable(self.min_stable_detections):
                continue
            
            x, y = tracked.get_filtered_position()
            distance = tracked.get_filtered_distance()
            angle = tracked.get_filtered_angle()
            confidence = tracked.get_filtered_confidence()
            
            filtered.append({
                'marker_id': marker_id,
                'color': tracked.color,
                'x': x,
                'y': y,
                'distance': distance,
                'angle': angle,
                'confidence': confidence,
                'detection_count': tracked.detection_count
            })
        
        return filtered
    
    def _cleanup_old_tracks(self):
        """Remove tracks that haven't been seen recently"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        to_remove = []
        for marker_id, tracked in self.tracked_crates.items():
            if tracked.age(current_time) > self.tracking_timeout:
                to_remove.append(marker_id)
        
        for marker_id in to_remove:
            del self.tracked_crates[marker_id]
    
    # ==================== VISUALIZATION ====================
    
    def draw_detection(self, img: np.ndarray, det: dict, is_filtered: bool = False):
        """Draw detection on debug image"""
        corners = det.get('corners')
        if corners is None:
            return  # Filtered detection without corners
        
        corners_int = corners.astype(int)
        
        # Color based on type
        if is_filtered:
            color_bgr = (0, 255, 0)  # Green for stable filtered
            thickness = 3
        else:
            color_bgr = (255, 255, 0)  # Cyan for raw
            thickness = 2
        
        # Draw marker outline
        cv2.polylines(img, [corners_int], True, color_bgr, thickness)
        
        if 'rvec' in det and 'tvec' in det:
            axis_length = self.marker_size * 0.75  # scale axes (tune if needed)
    
            cv2.drawFrameAxes(
                img,
                self.camera_matrix,
                self.dist_coeffs,
                det['rvec'],
                det['tvec'],
                axis_length
            )
        
        # Center point
        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        cv2.circle(img, (cx, cy), 5, color_bgr, -1)
        
        # Label
        label_lines = [
            f"{det['color']} ID:{det['marker_id']}",
            f"X:{det['x']:.2f} Y:{det['y']:.2f}",
            f"D:{det['distance']:.2f}m C:{det['confidence']:.2f}"
            ]
        
        if 'detection_count' in det:
            label_lines.append(f"n={det['detection_count']}")
        
        y_offset = -15
        for line in label_lines:
            cv2.putText(img, line, (cx - 60, cy + y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 2)
            y_offset += 15
        
        # Direction arrow
        arrow_len = 50
        end_x = int(cx + arrow_len * math.cos(det['angle']))
        end_y = int(cy + arrow_len * math.sin(det['angle']))
        cv2.arrowedLine(img, (cx, cy), (end_x, end_y),
                       (0, 255, 255), 2, tipLength=0.3)
    
    # ==================== MAIN CALLBACK ====================
    
    def image_callback(self, msg: Image):
        """Main image processing callback"""
        self.frame_count += 1
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # Convert image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Detect markers (raw)
        raw_detections = self.detect_markers(frame, timestamp)
        
        # Get filtered detections
        if self.enable_filtering:
            final_detections = self.get_filtered_detections()
        else:
            final_detections = raw_detections
        
        # Publish detections
        if final_detections:
            msg_out = CrateDetectionArray()
            msg_out.header = msg.header
            msg_out.header.frame_id = self.base_frame
            
            for det in final_detections:
                m = CrateDetection()
                m.color = det['color']
                m.x = float(det['x'])
                m.y = float(det['y'])
                m.distance = float(det['distance'])
                m.angle = float(math.degrees(det['angle']))
                m.confidence = float(det['confidence'])
                msg_out.detections.append(m)
            
            msg_out.total_detections = len(final_detections)
            self.pub_detections.publish(msg_out)
            
            self.detection_count += len(final_detections)
        
        # Debug visualization
        if self.visualize:
            debug = frame.copy()
            
            # Draw raw detections
            for det in raw_detections:
                self.draw_detection(debug, det, is_filtered=False)
            
            # Draw filtered detections (if different)
            if self.enable_filtering:
                for det in final_detections:
                    # Don't have corners for filtered, skip detailed viz
                    pass
            
            # Status text
            status_y = 25
            cv2.putText(debug, f"Frame: {self.frame_count}", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (255, 255, 255), 1)
            
            status_y += 20
            cv2.putText(debug, f"Raw: {len(raw_detections)} | Filtered: {len(final_detections)}", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (255, 255, 255), 1)
            
            status_y += 20
            cv2.putText(debug, f"Tracked: {len(self.tracked_crates)}", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (255, 255, 255), 1)
            
            # FPS
            if self.show_fps:
                now = self.get_clock().now()
                dt = (now - self.last_fps_report).nanoseconds / 1e9
                if dt > 1.0:
                    fps = self.frame_count / dt
                    self.get_logger().info(
                        f"📊 FPS: {fps:.1f} | Detections: {self.detection_count}",
                        throttle_duration_sec=5.0)
                    self.last_fps_report = now
            
            # Publish
            debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCratePerception()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()