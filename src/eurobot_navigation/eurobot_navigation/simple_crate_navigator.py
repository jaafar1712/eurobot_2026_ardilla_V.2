#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from eurobot_interfaces.msg import CrateDetectionArray
import math
from collections import deque


class SimpleCrateNavigator(Node):
    def __init__(self):
        super().__init__('simple_crate_navigator')

        # === PARAMETERS ===
        self.declare_parameter('target_color', 'yellow')
        self.declare_parameter('approach_distance', 0.4)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('angle_tolerance', 1.5)     # degrees
        self.declare_parameter('distance_tolerance', 0.015)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 3.0)
        self.declare_parameter('enable_approach', True)

        self.target_color = self.get_parameter('target_color').value
        self.approach_dist = self.get_parameter('approach_distance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.dist_tol = self.get_parameter('distance_tolerance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.enabled = self.get_parameter('enable_approach').value

        # === STATE ===
        self.state = 'SEARCHING'  # SEARCHING, ALIGNING, APPROACHING, ALIGNED, COMPLETED
        self.target_crate = None
        self.last_detection_time = None

        self.gripper_done = False
        self.return_home_sent = False

        # === FILTERING ===
        self.angle_buffer = deque(maxlen=5)
        self.distance_buffer = deque(maxlen=5)

        # === ROS INTERFACES ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper/command', 10)
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.det_sub = self.create_subscription(
            CrateDetectionArray,
            '/crate/detections',
            self.detection_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("=== Crate Navigator Started ===")
        self.get_logger().info(
            f"Target: {self.target_color} | Approach distance: {self.approach_dist} m"
        )

    # === DETECTION CALLBACK ===
    def detection_callback(self, msg):
        if not self.enabled or self.state in ['ALIGNED', 'COMPLETED']:
            return

        candidates = [d for d in msg.detections if d.color == self.target_color]
        if not candidates:
            return

        self.target_crate = min(candidates, key=lambda c: c.distance)
        self.last_detection_time = self.get_clock().now().nanoseconds / 1e9

        self.angle_buffer.append(self.target_crate.angle)
        self.distance_buffer.append(self.target_crate.distance)

        if self.state == 'SEARCHING':
            self.state = 'ALIGNING'
            self.get_logger().info("[STATE] SEARCHING -> ALIGNING")

    # === TARGET LOSS LOGIC WITH HYSTERESIS ===
    def is_target_lost(self):
        if self.last_detection_time is None:
            return True

        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == 'ALIGNING':
            timeout = 1.5
        elif self.state == 'APPROACHING':
            timeout = 2.5
        else:
            return False

        return (now - self.last_detection_time) > timeout

    # === CONTROL LOOP ===
    def control_loop(self):
        cmd = Twist()

        if not self.enabled:
            self.cmd_pub.publish(cmd)
            return

        # === HANDLE TARGET LOSS ===
        if self.is_target_lost():
            self.get_logger().warn("[STATE] Target lost -> SEARCHING")
            self.state = 'SEARCHING'
            self.target_crate = None
            self.angle_buffer.clear()
            self.distance_buffer.clear()

        # === SEARCHING ===
        if self.state == 'SEARCHING':
            cmd.angular.z = 0.2
            self.cmd_pub.publish(cmd)
            return

        if not self.target_crate:
            self.cmd_pub.publish(cmd)
            return

        # === SMOOTHED VALUES ===
        avg_angle = sum(self.angle_buffer) / len(self.angle_buffer)
        avg_distance = sum(self.distance_buffer) / len(self.distance_buffer)

        angle_error_rad = math.radians(avg_angle)
        distance_error = avg_distance - self.approach_dist

        # Reduce smoothing near target
        if self.state == 'APPROACHING' and avg_distance < 0.8:
            self.angle_buffer = deque(list(self.angle_buffer)[-2:], maxlen=2)
            self.distance_buffer = deque(list(self.distance_buffer)[-2:], maxlen=2)

        # === STATE MACHINE ===
        if self.state == 'ALIGNING':
            if abs(avg_angle) > self.angle_tol:
                cmd.angular.z = self.clamp(
                    -self.kp_angular * angle_error_rad,
                    -self.max_angular,
                    self.max_angular
                )
            else:
                self.state = 'APPROACHING'
                self.get_logger().info("[STATE] ALIGNING -> APPROACHING")

        elif self.state == 'APPROACHING':
            if abs(distance_error) <= self.dist_tol:
                self.state = 'ALIGNED'
                self.get_logger().info("[STATE] APPROACHING -> ALIGNED")
            else:
                cmd.linear.x = self.clamp(
                    self.kp_linear * distance_error,
                    -self.max_linear,
                    self.max_linear
                )

                if abs(avg_angle) > self.angle_tol * 0.5:
                    cmd.angular.z = self.clamp(
                        -0.4 * self.kp_angular * angle_error_rad,
                        -0.3 * self.max_angular,
                        0.3 * self.max_angular
                    )

        elif self.state == 'ALIGNED':
            if not self.gripper_done:
                self.gripper_pub.publish(String(data="close"))
                self.get_logger().info("[ALIGNED] Gripper closing")
                self.gripper_done = True

            if self.gripper_done and not self.return_home_sent:
                self.send_home_goal()
                self.return_home_sent = True
                self.state = 'COMPLETED'

        elif self.state == 'COMPLETED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # === SEND NAV2 GOAL ===
    def send_home_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0

        self.nav_pub.publish(goal)
        self.get_logger().info("[NAV2] Goal sent to (0,0)")

    # === CLAMP ===
    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min_val, min(value, max_val))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCrateNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
