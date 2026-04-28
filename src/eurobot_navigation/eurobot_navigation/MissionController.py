#!/usr/bin/env python3
"""
ULTRA SIMPLE MISSION CONTROLLER - EUROBOT 2026
Direct approach: See crate → Move to it → Grab → Go home
NO complex state machine - just simple logic that WORKS

Fixes applied:
  - Angular gain now works in radians (was in degrees → controller was always saturated)
  - Linear speed floor lowered to 0.02 m/s (was 0.10 → was overshooting crate)
  - GO_HOME now uses a real Nav2 NavigateToPose action (was a fake 2s sleep)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from eurobot_interfaces.msg import CrateDetectionArray


# ---------------------------------------------------------------------------
# Home pose — update to match your actual deposit zone for the match
# ---------------------------------------------------------------------------
HOME_X = 0.3   # metres from map origin
HOME_Y = 0.3
HOME_YAW = 0.0  # radians


class SimpleMissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # ── Config ──────────────────────────────────────────────────────────
        self.team_color   = 'yellow'
        self.grasp_distance = 0.15   # stop 15 cm away from crate

        # ── State ───────────────────────────────────────────────────────────
        self.state               = "INIT"
        self.start_time          = self.get_clock().now()
        self.current_detection   = None
        self.last_detection_time = None
        self.crate_grabbed       = False

        # Nav2 goal tracking
        self._nav_goal_handle    = None
        self._nav_result         = None   # 'succeeded' | 'failed' | None

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist,            '/cmd_vel',                      10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.det_sub  = self.create_subscription(
            CrateDetectionArray, '/crate/detections', self.on_detection, 10)

        # ── Nav2 action client ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Main loop — 10 Hz ────────────────────────────────────────────────
        self.create_timer(0.1, self.tick)

        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION CONTROLLER — Eurobot 2026")
        self.get_logger().info(f"Team: {self.team_color}")
        self.get_logger().info("=" * 60)

    # ════════════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════════════

    def time_in_state(self):
        """Seconds since entering current state."""
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def log(self, msg):
        self.get_logger().info(f"[{self.state:12s}] {msg}")

    def move(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x  = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.move(0.0, 0.0)

    def open_gripper(self):
        msg = Float64MultiArray()
        msg.data = [0.04, 0.04]
        self.grip_pub.publish(msg)

    def close_gripper(self):
        msg = Float64MultiArray()
        msg.data = [-0.04, -0.04]
        self.grip_pub.publish(msg)

    def change_state(self, new_state):
        self.log(f"→ {new_state}")
        self.state      = new_state
        self.start_time = self.get_clock().now()

    # ════════════════════════════════════════════════════════════════════════
    # Detection callback
    # ════════════════════════════════════════════════════════════════════════

    def on_detection(self, msg):
        """Pick the closest crate of our colour with sufficient confidence."""
        valid = [d for d in msg.detections
                 if d.color == self.team_color and d.confidence > 0.3]

        if valid:
            self.current_detection   = min(valid, key=lambda x: x.distance)
            self.last_detection_time = self.get_clock().now()
            d = self.current_detection
            self.log(f"Crate: {d.distance:.2f} m, angle={math.degrees(d.angle):.1f}°")
        else:
            if self.current_detection is not None:
                self.log("Lost crate")
            self.current_detection = None

    def has_recent_detection(self):
        if self.current_detection is None or self.last_detection_time is None:
            return False
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return age < 2.0

    # ════════════════════════════════════════════════════════════════════════
    # Nav2 helpers
    # ════════════════════════════════════════════════════════════════════════

    def _build_pose(self, x, y, yaw):
        """Build a PoseStamped in the map frame."""
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Convert yaw → quaternion (z, w only needed for 2-D)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_nav_goal(self, x, y, yaw):
        """Send a NavigateToPose goal asynchronously and reset result tracking."""
        self._nav_goal_handle = None
        self._nav_result      = None

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available!")
            self._nav_result = 'failed'
            return

        goal_msg           = NavigateToPose.Goal()
        goal_msg.pose      = self._build_pose(x, y, yaw)

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            self._nav_result = 'failed'
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_result = 'succeeded'
            self.log("Nav2 goal succeeded")
        else:
            self._nav_result = 'failed'
            self.log(f"Nav2 goal failed (status={status})")

    def _nav_feedback_cb(self, feedback):
        # Distance remaining is available in feedback for progress monitoring
        dist = feedback.feedback.distance_remaining
        self.log(f"Navigating… {dist:.2f} m remaining")

    def _cancel_nav_goal(self):
        """Cancel any active Nav2 goal (e.g. on emergency stop)."""
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    # ════════════════════════════════════════════════════════════════════════
    # Main control loop
    # ════════════════════════════════════════════════════════════════════════

    def tick(self):

        # ── INIT ─────────────────────────────────────────────────────────────
        if self.state == "INIT":
            if self.time_in_state() < 1.0:
                self.open_gripper()
            else:
                self.log("Ready!")
                self.change_state("EXIT_ZONE")

        # ── EXIT_ZONE ─────────────────────────────────────────────────────────
        elif self.state == "EXIT_ZONE":
            if self.time_in_state() < 1.5:
                self.move(linear=0.2)
            else:
                self.stop()
                self.change_state("SEARCH")

        # ── SEARCH ───────────────────────────────────────────────────────────
        elif self.state == "SEARCH":
            if self.has_recent_detection():
                self.stop()
                self.change_state("GO_TO_CRATE")
            else:
                # Rotate in place — safer than driving blind
                self.move(linear=0.0, angular=0.4)
                elapsed = self.time_in_state()
                # After a full rotation (~16 s at 0.4 rad/s) nudge forward
                if elapsed > 0 and int(elapsed) % 16 == 15:
                    self.move(linear=0.15, angular=0.0)

        # ── GO_TO_CRATE ──────────────────────────────────────────────────────
        elif self.state == "GO_TO_CRATE":
            if not self.has_recent_detection():
                self.log("Lost crate — back to search")
                self.stop()
                self.change_state("SEARCH")
                return

            d    = self.current_detection
            dist = d.distance
            # FIX 1: angle is stored in radians in the detection message.
            # If your detector publishes degrees, convert here:
            #   angle_rad = math.radians(d.angle)
            angle_rad = d.angle

            if dist <= self.grasp_distance:
                self.stop()
                self.log(f"✓ Reached crate at {dist:.2f} m!")
                self.change_state("GRAB")
                return

            # ── Angular control (proportional, in radians) ──
            # FIX 2: gain now operates on radians, not raw degrees.
            # At 0.5 rad error → 0.75 rad/s, well within 0.3 clamp territory.
            angular_speed = -angle_rad * 1.5
            angular_speed = max(-0.4, min(0.4, angular_speed))

            # ── Linear control ───────────────────────────────────────────────
            distance_error = dist - self.grasp_distance
            linear_speed   = distance_error * 0.8
            # FIX 3: floor lowered to 0.02 m/s — robot won't bulldoze the crate
            linear_speed = max(0.02, min(0.30, linear_speed))

            self.move(linear=linear_speed, angular=angular_speed)
            self.log(
                f"→ Crate: {dist:.2f} m | "
                f"lin={linear_speed:.2f} m/s | "
                f"ang={angular_speed:.2f} rad/s | "
                f"err={math.degrees(angle_rad):.1f}°"
            )

        # ── GRAB ─────────────────────────────────────────────────────────────
        elif self.state == "GRAB":
            if self.time_in_state() < 2.0:
                self.close_gripper()
                if self.time_in_state() < 0.1:
                    self.log("Grabbing…")
            else:
                self.log("✓ Grabbed!")
                self.crate_grabbed = True
                self.change_state("BACKUP")

        # ── BACKUP ───────────────────────────────────────────────────────────
        elif self.state == "BACKUP":
            if self.time_in_state() < 1.0:
                self.move(linear=-0.15)
            else:
                self.stop()
                self.log("✓ Backed up")
                self.change_state("GO_HOME")

        # ── GO_HOME ──────────────────────────────────────────────────────────
        # FIX 4: real Nav2 NavigateToPose action — not a fake sleep.
        elif self.state == "GO_HOME":
            # First entry: fire the goal once
            if self._nav_goal_handle is None and self._nav_result is None:
                self.log(f"Sending Nav2 goal → home ({HOME_X}, {HOME_Y})")
                self._send_nav_goal(HOME_X, HOME_Y, HOME_YAW)
                return  # wait for callbacks

            # Goal sent — wait for result
            if self._nav_result is None:
                return  # still navigating

            if self._nav_result == 'succeeded':
                self.change_state("RELEASE")

            elif self._nav_result == 'failed':
                # Simple retry: reset and try again
                self.log("Nav2 failed — retrying…")
                self._nav_result      = None
                self._nav_goal_handle = None
                # TODO: add a retry counter and give up after N attempts

        # ── RELEASE ──────────────────────────────────────────────────────────
        elif self.state == "RELEASE":
            if self.time_in_state() < 1.5:
                self.open_gripper()
                if self.time_in_state() < 0.1:
                    self.log("Releasing…")
            else:
                self.log("✓ Released")
                self.crate_grabbed = False
                self.change_state("SUCCESS")

        # ── SUCCESS ──────────────────────────────────────────────────────────
        elif self.state == "SUCCESS":
            self.stop()
            if self.time_in_state() < 0.5:
                self.log("MISSION COMPLETE! 🎉")
            # TODO: loop back to SEARCH if time remains


# ═══════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = SimpleMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()