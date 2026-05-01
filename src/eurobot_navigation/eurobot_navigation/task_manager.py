#!/usr/bin/env python3
"""
Eurobot 2026 Task Manager
Sequential controller for autonomous crate collection mission
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import math

from eurobot_interfaces.msg import CrateDetectionArray, PantryDetectionArray


class TaskManager(Node):
    """
    Main task manager for Eurobot 2026 competition
    Orchestrates: Perception → Navigation → Manipulation
    """

    # State definitions
    STATE_INITIALIZING = "INITIALIZING"
    STATE_SEARCHING_CRATE = "SEARCHING_CRATE"
    STATE_NAVIGATING_TO_CRATE = "NAVIGATING_TO_CRATE"
    STATE_ALIGNING_WITH_CRATE = "ALIGNING_WITH_CRATE"
    STATE_GRIPPING_CRATE = "GRIPPING_CRATE"
    STATE_NAVIGATING_TO_DROPOFF = "NAVIGATING_TO_DROPOFF"
    STATE_RELEASING_CRATE = "RELEASING_CRATE"
    STATE_RETURNING_TO_NEST = "RETURNING_TO_NEST"
    STATE_FINISHED = "FINISHED"

    def __init__(self):
        super().__init__('task_manager')

        # ========== PARAMETERS ==========
        self.declare_parameter('team_color', 'yellow')
        self.declare_parameter('match_duration', 100.0)
        self.declare_parameter('nest_return_time', 10.0)
        self.declare_parameter('max_crates_to_collect', 6)
        self.declare_parameter('alignment_distance', 0.50)  # distance from base_link when gripper jaws surround crate
        self.declare_parameter('alignment_tolerance', 0.04)  # ±4 cm tolerance
        self.declare_parameter('angle_tolerance', 8.0)       # degrees

        self.team_color = self.get_parameter('team_color').value
        self.match_duration = self.get_parameter('match_duration').value
        self.nest_return_time = self.get_parameter('nest_return_time').value
        self.max_crates = self.get_parameter('max_crates_to_collect').value
        self.alignment_dist = self.get_parameter('alignment_distance').value
        self.alignment_tol = self.get_parameter('alignment_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value

        # ========== STATE VARIABLES ==========
        self.state = self.STATE_INITIALIZING
        self.match_start_time = None
        self.crates_collected = 0
        self.current_crate = None
        self.home_position = None
        self.current_position = None
        self.current_yaw = 0.0

        # Perception data
        self.latest_crates = []
        self.latest_pantries = []
        self.gripper_state = "idle"

        # Navigation flags
        self.nav_goal_active = False
        self.nav_goal_result = None
        self.nav_goal_sent_time = None
        self.nav_timeout = 30.0

        # Grip timing
        self._grip_start_time = None

        # ========== SUBSCRIBERS ==========
        self.create_subscription(CrateDetectionArray, '/crate/detections', self.crate_callback, 10)
        self.create_subscription(PantryDetectionArray, '/pantry/detections', self.pantry_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/gripper/state', self.gripper_state_callback, 10)

        # ========== PUBLISHERS ==========
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_cmd_pub = self.create_publisher(String, '/gripper/command', 10)
        self.state_pub = self.create_publisher(String, '/task_manager/state', 10)

        # ========== ACTION CLIENTS ==========
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ========== TIMERS ==========
        self.main_timer = self.create_timer(0.05, self.main_loop)
        self.state_publish_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info(f"Task Manager initialized - Team: {self.team_color}")
        self.get_logger().info("Waiting for navigation action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Navigation server ready!")

    # ========================================================================
    # CALLBACKS
    # ========================================================================

    def crate_callback(self, msg):
        self.latest_crates = [det for det in msg.detections if det.color == self.team_color]

    def pantry_callback(self, msg):
        self.latest_pantries = msg.detections

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.home_position is None:
            self.home_position = {
                'x': self.current_position.x,
                'y': self.current_position.y,
            }
            self.get_logger().info(
                f"Home position set: ({self.home_position['x']:.2f}, {self.home_position['y']:.2f})"
            )

    def gripper_state_callback(self, msg):
        self.gripper_state = msg.data

    # ========================================================================
    # MAIN STATE MACHINE
    # ========================================================================

    def main_loop(self):
        # Navigation timeout watchdog
        if self.nav_goal_active and self.nav_goal_sent_time is not None:
            elapsed = (self.get_clock().now() - self.nav_goal_sent_time).nanoseconds / 1e9
            if elapsed > self.nav_timeout:
                self.get_logger().warn(f"Navigation timeout ({elapsed:.1f}s)!")
                self.nav_goal_active = False
                self.nav_goal_result = False
            elif elapsed > 5.0:
                self.get_logger().info(
                    f"Navigation in progress... {elapsed:.1f}s elapsed",
                    throttle_duration_sec=5.0
                )

        # Match timer
        if self.match_start_time is not None:
            elapsed = (self.get_clock().now() - self.match_start_time).nanoseconds / 1e9
            time_remaining = self.match_duration - elapsed

            if time_remaining > 0:
                self.get_logger().info(
                    f"Match time: {time_remaining:.1f}s remaining",
                    throttle_duration_sec=10.0
                )

            if (time_remaining <= self.nest_return_time
                    and self.state not in (self.STATE_RETURNING_TO_NEST, self.STATE_FINISHED)):
                self.get_logger().warn(f"{time_remaining:.1f}s remaining - returning to nest!")
                self.transition_to(self.STATE_RETURNING_TO_NEST)

            if elapsed >= self.match_duration:
                self.transition_to(self.STATE_FINISHED)

        # Dispatch
        dispatch = {
            self.STATE_INITIALIZING:        self.handle_initializing,
            self.STATE_SEARCHING_CRATE:     self.handle_searching_crate,
            self.STATE_NAVIGATING_TO_CRATE: self.handle_navigating_to_crate,
            self.STATE_ALIGNING_WITH_CRATE: self.handle_aligning_with_crate,
            self.STATE_GRIPPING_CRATE:      self.handle_gripping_crate,
            self.STATE_NAVIGATING_TO_DROPOFF: self.handle_navigating_to_dropoff,
            self.STATE_RELEASING_CRATE:     self.handle_releasing_crate,
            self.STATE_RETURNING_TO_NEST:   self.handle_returning_to_nest,
            self.STATE_FINISHED:            self.handle_finished,
        }
        handler = dispatch.get(self.state)
        if handler:
            handler()

    # ========================================================================
    # STATE HANDLERS
    # ========================================================================

    def handle_initializing(self):
        if self.home_position is None:
            return

        self.match_start_time = self.get_clock().now()
        self.get_logger().info("Match started! Gripper opening, searching for crates...")
        self.send_gripper_command('open')
        self.transition_to(self.STATE_SEARCHING_CRATE)

    def handle_searching_crate(self):
        if self.crates_collected >= self.max_crates:
            self.get_logger().info("Maximum crates collected! Returning to nest.")
            self.transition_to(self.STATE_RETURNING_TO_NEST)
            return

        if not self.latest_crates:
            self.rotate_in_place(0.3)
            return

        self.current_crate = min(self.latest_crates, key=lambda c: c.distance)
        self.get_logger().info(
            f"Target crate: distance={self.current_crate.distance:.2f}m, "
            f"angle={self.current_crate.angle:.1f}°"
        )
        self.transition_to(self.STATE_NAVIGATING_TO_CRATE)

    def handle_navigating_to_crate(self):
        """Drive toward the crate at full speed until close enough for fine alignment."""
        if self.latest_crates:
            self.current_crate = min(self.latest_crates, key=lambda c: c.distance)

        if self.current_crate is None:
            self.stop_robot()
            self.transition_to(self.STATE_SEARCHING_CRATE)
            return

        angle_deg = self.current_crate.angle
        distance = self.current_crate.distance

        # Hand off to fine alignment when close
        if distance <= self.alignment_dist + 0.05:
            self.stop_robot()
            self.get_logger().info("Reached crate vicinity — starting fine alignment")
            self.transition_to(self.STATE_ALIGNING_WITH_CRATE)
            return

        cmd = Twist()
        angle_rad = math.radians(angle_deg)

        # Positive angle = crate LEFT → positive angular.z turns left toward crate
        cmd.angular.z = max(-0.8, min(0.8, 2.0 * angle_rad))

        if abs(angle_deg) < 45.0:
            cmd.linear.x = max(0.05, min(0.5, 0.6 * (distance - self.alignment_dist)))

        self.get_logger().info(
            f"Approaching: dist={distance:.2f}m  angle={angle_deg:.1f}°",
            throttle_duration_sec=1.0
        )
        self.cmd_vel_pub.publish(cmd)

    def handle_aligning_with_crate(self):
        """
        Fine alignment with gripper open.
        Corrects angle and slowly drives forward until the crate sits exactly
        at alignment_dist — the position where the open gripper jaws surround it.
        """
        if not self.latest_crates:
            # If crate was very close before it disappeared, it's in the gripper — grip it
            if (self.current_crate is not None
                    and self.current_crate.distance <= self.alignment_dist + 0.10):
                self.stop_robot()
                self.get_logger().info("Crate at close range, disappeared from FOV — gripping")
                self.transition_to(self.STATE_GRIPPING_CRATE)
            else:
                self.stop_robot()
                self.get_logger().warn("Lost sight of crate during alignment — searching again")
                self.transition_to(self.STATE_SEARCHING_CRATE)
            return

        crate = min(self.latest_crates, key=lambda c: c.distance)
        distance_error = crate.distance - self.alignment_dist
        angle_error = abs(crate.angle)

        self.get_logger().info(
            f"Aligning: dist={crate.distance:.3f}m (want {self.alignment_dist:.2f}m)  "
            f"angle={crate.angle:.1f}°",
            throttle_duration_sec=1.0
        )

        # Crate is at the perfect grab position — close gripper
        if abs(distance_error) < self.alignment_tol and angle_error < self.angle_tol:
            self.stop_robot()
            self.get_logger().info("Crate in grip position — closing gripper")
            self.transition_to(self.STATE_GRIPPING_CRATE)
            return

        cmd = Twist()
        angle_rad = math.radians(crate.angle)

        # Proportional angular correction
        cmd.angular.z = max(-0.4, min(0.4, 1.5 * angle_rad))

        # Slow forward approach only when roughly aimed; back up if overshot
        if angle_error < 15.0:
            if distance_error > 0:
                # Approach slowly so we stop precisely at alignment_dist
                cmd.linear.x = max(0.03, min(0.12, 0.5 * distance_error))
            elif distance_error < -self.alignment_tol:
                cmd.linear.x = -0.05

        self.cmd_vel_pub.publish(cmd)

    def handle_gripping_crate(self):
        """
        Gripper was open during approach.  Now close it to grab the crate.
        A timeout prevents getting stuck if gripper feedback is absent.
        """
        if self._grip_start_time is None:
            self._grip_start_time = self.get_clock().now()
            self.send_gripper_command('close')
            self.get_logger().info("Gripper closing...")
            return

        elapsed = (self.get_clock().now() - self._grip_start_time).nanoseconds / 1e9

        if self.gripper_state == "gripping":
            self.get_logger().info("Crate gripped!")
            self.crates_collected += 1
            self.get_logger().info(f"Crates collected: {self.crates_collected}/{self.max_crates}")
            self.transition_to(self.STATE_NAVIGATING_TO_DROPOFF)

        elif self.gripper_state == "closing":
            if elapsed > 3.0:
                # Gripper taking too long — assume success and move on
                self.get_logger().warn("Gripper close timeout — proceeding to dropoff")
                self.crates_collected += 1
                self.transition_to(self.STATE_NAVIGATING_TO_DROPOFF)

        elif elapsed > 5.0:
            # Gripper never responded — release and retry
            self.get_logger().warn("Gripper did not respond — reopening and searching again")
            self.send_gripper_command('open')
            self.current_crate = None
            self.transition_to(self.STATE_SEARCHING_CRATE)

        else:
            # Keep sending close command until gripper responds
            self.send_gripper_command('close')

    def handle_navigating_to_dropoff(self):
        dropoff_x, dropoff_y, dropoff_yaw = None, None, 0.0
        dropoff_name = "NEST"

        valid_pantries = [p for p in self.latest_pantries if p.id != -1 and p.distance > 0.3]

        if valid_pantries:
            pantry = min(valid_pantries, key=lambda p: p.distance)
            pantry_angle_rad = math.radians(pantry.angle)
            dropoff_x = self.current_position.x + pantry.distance * math.cos(self.current_yaw + pantry_angle_rad)
            dropoff_y = self.current_position.y + pantry.distance * math.sin(self.current_yaw + pantry_angle_rad)
            dropoff_yaw = self.current_yaw + pantry_angle_rad
            dropoff_name = f"PANTRY {pantry.id}"
        else:
            dropoff_x = self.home_position['x']
            dropoff_y = self.home_position['y']
            dropoff_yaw = 0.0

        if not self.nav_goal_active:
            self.get_logger().info(f"Navigating to {dropoff_name} at ({dropoff_x:.2f}, {dropoff_y:.2f})")
            self.send_nav_goal(dropoff_x, dropoff_y, dropoff_yaw)
            return

        if self.nav_goal_result is not None:
            if self.nav_goal_result:
                self.get_logger().info(f"Reached {dropoff_name}")
                self.transition_to(self.STATE_RELEASING_CRATE)
            else:
                self.get_logger().warn(f"Failed to reach {dropoff_name}")
                if dropoff_name != "NEST":
                    self.latest_pantries = []
                    self.nav_goal_active = False
                    self.nav_goal_result = None
                else:
                    self.transition_to(self.STATE_RELEASING_CRATE)

    def handle_releasing_crate(self):
        if self.gripper_state in ("gripping", "closing"):
            self.send_gripper_command('open')
            return
        elif self.gripper_state == "opening":
            return
        else:
            self.get_logger().info("Crate released!")
            if self.crates_collected >= self.max_crates:
                self.transition_to(self.STATE_RETURNING_TO_NEST)
            else:
                self.transition_to(self.STATE_SEARCHING_CRATE)

    def handle_returning_to_nest(self):
        if self.home_position is None:
            self.get_logger().error("No home position!")
            self.transition_to(self.STATE_FINISHED)
            return

        if not self.nav_goal_active:
            self.get_logger().info("Returning to nest...")
            self.send_nav_goal(self.home_position['x'], self.home_position['y'], 0.0)
            return

        if self.nav_goal_result is not None:
            if self.nav_goal_result:
                self.get_logger().info("Safely in nest!")
            else:
                self.get_logger().warn("Failed to reach nest")
            self.transition_to(self.STATE_FINISHED)

    def handle_finished(self):
        self.stop_robot()
        self.get_logger().info(
            f"Match finished! Total crates collected: {self.crates_collected}",
            throttle_duration_sec=5.0
        )

    # ========================================================================
    # HELPER FUNCTIONS
    # ========================================================================

    def transition_to(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self.state = new_state
        self.nav_goal_active = False
        self.nav_goal_result = None
        self.nav_goal_sent_time = None

        if new_state == self.STATE_GRIPPING_CRATE:
            self._grip_start_time = None  # reset so timing starts fresh
        if new_state == self.STATE_NAVIGATING_TO_CRATE:
            self.send_gripper_command('open')  # ensure gripper is open before approach

    def send_nav_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.nav_goal_active = True
        self.nav_goal_result = None
        self.nav_goal_sent_time = self.get_clock().now()

        self.get_logger().info(f"Sending nav goal: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal REJECTED")
            self.nav_goal_result = False
            self.nav_goal_active = False
            return
        self.get_logger().info("Navigation goal accepted")
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        self.nav_goal_active = False
        if result.status == 4:
            self.get_logger().info("Navigation SUCCEEDED")
            self.nav_goal_result = True
        else:
            self.get_logger().warn(f"Navigation FAILED (status={result.status})")
            self.nav_goal_result = False

    def send_gripper_command(self, command):
        msg = String()
        msg.data = command
        self.gripper_cmd_pub.publish(msg)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def rotate_in_place(self, angular_vel):
        cmd = Twist()
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Task Manager shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
