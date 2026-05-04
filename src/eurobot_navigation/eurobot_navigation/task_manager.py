#!/usr/bin/env python3
"""
Eurobot 2026 Task Manager (with integrated gripper controller)
Single node — handles mission logic AND gripper hardware directly.

FIXES APPLIED:
  - Added angle_sign parameter to fix wrong navigation direction
  - Fixed premature gripper closing (tightened lost-crate margin)
  - Added crate_lost_counter to avoid reacting on single missed frames
  - Significantly increased speeds (approach, alignment, rotation)
  - Tightened alignment distance margin
  - Fast gripper-miss detection: closing→idle transition = missed grip
  - Failed-crate blacklist: prevents infinite retargeting of same crate
  - Fixed infinite nav loop in handle_navigating_to_dropoff
  - Fixed infinite nav loop in handle_returning_to_nest
  - Merged gripper_controller: velocity-based position control in-process
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from ros_gz_interfaces.msg import Contacts
import math

from eurobot_interfaces.msg import CrateDetectionArray, PantryDetectionArray


class TaskManager(Node):

    # ── Mission states ────────────────────────────────────────────────────────
    STATE_INITIALIZING        = "INITIALIZING"
    STATE_SEARCHING_CRATE     = "SEARCHING_CRATE"
    STATE_NAVIGATING_TO_CRATE = "NAVIGATING_TO_CRATE"
    STATE_ALIGNING_WITH_CRATE = "ALIGNING_WITH_CRATE"
    STATE_GRIPPING_CRATE      = "GRIPPING_CRATE"
    STATE_NAVIGATING_TO_DROPOFF = "NAVIGATING_TO_DROPOFF"
    STATE_RELEASING_CRATE     = "RELEASING_CRATE"
    STATE_RETURNING_TO_NEST   = "RETURNING_TO_NEST"
    STATE_FINISHED            = "FINISHED"

    # ── Gripper hardware constants (from model.sdf) ───────────────────────────
    GRIPPER_POS_OPEN   =  0.04   # m
    GRIPPER_POS_CLOSED = -0.04   # m
    GRIPPER_MAX_VEL    =  0.05   # m/s
    GRIPPER_KP         =  2.0
    GRIPPER_DEADBAND   =  0.002  # m

    def __init__(self):
        super().__init__('task_manager')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('team_color', 'yellow')
        self.declare_parameter('match_duration', 100.0)
        self.declare_parameter('nest_return_time', 10.0)
        self.declare_parameter('max_crates_to_collect', 6)
        self.declare_parameter('alignment_distance', 0.50)
        self.declare_parameter('alignment_tolerance', 0.04)
        self.declare_parameter('angle_tolerance', 8.0)
        self.declare_parameter('angle_sign', 1)

        self.team_color       = self.get_parameter('team_color').value
        self.match_duration   = self.get_parameter('match_duration').value
        self.nest_return_time = self.get_parameter('nest_return_time').value
        self.max_crates       = self.get_parameter('max_crates_to_collect').value
        self.alignment_dist   = self.get_parameter('alignment_distance').value
        self.alignment_tol    = self.get_parameter('alignment_tolerance').value
        self.angle_tol        = self.get_parameter('angle_tolerance').value
        self.angle_sign       = self.get_parameter('angle_sign').value

        # ── Mission state ─────────────────────────────────────────────────────
        self.state            = self.STATE_INITIALIZING
        self.match_start_time = None
        self.crates_collected = 0
        self.current_crate    = None
        self.home_position    = None
        self.current_position = None
        self.current_yaw      = 0.0

        self.latest_crates   = []
        self.latest_pantries = []

        self.nav_goal_active    = False
        self.nav_goal_result    = None
        self.nav_goal_sent_time = None
        self.nav_timeout        = 30.0
        self._dropoff_nav_sent  = False
        self._nest_nav_sent     = False

        self._grip_start_time   = None
        self._gripper_missed    = False

        self.failed_crate_positions = []
        self._BLACKLIST_RADIUS      = 0.30

        self._crate_lost_count    = 0
        self._CRATE_LOST_THRESHOLD = 5

        # ── Gripper hardware state ────────────────────────────────────────────
        self.gripper_state      = 'initializing'
        self._gripper_left_pos  = 0.0
        self._gripper_right_pos = 0.0
        self._gripper_left_contact  = False
        self._gripper_right_contact = False
        self._gripper_joints_ready  = False

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_vel_pub       = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.state_pub         = self.create_publisher(String, '/task_manager/state', 10)
        self.gripper_state_pub = self.create_publisher(String, '/gripper/state', 10)
        self._left_vel_pub  = self.create_publisher(
            Float64, '/model/simple_robot/joint/left_finger_joint/cmd_vel', 10)
        self._right_vel_pub = self.create_publisher(
            Float64, '/model/simple_robot/joint/right_finger_joint/cmd_vel', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(CrateDetectionArray,  '/crate/detections',   self.crate_callback,   10)
        self.create_subscription(PantryDetectionArray, '/pantry/detections',  self.pantry_callback,  10)
        self.create_subscription(Odometry,             '/odom',               self.odom_callback,    10)
        self.create_subscription(JointState,           '/joint_states',       self._joint_state_cb,  10)
        self.create_subscription(Contacts, '/gripper/left/contact',  self._left_contact_cb,  10)
        self.create_subscription(Contacts, '/gripper/right/contact', self._right_contact_cb, 10)

        # ── Action client ─────────────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.05,  self.main_loop)           # 20 Hz mission loop
        self.create_timer(0.05,  self._gripper_loop)       # 20 Hz gripper loop
        self.create_timer(1.0,   self.publish_state)

        self.get_logger().info(f"TaskManager+Gripper started — team: {self.team_color}")
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready!")

    # =========================================================================
    # PERCEPTION / ODOMETRY CALLBACKS
    # =========================================================================

    def crate_callback(self, msg):
        self.latest_crates = [d for d in msg.detections if d.color == self.team_color]

    def pantry_callback(self, msg):
        self.latest_pantries = msg.detections

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        if self.home_position is None:
            self.home_position = {'x': self.current_position.x, 'y': self.current_position.y}
            self.get_logger().info(
                f"Home set: ({self.home_position['x']:.2f}, {self.home_position['y']:.2f})")

    # =========================================================================
    # GRIPPER HARDWARE CALLBACKS
    # =========================================================================

    def _joint_state_cb(self, msg: JointState):
        try:
            li = msg.name.index('left_finger_joint')
            ri = msg.name.index('right_finger_joint')
            self._gripper_left_pos  = msg.position[li]
            self._gripper_right_pos = msg.position[ri]
            if not self._gripper_joints_ready:
                self._gripper_joints_ready = True
                self.gripper_state = 'opening'
                self.get_logger().info('Gripper joints ready — opening')
        except (ValueError, IndexError):
            pass

    def _left_contact_cb(self, msg: Contacts):
        had = self._gripper_left_contact
        self._gripper_left_contact = len(msg.contacts) > 0
        if self._gripper_left_contact and not had and self.gripper_state == 'closing':
            self.get_logger().info('[CONTACT] Left finger')

    def _right_contact_cb(self, msg: Contacts):
        had = self._gripper_right_contact
        self._gripper_right_contact = len(msg.contacts) > 0
        if self._gripper_right_contact and not had and self.gripper_state == 'closing':
            self.get_logger().info('[CONTACT] Right finger')

    # =========================================================================
    # GRIPPER CONTROL LOOP (20 Hz)
    # =========================================================================

    def _gripper_loop(self):
        s = String()
        s.data = self.gripper_state
        self.gripper_state_pub.publish(s)

        if not self._gripper_joints_ready:
            return

        if self.gripper_state == 'opening':
            lv = self._gripper_vel(self._gripper_left_pos,  self.GRIPPER_POS_OPEN)
            rv = self._gripper_vel(self._gripper_right_pos, self.GRIPPER_POS_OPEN)
            self._pub_gripper_vel(lv, rv)
            if (abs(self.GRIPPER_POS_OPEN - self._gripper_left_pos)  < self.GRIPPER_DEADBAND and
                    abs(self.GRIPPER_POS_OPEN - self._gripper_right_pos) < self.GRIPPER_DEADBAND):
                self.get_logger().info('[GRIPPER] Fully open → idle')
                self._pub_gripper_vel(0.0, 0.0)
                self.gripper_state = 'idle'

        elif self.gripper_state == 'closing':
            if self._gripper_left_contact and self._gripper_right_contact:
                self.get_logger().info('[GRIPPER] Object grasped!')
                self._pub_gripper_vel(0.0, 0.0)
                self.gripper_state = 'gripping'
                return

            lv = 0.0 if self._gripper_left_contact  else self._gripper_vel(self._gripper_left_pos,  self.GRIPPER_POS_CLOSED)
            rv = 0.0 if self._gripper_right_contact else self._gripper_vel(self._gripper_right_pos, self.GRIPPER_POS_CLOSED)
            self._pub_gripper_vel(lv, rv)

            # Fully closed with no contact → missed
            if (abs(self.GRIPPER_POS_CLOSED - self._gripper_left_pos)  < self.GRIPPER_DEADBAND and
                    abs(self.GRIPPER_POS_CLOSED - self._gripper_right_pos) < self.GRIPPER_DEADBAND and
                    not (self._gripper_left_contact and self._gripper_right_contact)):
                self.get_logger().warn('[GRIPPER] Fully closed — no contact (missed)')
                self._pub_gripper_vel(0.0, 0.0)
                self.gripper_state = 'idle'
                self._gripper_missed = True   # signal mission loop

        else:
            self._pub_gripper_vel(0.0, 0.0)  # idle / gripping — hold

    def _gripper_open(self):
        if self.gripper_state != 'opening':
            self.get_logger().info(f'[GRIPPER] Open command (was: {self.gripper_state})')
            self.gripper_state = 'opening'
            self._gripper_left_contact = self._gripper_right_contact = False

    def _gripper_close(self):
        if self.gripper_state in ('idle', 'opening'):
            self.get_logger().info('[GRIPPER] Close command')
            self.gripper_state = 'closing'
            self._gripper_left_contact = self._gripper_right_contact = False

    def _gripper_vel(self, current: float, target: float) -> float:
        err = target - current
        if abs(err) < self.GRIPPER_DEADBAND:
            return 0.0
        return max(-self.GRIPPER_MAX_VEL, min(self.GRIPPER_MAX_VEL, self.GRIPPER_KP * err))

    def _pub_gripper_vel(self, left: float, right: float):
        lm = Float64(); lm.data = left
        rm = Float64(); rm.data = right
        self._left_vel_pub.publish(lm)
        self._right_vel_pub.publish(rm)

    # =========================================================================
    # MISSION STATE MACHINE (20 Hz)
    # =========================================================================

    def main_loop(self):
        # Navigation timeout watchdog
        if self.nav_goal_active and self.nav_goal_sent_time is not None:
            elapsed = (self.get_clock().now() - self.nav_goal_sent_time).nanoseconds / 1e9
            if elapsed > self.nav_timeout:
                self.get_logger().warn(f"Nav timeout ({elapsed:.1f}s)!")
                self.nav_goal_active = False
                self.nav_goal_result = False
            elif elapsed > 5.0:
                self.get_logger().info(
                    f"Navigating... {elapsed:.1f}s", throttle_duration_sec=5.0)

        # Match timer
        if self.match_start_time is not None:
            elapsed = (self.get_clock().now() - self.match_start_time).nanoseconds / 1e9
            remaining = self.match_duration - elapsed
            if remaining > 0:
                self.get_logger().info(
                    f"Match: {remaining:.1f}s remaining", throttle_duration_sec=10.0)
            if (remaining <= self.nest_return_time
                    and self.state not in (self.STATE_RETURNING_TO_NEST, self.STATE_FINISHED)):
                self.get_logger().warn(f"{remaining:.1f}s left — returning to nest!")
                self.transition_to(self.STATE_RETURNING_TO_NEST)
            if elapsed >= self.match_duration:
                self.transition_to(self.STATE_FINISHED)

        dispatch = {
            self.STATE_INITIALIZING:          self.handle_initializing,
            self.STATE_SEARCHING_CRATE:       self.handle_searching_crate,
            self.STATE_NAVIGATING_TO_CRATE:   self.handle_navigating_to_crate,
            self.STATE_ALIGNING_WITH_CRATE:   self.handle_aligning_with_crate,
            self.STATE_GRIPPING_CRATE:        self.handle_gripping_crate,
            self.STATE_NAVIGATING_TO_DROPOFF: self.handle_navigating_to_dropoff,
            self.STATE_RELEASING_CRATE:       self.handle_releasing_crate,
            self.STATE_RETURNING_TO_NEST:     self.handle_returning_to_nest,
            self.STATE_FINISHED:              self.handle_finished,
        }
        handler = dispatch.get(self.state)
        if handler:
            handler()

    # =========================================================================
    # STATE HANDLERS
    # =========================================================================

    def handle_initializing(self):
        if self.home_position is None:
            return
        self.match_start_time = self.get_clock().now()
        self.get_logger().info("Match started!")
        self._gripper_open()
        self.transition_to(self.STATE_SEARCHING_CRATE)

    def handle_searching_crate(self):
        if self.crates_collected >= self.max_crates:
            self.get_logger().info("Max crates collected — returning to nest.")
            self.transition_to(self.STATE_RETURNING_TO_NEST)
            return

        if not self.latest_crates:
            self.rotate_in_place(0.6)
            return

        available = [c for c in self.latest_crates if not self._is_blacklisted(c)]
        if not available:
            self.get_logger().warn("All crates blacklisted — clearing blacklist")
            self.failed_crate_positions.clear()
            available = self.latest_crates

        self.current_crate = min(available, key=lambda c: c.distance)
        self.get_logger().info(
            f"Target: dist={self.current_crate.distance:.2f}m  angle={self.current_crate.angle:.1f}°")
        self.transition_to(self.STATE_NAVIGATING_TO_CRATE)

    def handle_navigating_to_crate(self):
        if self.latest_crates:
            self.current_crate = min(self.latest_crates, key=lambda c: c.distance)
            self._crate_lost_count = 0
        else:
            self._crate_lost_count += 1
            if self._crate_lost_count < self._CRATE_LOST_THRESHOLD:
                return
            self.stop_robot()
            self.transition_to(self.STATE_SEARCHING_CRATE)
            return

        if self.current_crate is None:
            self.stop_robot()
            self.transition_to(self.STATE_SEARCHING_CRATE)
            return

        angle_deg = self.current_crate.angle
        distance  = self.current_crate.distance

        if distance <= self.alignment_dist + 0.05:
            self.stop_robot()
            self.get_logger().info("Crate vicinity reached — fine alignment")
            self.transition_to(self.STATE_ALIGNING_WITH_CRATE)
            return

        cmd = Twist()
        cmd.angular.z = max(-1.2, min(1.2, 3.0 * self.angle_sign * math.radians(angle_deg)))
        if abs(angle_deg) < 45.0:
            cmd.linear.x = max(0.1, min(0.8, 0.8 * (distance - self.alignment_dist)))
        self.get_logger().info(
            f"Approaching: dist={distance:.2f}m  angle={angle_deg:.1f}°",
            throttle_duration_sec=1.0)
        self.cmd_vel_pub.publish(cmd)

    def handle_aligning_with_crate(self):
        if not self.latest_crates:
            self._crate_lost_count += 1
            if self._crate_lost_count >= self._CRATE_LOST_THRESHOLD:
                if (self.current_crate is not None
                        and self.current_crate.distance <= self.alignment_dist + 0.03):
                    self.stop_robot()
                    self.get_logger().info("Crate disappeared at close range — gripping")
                    self.transition_to(self.STATE_GRIPPING_CRATE)
                else:
                    self.stop_robot()
                    self.get_logger().warn("Lost crate during alignment — searching again")
                    self.transition_to(self.STATE_SEARCHING_CRATE)
            return

        self._crate_lost_count = 0
        crate = min(self.latest_crates, key=lambda c: c.distance)
        self.current_crate  = crate
        distance_error = crate.distance - self.alignment_dist
        angle_error    = abs(crate.angle)

        self.get_logger().info(
            f"Aligning: dist={crate.distance:.3f}m  angle={crate.angle:.1f}°",
            throttle_duration_sec=1.0)

        if abs(distance_error) < self.alignment_tol and angle_error < self.angle_tol:
            self.stop_robot()
            self.get_logger().info("In grip position — closing")
            self.transition_to(self.STATE_GRIPPING_CRATE)
            return

        cmd = Twist()
        cmd.angular.z = max(-0.6, min(0.6, 2.0 * self.angle_sign * math.radians(crate.angle)))
        if angle_error < 15.0:
            if distance_error > 0:
                cmd.linear.x = max(0.05, min(0.20, 0.6 * distance_error))
            elif distance_error < -self.alignment_tol:
                cmd.linear.x = -0.08
        self.cmd_vel_pub.publish(cmd)

    def handle_gripping_crate(self):
        if self._grip_start_time is None:
            self._grip_start_time = self.get_clock().now()
            self._gripper_missed  = False
            self._gripper_close()
            self.get_logger().info("Gripper closing...")
            return

        # Gripper loop detected closing→idle with no contact
        if self._gripper_missed:
            self._gripper_missed = False
            self.get_logger().warn("Grip missed — blacklisting crate")
            self._blacklist_current_crate()
            self._gripper_open()
            self.current_crate = None
            self.transition_to(self.STATE_SEARCHING_CRATE)
            return

        elapsed = (self.get_clock().now() - self._grip_start_time).nanoseconds / 1e9

        if self.gripper_state == 'gripping':
            self.get_logger().info("Crate gripped!")
            self.crates_collected += 1
            self.get_logger().info(f"Crates: {self.crates_collected}/{self.max_crates}")
            self.failed_crate_positions.clear()
            self.transition_to(self.STATE_NAVIGATING_TO_DROPOFF)

        elif elapsed > 8.0:
            self.get_logger().warn("Gripper timeout — is the JointController plugin loaded?")
            self._blacklist_current_crate()
            self._gripper_open()
            self.current_crate = None
            self.transition_to(self.STATE_SEARCHING_CRATE)

    def handle_navigating_to_dropoff(self):
        if not self._dropoff_nav_sent:
            valid_pantries = [p for p in self.latest_pantries if p.id != -1 and p.distance > 0.3]
            if valid_pantries:
                pantry = min(valid_pantries, key=lambda p: p.distance)
                a = math.radians(pantry.angle)
                dx = self.current_position.x + pantry.distance * math.cos(self.current_yaw + a)
                dy = self.current_position.y + pantry.distance * math.sin(self.current_yaw + a)
                self._dropoff_name = f"PANTRY {pantry.id}"
                self.send_nav_goal(dx, dy, self.current_yaw + a)
            else:
                self._dropoff_name = "NEST"
                self.send_nav_goal(self.home_position['x'], self.home_position['y'], 0.0)
            self.get_logger().info(f"Navigating to {self._dropoff_name}")
            self._dropoff_nav_sent = True
            return

        if self.nav_goal_result is not None:
            self.get_logger().info(
                f"{'Reached' if self.nav_goal_result else 'Failed to reach'} {self._dropoff_name}")
            self.transition_to(self.STATE_RELEASING_CRATE)

    def handle_releasing_crate(self):
        if self.gripper_state in ('gripping', 'closing'):
            self._gripper_open()
            return
        if self.gripper_state == 'opening':
            return
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
        if not self._nest_nav_sent:
            self.get_logger().info("Returning to nest...")
            self.send_nav_goal(self.home_position['x'], self.home_position['y'], 0.0)
            self._nest_nav_sent = True
            return
        if self.nav_goal_result is not None:
            self.get_logger().info(
                "In nest!" if self.nav_goal_result else "Failed to reach nest")
            self.transition_to(self.STATE_FINISHED)

    def handle_finished(self):
        self.stop_robot()
        self._pub_gripper_vel(0.0, 0.0)
        self.get_logger().info(
            f"Match finished! Crates collected: {self.crates_collected}",
            throttle_duration_sec=5.0)

    # =========================================================================
    # HELPERS
    # =========================================================================

    def _crate_world_pos(self, crate):
        a = self.current_yaw + math.radians(crate.angle)
        return (self.current_position.x + crate.distance * math.cos(a),
                self.current_position.y + crate.distance * math.sin(a))

    def _is_blacklisted(self, crate):
        if self.current_position is None:
            return False
        wx, wy = self._crate_world_pos(crate)
        return any(math.hypot(wx-fx, wy-fy) < self._BLACKLIST_RADIUS
                   for fx, fy in self.failed_crate_positions)

    def _blacklist_current_crate(self):
        if self.current_crate is None or self.current_position is None:
            return
        wx, wy = self._crate_world_pos(self.current_crate)
        self.failed_crate_positions.append((wx, wy))
        self.get_logger().warn(
            f"Blacklisted ({wx:.2f}, {wy:.2f}) [{len(self.failed_crate_positions)} total]")

    def transition_to(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self.state              = new_state
        self.nav_goal_active    = False
        self.nav_goal_result    = None
        self.nav_goal_sent_time = None
        self._crate_lost_count  = 0
        self._dropoff_nav_sent  = False
        self._nest_nav_sent     = False
        if new_state == self.STATE_GRIPPING_CRATE:
            self._grip_start_time = None
        if new_state == self.STATE_NAVIGATING_TO_CRATE:
            self._gripper_open()

    def send_nav_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav_goal_active    = True
        self.nav_goal_result    = None
        self.nav_goal_sent_time = self.get_clock().now()
        self.get_logger().info(f"Nav goal → ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.1f}°")
        self.nav_client.send_goal_async(goal).add_done_callback(self._nav_goal_cb)

    def _nav_goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav goal REJECTED")
            self.nav_goal_result = False
            self.nav_goal_active = False
            return
        self.get_logger().info("Nav goal accepted")
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        self.nav_goal_active = False
        result = future.result()
        if result.status == 4:
            self.get_logger().info("Navigation SUCCEEDED")
            self.nav_goal_result = True
        else:
            self.get_logger().warn(f"Navigation FAILED (status={result.status})")
            self.nav_goal_result = False

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
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
