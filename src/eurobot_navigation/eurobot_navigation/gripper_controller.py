#!/usr/bin/env python3
"""
Gripper Controller — Eurobot 2026
Translates /gripper/command (String) into direct Gazebo joint position
commands via gz_bridge, and reports /gripper/state (String) for task_manager.

Joint limits (from model.sdf):
  left_finger_joint  — prismatic, axis +Y, limits [-0.04, +0.04] m
  right_finger_joint — prismatic, axis -Y, limits [-0.04, +0.04] m
  Open  = +0.04 m (both joints)
  Closed = -0.04 m (both joints)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts


class GripperController(Node):

    POS_OPEN   =  0.04   # m — both joints fully open
    POS_CLOSED = -0.04   # m — both joints fully closed
    STEP       =  0.002  # m per tick at 20 Hz → ~2 s full travel

    def __init__(self):
        super().__init__('gripper_controller')

        # ── Publishers ───────────────────────────────────────────────────────
        self.left_pub  = self.create_publisher(
            Float64, '/model/simple_robot/joint/left_finger_joint/cmd_pos', 10)
        self.right_pub = self.create_publisher(
            Float64, '/model/simple_robot/joint/right_finger_joint/cmd_pos', 10)
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,    '/gripper/command',       self._cmd_cb,          10)
        self.create_subscription(JointState, '/joint_states',         self._joint_state_cb,  10)
        self.create_subscription(Contacts,  '/gripper/left/contact',  self._left_contact_cb,  10)
        self.create_subscription(Contacts,  '/gripper/right/contact', self._right_contact_cb, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self.state          = 'initializing'
        self.target_left    = 0.0
        self.target_right   = 0.0
        self.left_contact   = False
        self.right_contact  = False
        self._joints_ready  = False

        # ── 20 Hz control loop ───────────────────────────────────────────────
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('GripperController ready (gz_bridge direct mode)')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == 'open':
            if self.state != 'opening':
                self.get_logger().info(f'[CMD] OPEN  (was: {self.state})')
                self.state = 'opening'
                self.left_contact = self.right_contact = False
        elif cmd == 'close':
            if self.state in ('idle', 'opening'):
                self.get_logger().info('[CMD] CLOSE')
                self.state = 'closing'
                self.left_contact = self.right_contact = False
            else:
                self.get_logger().warn(f'[CMD] Cannot close in state: {self.state}')
        else:
            self.get_logger().warn(f'[CMD] Unknown command: {cmd!r}')

    def _joint_state_cb(self, msg: JointState):
        if self._joints_ready:
            return
        try:
            li = msg.name.index('left_finger_joint')
            ri = msg.name.index('right_finger_joint')
            self.target_left  = msg.position[li]
            self.target_right = msg.position[ri]
            self._joints_ready = True
            self.state = 'opening'
            self.get_logger().info(
                f'Joints initialised — '
                f'L={self.target_left:.3f}  R={self.target_right:.3f} — opening')
        except (ValueError, IndexError):
            pass

    def _left_contact_cb(self, msg: Contacts):
        had = self.left_contact
        self.left_contact = len(msg.contacts) > 0
        if self.left_contact and not had and self.state == 'closing':
            self.get_logger().info('[CONTACT] Left finger')

    def _right_contact_cb(self, msg: Contacts):
        had = self.right_contact
        self.right_contact = len(msg.contacts) > 0
        if self.right_contact and not had and self.state == 'closing':
            self.get_logger().info('[CONTACT] Right finger')

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        # Always broadcast current state
        s = String()
        s.data = self.state
        self.state_pub.publish(s)

        if not self._joints_ready:
            return

        if self.state == 'initializing':
            # Shouldn't stay here after joints are ready, but guard anyway
            self.state = 'opening'

        elif self.state == 'opening':
            self._step_both_toward(self.POS_OPEN)
            if (self.target_left  >= self.POS_OPEN - 1e-4 and
                    self.target_right >= self.POS_OPEN - 1e-4):
                self.get_logger().info('[OPEN] Fully open → idle')
                self.state = 'idle'

        elif self.state == 'closing':
            if self.left_contact and self.right_contact:
                self.get_logger().info('[GRASP] Object grasped!')
                self.state = 'gripping'
                self._publish_targets()
                return

            # Step each finger inward only while it has no contact
            if not self.left_contact:
                self.target_left  = max(self.POS_CLOSED, self.target_left  - self.STEP)
            if not self.right_contact:
                self.target_right = max(self.POS_CLOSED, self.target_right - self.STEP)

            # Fully closed but no contact → missed the crate
            if (self.target_left  <= self.POS_CLOSED + 1e-4 and
                    self.target_right <= self.POS_CLOSED + 1e-4 and
                    not (self.left_contact and self.right_contact)):
                self.get_logger().warn('[CLOSE] Fully closed — no contact (crate missed)')
                self.state = 'idle'

        # 'gripping' and 'idle': hold current target, no movement

        self._publish_targets()

    def _step_both_toward(self, target: float):
        self.target_left  = min(target, self.target_left  + self.STEP)
        self.target_right = min(target, self.target_right + self.STEP)

    def _publish_targets(self):
        self.target_left  = max(self.POS_CLOSED, min(self.POS_OPEN, self.target_left))
        self.target_right = max(self.POS_CLOSED, min(self.POS_OPEN, self.target_right))
        lm = Float64(); lm.data = self.target_left
        rm = Float64(); rm.data = self.target_right
        self.left_pub.publish(lm)
        self.right_pub.publish(rm)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
