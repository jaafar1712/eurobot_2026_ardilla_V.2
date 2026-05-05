#!/usr/bin/env python3
"""
Gripper Controller — Eurobot 2026
Ignition Fortress only exposes velocity control (cmd_vel) for joints via
JointController plugin. This node implements a proportional position
controller in ROS2, reading /joint_states and publishing velocity commands.

Joint limits (model.sdf):
  left_finger_joint  — prismatic, axis +Y, [-0.04, +0.04] m
  right_finger_joint — prismatic, axis -Y, [-0.04, +0.04] m
  Open = +0.04 m  |  Closed = -0.04 m  (both joints)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts


class GripperController(Node):

    POS_OPEN      =  0.04    # m
    POS_CLOSED    = -0.04    # m
    MAX_VEL       =  0.05    # m/s (matches SDF velocity limit)
    KP            =  2.0     # proportional gain for position → velocity
    DEADBAND      =  0.002   # m — position error considered "reached"

    def __init__(self):
        super().__init__('gripper_controller')

        # ── Publishers ────────────────────────────────────────────────────────
        self.left_vel_pub  = self.create_publisher(
            Float64, '/model/simple_robot/joint/left_finger_joint/cmd_vel', 10)
        self.right_vel_pub = self.create_publisher(
            Float64, '/model/simple_robot/joint/right_finger_joint/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String,     '/gripper/command',       self._cmd_cb,           10)
        self.create_subscription(JointState,  '/joint_states',         self._joint_state_cb,   10)
        self.create_subscription(Contacts,   '/gripper/left/contact',  self._left_contact_cb,  10)
        self.create_subscription(Contacts,   '/gripper/right/contact', self._right_contact_cb, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self.state         = 'initializing'
        self.left_pos      = 0.0
        self.right_pos     = 0.0
        self.left_contact  = False
        self.right_contact = False
        self._joints_ready = False

        # ── 20 Hz control loop ────────────────────────────────────────────────
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('GripperController started — velocity-based position control')

    # ── Callbacks ─────────────────────────────────────────────────────────────

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
            self.get_logger().warn(f'[CMD] Unknown: {cmd!r}')

    def _joint_state_cb(self, msg: JointState):
        try:
            li = msg.name.index('left_finger_joint')
            ri = msg.name.index('right_finger_joint')
            self.left_pos  = msg.position[li]
            self.right_pos = msg.position[ri]
            if not self._joints_ready:
                self._joints_ready = True
                self.state = 'opening'
                self.get_logger().info(
                    f'Joints ready — L={self.left_pos:.3f} R={self.right_pos:.3f} — opening')
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

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        s = String()
        s.data = self.state
        self.state_pub.publish(s)

        if not self._joints_ready:
            return

        if self.state == 'opening':
            lv = self._vel_toward(self.left_pos,  self.POS_OPEN)
            rv = self._vel_toward(self.right_pos, self.POS_OPEN)
            self._publish_vel(lv, rv)
            if abs(self.POS_OPEN - self.left_pos)  < self.DEADBAND and \
               abs(self.POS_OPEN - self.right_pos) < self.DEADBAND:
                self.get_logger().info('[OPEN] Fully open → idle')
                self._publish_vel(0.0, 0.0)
                self.state = 'idle'

        elif self.state == 'closing':
            if self.left_contact and self.right_contact:
                self.get_logger().info('[GRASP] Object grasped!')
                self._publish_vel(0.0, 0.0)
                self.state = 'gripping'
                return

            # Stop a finger once it has contact; keep closing the other
            lv = 0.0 if self.left_contact  else self._vel_toward(self.left_pos,  self.POS_CLOSED)
            rv = 0.0 if self.right_contact else self._vel_toward(self.right_pos, self.POS_CLOSED)
            self._publish_vel(lv, rv)

            # Both fully closed with no contact → missed the crate
            if abs(self.POS_CLOSED - self.left_pos)  < self.DEADBAND and \
               abs(self.POS_CLOSED - self.right_pos) < self.DEADBAND and \
               not (self.left_contact and self.right_contact):
                self.get_logger().warn('[CLOSE] Fully closed — no contact (crate missed)')
                self._publish_vel(0.0, 0.0)
                self.state = 'idle'

        elif self.state == 'gripping':
            self._publish_vel(0.0, 0.0)   # hold via joint damping

        elif self.state == 'idle':
            self._publish_vel(0.0, 0.0)

    def _vel_toward(self, current: float, target: float) -> float:
        error = target - current
        if abs(error) < self.DEADBAND:
            return 0.0
        vel = self.KP * error
        return max(-self.MAX_VEL, min(self.MAX_VEL, vel))

    def _publish_vel(self, left: float, right: float):
        lm = Float64(); lm.data = left
        rm = Float64(); rm.data = right
        self.left_vel_pub.publish(lm)
        self.right_vel_pub.publish(rm)


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
