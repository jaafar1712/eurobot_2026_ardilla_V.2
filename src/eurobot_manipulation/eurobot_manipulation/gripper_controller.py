#!/usr/bin/env python3
"""
Eurobot 2026 Gripper Controller - Fixed contact latching
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState

OPEN_POS   =  0.04
CLOSED_POS = -0.04
POSITION_TOLERANCE = 0.003
MOVE_TIMEOUT = 3.0


class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller')

        self._state           = "idle"
        self._left_pos        = 0.04
        self._right_pos       = 0.04
        self._left_contact    = False
        self._right_contact   = False
        self._contact_latched = False
        self._move_start_time = None
        self._joints_ready    = False

        self.create_subscription(String,       '/gripper/command',       self._cmd_cb,           10)
        self.create_subscription(JointState,   '/joint_states',          self._joint_state_cb,   10)
        self.create_subscription(ContactsState,'/gripper/left/contact',  self._left_contact_cb,  10)
        self.create_subscription(ContactsState,'/gripper/right/contact', self._right_contact_cb, 10)

        self._left_pub  = self.create_publisher(Float64, '/model/simple_robot/joint/left_finger_joint/cmd_pos',  10)
        self._right_pub = self.create_publisher(Float64, '/model/simple_robot/joint/right_finger_joint/cmd_pos', 10)
        self._state_pub = self.create_publisher(String,  '/gripper/state', 10)

        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.1,  self._publish_state)

        self.get_logger().info("GripperController ready")

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'open':
            if self._state != 'opening':
                self.get_logger().info(f"[CMD] OPEN  (was: {self._state})")
                self._state = 'opening'
                self._contact_latched = False
                self._move_start_time = self.get_clock().now()
                self._send_pos(OPEN_POS)
        elif cmd == 'close':
            if self._state == 'idle':
                self.get_logger().info("[CMD] CLOSE")
                self._state = 'closing'
                self._contact_latched = False
                self._left_contact    = False
                self._right_contact   = False
                self._move_start_time = self.get_clock().now()
                self._send_pos(CLOSED_POS)
            elif self._state in ('closing', 'gripping'):
                pass  # silently ignore repeated close commands
            else:
                self.get_logger().warn(f"[CMD] Cannot close in state: {self._state}")

    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'left_finger_joint':
                self._left_pos = msg.position[i]
            elif name == 'right_finger_joint':
                self._right_pos = msg.position[i]
        if not self._joints_ready:
            self._joints_ready = True
            self.get_logger().info(
                f"Joints initialised — L={self._left_pos:.3f}  R={self._right_pos:.3f} — opening"
            )
            self._state = 'opening'
            self._move_start_time = self.get_clock().now()
            self._send_pos(OPEN_POS)

    def _left_contact_cb(self, msg: ContactsState):
        had = self._left_contact
        self._left_contact = len(msg.states) > 0
        if self._left_contact and not had:
            self.get_logger().info("[CONTACT] Left finger")
        if self._left_contact and self._state == 'closing':
            self._contact_latched = True

    def _right_contact_cb(self, msg: ContactsState):
        had = self._right_contact
        self._right_contact = len(msg.states) > 0
        if self._right_contact and not had:
            self.get_logger().info("[CONTACT] Right finger")
        if self._right_contact and self._state == 'closing':
            self._contact_latched = True

    def _control_loop(self):
        if not self._joints_ready:
            return
        now = self.get_clock().now()
        elapsed = (now - self._move_start_time).nanoseconds / 1e9 if self._move_start_time else 0

        if self._state == 'opening':
            self._send_pos(OPEN_POS)
            if self._at_target(OPEN_POS) or elapsed > MOVE_TIMEOUT:
                self.get_logger().info("[OPEN] Fully open → idle")
                self._state = 'idle'

        elif self._state == 'closing':
            if self._contact_latched:
                hold = (self._left_pos + self._right_pos) / 2.0
                self._send_pos(hold)
                self.get_logger().info(
                    f"[GRIP] Contact latched L={self._left_pos:.3f} R={self._right_pos:.3f} → gripping"
                )
                self._state = 'gripping'
                return
            if self._at_target(CLOSED_POS):
                self.get_logger().warn("[MISS] Fully closed, no contact → idle")
                self._state = 'idle'
                return
            if elapsed > MOVE_TIMEOUT:
                self.get_logger().warn("[MISS] Timeout, no contact → idle")
                self._state = 'idle'
                return
            self._send_pos(CLOSED_POS)

        elif self._state == 'gripping':
            hold = (self._left_pos + self._right_pos) / 2.0
            self._send_pos(hold)

    def _publish_state(self):
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _send_pos(self, pos: float):
        msg = Float64()
        msg.data = float(pos)
        self._left_pub.publish(msg)
        self._right_pub.publish(msg)

    def _at_target(self, target: float) -> bool:
        return (abs(self._left_pos  - target) < POSITION_TOLERANCE and
                abs(self._right_pos - target) < POSITION_TOLERANCE)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Gripper Controller shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()