#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 1.0  # rad/s
        self.get_logger().info("Keyboard teleop started. Use W/A/S/D to move, Q to quit.")

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = get_key()
            if key.lower() == 'w':
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif key.lower() == 's':
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif key.lower() == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key.lower() == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key.lower() == 'q':
                twist = Twist()
                self.publisher_.publish(twist)
                self.get_logger().info("Exiting teleop.")
                break
            else:
                twist = Twist()

            self.publisher_.publish(twist)
            self.get_logger().info(f"cmd_vel -> linear: {twist.linear.x:.2f}, angular: {twist.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.publisher_.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
