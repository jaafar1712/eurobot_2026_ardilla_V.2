#!/usr/bin/env python3
"""
Clean Gripper Controller - Built from scratch
Simple and reliable gripper control with proper state management
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')

        # ========== Publishers ==========
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/gripper_controller/commands', 
            10
        )
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)

        # ========== Subscribers ==========
        self.create_subscription(
            String, 
            '/gripper/command', 
            self.command_callback, 
            10
        )
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.create_subscription(
            Contacts, 
            '/gripper/left/contact', 
            self.left_contact_callback, 
            10
        )
        self.create_subscription(
            Contacts, 
            '/gripper/right/contact', 
            self.right_contact_callback, 
            10
        )

        # ========== State Variables ==========
        self.state = "initializing"
        
        # Position limits
        self.MIN_POS = -0.04  # Fully closed
        self.MAX_POS = 0.04   # Fully open
        self.STEP = 0.01      # Movement increment
        
        # Current positions (read from sensors)
        self.current_left = 0.0
        self.current_right = 0.0
        self.joint_states_received = False
        
        # Target positions (what we command)
        self.target_left = 0.0
        self.target_right = 0.0
        
        # Contact sensors
        self.left_contact = False
        self.right_contact = False

        # ========== Control Timer ==========
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("="*60)
        self.get_logger().info("Gripper Controller Started")
        self.get_logger().info("="*60)

    # ========== Callbacks ==========

    def command_callback(self, msg):
        """Handle open/close commands"""
        command = msg.data.lower().strip()
        
        if command == 'open':
            if self.state in ['idle', 'gripping', 'closing']:
                self.get_logger().info("[CMD] OPEN")
                self.state = 'opening'
                self.left_contact = False
                self.right_contact = False
            else:
                self.get_logger().warn(f"[CMD] Cannot open in state: {self.state}")
                
        elif command == 'close':
            if self.state in ['idle', 'opening']:
                self.get_logger().info("[CMD] CLOSE")
                self.state = 'closing'
                self.left_contact = False
                self.right_contact = False
            else:
                self.get_logger().warn(f"[CMD] Cannot close in state: {self.state}")
        else:
            self.get_logger().warn(f"[CMD] Unknown command: '{command}'")

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        try:
            left_idx = msg.name.index('left_finger_joint')
            right_idx = msg.name.index('right_finger_joint')
            
            self.current_left = msg.position[left_idx]
            self.current_right = msg.position[right_idx]
            
            if not self.joint_states_received:
                self.joint_states_received = True
                self.target_left = self.current_left
                self.target_right = self.current_right
                self.get_logger().info(
                    f"[INIT] Joint states received: "
                    f"left={self.current_left:.4f}, right={self.current_right:.4f}"
                )
        except (ValueError, IndexError):
            pass

    def left_contact_callback(self, msg):
        """Left finger contact sensor"""
        was_contact = self.left_contact
        self.left_contact = len(msg.contacts) > 0
        
        if self.left_contact and not was_contact and self.state == 'closing':
            self.get_logger().info("[CONTACT] Left finger")

    def right_contact_callback(self, msg):
        """Right finger contact sensor"""
        was_contact = self.right_contact
        self.right_contact = len(msg.contacts) > 0
        
        if self.right_contact and not was_contact and self.state == 'closing':
            self.get_logger().info("[CONTACT] Right finger")

    # ========== Control Loop ==========

    def control_loop(self):
        """Main control loop - runs at 20 Hz"""
        
        # Always publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        # State machine
        if self.state == 'initializing':
            self.handle_initializing()
        elif self.state == 'opening':
            self.handle_opening()
        elif self.state == 'closing':
            self.handle_closing()
        elif self.state == 'gripping':
            self.handle_gripping()
        elif self.state == 'idle':
            self.handle_idle()
        
        # Always publish target positions
        self.publish_targets()

    # ========== State Handlers ==========

    def handle_initializing(self):
        """Wait for joint states, then open gripper"""
        if not self.joint_states_received:
            return
        
        # Got joint states - start opening
        self.get_logger().info("[INIT] Opening gripper to start position...")
        self.state = 'opening'

    def handle_opening(self):
        """Open the gripper"""
        # Increment targets toward max
        if self.target_left < self.MAX_POS:
            self.target_left = min(self.MAX_POS, self.target_left + self.STEP)
        if self.target_right < self.MAX_POS:
            self.target_right = min(self.MAX_POS, self.target_right + self.STEP)
        
        # Check if fully open
        if self.target_left >= self.MAX_POS and self.target_right >= self.MAX_POS:
            self.get_logger().info("[OPEN] Gripper fully opened")
            self.state = 'idle'

    def handle_closing(self):
        """Close the gripper"""
        # Check if both fingers have contact
        if self.left_contact and self.right_contact:
            self.get_logger().info("[GRASP] Object grasped!")
            self.state = 'gripping'
            return
        
        # Decrement targets toward min (only if no contact)
        if not self.left_contact and self.target_left > self.MIN_POS:
            self.target_left = max(self.MIN_POS, self.target_left - self.STEP)
        if not self.right_contact and self.target_right > self.MIN_POS:
            self.target_right = max(self.MIN_POS, self.target_right - self.STEP)
        
        # Check if fully closed without object
        if self.target_left <= self.MIN_POS and self.target_right <= self.MIN_POS:
            if not (self.left_contact and self.right_contact):
                self.get_logger().warn("[CLOSE] Fully closed but no object detected")
                self.state = 'idle'

    def handle_gripping(self):
        """Hold the object - maintain current position"""
        pass  # Targets don't change

    def handle_idle(self):
        """Idle - maintain current position"""
        pass  # Targets don't change

    # ========== Utilities ==========

    def publish_targets(self):
        """Publish target positions to Gazebo"""
        # Clamp to limits
        self.target_left = max(self.MIN_POS, min(self.MAX_POS, self.target_left))
        self.target_right = max(self.MIN_POS, min(self.MAX_POS, self.target_right))
        
        # Publish
        msg = Float64MultiArray()
        msg.data = [self.target_left, self.target_right]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Gripper controller stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()