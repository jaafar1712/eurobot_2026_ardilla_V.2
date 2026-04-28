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
import time

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
        self.declare_parameter('team_color', 'yellow')  # 'blue' or 'yellow'
        self.declare_parameter('match_duration', 100.0)  # seconds
        self.declare_parameter('nest_return_time', 10.0)  # seconds before end to return
        self.declare_parameter('max_crates_to_collect', 6)  # nest capacity
        self.declare_parameter('alignment_distance', 0.15)  # meters in front of crate
        self.declare_parameter('alignment_tolerance', 0.05)  # meters
        self.declare_parameter('angle_tolerance', 10.0)  # degrees
        
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
        self.current_pantry = None
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
        self.nav_timeout = 30.0  # 30 second timeout for navigation
        
        # ========== SUBSCRIBERS ==========
        self.create_subscription(
            CrateDetectionArray, 
            '/crate/detections', 
            self.crate_callback, 
            10
        )
        self.create_subscription(
            PantryDetectionArray,
            '/pantry/detections',
            self.pantry_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.create_subscription(
            String,
            '/gripper/state',
            self.gripper_state_callback,
            10
        )
        
        # ========== PUBLISHERS ==========
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_cmd_pub = self.create_publisher(String, '/gripper/command', 10)
        self.state_pub = self.create_publisher(String, '/task_manager/state', 10)
        
        # ========== ACTION CLIENTS ==========
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ========== TIMERS ==========
        self.main_timer = self.create_timer(0.2, self.main_loop)  # 5 Hz
        self.state_publish_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info(f"Task Manager initialized - Team: {self.team_color}")
        self.get_logger().info("Waiting for navigation action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Navigation server ready!")
        
    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def crate_callback(self, msg):
        """Store latest crate detections"""
        self.latest_crates = [det for det in msg.detections if det.color == self.team_color]
    
    def pantry_callback(self, msg):
        """Store latest pantry detections"""
        self.latest_pantries = msg.detections
    
    def odom_callback(self, msg):
        """Track robot position and orientation"""
        self.current_position = msg.pose.pose.position
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Capture home position on first callback
        if self.home_position is None:
            self.home_position = {
                'x': self.current_position.x,
                'y': self.current_position.y,
                'z': self.current_position.z
            }
            self.get_logger().info(
                f"Home position set: ({self.home_position['x']:.2f}, "
                f"{self.home_position['y']:.2f})"
            )
    
    def gripper_state_callback(self, msg):
        """Track gripper state"""
        self.gripper_state = msg.data
    
    # ========================================================================
    # MAIN STATE MACHINE
    # ========================================================================
    
    def main_loop(self):
        """Main control loop - runs at 5 Hz"""
        
        # Check navigation timeout
        if self.nav_goal_active and self.nav_goal_sent_time is not None:
            elapsed = (self.get_clock().now() - self.nav_goal_sent_time).nanoseconds / 1e9
            if elapsed > self.nav_timeout:
                self.get_logger().warn(f"⏱️ Navigation timeout ({elapsed:.1f}s)!")
                self.nav_goal_active = False
                self.nav_goal_result = False
            elif elapsed > 5.0:  # Log progress every 5s
                self.get_logger().info(
                    f"Navigation in progress... {elapsed:.1f}s elapsed",
                    throttle_duration_sec=5.0
                )
        
        # Check match time
        if self.match_start_time is not None:
            elapsed = (self.get_clock().now() - self.match_start_time).nanoseconds / 1e9
            time_remaining = self.match_duration - elapsed
            
            # Log time remaining periodically
            if time_remaining > 0:
                self.get_logger().info(
                    f"⏱️ Match time: {time_remaining:.1f}s remaining",
                    throttle_duration_sec=10.0
                )
            
            # Return to nest if time is running out
            if time_remaining <= self.nest_return_time and self.state != self.STATE_RETURNING_TO_NEST and self.state != self.STATE_FINISHED:
                self.get_logger().warn(f"⏰ {time_remaining:.1f}s remaining - returning to nest!")
                self.transition_to(self.STATE_RETURNING_TO_NEST)
            
            if elapsed >= self.match_duration:
                self.transition_to(self.STATE_FINISHED)
        
        # Execute current state
        if self.state == self.STATE_INITIALIZING:
            self.handle_initializing()
        elif self.state == self.STATE_SEARCHING_CRATE:
            self.handle_searching_crate()
        elif self.state == self.STATE_NAVIGATING_TO_CRATE:
            self.handle_navigating_to_crate()
        elif self.state == self.STATE_ALIGNING_WITH_CRATE:
            self.handle_aligning_with_crate()
        elif self.state == self.STATE_GRIPPING_CRATE:
            self.handle_gripping_crate()
        elif self.state == self.STATE_NAVIGATING_TO_DROPOFF:
            self.handle_navigating_to_dropoff()
        elif self.state == self.STATE_RELEASING_CRATE:
            self.handle_releasing_crate()
        elif self.state == self.STATE_RETURNING_TO_NEST:
            self.handle_returning_to_nest()
        elif self.state == self.STATE_FINISHED:
            self.handle_finished()
    
    # ========================================================================
    # STATE HANDLERS
    # ========================================================================
    
    def handle_initializing(self):
        """Initialize and start the match"""
        if self.home_position is None:
            return  # Wait for odometry
        
        # Start match timer
        self.match_start_time = self.get_clock().now()
        self.get_logger().info("🚀 Match started! Looking for crates...")
        
        # Open gripper at start
        self.send_gripper_command('open')
        
        self.transition_to(self.STATE_SEARCHING_CRATE)
    
    def handle_searching_crate(self):
        """Search for crates of team color"""
        
        # Check if we've collected enough crates
        if self.crates_collected >= self.max_crates:
            self.get_logger().info("✅ Maximum crates collected! Returning to nest.")
            self.transition_to(self.STATE_RETURNING_TO_NEST)
            return
        
        if not self.latest_crates:
            # No crates detected - rotate to search
            self.rotate_in_place(0.3)  # slow rotation
            return
        
        # Select nearest crate
        self.current_crate = min(self.latest_crates, key=lambda c: c.distance)
        
        self.get_logger().info(
            f"🎯 Target crate found at ({self.current_crate.x:.2f}, "
            f"{self.current_crate.y:.2f}), distance: {self.current_crate.distance:.2f}m"
        )
        
        self.transition_to(self.STATE_NAVIGATING_TO_CRATE)
    
    def handle_navigating_to_crate(self):
        """Navigate close to the crate"""
        
        if self.current_crate is None:
            self.transition_to(self.STATE_SEARCHING_CRATE)
            return
        
        # Calculate approach position in map frame
        # Crate x,y are in robot's local frame - need to transform to map
        crate_distance = self.current_crate.distance - self.alignment_dist
        crate_angle_rad = math.radians(self.current_crate.angle)
        
        # Transform to map frame using current robot pose
        approach_x = self.current_position.x + crate_distance * math.cos(self.current_yaw + crate_angle_rad)
        approach_y = self.current_position.y + crate_distance * math.sin(self.current_yaw + crate_angle_rad)
        
        # Calculate desired orientation (face the crate)
        approach_yaw = self.current_yaw + crate_angle_rad
        
        # Send navigation goal if not already sent
        if not self.nav_goal_active:
            self.get_logger().info(f"Navigating to ({approach_x:.2f}, {approach_y:.2f}), yaw={math.degrees(approach_yaw):.1f}°")
            self.send_nav_goal(approach_x, approach_y, approach_yaw)
            return
        
        # Check if navigation completed
        if self.nav_goal_result is not None:
            if self.nav_goal_result:
                self.get_logger().info("✅ Reached crate vicinity")
                self.transition_to(self.STATE_ALIGNING_WITH_CRATE)
            else:
                self.get_logger().warn("❌ Navigation to crate failed - searching again")
                self.current_crate = None
                self.transition_to(self.STATE_SEARCHING_CRATE)
    
    def handle_aligning_with_crate(self):
        """Fine alignment with crate using visual servoing"""
        
        # Find current crate in latest detections
        if not self.latest_crates:
            self.get_logger().warn("⚠️ Lost sight of crate during alignment")
            self.transition_to(self.STATE_SEARCHING_CRATE)
            return
        
        # Get closest crate (should be our target)
        crate = min(self.latest_crates, key=lambda c: c.distance)
        
        # Check alignment
        distance_error = abs(crate.distance - self.alignment_dist)
        angle_error = abs(crate.angle)
        
        self.get_logger().info(
            f"Aligning: dist_err={distance_error:.3f}m, ang_err={angle_error:.1f}°",
            throttle_duration_sec=2.0
        )
        
        # If well aligned, proceed to grip
        if distance_error < self.alignment_tol and angle_error < self.angle_tol:
            self.stop_robot()
            self.get_logger().info("✅ Aligned with crate!")
            self.transition_to(self.STATE_GRIPPING_CRATE)
            return
        
        # Visual servoing control
        cmd = Twist()
        
        # Angular control (turn to face crate)
        if angle_error > self.angle_tol:
            cmd.angular.z = 0.3 if crate.angle > 0 else -0.3
        
        # Linear control (move forward/backward)
        if distance_error > self.alignment_tol:
            if crate.distance > self.alignment_dist:
                cmd.linear.x = 0.1  # move forward
            else:
                cmd.linear.x = -0.1  # move backward
        
        self.cmd_vel_pub.publish(cmd)
    
    def handle_gripping_crate(self):
        """Close gripper to grab crate"""
        
        if self.gripper_state == "idle" or self.gripper_state == "opening":
            # Start closing
            self.send_gripper_command('close')
            return
        
        elif self.gripper_state == "closing":
            # Wait for gripper to finish
            return
        
        elif self.gripper_state == "gripping":
            # Successfully gripped!
            self.get_logger().info("✅ Crate gripped!")
            self.crates_collected += 1
            self.get_logger().info(f"📦 Crates collected: {self.crates_collected}/{self.max_crates}")
            self.transition_to(self.STATE_NAVIGATING_TO_DROPOFF)
        
        else:
            # Gripper failed to grip (fully closed but no contact)
            self.get_logger().warn("❌ Failed to grip crate - searching again")
            self.send_gripper_command('open')
            self.current_crate = None
            self.transition_to(self.STATE_SEARCHING_CRATE)
    
    def handle_navigating_to_dropoff(self):
        """Navigate to pantry or nest for dropoff"""
        
        # Determine dropoff location
        dropoff_x, dropoff_y, dropoff_yaw = None, None, 0.0
        dropoff_name = "NEST"
        
        # Prefer pantry if detected (excluding home pantry)
        valid_pantries = [p for p in self.latest_pantries if p.id != -1 and p.distance > 0.3]
        
        if valid_pantries:
            # Choose nearest pantry
            pantry = min(valid_pantries, key=lambda p: p.distance)
            
            # Transform pantry position from robot frame to map frame
            pantry_angle_rad = math.radians(pantry.angle)
            dropoff_x = self.current_position.x + pantry.distance * math.cos(self.current_yaw + pantry_angle_rad)
            dropoff_y = self.current_position.y + pantry.distance * math.sin(self.current_yaw + pantry_angle_rad)
            dropoff_yaw = self.current_yaw + pantry_angle_rad
            dropoff_name = f"PANTRY {pantry.id}"
        else:
            # Default to nest
            dropoff_x = self.home_position['x']
            dropoff_y = self.home_position['y']
            dropoff_yaw = 0.0
        
        # Send navigation goal
        if not self.nav_goal_active:
            self.get_logger().info(f"🎯 Navigating to {dropoff_name} at ({dropoff_x:.2f}, {dropoff_y:.2f})")
            self.send_nav_goal(dropoff_x, dropoff_y, dropoff_yaw)
            return
        
        # Check navigation result
        if self.nav_goal_result is not None:
            if self.nav_goal_result:
                self.get_logger().info(f"✅ Reached {dropoff_name}")
                self.transition_to(self.STATE_RELEASING_CRATE)
            else:
                self.get_logger().warn(f"❌ Failed to reach {dropoff_name} - going to nest")
                # Fallback to nest
                if dropoff_name != "NEST":
                    self.latest_pantries = []  # Clear pantries
                    self.nav_goal_active = False
                    self.nav_goal_result = None
                else:
                    # Even nest failed - just release here
                    self.transition_to(self.STATE_RELEASING_CRATE)
    
    def handle_releasing_crate(self):
        """Release the crate"""
        
        if self.gripper_state == "gripping" or self.gripper_state == "closing":
            # Start opening
            self.send_gripper_command('open')
            return
        
        elif self.gripper_state == "opening":
            # Wait for gripper to open
            return
        
        elif self.gripper_state == "idle":
            # Released!
            self.get_logger().info("✅ Crate released!")
            
            # Decide next action
            if self.crates_collected >= self.max_crates:
                self.transition_to(self.STATE_RETURNING_TO_NEST)
            else:
                self.transition_to(self.STATE_SEARCHING_CRATE)
    
    def handle_returning_to_nest(self):
        """Return to nest for end of match"""
        
        if self.home_position is None:
            self.get_logger().error("❌ No home position!")
            self.transition_to(self.STATE_FINISHED)
            return
        
        # Send navigation goal to home
        if not self.nav_goal_active:
            self.get_logger().info("🏠 Returning to nest...")
            self.send_nav_goal(
                self.home_position['x'],
                self.home_position['y'],
                0.0
            )
            return
        
        # Check result
        if self.nav_goal_result is not None:
            if self.nav_goal_result:
                self.get_logger().info("✅ Safely in nest!")
            else:
                self.get_logger().warn("⚠️ Failed to reach nest")
            
            self.transition_to(self.STATE_FINISHED)
    
    def handle_finished(self):
        """Match finished"""
        self.stop_robot()
        self.get_logger().info(
            f"🏁 Match finished! Total crates collected: {self.crates_collected}",
            throttle_duration_sec=5.0
        )
    
    # ========================================================================
    # HELPER FUNCTIONS
    # ========================================================================
    
    def transition_to(self, new_state):
        """Transition to a new state"""
        if new_state != self.state:
            self.get_logger().info(f"State: {self.state} → {new_state}")
            self.state = new_state
            
            # Reset navigation flags on state change
            self.nav_goal_active = False
            self.nav_goal_result = None
            self.nav_goal_sent_time = None
    
    def send_nav_goal(self, x, y, yaw):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        self.nav_goal_active = True
        self.nav_goal_result = None
        self.nav_goal_sent_time = self.get_clock().now()
        
        self.get_logger().info(f"📍 Sending nav goal: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Navigation goal REJECTED by server")
            self.nav_goal_result = False
            self.nav_goal_active = False
            return
        
        self.get_logger().info("✓ Navigation goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        self.nav_goal_active = False
        
        # Status codes: https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
        # STATUS_SUCCEEDED = 4
        # STATUS_ABORTED = 6
        # STATUS_CANCELED = 5
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info("✅ Navigation SUCCEEDED")
            self.nav_goal_result = True
        else:
            self.get_logger().warn(f"❌ Navigation FAILED (status={status})")
            self.nav_goal_result = False
    
    def send_gripper_command(self, command):
        """Send command to gripper"""
        msg = String()
        msg.data = command
        self.gripper_cmd_pub.publish(msg)
    
    def stop_robot(self):
        """Stop robot motion"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def rotate_in_place(self, angular_vel):
        """Rotate robot in place"""
        cmd = Twist()
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
    
    def publish_state(self):
        """Publish current state for monitoring"""
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