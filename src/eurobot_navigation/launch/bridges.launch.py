#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',

            # TF
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # Laser scan
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

            # Odometry
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

            # Cmd vel
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',

            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            #Camera
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            
            # Gripper contacts (Ignition → ROS)
            '/world/eurobot_2026_arena/model/simple_robot/link/left_finger/sensor/left_finger_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/eurobot_2026_arena/model/simple_robot/link/right_finger/sensor/right_finger_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',

            # Gripper joint velocity commands (ROS → Ignition, Fortress only supports cmd_vel)
            '/model/simple_robot/joint/left_finger_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/simple_robot/joint/right_finger_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',

            # Remap joint state topic
            '--ros-args',
            '-r', '/world/eurobot_2026_arena/model/simple_robot/joint_state:=/joint_states',
            '-r', '/world/eurobot_2026_arena/model/simple_robot/link/left_finger/sensor/left_finger_contact/contact:=/gripper/left/contact',
            '-r', '/world/eurobot_2026_arena/model/simple_robot/link/right_finger/sensor/right_finger_contact/contact:=/gripper/right/contact'
        ],
        parameters=[{
                'use_sim_time': True 
            }],
    )

    return LaunchDescription([bridge])
