#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='odom',  # Changed default to 'odom'
        description='Global frame for navigation (map or odom)'
    )
    
    # Get launch configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    global_frame = LaunchConfiguration('global_frame')
    
    # Crate perception node with parameters
    crate_node = Node(
        package='eurobot_perception',
        executable='crate_perception',
        name='crate_perception',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': global_frame,
            'camera_frame': 'camera_link',
            'base_frame': 'base_link',
            'focal_length_px': 277.0,
            'known_crate_width': 0.05,
            'camera_center_offset': 0.10,
            'approach_distance': 0.30,
        }]
    )
    
    # Pantry perception node with parameters
    pantry_node = Node(
        package='eurobot_perception',
        executable='pantry_detection',
        name='pantry_detection',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': global_frame,
            'camera_frame': 'camera_link',
            'base_frame': 'base_link',
            'focal_length_px': 277.0,
            'known_pantry_width': 0.20,
            'camera_center_offset': 0.10,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        global_frame_arg,
        crate_node,
        pantry_node
    ])