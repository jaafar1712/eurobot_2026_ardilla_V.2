"""
ROS2 Launch File for ArUco Crate Perception Node
Eurobot 2026 "Winter is Coming"

Usage:
    ros2 launch eurobot_perception aruco_crate_perception.launch.py
    
    # With custom team color:
    ros2 launch eurobot_perception aruco_crate_perception.launch.py team_color:=blue
    
    # Without visualization (performance mode):
    ros2 launch eurobot_perception aruco_crate_perception.launch.py visualize:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for ArUco crate perception"""
    
    # Declare launch arguments
    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value='yellow',
        description='Team color: blue or yellow'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable debug visualization'
    )
    
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='DICT_4X4_50',
        description='ArUco dictionary type (verify with organizers!)'
    )
    
    min_confidence_arg = DeclareLaunchArgument(
        'min_confidence',
        default_value='0.35',
        description='Minimum detection confidence threshold'
    )
    
    # Path to parameter file
    # Adjust package name to match your package
    param_file = PathJoinSubstitution([
        FindPackageShare('eurobot_perception'),  # Change to your package name
        'config',
        'aruco_crate_perception_params.yaml'
    ])
    
    # ArUco crate perception node
    aruco_perception_node = Node(
        package='eurobot_perception',  # Change to your package name
        executable='aruco_crate_perception',
        name='aruco_crate_perception',
        output='screen',
        parameters=[
            param_file,
            {
                'team_color': LaunchConfiguration('team_color'),
                'visualize': LaunchConfiguration('visualize'),
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'min_confidence': LaunchConfiguration('min_confidence'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
            # e.g., ('/camera', '/front_camera/image_raw'),
        ]
    )
    
    # Optional: Image view for debugging
    # Uncomment to auto-launch image viewer
    # image_view_node = Node(
    #     package='rqt_image_view',
    #     executable='rqt_image_view',
    #     name='crate_debug_viewer',
    #     arguments=['/crate/image_debug']
    # )
    
    return LaunchDescription([
        # Launch arguments
        team_color_arg,
        visualize_arg,
        aruco_dict_arg,
        min_confidence_arg,
        
        # Nodes
        aruco_perception_node,
        # image_view_node,  # Uncomment to enable
    ])


# Alternative: Simplified launch without parameter file
def generate_launch_description_simple():
    """
    Simplified launch with all parameters inline
    Use this if you don't want a separate YAML file
    """
    
    aruco_perception_node = Node(
        package='eurobot_perception',
        executable='aruco_crate_perception.py',
        name='aruco_crate_perception',
        output='screen',
        parameters=[{
            # Camera
            'camera_topic': '/camera',
            'camera_info_topic': '/camera_info',
            
            # Team
            'team_color': 'yellow',
            
            # ArUco
            'aruco_dict': 'DICT_4X4_50',
            'marker_size': 0.040,
            
            # Detection
            'min_confidence': 0.35,
            'max_detection_distance': 3.0,
            
            # Crate dimensions
            'crate_length': 0.15,
            'crate_width': 0.05,
            'crate_height': 0.03,
            
            # Frames
            'camera_frame': 'camera_link',
            'base_frame': 'base_link',
            'global_frame': 'odom',
            
            # Visualization
            'visualize': True,
        }]
    )
    
    return LaunchDescription([aruco_perception_node])