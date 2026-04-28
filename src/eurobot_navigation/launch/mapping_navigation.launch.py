from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    eurobot_nav_share = get_package_share_directory("eurobot_navigation")
    mam_share = get_package_share_directory("mam_eurobot_2026")

    robot_description_path = os.path.join(mam_share, "models", "simple_robot", "simple_robot.urdf")
    nav2_params_path = os.path.join(eurobot_nav_share, "config", "nav2_params.yaml")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": open(robot_description_path).read()},
            {"use_sim_time": True}
        ],
        output="screen"
    )

    # SLAM Toolbox (online async)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_nav_share, "launch", "online_async_launch.py")
        )
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params_path
        }.items()
    )

    return LaunchDescription([
        robot_state_publisher_node,
        slam_launch,
        nav2_launch
    ])
