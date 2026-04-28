from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    eurobot_nav_dir = get_package_share_directory('eurobot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')


    scan_frame_fixer_node = Node(
        package='eurobot_navigation',
        executable='scan_frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_nav_dir, 'launch', 'online_async_launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(
                eurobot_nav_dir,
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', '/root/.rviz2/eurobot.rviz'
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([

        # Step 1
        scan_frame_fixer_node,

        # Step 2
        TimerAction(
            period=2.0,
            actions=[online_async_launch]
        ),

        # Step 3
        TimerAction(
            period=5.0,
            actions=[nav2_launch]
        ),

        # Step 4 
        TimerAction(
            period=8.0,
            actions=[rviz_node]
        ),
    ])
