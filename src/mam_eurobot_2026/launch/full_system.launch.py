from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('mam_eurobot_2026')
    eurobot_nav_dir = get_package_share_directory('eurobot_navigation')
    eurobot_perception_dir = get_package_share_directory('eurobot_perception')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    models_path = os.path.join(pkg_share, 'models')
    urdf_path = os.path.join(pkg_share, 'models', 'simple_robot', 'simple_robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'arena_world.sdf')
    gripper_yaml = os.path.join(pkg_share, 'models', 'simple_robot', 'gripper_controller.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'arena.rviz')

    # --- Environment ---
    set_resource_path    = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path)
    set_gripper_yaml     = SetEnvironmentVariable('GRIPPER_CONTROLLER_YAML', gripper_yaml)
    set_libgl_sw         = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_gallium_driver   = SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe')
    set_mesa_override    = SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe')

    # --- Simulation (t=0) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': open(urdf_path).read()},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    navigation_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_nav_dir, 'launch', 'bridges.launch.py')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- Controllers (t=5s, after Gazebo is up) ---
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    # --- Navigation (t=5s) ---
    scan_frame_fixer = Node(
        package='eurobot_navigation',
        executable='scan_frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # SLAM (t=7s)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_nav_dir, 'launch', 'online_async_launch.py')
        )
    )

    # Nav2 (t=10s)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(eurobot_nav_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Teleop (t=13s)
    teleop = Node(
        package='eurobot_navigation',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen'
    )

    # ArUco crate perception (t=5s, after camera bridge is up)
    aruco_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_perception_dir, 'launch', 'aruco_crate_perception.launch.py')
        )
    )

    # Camera debug view showing ArUco detections (t=6s, after perception node is up)
    image_view = Node(
        package='image_view',
        executable='image_view',
        name='aruco_debug_view',
        remappings=[('image', '/crate/debug')],
        output='screen'
    )

    return LaunchDescription([
        # Environment
        set_resource_path,
        set_gripper_yaml,
        set_libgl_sw,
        set_gallium_driver,
        set_mesa_override,

        # t=0: Simulation
        robot_state_publisher,
        gazebo,
        navigation_bridges,
        rviz,

        # t=5s: Controllers + scan_frame_fixer
        TimerAction(period=5.0, actions=[
            joint_state_broadcaster,
            gripper_controller,
            scan_frame_fixer,
        ]),

        # t=7s: SLAM
        TimerAction(period=7.0, actions=[slam]),

        # t=10s: Nav2
        TimerAction(period=10.0, actions=[nav2]),

        # t=13s: Teleop
        TimerAction(period=13.0, actions=[teleop]),

        # t=5s: ArUco perception
        TimerAction(period=5.0, actions=[aruco_perception]),

        # t=6s: Debug image view (/crate/debug)
        TimerAction(period=6.0, actions=[image_view]),
    ])
