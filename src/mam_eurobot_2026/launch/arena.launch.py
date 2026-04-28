from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('mam_eurobot_2026')
    models_path = os.path.join(pkg_share, 'models')

    urdf_path = os.path.join(
        pkg_share,
        'models',
        'simple_robot',
        'simple_robot.urdf'
    )

    world_path = os.path.join(
        pkg_share,
        'worlds',
        'arena_world.sdf'
    )

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

    navigation_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('eurobot_navigation'),
                'launch',
                'bridges.launch.py'
            )
        )
    )

    rviz_config = os.path.join(pkg_share, 'config', 'arena.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    gripper_yaml = os.path.join(pkg_share, 'models', 'simple_robot', 'gripper_controller.yaml')

    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', models_path
    )
    set_gripper_yaml = SetEnvironmentVariable('GRIPPER_CONTROLLER_YAML', gripper_yaml)

    set_libgl_sw = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_gallium_driver = SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe')
    set_mesa_override = SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe')

    return LaunchDescription([
        set_resource_path,
        set_gripper_yaml,
        set_libgl_sw,
        set_gallium_driver,
        set_mesa_override,
        robot_state_publisher,
        gazebo,
        joint_state_broadcaster,
        gripper_controller,
        navigation_bridges,
        rviz,
    ])
