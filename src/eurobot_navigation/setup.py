from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'eurobot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # <-- add this
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'slam_node = eurobot_navigation.slam_node:main',
        'localization_node = eurobot_navigation.localization_node:main',
        'nav2_global_planner = eurobot_navigation.nav2_global_planner:main',
        'nav2_local_planner = eurobot_navigation.nav2_local_planner:main',
        'task_manager = eurobot_navigation.task_manager:main',
        'teleop_keyboard = eurobot_navigation.teleop_keyboard:main',
        'scan_frame_fixer=eurobot_navigation.scan_frame_fixer:main',
        'simple_crate_navigator = eurobot_navigation.simple_crate_navigator:main',
        'MissionController = eurobot_navigation.MissionController:main',
      
    ],
}
)

