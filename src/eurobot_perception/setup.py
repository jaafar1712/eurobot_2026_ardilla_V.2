from setuptools import setup

package_name = 'eurobot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/launch', ['launch/aruco_crate_perception.launch.py']),
        ('share/' + package_name + '/config', ['config/aruco_crate_perception_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='Perception package for Eurobot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'crate_perception = eurobot_perception.crate_perception:main',
            'pantry_detection = eurobot_perception.pantry_detection:main',
            'crate_perception_V2 = eurobot_perception.crate_perception_V2:main',
            'rotation_tester = eurobot_perception.RotationTester:main',
            'aruco_crate_perception = eurobot_perception.aruco_crate_perception:main',    
        ],
    },
)
