from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mam_eurobot_2026'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'crate'), glob('models/crate/*.sdf') + glob('models/crate/*.config')),
        (os.path.join('share', package_name, 'models', 'crate', 'materials', 'textures'), glob('models/crate/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'crate_yellow'), glob('models/crate_yellow/*.sdf') + glob('models/crate_yellow/*.config')),
        (os.path.join('share', package_name, 'models', 'crate_yellow', 'materials', 'textures'), glob('models/crate_yellow/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'mat'), glob('models/mat/*')),
        (os.path.join('share', package_name, 'models', 'simple_robot'), glob('models/simple_robot/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Freddy Liendo',
    maintainer_email='liendomf@univ-smb.fr',
    description='Template package for Eurobot 2026',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            
        ],
    },
)