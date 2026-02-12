from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bowling_target_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.lua') + glob('config/*.rviz')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        # Map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
        # Models
        (os.path.join('share', package_name, 'models'),
            glob('models/*.onnx')),
        # Scripts (shell and python)
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh') + glob('scripts/*.py') + glob('scripts/*.service')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Bowling Target Navigation for RZ/V2N Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_gui = bowling_target_nav.nodes.main_gui:main',
            'vision_node = bowling_target_nav.nodes.vision_node:main',
            'arduino_driver_node = bowling_target_nav.nodes.arduino_driver_node:main',
            'target_follower_node = bowling_target_nav.nodes.target_follower_node:main',
            'target_gui_node = bowling_target_nav.nodes.target_gui_node:main',
            'odometry_node = bowling_target_nav.nodes.odometry_node:main',
            'map_viewer_node = bowling_target_nav.nodes.map_viewer_node:main',
            'slam_camera_gui = bowling_target_nav.nodes.slam_camera_gui:main',
            'autonomous_explorer = bowling_target_nav.nodes.autonomous_explorer:main',
        ],
    },
)
