from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'target_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'target_nav': ['gui/*.png'],
    },
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
            glob('config/*.yaml')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        # Scripts (shell and python)
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh') + glob('scripts/*.py') + glob('scripts/*.service')),
        (os.path.join('share', package_name, 'scripts', 'setup'),
            glob('scripts/setup/*.sh')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Vision-guided target navigation for RZ/V2N mecanum robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_gui = target_nav.app.main:main',
            'arduino_driver_node = target_nav.app.arduino_node:main',
            'odometry_node = target_nav.app.odometry_node:main',
        ],
    },
)
