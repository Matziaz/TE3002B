# setup.py tells ROS 2/ament how to install this Python package and which
# console commands should be created for each node.
from setuptools import find_packages, setup
import os
from glob import glob

# Package name used for installation paths and console entry points.
package_name = 'minichallenge_5'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    # Install package metadata, launch files, configuration files, and profiles
    # into the package share directory so ros2 launch can find them.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'profiles'), glob('config/profiles/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mati',
    maintainer_email='A01772503@tec.mx',
    description='Minichallenge 5: Traffic light navigation with color detection and robust state handling.',
    license='MIT',
    tests_require=['pytest'],
    # These console scripts allow each node to be launched by ROS 2 using the
    # executable names shown below.
    entry_points={
        'console_scripts': [
            'color_detector_node = minichallenge_5.color_detector_node:main',
            'traffic_light_controller = minichallenge_5.traffic_light_controller:main',
            'line_follower_node = minichallenge_5.line_follower_node:main',
        ],
    },
)
