from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'minichallenge_4'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'profiles'), glob('config/profiles/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LosFELM',
    maintainer_email='noreply@example.com',
    description='Minichallenge 4: Traffic light navigation with color detection and robust state handling.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = minichallenge_4.odometry_node:main',
            'color_detector_node = minichallenge_4.color_detector_node:main',
            'traffic_light_controller = minichallenge_4.traffic_light_controller:main',
            'go_to_goal_node = minichallenge_4.go_to_goal_node:main',
        ],
    },
)
