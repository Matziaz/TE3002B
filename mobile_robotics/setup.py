from setuptools import setup
import os
from glob import glob

package_name = "mobile_robotics"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liz",
    maintainer_email="A01711740@tec.mx",
    description="Mobile Robotics",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "encoder_simulator = mobile_robotics.encoder_simulator:main",
            "odometry_speed_estimator = mobile_robotics.odometry_speed_estimator:main",
            "go_to_goal_controller = mobile_robotics.go_to_goal_controller:main",
        ],
    },
)
