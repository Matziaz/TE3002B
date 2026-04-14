from setuptools import setup
import os
from glob import glob

package_name = "square_controller"

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
    maintainer="Mati",
    maintainer_email="A01772503@tec.mx",
    description="MCR Activities 1-3: speed estimation, odometry, and go-to-goal for Puzzlebot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speed_estimator_node = square_controller.speed_estimator_node:main",
            "odometry_node = square_controller.odometry_node:main",
            "go_to_goal_node = square_controller.go_to_goal_node:main",
        ],
    },
)
