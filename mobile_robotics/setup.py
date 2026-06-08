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
    maintainer="Mati",
    maintainer_email="A01772503@tec.mx",
    description="Bug 0 reactive navigation and odometry for Puzzlebot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry_node = mobile_robotics.odometry_node:main",
            "bug0_node = mobile_robotics.bug0_node:main",
        ],
    },
)
