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
    maintainer="MCR2 Student",
    maintainer_email="student@mcr2.com",
    description="MCR2 Mini Challenge 1 – Square path open-loop controller",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "square_controller_node = square_controller.square_controller_node:main",
            "path_generator_node    = square_controller.path_generator_node:main",
        ],
    },
)
