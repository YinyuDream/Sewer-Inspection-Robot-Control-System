from glob import glob
from setuptools import setup

package_name = "four_wheel_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
    ("share/" + package_name + "/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Maintainer Name",
    maintainer_email="maintainer@example.com",
    description="Four wheel mobile robot simulation with ROS 2 Humble, Gazebo Fortress, and RViz2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = four_wheel_robot.keyboard_teleop:main",
        ],
    },
)
