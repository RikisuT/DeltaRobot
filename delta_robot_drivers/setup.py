from setuptools import find_packages, setup

package_name = "delta_robot_drivers"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Likhithraj T Acharya",
    maintainer_email="likhiacharya@gmail.com",
    description="Python hardware and simulation driver nodes for the delta robot.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "motor_control_node = delta_robot_drivers.motor_control_node:main",
            "joint_state_bridge = delta_robot_drivers.joint_state_bridge:main",
            "ee_tf_broadcaster = delta_robot_drivers.ee_tf_broadcaster:main",
        ],
    },
)
