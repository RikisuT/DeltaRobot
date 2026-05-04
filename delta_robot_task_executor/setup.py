from setuptools import setup

package_name = "delta_robot_task_executor"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Likhithraj T Acharya",
    maintainer_email="likhiacharya@gmail.com",
    description="Task playback tools for G-code and JSON sequences.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "gcode_parser=delta_robot_task_executor.gcode_parser:main",
            "json_task_sequencer=delta_robot_task_executor.json_task_sequencer:main",
        ],
    },
)
