from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_gcode = LaunchConfiguration("run_gcode")
    run_json = LaunchConfiguration("run_json")
    gcode_file = LaunchConfiguration("gcode_file")
    task_file = LaunchConfiguration("task_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_gcode",
                default_value="false",
                description="Run G-code parser node",
            ),
            DeclareLaunchArgument(
                "run_json",
                default_value="false",
                description="Run JSON task sequencer node",
            ),
            DeclareLaunchArgument(
                "gcode_file",
                default_value="",
                description="Path to input .gcode file",
            ),
            DeclareLaunchArgument(
                "task_file",
                default_value="",
                description="Path to input task .json file",
            ),
            Node(
                package="delta_robot",
                executable="gcode_parser.py",
                name="gcode_parser",
                output="screen",
                arguments=[gcode_file],
                condition=IfCondition(run_gcode),
            ),
            Node(
                package="delta_robot",
                executable="json_task_sequencer.py",
                name="json_task_sequencer",
                output="screen",
                arguments=[task_file],
                condition=IfCondition(run_json),
            ),
        ]
    )
