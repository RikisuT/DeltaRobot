import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def _motor_actions(context):
    run_motor_control = (
        LaunchConfiguration("run_motor_control").perform(context).strip().lower()
    )
    motor_port = LaunchConfiguration("motor_port").perform(context)
    motor_mode = LaunchConfiguration("motor_mode").perform(context).strip().lower()

    if run_motor_control not in ("true", "1", "yes", "on"):
        return [
            LogInfo(
                msg="[delta_robot.launch.py] run_motor_control=false; skipping motor_control_node.py"
            )
        ]

    if motor_mode == "sim_only":
        return [
            LogInfo(
                msg="[delta_robot.launch.py] motor_mode=sim_only; skipping motor_control_node.py"
            )
        ]

    if motor_mode == "auto" and not os.path.exists(motor_port):
        return [
            LogInfo(
                msg=(
                    "[delta_robot.launch.py] No motor device found at "
                    + motor_port
                    + "; running sim-only (motor node skipped)"
                )
            )
        ]

    return [
        Node(
            package="delta_robot",
            executable="motor_control_node.py",
            name="delta_motor_control",
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_motor_control",
                default_value="false",
                description="Set true to start hardware motor_control_node.py",
            ),
            DeclareLaunchArgument(
                "motor_mode",
                default_value="auto",
                description="auto|real|sim_only",
            ),
            DeclareLaunchArgument(
                "motor_port",
                default_value="/dev/ttyUSB0",
                description="Serial device path for motor controller",
            ),
            DeclareLaunchArgument(
                "run_range_scanner",
                default_value="false",
                description="Set true to start range_scanner (requires range sensor topic)",
            ),
            DeclareLaunchArgument(
                "run_sensor_fusion",
                default_value="false",
                description="Set true to include delta_robot_sensors launch (ToF/IMU/Kalman)",
            ),
            Node(
                package="delta_robot",
                executable="kinematics",
                name="delta_kinematics",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("delta_robot"),
                            "config",
                            "delta_config.yaml",
                        ]
                    )
                ],
            ),
            Node(
                package="delta_robot",
                executable="motion_planner",
                name="motion_planner",
                output="screen",
            ),
            OpaqueFunction(function=_motor_actions),
            Node(
                package="delta_robot",
                executable="range_scanner",
                name="range_scanner",
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_range_scanner")),
            ),
            Node(
                package="delta_robot",
                executable="delta_trajectory_generator",
                name="trajectory_generator",
                output="screen",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("delta_robot_sensors"),
                            "launch",
                            "sensors.launch.xml",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("run_sensor_fusion")),
            ),
        ]
    )
