import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import yaml


def load_joint_name_config():
    config_path = os.path.join(
        get_package_share_directory("delta_robot"),
        "config",
        "joint_names.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def generate_launch_description():
    joint_name_config = load_joint_name_config()
    controller_joint_names = joint_name_config.get("controller_joint_names", [])

    return LaunchDescription(
        [
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
                parameters=[
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("delta_robot"),
                            "config",
                            "delta_config.yaml",
                        ]
                    ),
                    {"controller_joint_names": controller_joint_names},
                ],
            ),
            Node(
                package="delta_robot_drivers",
                executable="motor_control_node",
                name="delta_motor_control",
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
            # Node(
            #     package="delta_robot",
            #     executable="range_scanner",
            #     name="range_scanner",
            #     output="screen",
            # ),
            # IncludeLaunchDescription(
            #     AnyLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [
            #                 get_package_share_directory("delta_robot_sensors"),
            #                 "launch",
            #                 "sensors.launch.xml",
            #             ]
            #         )
            #     )
            # ),
        ]
    )
