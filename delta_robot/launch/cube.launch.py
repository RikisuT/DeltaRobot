from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="delta_robot_drivers",
                executable="ee_tf_broadcaster",
                name="ee_tf_broadcaster",
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
                package="delta_robot_visualization",
                executable="plot_ee_tf.py",
                name="plot_ee_tf",
                output="screen",
                arguments=[
                    "--parent-frame",
                    "delta_robot/world_link",
                    "--commanded",
                    "delta_robot/commanded_end_effector_pin",
                    "--ee",
                    "ee_link",
                    "--calculated",
                    "delta_robot/calculated_fk_end_effector_pin",
                    "--actual",
                    "delta_robot/actual_fk_end_effector_pin",
                ],
            ),
        ]
    )
