from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    move_to_point_service = LaunchConfiguration("move_to_point_service")
    set_motion_mode_service = LaunchConfiguration("set_motion_mode_service")
    live_target_topic = LaunchConfiguration("live_target_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "move_to_point_service",
                default_value="delta_motion_planner/move_to_point",
                description="Service used to command Cartesian targets",
            ),
            DeclareLaunchArgument(
                "set_motion_mode_service",
                default_value="delta_motion_planner/set_motion_mode",
                description="Service used to switch between task and live modes",
            ),
            DeclareLaunchArgument(
                "live_target_topic",
                default_value="delta_motion_planner/live_target",
                description="Topic used for live Cartesian targets",
            ),
            Node(
                package="delta_robot_gui",
                executable="delta_robot_gui",
                name="delta_robot_gui",
                output="screen",
                parameters=[
                    {"move_to_point_service": move_to_point_service},
                    {"set_motion_mode_service": set_motion_mode_service},
                    {"live_target_topic": live_target_topic},
                ],
            ),
        ]
    )
