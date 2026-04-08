from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    move_to_point_service = LaunchConfiguration("move_to_point_service")
    move_to_pose_service = LaunchConfiguration("move_to_pose_service")
    set_motion_mode_service = LaunchConfiguration("set_motion_mode_service")
    live_target_topic = LaunchConfiguration("live_target_topic")
    play_demo_trajectory_service = LaunchConfiguration("play_demo_trajectory_service")
    motion_demo_service = LaunchConfiguration("motion_demo_service")
    qt_qpa_platform = LaunchConfiguration("qt_qpa_platform")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "move_to_point_service",
                default_value="delta_motion_planner/move_to_point",
                description="Service used to command Cartesian targets",
            ),
            DeclareLaunchArgument(
                "move_to_pose_service",
                default_value="delta_motion_planner/move_to_pose",
                description="Service used to command 5DOF Cartesian targets",
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
            DeclareLaunchArgument(
                "play_demo_trajectory_service",
                default_value="delta_motion_planner/play_demo_trajectory",
                description="Service used to play a single demo trajectory",
            ),
            DeclareLaunchArgument(
                "motion_demo_service",
                default_value="delta_motion_planner/motion_demo",
                description="Service used to start or stop the demo loop",
            ),
            DeclareLaunchArgument(
                "qt_qpa_platform",
                default_value="wayland",
                description="Qt platform plugin used by the GUI process",
            ),
            Node(
                package="delta_robot_gui",
                executable="delta_robot_gui",
                name="delta_robot_gui",
                output="screen",
                additional_env={"QT_QPA_PLATFORM": qt_qpa_platform},
                parameters=[
                    {"move_to_point_service": move_to_point_service},
                    {"move_to_pose_service": move_to_pose_service},
                    {"set_motion_mode_service": set_motion_mode_service},
                    {"live_target_topic": live_target_topic},
                    {"play_demo_trajectory_service": play_demo_trajectory_service},
                    {"motion_demo_service": motion_demo_service},
                ],
            ),
        ]
    )
