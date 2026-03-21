import os
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Add these imports at the top
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # --- 1. Setup Paths ---
    delta_robot_description_path = get_package_share_directory(
        "delta_robot_description"
    )
    delta_robot_sim_path = get_package_share_directory("delta_robot_sim")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(delta_robot_description_path).parent.resolve()),
    )

    # Gazebo plugin path
    gazebo_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value="/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/",
    )

    use_sim_feedback_arg = DeclareLaunchArgument(
        "use_sim_feedback",
        default_value="true",
        description="Set false when real motors are running to avoid feedback conflict",
    )

    rsp_log_level_arg = DeclareLaunchArgument(
        "rsp_log_level",
        default_value="error",
        description="Log level for robot_state_publisher",
    )

    # Load SDF for Gazebo simulation
    sdf_file = os.path.join(delta_robot_description_path, "models", "model.sdf")
    if not os.path.exists(sdf_file):
        raise FileNotFoundError(f"Missing robot model SDF: {sdf_file}")
    with open(sdf_file, "r") as infp:
        robot_desc_sdf = infp.read()

    # Load Box SDF
    box_sdf_file = os.path.join(delta_robot_description_path, "models", "box.sdf")
    if not os.path.exists(box_sdf_file):
        raise FileNotFoundError(f"Missing box SDF: {box_sdf_file}")
    with open(box_sdf_file, "r") as infp:
        infp.read()

    # Pre-process SDF to replace explicit launch-time placeholder with actual path
    placeholder = "__DELTA_ROBOT_SIM_SHARE__"
    if placeholder not in robot_desc_sdf:
        raise RuntimeError(
            "Expected placeholder '__DELTA_ROBOT_SIM_SHARE__' not found in model.sdf"
        )
    robot_desc_sdf = robot_desc_sdf.replace(placeholder, delta_robot_sim_path)

    # Setup World
    world_file = os.path.join(delta_robot_sim_path, "worlds", "empty.sdf")
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"Missing simulation world: {world_file}")

    # Path to RViz config (we will create this in Step 2)

    # --- 2. Launch Gazebo ---
    gz_gui_config = os.path.join(delta_robot_sim_path, "config", "config.config")
    if not os.path.exists(gz_gui_config):
        raise FileNotFoundError(f"Missing Gazebo GUI config: {gz_gui_config}")

    bridge_config = os.path.join(delta_robot_sim_path, "config", "ros_gz_bridge.yaml")
    if not os.path.exists(bridge_config):
        raise FileNotFoundError(f"Missing ROS-Gazebo bridge config: {bridge_config}")

    rviz_config = os.path.join(delta_robot_sim_path, "config", "delta_robot.rviz")
    if not os.path.exists(rviz_config):
        raise FileNotFoundError(f"Missing RViz config: {rviz_config}")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 0 --gui-config {gz_gui_config} {world_file}"
        }.items(),
    )

    # --- 3. Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("rsp_log_level"),
        ],
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_desc_sdf},
        ],
        remappings=[
            ("/tf", "/tf_rsp_ignore"),
            ("/tf_static", "/tf_static_rsp_ignore"),
        ],
    )

    # --- 4. Spawn Entity ---
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-string",
            robot_desc_sdf,
            "-name",
            "delta_robot",
            "-allow_renaming",
            "false",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.50",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            str(math.pi),
        ],
    )

    # --- 5. Bridge ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "config_file": bridge_config,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="log",
    )

    # --- Foxglove Bridge ---
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{"port": 8765}],
        output="log",
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # --- 6. RViz2 ---
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
            "--ros-args",
            "--log-level",
            "error",
        ],
        parameters=[{"use_sim_time": False}],
    )

    # --- 7. ROS2 Control Spawners ---
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--ros-args",
            "--log-level",
            "warn",
        ],
        output="log",
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--ros-args",
            "--log-level",
            "warn",
        ],
        output="log",
    )
    load_joint_feedback = Node(
        package="delta_robot",
        executable="joint_state_bridge.py",
        name="joint_state_bridge",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_sim_feedback")),  # ← add this
    )

    plotter3d = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="delta_robot_sim",
                executable="plotter3d.py",
                name="delta_ee_plotter",
                output="log",
            )
        ],
    )

    return LaunchDescription(
        [
            use_sim_feedback_arg,
            rsp_log_level_arg,
            gazebo_resource_path,
            gazebo_plugin_path,
            gz_sim,
            gz_spawn_entity,
            bridge,
            robot_state_publisher,
            rviz2,
            load_joint_state_broadcaster,
            load_joint_trajectory_controller,
            load_joint_feedback,
            plotter3d,
        ]
    )
