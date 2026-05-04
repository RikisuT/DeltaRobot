# `delta_robot` Package

Pure C++ core control package. All motion math lives in the `delta_math` header-only
library; the ROS nodes are thin service/action wrappers.

> Python driver nodes (motor control, joint state bridge, EE TF broadcaster) have
> been moved to the [`delta_robot_drivers`](../delta_robot_drivers/) package.

## Math Library

### `delta_math.hpp`

Header-only C++ library with **zero ROS dependency**, located at
`include/delta_robot/delta_math.hpp`. Contains:

- **FK / IK:** `forward_kinematics()`, `inverse_kinematics()`, `ik_angle_yz()`
- **Jacobian:** `calc_jacobian()`, `calc_aux_angles()`, `calc_theta_dot()`
- **Velocity:** `compute_gradient()` (finite-difference end-effector velocities)
- **Trajectories:** `straight_up_down_trajectory()`, `pringle_trajectory()`, `circle_trajectory()`, `axes_trajectory()`
- **I/O:** `read_csv()`, `random_sample_trajectory()`

All functions operate on lightweight POD structs (`Point3D`, `JointAngles`,
`JointVelocities`) and a `DeltaRobotParams` struct for robot geometry.

## Nodes

### `kinematics`

Provides forward and inverse kinematics as ROS 2 services. Accepts a config YAML for robot geometry (link lengths, joint limits). Also publishes `robot_config` (pose + joint angles) at a configurable rate.

**Services:** `delta_fk`, `delta_ik`, `convert_to_joint_trajectory`, `convert_to_joint_vel_trajectory`

### `motion_planner`

Top-level control node. Sends joint trajectory commands to both the physical motors and the Gazebo simulation simultaneously.

**Services:** `play_demo_trajectory`, `move_to_point`, `move_to_pose`, `move_to_configuration`, `motion_demo`, `play_custom_trajectory`, `set_motion_mode`

**Actions:** `execute_trajectory` (lightweight action for executing batched, long-running end-effector trajectories)

Built-in demos: `circle`, `pringle`, `axes`, `up_down`, `scan`

### `range_scanner`

3D scanning node. Moves the end-effector through the `scan` trajectory while recording (x, y, z, distance) data from the ToF sensor to map surfaces.

## G-code and JSON Usage

Prerequisite: start the core stack (`kinematics` + `motion_planner`) before running parser tools.

Run G-code:

```bash
ros2 run delta_robot_task_executor gcode_parser \
  $(ros2 pkg prefix delta_robot)/share/delta_robot/config/examples/circle_test.gcode
```

Smoother G-code execution (more interpolation steps):

```bash
ros2 run delta_robot_task_executor gcode_parser \
  $(ros2 pkg prefix delta_robot)/share/delta_robot/config/examples/circle_test.gcode \
  --ros-args -p motion_rate_hz:=100.0 -p units:=meters
```

Run JSON task sequence:

```bash
ros2 run delta_robot_task_executor json_task_sequencer \
  $(ros2 pkg prefix delta_robot)/share/delta_robot/config/examples/example_task.json
```

Smoother timed JSON moves:

```bash
ros2 run delta_robot_task_executor json_task_sequencer \
  $(ros2 pkg prefix delta_robot)/share/delta_robot/config/examples/example_task.json \
  --ros-args -p motion_rate_hz:=100.0 -p units:=meters
```

When `execute_trajectory` action or `play_custom_trajectory` service is available, both tools now queue a single batched trajectory per file/sequence.

Optional launch wrapper:

```bash
ros2 launch delta_robot gcode_json_tools.launch.py \
  run_gcode:=true \
  gcode_file:=$(ros2 pkg prefix delta_robot)/share/delta_robot/config/examples/circle_test.gcode
```

## Configuration Index

The primary configuration file is `config/delta_config.yaml`. Key namespaces:

- **`delta_kinematics`**: Geometry lengths (`base_triangle_side_length`, `active_link_length`), joint limits.
- **`motion_planner`**: Controller timings (`traj_step_ms`, `live_controller_ms`), TF frames, and end-effector/tool-tip offsets (`ee_to_tilt_axis_offset_m`, `tilt_axis_to_tool_tip_offset_m`).
- **`delta_motor_control`**: Serial bus timings and feedback stream rates. *(now in `delta_robot_drivers`)*
- **`gcode_parser` & `json_task_sequencer`**: `units`, `motion_rate_hz`, and action server names.
- **`ee_tf_broadcaster`**: IMU serial port configuration. *(now in `delta_robot_drivers`)*
