# API Boundaries — C++ Core vs Python Tooling

This document defines the boundary between C++ "core control" nodes and Python
"tooling" nodes in the Delta Robot workspace.

## Rules

1. **C++ owns all motion math**: IK, FK, Jacobian, trajectory interpolation, and
   real-time motor command sequencing live exclusively in C++.
2. **Python is a thin client**: Python tools call services, actions, or publish to
   topics. They never implement kinematic math.
3. **Boundary contract**: Python ↔ C++ interaction happens only through the ROS
   interfaces listed below. C++ never depends on Python packages.

## C++ Core Nodes (`delta_robot` package)

| Node | Executable | Purpose |
|------|-----------|---------|
| `delta_kinematics` | `kinematics` | FK / IK services, Jacobian, trajectory conversion |
| `delta_motion_planner` | `motion_planner` | Service/action handlers, trajectory execution, TF broadcasting |
| `range_scanner` | `range_scanner` | Lidar scan aggregation (currently disabled) |

### Shared Math Library

`delta_math.hpp` — a header-only C++ library providing pure math functions
(no ROS dependency) used by kinematics and motion_planner nodes.

## Python Driver Nodes (`delta_robot_drivers` package)

| Node | Script | Purpose |
|------|--------|---------|
| `delta_motor_control` | `motor_control_node.py` | Serial bridge to ESP32 servos |
| `joint_state_bridge` | `joint_state_bridge.py` | Gazebo `/joint_states` → DeltaJoints bridge |
| `ee_tf_broadcaster` | `ee_tf_broadcaster.py` | IMU + ToF sensor → TF broadcaster |

## Python Tooling Packages

| Package | Nodes | Purpose |
|---------|-------|---------|
| `delta_robot_gui` | `delta_robot_gui` | PyQt control center (service/topic client) |
| `delta_robot_task_executor` | `gcode_parser`, `json_task_sequencer` | File-based task playback |
| `delta_robot_testing` | `motor_slider_tester`, `see_motors`, `test` | Debug utilities |
| `delta_robot_visualization` | `plot_ee_tf`, `plotter3d` | TF/trajectory plotters |

## Service / Action / Topic Contract

### Services (C++ servers, called by Python tools)

| Service | Type | Provider |
|---------|------|----------|
| `delta_kinematics/delta_fk` | `DeltaFK` | kinematics |
| `delta_kinematics/delta_ik` | `DeltaIK` | kinematics |
| `delta_kinematics/convert_to_joint_trajectory` | `ConvertToJointTrajectory` | kinematics |
| `delta_kinematics/convert_to_joint_vel_trajectory` | `ConvertToJointVelTrajectory` | kinematics |
| `delta_motion_planner/move_to_point` | `MoveToPoint` | motion_planner |
| `delta_motion_planner/move_to_pose` | `MoveToPose` | motion_planner |
| `delta_motion_planner/move_to_configuration` | `MoveToConfiguration` | motion_planner |
| `delta_motion_planner/play_demo_trajectory` | `PlayDemoTrajectory` | motion_planner |
| `delta_motion_planner/play_custom_trajectory` | `PlayCustomTrajectory` | motion_planner |
| `delta_motion_planner/motion_demo` | `MotionDemo` | motion_planner |
| `delta_motion_planner/set_motion_mode` | `SetMotionMode` | motion_planner |
| `delta_motors/set_joint_limits` | `SetJointLimits` | motor_control_node |

### Actions (C++ server, called by Python tools)

| Action | Type | Provider |
|--------|------|----------|
| `delta_motion_planner/execute_trajectory` | `ExecuteTrajectory` | motion_planner |

The `ExecuteTrajectory` action is the preferred interface for task executors.
`TaskExecutorBase` automatically selects it over the `PlayCustomTrajectory`
service when available.

### Topics (inter-node communication)

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `delta_motors/set_joints` | `DeltaJoints` | motion_planner | motor_control_node |
| `delta_motors/set_joint_vels` | `DeltaJointVels` | motion_planner | motor_control_node |
| `delta_motors/motor_position_feedback` | `DeltaJoints` | motor_control_node / joint_state_bridge | kinematics, motion_planner, GUI |
| `delta_motors/motor_velocity_feedback` | `DeltaJointVels` | motor_control_node / joint_state_bridge | kinematics |
| `delta_robot/robot_config` | `RobotConfig` | kinematics | range_scanner |
| `delta_motion_planner/live_target` | `Point` | GUI | motion_planner |
| `delta_motion_planner/live_orientation` | `Float64MultiArray` | GUI | motion_planner |
