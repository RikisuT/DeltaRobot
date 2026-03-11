# `delta_robot` Package

## Nodes

### `kinematics`
Provides forward and inverse kinematics as ROS 2 services. Accepts a config YAML for robot geometry (link lengths, joint limits). Also publishes `robot_config` (pose + joint angles) at a configurable rate.

**Services:** `delta_fk`, `delta_ik`, `convert_to_joint_trajectory`, `convert_to_joint_vel_trajectory`

### `motion_planner`
Top-level control node. Sends joint trajectory commands to both the physical motors and the Gazebo simulation simultaneously.

**Services:** `play_demo_trajectory`, `move_to_point`, `move_to_configuration`, `motion_demo`

Built-in demos: `circle`, `pringle`, `axes`, `up_down`, `scan`

### `motor_control_node.py` *(Python)*
Interfaces with the **Waveshare ST3215** serial bus servos via the [`stservo`](https://github.com/iltlo/waveshare_stservo_python) Python SDK (install with `pip install -e repos/waveshare_stservo_python`).

Converts radians ↔ motor ticks, handles position and velocity control via GroupSyncWrite, and publishes joint position/velocity feedback at 25 Hz.

**Subscribes:** `delta_motors/set_joints`, `delta_motors/set_joint_vels`
**Publishes:** `delta_motors/motor_position_feedback`, `delta_motors/motor_velocity_feedback`
**Service:** `delta_motors/set_joint_limits`

### `joint_state_bridge.py` *(Python)*
Bridges `/joint_states` (from `joint_state_broadcaster` in simulation) to the motor feedback topics expected by `kinematics`. Only needed in simulation — replaced by `motor_control_node` on hardware.

### `range_scanner`
3D scanning node. Moves the end-effector through the `scan` trajectory while recording (x, y, z, distance) data from the VL53L1X ToF sensor to map surfaces.

### `delta_trajectory_generator` *(Deprecated)*
Legacy standalone trajectory generation node. Functionality is now fully covered by `motion_planner`.