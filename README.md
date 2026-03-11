# DeltaRobot
Author: Sharwin Patil *(2025 MSR Winter Project)*

<p align="center">
  <img src="images/MSI_demo_desk.jpg" alt="Demo Table at Museum of Science" style="border-radius: 15px; width: 69%; display: inline-block; margin-right: 2%;">
  <img src="images/robot_white_background.png" alt="Delta Robot Workspace" style="border-radius: 15px; width: 30%; display: inline-block;">
</p>


_Check out [my portfolio post](https://www.sharwinpatil.info/posts/delta-robot/) for more media and information._

[![wakatime](https://wakatime.com/badge/user/b25c3469-3f3c-4aff-90ef-5723a788454c/project/c9d6563e-f5b2-4049-b33c-ff699139a47a.svg)](https://wakatime.com/badge/user/b25c3469-3f3c-4aff-90ef-5723a788454c/project/c9d6563e-f5b2-4049-b33c-ff699139a47a)

# Hardware

- **Servos:** Waveshare ST3215 (×3) — STS serial bus servos, 12-bit encoder, 1 Mbaud
- **Driver:** Waveshare Servo Driver with ESP32 — UART control over USB (`/dev/ttyUSB0`)
- **Sensors:** VL53L1X Time-of-Flight range sensor, BNO055 9-DoF IMU

# Dependencies

- ROS 2 Jazzy
- Gazebo (Ignition) — for simulation

### Servo SDK (required for hardware)
```bash
# Clone the STServo Python library into the repos folder
git clone https://github.com/iltlo/waveshare_stservo_python.git repos/waveshare_stservo_python

# Install it (editable install — keep the folder in place)
pip install -e repos/waveshare_stservo_python
```

# ROS Package Structure

## `delta_robot` Package
Contains main delta robot nodes: kinematics, motion planning, motor control, and 3D scanning.

## `delta_robot_sim` Package
Gazebo simulation environment for the delta robot. Includes:
- SDF world + robot model with `gz_ros2_control` integration
- `joint_state_broadcaster` + `joint_trajectory_controller` via `ros2_controllers.yaml`
- `joint_state_bridge` node bridging `/joint_states` → motor feedback topics for `delta_kinematics`

## `delta_robot_description` Package
Robot description (SDF model, meshes, materials).

## `deltarobot_interfaces` Package
Custom ROS 2 messages and services used across all nodes.

## `delta_robot_sensors` Package
IMU (BNO055) and ToF (VL53L1X) sensor nodes for the physical robot.

# Running

### Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Simulation Only
```bash
# Terminal 1 — Gazebo + controllers
ros2 launch delta_robot_sim delta_robot_spawn.launch.py

# Terminal 2 — Kinematics
ros2 run delta_robot kinematics --ros-args \
  --params-file install/delta_robot/share/delta_robot/config/delta_config.yaml

# Terminal 3 — Motion Planner
ros2 run delta_robot motion_planner

# Terminal 4 — Run a demo
ros2 service call /delta_motion_planner/play_demo_trajectory \
  deltarobot_interfaces/srv/PlayDemoTrajectory '{type: {data: circle}}'
```

Available demos: `circle`, `pringle`, `axes`, `up_down`, `scan`

### Hardware Only
```bash
ros2 run delta_robot kinematics --ros-args \
  --params-file install/delta_robot/share/delta_robot/config/delta_config.yaml
ros2 run delta_robot motion_planner
ros2 run delta_robot motor_control_node.py
```

