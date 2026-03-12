# DeltaRobot — 5-DOF Extension

**Maintainer:** Likhithraj T Acharya (rikisu) — likhiacharya@gmail.com

A 3-DOF delta robot ROS 2 control stack, being extended to a **5-DOF system** with 2 additional end-effector degrees of freedom. Intended for deployment in a university control engineering lab.

---

> **Based on** the original work by [Sharwin Patil](https://www.sharwinpatil.info/posts/delta-robot/) — *2025 MSR Winter Project, Northwestern University.*
> Original repository: [sharwinpatil/DeltaRobot](https://github.com/sharwinpatil/DeltaRobot)

---

## Hardware

| Component | Details |
|---|---|
| Servos | Waveshare ST3215 × 3 (STS serial bus, 12-bit, 1 Mbaud) |
| Driver | Waveshare Servo Driver with ESP32 via `/dev/ttyUSB0` |
| Sensors | VL53L1X ToF range sensor, BNO055 9-DoF IMU |
| Simulation | ROS 2 Jazzy + Gazebo (Ignition) |

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic

### Servo Python SDK
```bash
git clone https://github.com/iltlo/waveshare_stservo_python.git repos/waveshare_stservo_python
pip install -e repos/waveshare_stservo_python
```
> The `repos/` folder is gitignored — keep the clone in place for the editable install to work.

## Package Structure

| Package | Description |
|---|---|
| `delta_robot` | Core nodes: kinematics, motion planner, motor control, 3D scanner |
| `delta_robot_sim` | Gazebo simulation with `ros2_control` integration |
| `delta_robot_description` | SDF robot model and meshes |
| `deltarobot_interfaces` | Custom ROS 2 messages and services |
| `delta_robot_sensors` | IMU and ToF sensor nodes |

## Build & Run

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Simulation
```bash
# Terminal 1
ros2 launch delta_robot_sim delta_robot_spawn.launch.py

# Terminal 2
ros2 run delta_robot kinematics --ros-args \
  --params-file install/delta_robot/share/delta_robot/config/delta_config.yaml

# Terminal 3
ros2 run delta_robot motion_planner

# Terminal 4 — run a demo
ros2 service call /delta_motion_planner/play_demo_trajectory \
  deltarobot_interfaces/srv/PlayDemoTrajectory '{type: {data: circle}}'
```

Available demos: `circle`, `pringle`, `axes`, `up_down`, `scan`

### Hardware
```bash
ros2 run delta_robot kinematics --ros-args \
  --params-file install/delta_robot/share/delta_robot/config/delta_config.yaml
ros2 run delta_robot motion_planner
ros2 run delta_robot motor_control_node.py
```

## License

BSD-3-Clause — see [LICENSE](LICENSE).
Original work Copyright © 2025 Sharwin Patil.
Modifications Copyright © 2025 Likhithraj T Acharya.


