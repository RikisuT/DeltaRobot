# DeltaRobot — 5-DOF Extension

**Maintainer:** Likhithraj T Acharya (rikisu) — likhiacharya@gmail.com

A 3-DOF delta robot ROS 2 control stack, being extended to a **5-DOF system** with 2 additional end-effector degrees of freedom.

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
| Simulation | ROS 2 Jazzy + Gazebo Harmonic |

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

## Build

```bash
# Source the ROS 2 workspace (run once per terminal)
source /opt/ros/jazzy/setup.bash

# Build all packages (from the workspace root)
cd ~/major_project_ws2
colcon build --symlink-install

# Source the overlay (run once per terminal, after build)
source install/setup.bash
```

> **Tip:** Add `source /opt/ros/jazzy/setup.bash` and
> `source ~/major_project_ws2/install/setup.bash` to your `~/.bashrc`
> so every new terminal is ready to go.

---

### Simulation

Launch everything (Gazebo, RViz, controllers, Foxglove bridge, 3D plotter) with a single command:

```bash
# Terminal 1 — Spawn the simulation
ros2 launch delta_robot_sim delta_robot_spawn.launch.py
```

Then start the control stack:

```bash
# Terminal 2 — Kinematics server
ros2 run delta_robot kinematics --ros-args \
  --params-file install/delta_robot/share/delta_robot/config/delta_config.yaml

# Terminal 3 — Motion planner
ros2 run delta_robot motion_planner
```

Run a demo trajectory:

```bash
# Terminal 4 — Trigger a demo
ros2 service call /delta_motion_planner/play_demo_trajectory \
  deltarobot_interfaces/srv/PlayDemoTrajectory '{type: {data: circle}}'
```

Available demos: `circle`, `pringle`, `axes`, `up_down`, `scan`

> The simulation launch also starts a **Foxglove Bridge** on port `8765`.
> Open [Foxglove Studio](https://foxglove.dev/) and connect to `ws://localhost:8765`
> to visualise topics in real time.

---

### Hardware

> **Prerequisite:** Connect the Waveshare Servo Driver (ESP32) via USB
> and verify it appears as `/dev/ttyUSB0`.

Open **three** terminals (source the workspace in each):

```bash
# Terminal 1 — Kinematics server
ros2 run delta_robot kinematics --ros-args \
  --params-file install/delta_robot/share/delta_robot/config/delta_config.yaml

# Terminal 2 — Motion planner
ros2 run delta_robot motion_planner

# Terminal 3 — Motor control (communicates with physical servos)
ros2 run delta_robot motor_control_node.py
```

You can then send commands exactly as in the simulation (e.g. the demo
service call above).

## License

BSD-3-Clause — see [LICENSE](LICENSE).
Original work Copyright © 2025 Sharwin Patil.
Modifications Copyright © 2025 Likhithraj T Acharya.


