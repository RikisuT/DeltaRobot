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
| Bicep Servos | Waveshare ST3215-HS × 3 (STS serial bus, 12-bit, 1 Mbaud) |
| EE Servos | Waveshare STS3032 × 2 (tilt + spin, differential mechanism) |
| Driver | Waveshare Servo Driver with ESP32 via `/dev/ttyUSB0` |
| Sensors | VL53L4CD ToF range sensors × 3, ISM330DHCX 6-DoF IMU |
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

## Architecture

The workspace enforces a clean separation between C++ core control and Python
tooling. See [API_BOUNDARIES.md](API_BOUNDARIES.md) for the full contract.

```
C++ core (delta_robot)
├── delta_math.hpp        — Pure math library (FK, IK, Jacobian, trajectories)
├── kinematics node       — FK/IK services, trajectory conversion
├── motion_planner node   — Service/action handlers, trajectory execution, TF
└── range_scanner node    — Lidar scan aggregation

Python drivers (delta_robot_drivers)
├── motor_control_node    — ESP32 serial bridge
├── joint_state_bridge    — Gazebo joint states bridge (sim only)
└── ee_tf_broadcaster     — IMU + ToF sensor TF broadcaster

Python tooling (separate packages)
├── delta_robot_gui       — PyQt control center
├── delta_robot_task_executor — G-code / JSON task playback
├── delta_robot_visualization — TF and trajectory plotters
└── delta_robot_testing   — Debug and testing utilities
```

## Package Structure

| Package | Language | Description |
|---|---|---|
| `delta_robot` | C++ | Core nodes: kinematics, motion planner, range scanner, and `delta_math` library |
| `delta_robot_drivers` | Python | Hardware/sim driver nodes: motor control, joint state bridge, EE TF broadcaster |
| `delta_robot_gui` | Python | PyQt control center for Cartesian targets, G-code, and JSON tasks |
| `delta_robot_sim` | C++/Launch | Gazebo simulation with `ros2_control` integration |
| `delta_robot_description` | SDF/Meshes | Robot model and meshes |
| `deltarobot_interfaces` | ROS IDL | Custom ROS 2 messages, services, and actions |
| `delta_robot_task_executor` | Python | G-code and JSON task playback tools |
| `delta_robot_visualization` | Python | TF and trajectory visualization tools |
| `delta_robot_testing` | Python | Debug and testing utilities |

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
# Terminal 2 — Core stack (kinematics + motion planner + motor control)
ros2 launch delta_robot delta_robot.launch.py

# Terminal 3 — GUI controller (Cartesian / G-code / JSON)
ros2 launch delta_robot_gui delta_robot_gui.launch.py

# Terminal 4 — Measurement cube tooling (TF broadcaster + plotter)
ros2 launch delta_robot cube.launch.py
```

Run a demo trajectory:

```bash
# Terminal 5 — Trigger a demo
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

Open **two** terminals (source the workspace in each):

```bash
# Terminal 1 — Core stack (kinematics + motion planner + motor control)
ros2 launch delta_robot delta_robot.launch.py
```

You can then send commands exactly as in the simulation (e.g. the demo
service call above).

## License

BSD-3-Clause — see [LICENSE](LICENSE).
Original work Copyright © 2025 Sharwin Patil.
Modifications Copyright © 2025 Likhithraj T Acharya.
