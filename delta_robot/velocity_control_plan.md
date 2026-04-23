# Velocity-control Plan for Delta Robot (simulation)

Date: 2026-04-23

Goal: Implement a closed-loop end-effector velocity controller in simulation to smoothly move to targets (teleop or automated setpoints).

Overview
- Use a Jacobian-based mapping with damped least squares (DLS): theta_dot = J^T (J J^T + lambda^2 I)^{-1} p_dot.
- Run a fixed-rate control loop (default 100 Hz) that computes an EE velocity command p_dot from a PD position controller and maps it to joint velocities.
- Publish joint velocities to `delta_motors/set_joint_vels` (use sim velocity controller or JointGroupVelocityController).

Parameters (configurable)
- `control_rate_hz` (default 100)
- `Kp` (mm->mm/s gain), `Kd` (damping on EE vel)
- `lambda_dls` (damped least-squares factor, e.g. 0.01)
- `max_ee_speed_mm_s` (saturate EE speed)
- `max_joint_vel_rad_s` (use delta_kinematics parameter or user-specified)
- `max_joint_acc_rad_s` (optional accel limit)
- `position_tolerance_mm` and `stopping_cycles`

Topics / Services
- Subscribe: `delta_motion_planner/live_target` or custom `delta_velocity_controller/target` (geometry_msgs/Point) — target EE point in mm.
- Subscribe: `delta_motors/motor_position_feedback` (deltarobot_interfaces::msg::DeltaJoints) — current joint angles (rad).
- Optionally subscribe: `delta_motors/motor_velocity_feedback` (joint vel feedback).
- Publish: `delta_motors/set_joint_vels` (deltarobot_interfaces::msg::DeltaJointVels).
- Optionally call: `delta_kinematics/delta_fk` to get EE position, or compute FK locally by porting `delta_kinematics::deltaFK`.

Control loop (per tick, dt = 1/control_rate_hz)
1. Read current joint angles theta (and optionally joint velocities).
2. Compute current end-effector pose p_current (mm) via FK or `delta_kinematics/delta_fk`.
3. Compute position error e = p_target - p_current (mm).
4. Compute EE velocity command p_dot_cmd = Kp * e + Kd * (v_target - v_current) (mm/s). For simple P-only, set Kd=0.
5. Limit |p_dot_cmd| to `max_ee_speed_mm_s`.
6. Compute Jacobian J (3x3) at current theta. Option A: port `calcJacobian` from `delta_robot/src/kinematics.cpp`. Option B: call a service that returns joint vels (not ideal for per-tick control).
7. Compute theta_dot via DLS: theta_dot = J^T * (J * J^T + lambda^2 I)^{-1} * p_dot_cmd. (If J is invertible, theta_dot = J^{-1} p_dot_cmd.)
8. Clip theta_dot per-joint to +/- `max_joint_vel_rad_s`. Optionally apply accel limits using previous theta_dot.
9. Publish `DeltaJointVels` with theta_dot and loop.
10. Stop when |e| < position_tolerance_mm for `stopping_cycles` ticks.

Numerical / Units notes
- The codebase uses millimeters for EE position in many places; TFs use meters. Keep p_dot in mm/s (or convert consistently) — ensure Jacobian scaling matches p units.
- `delta_kinematics` stores `max_joint_velocity` (rad/s) as a parameter; use it to clip commands.

Safety & Robustness
- Damped least squares prevents excessive commands near singularities. Increase `lambda_dls` when conditioning is poor.
- Detect joints approaching limits and reduce speed or switch to a position-mode safe stop.
- Add watchdog / emergency stop topic; zero velocities on lost feedback.

Implementation options
- Quick prototype: Python node (rclpy) that ports `calcJacobian` (using numpy) and implements the loop. Faster iteration in sim.
- Production: C++ node (rclcpp) integrated into `delta_motion_planner` or separate `delta_velocity_controller`.
- Alternative (simpler): send time-parameterized `trajectory_msgs/JointTrajectory` with velocities/time_from_start to `joint_trajectory_controller`. Less ideal for live teleop.

Testing in simulation
1. Ensure sim has a velocity-capable controller for the motors (e.g., `JointGroupVelocityController`) subscribed to `delta_motors/set_joint_vels`.
2. Launch sim and start the controller node.
3. Publish a few `live_target` messages (mm) and watch `delta_robot/robot_config` or FK TFs to verify tracking.
4. Tune `Kp`, `Kd`, `lambda_dls`, rate, and limits.

References / TODO
- Reference `delta_robot/src/kinematics.cpp` `calcJacobian` and `calcThetaDot` for Jacobian math.
- Decide language (Python/C++). If you want, I can scaffold the node (C++ or Python) with the DLS Jacobian implementation and a runnable test harness.
