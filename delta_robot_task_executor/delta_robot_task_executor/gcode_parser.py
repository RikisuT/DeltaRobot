#!/usr/bin/env python3
"""G-code interpreter for delta robot motion planner service execution.

Boundary note (PY_CPP_SPLIT):
  This module is a *thin Python client* in the tooling layer.  It parses G-code
  commands and maps them to ROS 2 service/action calls (MoveToPose,
  PlayCustomTrajectory, ExecuteTrajectory).  Any linear interpolation performed
  here is **waypoint generation** (building a list of XYZ points to send to the
  C++ motion planner), NOT kinematic math.  All IK/FK and trajectory smoothing
  is handled exclusively by the C++ kinematics and motion_planner nodes.
"""

import math
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point

from delta_robot_task_executor.task_base import TaskExecutorBase


class GCodeParserNode(TaskExecutorBase):
    """Parse and execute a subset of G-code against the move_to_pose service."""

    def __init__(self, gcode_file: str):
        super().__init__("gcode_parser")

        self.declare_parameter("move_to_pose_service", "delta_motion_planner/move_to_pose")
        self.declare_parameter("play_custom_trajectory_service", "delta_motion_planner/play_custom_trajectory")
        self.declare_parameter("execute_trajectory_action", "delta_motion_planner/execute_trajectory")
        self.declare_parameter("get_commanded_pose_service", "delta_motion_planner/get_commanded_pose")
        self.declare_parameter("units", "meters")
        self.declare_parameter("home_z_mm", -300.0)
        self.declare_parameter("min_move_time_s", 0.02)
        self.declare_parameter("default_speed_mm_s", 300.0)
        self.declare_parameter("motion_rate_hz", 100.0)

        self.move_to_pose_service = (
            self.get_parameter("move_to_pose_service").get_parameter_value().string_value
        )
        self.play_custom_trajectory_service = (
            self.get_parameter("play_custom_trajectory_service").get_parameter_value().string_value
        )
        self.execute_trajectory_action = (
            self.get_parameter("execute_trajectory_action").get_parameter_value().string_value
        )
        self.get_commanded_pose_service = (
            self.get_parameter("get_commanded_pose_service").get_parameter_value().string_value
        )
        default_units = self.get_parameter("units").get_parameter_value().string_value.lower()
        self.home_z_mm = (
            self.get_parameter("home_z_mm").get_parameter_value().double_value
        )
        self.min_move_time_s = (
            self.get_parameter("min_move_time_s").get_parameter_value().double_value
        )
        self.default_speed_mm_s = (
            self.get_parameter("default_speed_mm_s").get_parameter_value().double_value
        )
        self.motion_rate_hz = max(
            1.0,
            self.get_parameter("motion_rate_hz").get_parameter_value().double_value,
        )

        self.mode_abs = True
        self.unit_scale_mm = 1000.0 if default_units == "meters" else 1.0
        self.speed_mm_s = self.default_speed_mm_s
        self.pos = {"X": 0.0, "Y": 0.0, "Z": self.home_z_mm, "A": 0.0, "C": 0.0}

        self.setup_services(
            self.move_to_pose_service,
            self.play_custom_trajectory_service,
            self.execute_trajectory_action,
            self.get_commanded_pose_service,
        )
        self.custom_trajectory_available = False

        self.gcode_file = Path(gcode_file).expanduser().resolve()

    def run(self) -> int:
        if not self.gcode_file.exists():
            self.get_logger().error(f"G-code file not found: {self.gcode_file}")
            return 1

        self.get_logger().info(
            f"Waiting for service {self.move_to_pose_service} before executing {self.gcode_file}"
        )
        if not self.wait_for_services(self.move_to_pose_service, timeout_sec=10.0):
            return 1

        self.custom_trajectory_available = self.check_custom_trajectory(
            self.play_custom_trajectory_service,
            self.execute_trajectory_action,
            timeout_sec=2.0,
        )

        pt, tilt, spin = self.fetch_current_pose()
        if pt is not None:
            self.pos["X"] = pt.x
            self.pos["Y"] = pt.y
            self.pos["Z"] = pt.z
            self.pos["A"] = tilt
            self.pos["C"] = spin
            self.get_logger().info(f"Starting from current physical position: ({pt.x:.1f}, {pt.y:.1f}, {pt.z:.1f})")
        if self.custom_trajectory_available:
            batched_result = self._run_batched_file()
            if batched_result == -1:
                self.get_logger().info(
                    "Batched mode disabled for this file due to A/C orientation commands; retrying point-wise"
                )
                self.custom_trajectory_available = False
            else:
                return batched_result

        self.get_logger().info("Service available, starting G-code execution")
        with self.gcode_file.open("r", encoding="utf-8") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                if not rclpy.ok():
                    return 0

                command = self._parse_line(raw_line)
                if command is None:
                    continue

                success = self._execute_command(command, line_number)
                if not success:
                    return 1

        self.get_logger().info("G-code execution complete")
        return 0

    def _run_batched_file(self) -> int:
        points: List[Point] = []
        step_dt = 1.0 / self.motion_rate_hz
        total_duration_s = 0.0

        with self.gcode_file.open("r", encoding="utf-8") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                if not rclpy.ok():
                    return 0

                command = self._parse_line(raw_line)
                if command is None:
                    continue

                cmd = command["cmd"]
                params = command["params"]

                if "F" in params:
                    self.speed_mm_s = max(0.1, float(params["F"]))

                if cmd == "G90":
                    self.mode_abs = True
                    continue
                if cmd == "G91":
                    self.mode_abs = False
                    continue
                if cmd == "G20":
                    self.unit_scale_mm = 25.4
                    continue
                if cmd == "G21":
                    self.unit_scale_mm = 1.0
                    continue

                if cmd == "G28":
                    target = {"X": 0.0, "Y": 0.0, "Z": self.home_z_mm}
                elif cmd in {"G0", "G1"}:
                    target = self._calculate_target(params)
                else:
                    self.get_logger().warn(f"Line {line_number}: unsupported command {cmd}, skipping")
                    continue

                x0 = self.pos["X"]
                y0 = self.pos["Y"]
                z0 = self.pos["Z"]
                a0 = self.pos["A"]
                c0 = self.pos["C"]
                dx = target["X"] - x0
                dy = target["Y"] - y0
                dz = target["Z"] - z0
                da = target["A"] - a0
                dc = target["C"] - c0
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                duration_s = max(self.min_move_time_s, distance / self.speed_mm_s)
                steps = max(1, int(math.ceil(duration_s / step_dt)))

                orientation_changed = (abs(da) > 1e-12) or (abs(dc) > 1e-12)
                if orientation_changed:
                    # Batched trajectory currently carries XYZ only.
                    # When orientation is commanded, fall back to point-wise pose calls.
                    return -1

                for step in range(1, steps + 1):
                    alpha = step / steps
                    points.append(
                        Point(
                            x=float(x0 + dx * alpha),
                            y=float(y0 + dy * alpha),
                            z=float(z0 + dz * alpha),
                        )
                    )

                total_duration_s += duration_s
                self.pos["X"] = target["X"]
                self.pos["Y"] = target["Y"]
                self.pos["Z"] = target["Z"]
                self.pos["A"] = target["A"]
                self.pos["C"] = target["C"]

        if not points:
            self.get_logger().warn("No motion points generated from G-code")
            return 0

        step_ms = max(1, int(round(step_dt * 1000.0)))
        if not self._call_custom_trajectory_service(points, step_ms, 0):
            return 1

        self.get_logger().info(
            f"Queued batched G-code trajectory with {len(points)} point(s), step={step_ms}ms"
        )
        time.sleep(total_duration_s)
        self.get_logger().info("G-code execution complete")
        return 0

    def _parse_line(self, raw_line: str) -> Optional[Dict[str, object]]:
        stripped = raw_line.strip()
        if not stripped:
            return None
        if stripped.startswith(";"):
            return None

        # Remove bracket comments and inline semicolon comments.
        stripped = re.sub(r"\(.*?\)", "", stripped)
        stripped = stripped.split(";", maxsplit=1)[0].strip()
        if not stripped:
            return None

        words = re.findall(r"([A-Za-z])\s*([-+]?\d*\.?\d+)", stripped)
        if not words:
            return None

        cmd = None
        params: Dict[str, float] = {}
        for key, value_text in words:
            key = key.upper()
            try:
                value = float(value_text)
            except ValueError:
                self.get_logger().warn(f"Skipping invalid token: {key}{value_text}")
                continue

            if key == "G" and cmd is None:
                cmd = f"G{int(value)}"
                continue

            if key in {"X", "Y", "Z", "A", "C", "F"}:
                params[key] = value

        if cmd is None:
            return None

        return {"cmd": cmd, "params": params}

    def _execute_command(self, command: Dict[str, object], line_number: int) -> bool:
        cmd = command["cmd"]
        params = command["params"]

        if "F" in params:
            self.speed_mm_s = max(0.1, float(params["F"]))
            self.get_logger().info(
                f"Line {line_number}: feed F={params['F']} -> {self.speed_mm_s:.2f} mm/s"
            )

        if cmd == "G90":
            self.mode_abs = True
            return True

        if cmd == "G91":
            self.mode_abs = False
            return True

        if cmd == "G20":
            self.unit_scale_mm = 25.4
            self.get_logger().info(f"Line {line_number}: units set to inches")
            return True

        if cmd == "G21":
            self.unit_scale_mm = 1.0
            self.get_logger().info(f"Line {line_number}: units set to millimeters")
            return True

        if cmd == "G28":
            return self._move_to(0.0, 0.0, self.home_z_mm, self.pos["A"], self.pos["C"], False, line_number)

        if cmd in {"G0", "G1"}:
            target = self._calculate_target(params)
            use_orientation = ("A" in params) or ("C" in params)
            return self._move_to(
                target["X"],
                target["Y"],
                target["Z"],
                target["A"],
                target["C"],
                use_orientation,
                line_number,
            )

        self.get_logger().warn(f"Line {line_number}: unsupported command {cmd}, skipping")
        return True

    def _calculate_target(self, params: Dict[str, float]) -> Dict[str, float]:
        next_pos = dict(self.pos)
        for axis in ("X", "Y", "Z", "A", "C"):
            if axis not in params:
                continue
            delta = float(params[axis])
            if axis in {"X", "Y", "Z"}:
                delta *= self.unit_scale_mm
            elif axis in {"A", "C"}:
                # G-code A/C tilt/spin are commonly specified in degrees;
                # convert to radians for the motion planner API which expects radians.
                delta = math.radians(delta)
            if self.mode_abs:
                next_pos[axis] = delta
            else:
                next_pos[axis] += delta
        return next_pos

    def _move_to(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        a_rad: float,
        c_rad: float,
        use_orientation: bool,
        line_number: int,
    ) -> bool:
        x0 = self.pos["X"]
        y0 = self.pos["Y"]
        z0 = self.pos["Z"]
        a0 = self.pos["A"]
        c0 = self.pos["C"]

        dx = x_mm - x0
        dy = y_mm - y0
        dz = z_mm - z0
        da = a_rad - a0
        dc = c_rad - c0

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Angular "distance" scaled to a comparable speed
        # Treat 1 rad of rotation as equivalent to some mm of travel
        ANGULAR_SCALE_MM_PER_RAD = 50.0  # tune to your robot
        angular_distance = math.sqrt(da*da + dc*dc) * ANGULAR_SCALE_MM_PER_RAD

        effective_distance = max(distance, angular_distance)
        duration_s = max(self.min_move_time_s, effective_distance / self.speed_mm_s)
        steps = max(1, int(math.ceil(duration_s * self.motion_rate_hz)))
        step_dt = duration_s / steps

        if self.custom_trajectory_available:
            trajectory_points = []
            for step in range(1, steps + 1):
                alpha = step / steps
                trajectory_points.append(
                    Point(
                        x=float(x0 + dx * alpha),
                        y=float(y0 + dy * alpha),
                        z=float(z0 + dz * alpha),
                    )
                )

            step_ms = max(1, int(round(step_dt * 1000.0)))
            if not self._call_custom_trajectory_service(trajectory_points, step_ms, line_number):
                return False

            self.pos["X"] = x_mm
            self.pos["Y"] = y_mm
            self.pos["Z"] = z_mm
            self.pos["A"] = a_rad
            self.pos["C"] = c_rad
            # Service returns when accepted; sleep for commanded duration to preserve ordering.
            time.sleep(duration_s)
            self.get_logger().info(
                f"Line {line_number}: move ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f}) in {duration_s:.2f}s ({steps} batched step(s))"
            )
            return True

        for step in range(1, steps + 1):
            alpha = step / steps
            xi = x0 + dx * alpha
            yi = y0 + dy * alpha
            zi = z0 + dz * alpha
            ai = a0 + (a_rad - a0) * alpha
            ci = c0 + (c_rad - c0) * alpha

            tick_start = time.monotonic()
            if not self._call_move_pose_service(xi, yi, zi, ai, ci, use_orientation, line_number):
                return False

            elapsed = time.monotonic() - tick_start
            remaining = step_dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

        self.pos["X"] = x_mm
        self.pos["Y"] = y_mm
        self.pos["Z"] = z_mm
        self.pos["A"] = a_rad
        self.pos["C"] = c_rad
        self.get_logger().info(
            f"Line {line_number}: move ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f}) in {duration_s:.2f}s ({steps} fallback step(s))"
        )
        return True

    def _call_custom_trajectory_service(
        self, trajectory: List[Point], step_ms: int, line_number: int
    ) -> bool:
        return self.call_custom_trajectory(
            trajectory, step_ms, f"Line {line_number}"
        )

    def _call_move_pose_service(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        tilt_rad: float,
        spin_rad: float,
        use_orientation: bool,
        line_number: int,
    ) -> bool:
        return self.call_move_pose(
            x_mm,
            y_mm,
            z_mm,
            tilt_rad,
            spin_rad,
            use_orientation,
            f"Line {line_number}",
        )


def main(args=None) -> int:
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run delta_robot_task_executor gcode_parser <file.gcode>")
        rclpy.shutdown()
        return 1

    node = GCodeParserNode(sys.argv[1])
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
