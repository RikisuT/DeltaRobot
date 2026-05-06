#!/usr/bin/env python3
"""JSON task sequencer for delta robot motion execution.

Boundary note (PY_CPP_SPLIT):
  This module is a *thin Python client* in the tooling layer.  It reads JSON
  task files and maps each step to ROS 2 service/action calls (MoveToPose,
  PlayCustomTrajectory, ExecuteTrajectory).  Any point-list construction here
  is **waypoint generation** for the service API, NOT kinematic math.  All
  IK/FK computation and trajectory smoothing is handled exclusively by the
  C++ kinematics and motion_planner nodes.
"""

import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point

from delta_robot_task_executor.task_base import TaskExecutorBase


class JsonTaskSequencer(TaskExecutorBase):
    """Execute source-compatible JSON action sequences."""

    def __init__(self, task_file: str):
        super().__init__("json_task_sequencer")

        self.declare_parameter(
            "move_to_pose_service", "delta_motion_planner/move_to_pose"
        )
        self.declare_parameter(
            "play_custom_trajectory_service",
            "delta_motion_planner/play_custom_trajectory",
        )
        self.declare_parameter(
            "execute_trajectory_action", "delta_motion_planner/execute_trajectory"
        )
        self.declare_parameter(
            "get_commanded_pose_service", "delta_motion_planner/get_commanded_pose"
        )
        self.declare_parameter("units", "meters")
        self.declare_parameter("default_duration_s", 1.0)
        self.declare_parameter("motion_rate_hz", 100.0)
        self.declare_parameter("home_x_mm", 0.0)
        self.declare_parameter("home_y_mm", 0.0)
        self.declare_parameter("home_z_mm", -300.0)

        self.move_to_pose_service = (
            self.get_parameter("move_to_pose_service")
            .get_parameter_value()
            .string_value
        )
        self.play_custom_trajectory_service = (
            self.get_parameter("play_custom_trajectory_service")
            .get_parameter_value()
            .string_value
        )
        self.execute_trajectory_action = (
            self.get_parameter("execute_trajectory_action")
            .get_parameter_value()
            .string_value
        )
        self.get_commanded_pose_service = (
            self.get_parameter("get_commanded_pose_service")
            .get_parameter_value()
            .string_value
        )
        self.json_units = self.get_parameter("units").get_parameter_value().string_value
        self.default_duration_s = (
            self.get_parameter("default_duration_s").get_parameter_value().double_value
        )
        self.motion_rate_hz = max(
            1.0,
            self.get_parameter("motion_rate_hz").get_parameter_value().double_value,
        )
        self.home = {
            "x": self.get_parameter("home_x_mm").get_parameter_value().double_value,
            "y": self.get_parameter("home_y_mm").get_parameter_value().double_value,
            "z": self.get_parameter("home_z_mm").get_parameter_value().double_value,
        }

        self.scale_to_mm = 1000.0 if self.json_units.lower() == "meters" else 1.0

        self.task_file = Path(task_file).expanduser().resolve()
        self.current_pos = dict(self.home)
        self.current_tilt = 0.0
        self.current_spin = 0.0

        self.setup_services(
            self.move_to_pose_service,
            self.play_custom_trajectory_service,
            self.execute_trajectory_action,
            self.get_commanded_pose_service,
        )
        self.custom_trajectory_available = False

    def run(self) -> int:
        if not self.task_file.exists():
            self.get_logger().error(f"Task file not found: {self.task_file}")
            return 1

        if not self.wait_for_services(self.move_to_pose_service, timeout_sec=10.0):
            return 1

        self.custom_trajectory_available = self.check_custom_trajectory(
            self.play_custom_trajectory_service,
            self.execute_trajectory_action,
            timeout_sec=2.0,
        )

        pt, tilt, spin = self.fetch_current_pose()
        if pt is not None:
            self.current_pos["x"] = pt.x
            self.current_pos["y"] = pt.y
            self.current_pos["z"] = pt.z
            self.current_tilt = tilt
            self.current_spin = spin
            self.get_logger().info(f"Starting from current physical position: ({pt.x:.1f}, {pt.y:.1f}, {pt.z:.1f})")

        tasks = self._load_tasks(self.task_file)
        if tasks is None:
            return 1

        self.get_logger().info(
            f"Loaded {len(tasks)} task(s) from {self.task_file} with units={self.json_units}"
        )

        if self.custom_trajectory_available and not self._tasks_require_orientation(
            tasks
        ):
            return self._run_batched_tasks(tasks)
        if self.custom_trajectory_available:
            self.get_logger().info(
                "Detected tilt/spin actions. Switching to point-wise pose mode to preserve orientation."
            )

        for idx, task in enumerate(tasks, start=1):
            if not rclpy.ok():
                return 0
            if not isinstance(task, dict):
                self.get_logger().warn(f"Task {idx}: expected object, skipping")
                continue
            if not self._execute_task(task, idx):
                return 1

        self.get_logger().info("Task sequence complete")
        return 0

    def _run_batched_tasks(self, tasks: List[Dict[str, Any]]) -> int:
        step_dt = 1.0 / self.motion_rate_hz
        points: List[Point] = []
        total_duration_s = 0.0

        for idx, task in enumerate(tasks, start=1):
            action = str(task.get("action", "")).lower().strip()

            if action == "move":
                x_mm = self._to_mm(task.get("x", self.current_pos["x"]))
                y_mm = self._to_mm(task.get("y", self.current_pos["y"]))
                z_mm = self._to_mm(task.get("z", self.current_pos["z"]))
                duration = max(
                    0.0, float(task.get("duration", self.default_duration_s))
                )
                self._append_segment_points(points, x_mm, y_mm, z_mm, duration, step_dt)
                self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
                total_duration_s += duration
                continue

            if action == "home":
                duration = max(
                    0.0, float(task.get("duration", self.default_duration_s))
                )
                self._append_segment_points(
                    points,
                    self.home["x"],
                    self.home["y"],
                    self.home["z"],
                    duration,
                    step_dt,
                )
                self.current_pos.update(self.home)
                total_duration_s += duration
                continue

            if action == "wait":
                duration = max(0.0, float(task.get("seconds", 1.0)))
                hold_steps = max(1, int(math.ceil(duration / step_dt)))
                for _ in range(hold_steps):
                    points.append(
                        Point(
                            x=float(self.current_pos["x"]),
                            y=float(self.current_pos["y"]),
                            z=float(self.current_pos["z"]),
                        )
                    )
                total_duration_s += duration
                continue

            if action in {"tilt", "spin", "suction"}:
                self.get_logger().warn(
                    f"[{idx}] action '{action}' not wired in this stack, skipping"
                )
                duration = max(0.0, float(task.get("duration", 0.0) or 0.0))
                if duration > 0.0:
                    hold_steps = max(1, int(math.ceil(duration / step_dt)))
                    for _ in range(hold_steps):
                        points.append(
                            Point(
                                x=float(self.current_pos["x"]),
                                y=float(self.current_pos["y"]),
                                z=float(self.current_pos["z"]),
                            )
                        )
                    total_duration_s += duration
                continue

            self.get_logger().warn(f"[{idx}] unknown action '{action}', skipping")

        if not points:
            self.get_logger().warn("No motion points generated from JSON tasks")
            return 0

        step_ms = max(1, int(round(step_dt * 1000.0)))
        if not self._call_custom_trajectory_service(points, step_ms, 0):
            return 1

        self.get_logger().info(
            f"Queued batched JSON trajectory with {len(points)} point(s), step={step_ms}ms"
        )
        time.sleep(total_duration_s)
        self.get_logger().info("Task sequence complete")
        return 0

    def _append_segment_points(
        self,
        points: List[Point],
        x_mm: float,
        y_mm: float,
        z_mm: float,
        duration: float,
        step_dt: float,
    ) -> None:
        x0 = self.current_pos["x"]
        y0 = self.current_pos["y"]
        z0 = self.current_pos["z"]

        if duration <= 0.0:
            points.append(Point(x=float(x_mm), y=float(y_mm), z=float(z_mm)))
            return

        dx = x_mm - x0
        dy = y_mm - y0
        dz = z_mm - z0
        steps = max(1, int(math.ceil(duration / step_dt)))

        for step in range(1, steps + 1):
            alpha = step / steps
            points.append(
                Point(
                    x=float(x0 + dx * alpha),
                    y=float(y0 + dy * alpha),
                    z=float(z0 + dz * alpha),
                )
            )

    def _load_tasks(self, path: Path) -> Optional[List[Dict[str, Any]]]:
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON in {path}: {exc}")
            return None
        except OSError as exc:
            self.get_logger().error(f"Failed to read {path}: {exc}")
            return None

        if not isinstance(data, list):
            self.get_logger().error("Task JSON must be a list of action objects")
            return None

        return data

    def _execute_task(self, task: Dict[str, Any], idx: int) -> bool:
        action = str(task.get("action", "")).lower().strip()
        self.get_logger().info(f"[{idx}] action={action}")

        if action == "move":
            return self._do_move(task, idx)
        if action == "wait":
            return self._do_wait(task)
        if action == "home":
            return self._do_home(task, idx)
        if action == "tilt":
            return self._do_tilt(task, idx)
        if action == "spin":
            return self._do_spin(task, idx)
        if action == "suction":
            self.get_logger().warn(
                f"[{idx}] action 'suction' not wired in this stack, skipping"
            )
            duration = float(task.get("duration", 0.0) or 0.0)
            if duration > 0.0:
                time.sleep(duration)
            return True

        self.get_logger().warn(f"[{idx}] unknown action '{action}', skipping")
        return True

    def _do_move(self, task: Dict[str, Any], idx: int) -> bool:
        x_mm = self._to_mm(task.get("x", self.current_pos["x"]))
        y_mm = self._to_mm(task.get("y", self.current_pos["y"]))
        z_mm = self._to_mm(task.get("z", self.current_pos["z"]))
        tilt_rad = self._read_optional_angle_rad(task, "tilt", "a")
        spin_rad = self._read_optional_angle_rad(task, "spin", "c")
        use_orientation = tilt_rad is not None or spin_rad is not None
        target_tilt = self.current_tilt if tilt_rad is None else tilt_rad
        target_spin = self.current_spin if spin_rad is None else spin_rad
        duration = max(0.0, float(task.get("duration", self.default_duration_s)))
        return self._move_interpolated(
            x_mm,
            y_mm,
            z_mm,
            duration,
            idx,
            target_tilt=target_tilt,
            target_spin=target_spin,
            use_orientation=use_orientation,
        )

    def _do_home(self, task: Dict[str, Any], idx: int) -> bool:
        duration = max(0.0, float(task.get("duration", self.default_duration_s)))
        return self._move_interpolated(
            self.home["x"], self.home["y"], self.home["z"], duration, idx
        )

    def _do_tilt(self, task: Dict[str, Any], idx: int) -> bool:
        target_tilt = self._read_optional_angle_rad(task, "value", "tilt", "a")
        if target_tilt is None:
            self.get_logger().warn(f"[{idx}] tilt action missing value, skipping")
            return True
        duration = max(0.0, float(task.get("duration", self.default_duration_s)))
        return self._move_interpolated(
            self.current_pos["x"],
            self.current_pos["y"],
            self.current_pos["z"],
            duration,
            idx,
            target_tilt=target_tilt,
            target_spin=self.current_spin,
            use_orientation=True,
        )

    def _do_spin(self, task: Dict[str, Any], idx: int) -> bool:
        target_spin = self._read_optional_angle_rad(task, "value", "spin", "c")
        if target_spin is None:
            self.get_logger().warn(f"[{idx}] spin action missing value, skipping")
            return True
        duration = max(0.0, float(task.get("duration", self.default_duration_s)))
        return self._move_interpolated(
            self.current_pos["x"],
            self.current_pos["y"],
            self.current_pos["z"],
            duration,
            idx,
            target_tilt=self.current_tilt,
            target_spin=target_spin,
            use_orientation=True,
        )

    def _do_wait(self, task: Dict[str, Any]) -> bool:
        seconds = float(task.get("seconds", 1.0))
        time.sleep(max(0.0, seconds))
        return True

    def _move_interpolated(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        duration: float,
        idx: int,
        target_tilt: float | None = None,
        target_spin: float | None = None,
        use_orientation: bool = False,
    ) -> bool:
        x0 = self.current_pos["x"]
        y0 = self.current_pos["y"]
        z0 = self.current_pos["z"]
        tilt0 = self.current_tilt
        spin0 = self.current_spin
        tilt1 = tilt0 if target_tilt is None else target_tilt
        spin1 = spin0 if target_spin is None else target_spin

        dx = x_mm - x0
        dy = y_mm - y0
        dz = z_mm - z0

        if duration <= 0.0:
            if not self._call_move_pose_service(
                x_mm, y_mm, z_mm, tilt1, spin1, use_orientation, idx
            ):
                return False
            self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
            if use_orientation:
                self.current_tilt = tilt1
                self.current_spin = spin1
            return True

        steps = max(1, int(math.ceil(duration * self.motion_rate_hz)))
        step_dt = duration / steps

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
            if not self._call_custom_trajectory_service(
                trajectory_points, step_ms, idx
            ):
                return False

            self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
            # Service returns when accepted; sleep for commanded duration to preserve ordering.
            time.sleep(duration)
            return True

        for step in range(1, steps + 1):
            alpha = step / steps
            xi = x0 + dx * alpha
            yi = y0 + dy * alpha
            zi = z0 + dz * alpha
            tilt_i = tilt0 + (tilt1 - tilt0) * alpha
            spin_i = spin0 + (spin1 - spin0) * alpha

            tick_start = time.monotonic()
            if not self._call_move_pose_service(
                xi, yi, zi, tilt_i, spin_i, use_orientation, idx
            ):
                return False

            elapsed = time.monotonic() - tick_start
            remaining = step_dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

        self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
        if use_orientation:
            self.current_tilt = tilt1
            self.current_spin = spin1
        return True

    def _call_custom_trajectory_service(
        self, trajectory: List[Point], step_ms: int, idx: int
    ) -> bool:
        return self.call_custom_trajectory(trajectory, step_ms, f"[{idx}]")

    def _call_move_pose_service(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        tilt_rad: float,
        spin_rad: float,
        use_orientation: bool,
        idx: int,
    ) -> bool:
        ok = self.call_move_pose(
            x_mm,
            y_mm,
            z_mm,
            tilt_rad,
            spin_rad,
            use_orientation,
            f"[{idx}]",
        )
        if ok:
            self.get_logger().info(
                f"[{idx}] moved to ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f})"
            )
        return ok

    def _tasks_require_orientation(self, tasks: List[Dict[str, Any]]) -> bool:
        for task in tasks:
            if not isinstance(task, dict):
                continue
            action = str(task.get("action", "")).lower().strip()
            if action in {"tilt", "spin"}:
                return True
            if any(key in task for key in ("tilt", "spin", "a", "c")):
                return True
        return False

    def _read_optional_angle_rad(
        self, task: Dict[str, Any], *keys: str
    ) -> Optional[float]:
        for key in keys:
            if key in task and task[key] is not None:
                return float(task[key])
        return None

    def _to_mm(self, value: Any) -> float:
        return float(value) * self.scale_to_mm


def main(args=None) -> int:
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print(
            "Usage: ros2 run delta_robot_task_executor json_task_sequencer <tasks.json>"
        )
        rclpy.shutdown()
        return 1

    node = JsonTaskSequencer(sys.argv[1])
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
