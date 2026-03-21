#!/usr/bin/env python3
"""JSON task sequencer for delta robot motion execution."""

import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from deltarobot_interfaces.srv import MoveToPoint, PlayCustomTrajectory


class JsonTaskSequencer(Node):
    """Execute source-compatible JSON action sequences."""

    def __init__(self, task_file: str):
        super().__init__("json_task_sequencer")

        self.declare_parameter("move_to_point_service", "delta_motion_planner/move_to_point")
        self.declare_parameter("play_custom_trajectory_service", "delta_motion_planner/play_custom_trajectory")
        self.declare_parameter("json_units", "meters")
        self.declare_parameter("default_duration_s", 1.0)
        self.declare_parameter("motion_rate_hz", 100.0)
        self.declare_parameter("home_x_mm", 0.0)
        self.declare_parameter("home_y_mm", 0.0)
        self.declare_parameter("home_z_mm", -220.0)

        self.move_to_point_service = (
            self.get_parameter("move_to_point_service").get_parameter_value().string_value
        )
        self.play_custom_trajectory_service = (
            self.get_parameter("play_custom_trajectory_service").get_parameter_value().string_value
        )
        self.json_units = self.get_parameter("json_units").get_parameter_value().string_value
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

        self.move_to_point_client = self.create_client(MoveToPoint, self.move_to_point_service)
        self.play_custom_trajectory_client = self.create_client(
            PlayCustomTrajectory, self.play_custom_trajectory_service
        )
        self.custom_trajectory_available = False

    def run(self) -> int:
        if not self.task_file.exists():
            self.get_logger().error(f"Task file not found: {self.task_file}")
            return 1

        if not self.move_to_point_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"Service not available: {self.move_to_point_service}"
            )
            return 1

        self.custom_trajectory_available = self.play_custom_trajectory_client.wait_for_service(
            timeout_sec=2.0
        )
        if self.custom_trajectory_available:
            self.get_logger().info(
                f"Using batched trajectory service: {self.play_custom_trajectory_service}"
            )
        else:
            self.get_logger().warn(
                "Batched trajectory service not available, falling back to point-wise MoveToPoint"
            )

        tasks = self._load_tasks(self.task_file)
        if tasks is None:
            return 1

        self.get_logger().info(
            f"Loaded {len(tasks)} task(s) from {self.task_file} with units={self.json_units}"
        )

        if self.custom_trajectory_available:
            return self._run_batched_tasks(tasks)

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
                duration = max(0.0, float(task.get("duration", self.default_duration_s)))
                self._append_segment_points(points, x_mm, y_mm, z_mm, duration, step_dt)
                self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
                total_duration_s += duration
                continue

            if action == "home":
                duration = max(0.0, float(task.get("duration", self.default_duration_s)))
                self._append_segment_points(
                    points, self.home["x"], self.home["y"], self.home["z"], duration, step_dt
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
        if action in {"tilt", "spin", "suction"}:
            self.get_logger().warn(
                f"[{idx}] action '{action}' not wired in this stack, skipping"
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
        duration = max(0.0, float(task.get("duration", self.default_duration_s)))
        return self._move_interpolated(x_mm, y_mm, z_mm, duration, idx)

    def _do_home(self, task: Dict[str, Any], idx: int) -> bool:
        duration = max(0.0, float(task.get("duration", self.default_duration_s)))
        return self._move_interpolated(
            self.home["x"], self.home["y"], self.home["z"], duration, idx
        )

    def _do_wait(self, task: Dict[str, Any]) -> bool:
        seconds = float(task.get("seconds", 1.0))
        time.sleep(max(0.0, seconds))
        return True

    def _move_interpolated(
        self, x_mm: float, y_mm: float, z_mm: float, duration: float, idx: int
    ) -> bool:
        x0 = self.current_pos["x"]
        y0 = self.current_pos["y"]
        z0 = self.current_pos["z"]

        dx = x_mm - x0
        dy = y_mm - y0
        dz = z_mm - z0

        if duration <= 0.0:
            if not self._call_move_service(x_mm, y_mm, z_mm, idx):
                return False
            self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
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
            if not self._call_custom_trajectory_service(trajectory_points, step_ms, idx):
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

            tick_start = time.monotonic()
            if not self._call_move_service(xi, yi, zi, idx):
                return False

            elapsed = time.monotonic() - tick_start
            remaining = step_dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

        self.current_pos.update({"x": x_mm, "y": y_mm, "z": z_mm})
        return True

    def _call_custom_trajectory_service(
        self, trajectory: List[Point], step_ms: int, idx: int
    ) -> bool:
        request = PlayCustomTrajectory.Request()
        request.trajectory = trajectory
        request.step_ms = int(step_ms)

        future = self.play_custom_trajectory_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f"[{idx}] custom trajectory service failed ({len(trajectory)} point(s))"
            )
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(
                f"[{idx}] planner rejected custom trajectory ({len(trajectory)} point(s))"
            )
            return False

        return True

    def _call_move_service(self, x_mm: float, y_mm: float, z_mm: float, idx: int) -> bool:
        request = MoveToPoint.Request()
        request.target = Point(x=float(x_mm), y=float(y_mm), z=float(z_mm))

        future = self.move_to_point_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f"[{idx}] move service failed at ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f})"
            )
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(
                f"[{idx}] planner rejected ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f})"
            )
            return False

        self.get_logger().info(
            f"[{idx}] moved to ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f})"
        )
        return True

    def _to_mm(self, value: Any) -> float:
        return float(value) * self.scale_to_mm


def main(args=None) -> int:
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run delta_robot json_task_sequencer.py <tasks.json>")
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
