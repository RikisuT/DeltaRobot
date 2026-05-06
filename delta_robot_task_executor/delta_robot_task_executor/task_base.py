"""Shared helpers for task playback nodes."""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from rclpy.node import Node

from deltarobot_interfaces.action import ExecuteTrajectory
from deltarobot_interfaces.srv import MoveToPose, PlayCustomTrajectory, GetCommandedPose
import time


class TaskExecutorBase(Node):
    """Common service helpers for task playback."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.move_to_pose_client = None
        self.play_custom_trajectory_client = None
        self.execute_trajectory_client = None
        self.get_commanded_pose_client = None
        self.use_execute_action = False

    def setup_services(
        self,
        move_to_pose_service: str,
        play_custom_trajectory_service: str,
        execute_trajectory_action: str,
        get_commanded_pose_service: str,
    ) -> None:
        self.move_to_pose_client = self.create_client(MoveToPose, move_to_pose_service)
        self.play_custom_trajectory_client = self.create_client(
            PlayCustomTrajectory, play_custom_trajectory_service
        )
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, execute_trajectory_action
        )
        self.get_commanded_pose_client = self.create_client(
            GetCommandedPose, get_commanded_pose_service
        )

    def wait_for_services(self, move_to_pose_service: str, timeout_sec: float) -> bool:
        if not self.move_to_pose_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Service not available: {move_to_pose_service}")
            return False
        return True

    def check_custom_trajectory(
        self,
        play_custom_trajectory_service: str,
        execute_trajectory_action: str,
        timeout_sec: float,
    ) -> bool:
        if self.execute_trajectory_client.wait_for_server(timeout_sec=timeout_sec):
            self.use_execute_action = True
            self.get_logger().info(
                f"Using ExecuteTrajectory action: {execute_trajectory_action}"
            )
            return True

        self.use_execute_action = False
        available = self.play_custom_trajectory_client.wait_for_service(timeout_sec=timeout_sec)
        if available:
            self.get_logger().info(
                f"Using batched trajectory service: {play_custom_trajectory_service}"
            )
        else:
            self.get_logger().warn(
                "Batched trajectory service not available, falling back to point-wise MoveToPose"
            )
        return available

    def call_custom_trajectory(
        self, trajectory: List[Point], step_ms: int, label: str
    ) -> bool:
        if self.use_execute_action:
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory
            goal.step_ms = int(step_ms)

            send_future = self.execute_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            if not send_future.done() or send_future.result() is None:
                self.get_logger().error(f"{label}: ExecuteTrajectory action send failed")
                return False

            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"{label}: ExecuteTrajectory goal rejected")
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            if not result_future.done() or result_future.result() is None:
                self.get_logger().error(f"{label}: ExecuteTrajectory action timed out")
                return False

            result = result_future.result().result
            if not result.success:
                self.get_logger().error(f"{label}: ExecuteTrajectory failed: {result.message}")
                return False
            return True

        request = PlayCustomTrajectory.Request()
        request.trajectory = trajectory
        request.step_ms = int(step_ms)

        future = self.play_custom_trajectory_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f"{label}: custom trajectory service failed ({len(trajectory)} point(s))"
            )
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(
                f"{label}: planner rejected custom trajectory ({len(trajectory)} point(s))"
            )
            return False
        return True

    def call_move_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        tilt_rad: float,
        spin_rad: float,
        use_orientation: bool,
        label: str,
    ) -> bool:
        request = MoveToPose.Request()
        request.target = Point(x=float(x_mm), y=float(y_mm), z=float(z_mm))
        request.tilt = float(tilt_rad)
        request.spin = float(spin_rad)
        request.use_orientation = bool(use_orientation)

        future = self.move_to_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f"{label}: move service failed at ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f})"
            )
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(
                f"{label}: planner rejected ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f})"
            )
            return False
        return True

    def fetch_current_pose(self) -> tuple[Optional[Point], float, float]:
        """Fetch the current commanded pose from the motion planner.
        Returns (Point, tilt_rad, spin_rad) or (None, 0.0, 0.0) if unavailable.
        """
        if self.get_commanded_pose_client is None:
            self.get_logger().warn("GetCommandedPose client not initialized. Defaulting to home.")
            return None, 0.0, 0.0

        if not self.get_commanded_pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("GetCommandedPose service unavailable. Defaulting to home.")
            return None, 0.0, 0.0

        request = GetCommandedPose.Request()
        future = self.get_commanded_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done() or future.result() is None:
            self.get_logger().warn("GetCommandedPose failed. Defaulting to home.")
            return None, 0.0, 0.0

        response = future.result()
        return response.pose, response.tilt, response.spin

