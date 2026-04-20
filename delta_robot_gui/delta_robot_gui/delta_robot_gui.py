#!/usr/bin/env python3
"""Versatile delta robot GUI for Cartesian moves, G-code, and JSON tasks."""

import os
import sys
import math
import threading
import time
import json
from dataclasses import dataclass

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from PyQt5.QtCore import QEvent, QProcess, QPoint, Qt, QTimer
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QGridLayout,
    QSlider,
    QSpacerItem,
    QSizePolicy,
    QTabWidget,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
    QGraphicsDropShadowEffect,
    QTextEdit,
    QSpinBox,
    QListWidget,
    QListWidgetItem,
)
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8MultiArray, String

from deltarobot_interfaces.srv import (
    MotionDemo,
    MoveToPoint,
    MoveToPose,
    PlayDemoTrajectory,
    SetMotionMode,
)
from deltarobot_interfaces.msg import DeltaJoints


@dataclass
class SliderSpec:
    label: str
    minimum_mm: int
    maximum_mm: int
    default_mm: int


MOTOR_IDS = [1, 2, 3, 4, 5]


class MotorFeedbackNode(Node):
    def __init__(self):
        super().__init__("delta_motor_feedback_window")
        self.latest_angles = {mid: None for mid in MOTOR_IDS}
        self.create_subscription(
            DeltaJoints,
            "delta_motors/motor_position_feedback",
            self._feedback_callback,
            10,
        )

    def _feedback_callback(self, msg):
        self.latest_angles[1] = msg.theta1
        self.latest_angles[2] = msg.theta2
        self.latest_angles[3] = msg.theta3
        self.latest_angles[4] = msg.theta4
        self.latest_angles[5] = msg.theta5


class MotorAnglesWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Motor Angles")
        self.setMinimumSize(620, 300)

        self.node = MotorFeedbackNode()
        self.angle_labels = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        title = QLabel("Live motor_position_feedback")
        title.setObjectName("titleLabel")
        title.setStyleSheet("font-size: 18px; padding: 0;")
        layout.addWidget(title)

        subtitle = QLabel("Shows actual joint angles for motors 1 to 5.")
        subtitle.setObjectName("hintLabel")
        subtitle.setStyleSheet("padding: 0; background: transparent;")
        layout.addWidget(subtitle)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        grid.addWidget(QLabel("Motor"), 0, 0)
        grid.addWidget(QLabel("Actual angle"), 0, 1)

        for row, motor_id in enumerate(MOTOR_IDS, start=1):
            motor_label = QLabel(f"ID {motor_id}")
            motor_label.setStyleSheet("font-weight: 700;")
            angle_label = QLabel("Waiting for feedback...")
            angle_label.setObjectName("previewLabel")
            angle_label.setStyleSheet(
                "color: #d7ecff; padding: 6px 10px; background: rgba(42, 108, 176, 0.24); border: 1px solid rgba(96, 169, 238, 0.45); border-radius: 8px;"
            )
            grid.addWidget(motor_label, row, 0)
            grid.addWidget(angle_label, row, 1)
            self.angle_labels[motor_id] = angle_label

        layout.addLayout(grid)

        self.status_label = QLabel(
            "Listening to delta_motors/motor_position_feedback..."
        )
        self.status_label.setObjectName("feedbackLabel")
        layout.addWidget(self.status_label)

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_feedback)
        self.poll_timer.start(50)

        self._poll_feedback()

    def _format_angle(self, value: float) -> str:
        return f"{value:+.4f} rad   ({math.degrees(value):+.1f} deg)"

    def _poll_feedback(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

        latest_angles = dict(self.node.latest_angles)
        received_any = False
        for motor_id in MOTOR_IDS:
            value = latest_angles.get(motor_id)
            if value is None:
                continue
            received_any = True
            self.angle_labels[motor_id].setText(self._format_angle(value))

        if received_any:
            self.status_label.setText("Receiving live motor feedback.")

    def closeEvent(self, event):
        self.poll_timer.stop()
        self.node.destroy_node()
        event.accept()


class DeltaGuiNode(Node):
    def __init__(self):
        super().__init__("delta_robot_gui")
        self.declare_parameter(
            "move_to_point_service", "delta_motion_planner/move_to_point"
        )
        self.declare_parameter(
            "move_to_pose_service", "delta_motion_planner/move_to_pose"
        )
        self.declare_parameter(
            "set_motion_mode_service", "delta_motion_planner/set_motion_mode"
        )
        self.declare_parameter("live_target_topic", "delta_motion_planner/live_target")
        self.declare_parameter(
            "live_orientation_topic", "delta_motion_planner/live_orientation"
        )
        self.declare_parameter("motion_planner_node_name", "motion_planner")
        self.declare_parameter(
            "play_demo_trajectory_service", "delta_motion_planner/play_demo_trajectory"
        )
        self.declare_parameter(
            "motion_demo_service", "delta_motion_planner/motion_demo"
        )
        self.move_to_point_service = (
            self.get_parameter("move_to_point_service")
            .get_parameter_value()
            .string_value
        )
        self.move_to_pose_service = (
            self.get_parameter("move_to_pose_service")
            .get_parameter_value()
            .string_value
        )
        self.set_motion_mode_service = (
            self.get_parameter("set_motion_mode_service")
            .get_parameter_value()
            .string_value
        )
        self.live_target_topic = (
            self.get_parameter("live_target_topic").get_parameter_value().string_value
        )
        self.live_orientation_topic = (
            self.get_parameter("live_orientation_topic")
            .get_parameter_value()
            .string_value
        )
        self.motion_planner_node_name = (
            self.get_parameter("motion_planner_node_name")
            .get_parameter_value()
            .string_value
        )
        self.play_demo_trajectory_service = (
            self.get_parameter("play_demo_trajectory_service")
            .get_parameter_value()
            .string_value
        )
        self.motion_demo_service = (
            self.get_parameter("motion_demo_service").get_parameter_value().string_value
        )
        self.client = self.create_client(MoveToPoint, self.move_to_point_service)
        self.pose_client = self.create_client(MoveToPose, self.move_to_pose_service)
        self.set_motion_mode_client = self.create_client(
            SetMotionMode, self.set_motion_mode_service
        )
        self.live_target_publisher = self.create_publisher(
            Point, self.live_target_topic, 10
        )
        self.live_orientation_publisher = self.create_publisher(
            Float64MultiArray, self.live_orientation_topic, 10
        )
        self.joint_command_publisher = self.create_publisher(
            DeltaJoints, "delta_motors/set_joints", 10
        )
        self.torque_command_publisher = self.create_publisher(
            UInt8MultiArray, "delta_motors/torque_command", 10
        )
        self.motor_feedback_subscription = self.create_subscription(
            DeltaJoints,
            "delta_motors/motor_position_feedback",
            self._recording_motor_feedback_callback,
            10,
        )
        self.motion_planner_param_client = AsyncParameterClient(
            self, self.motion_planner_node_name
        )
        self.play_demo_trajectory_client = self.create_client(
            PlayDemoTrajectory, self.play_demo_trajectory_service
        )
        self.motion_demo_client = self.create_client(
            MotionDemo, self.motion_demo_service
        )

        # Recording mode state
        self.recording_enabled = False
        self.is_recording = False
        self.recorded_positions = []
        self.recording_lock = threading.Lock()
        self.latest_motor_positions = None
        self.recording_gui_ref = None  # Reference to GUI for thread-safe callbacks

    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def wait_for_pose_service(self, timeout_sec: float = 5.0) -> bool:
        return self.pose_client.wait_for_service(timeout_sec=timeout_sec)

    def wait_for_mode_service(self, timeout_sec: float = 5.0) -> bool:
        return self.set_motion_mode_client.wait_for_service(timeout_sec=timeout_sec)

    def wait_for_demo_trajectory_service(self, timeout_sec: float = 5.0) -> bool:
        return self.play_demo_trajectory_client.wait_for_service(
            timeout_sec=timeout_sec
        )

    def wait_for_motion_demo_service(self, timeout_sec: float = 5.0) -> bool:
        return self.motion_demo_client.wait_for_service(timeout_sec=timeout_sec)

    def _recording_motor_feedback_callback(self, msg):
        """Callback for motor feedback - records positions if recording is enabled."""
        with self.recording_lock:
            self.latest_motor_positions = {
                1: msg.theta1,
                2: msg.theta2,
                3: msg.theta3,
                4: msg.theta4,
                5: msg.theta5,
            }
            if self.is_recording:
                # Store raw position feedback (convert radians to approximate ticks for display)
                tick_positions = [
                    int((msg.theta1 * 2048 / 1.57) + 2048),
                    int((msg.theta2 * 2048 / 1.57) + 2048),
                    int((msg.theta3 * 2048 / 1.57) + 2048),
                    int((msg.theta4 * 2048 / 1.57) + 2048),
                    int((msg.theta5 * 2048 / 1.57) + 2048),
                ]
                self.recorded_positions.append(
                    {
                        "timestamp": time.time(),
                        "radians": [
                            msg.theta1,
                            msg.theta2,
                            msg.theta3,
                            msg.theta4,
                            msg.theta5,
                        ],
                        "ticks": tick_positions,
                    }
                )

    def send_torque_command(self, motor_id: int, enable: int) -> str:
        """Send TORQUE command to ESP32 board via motor control node.
        Args:
            motor_id: Motor ID (1-5)
            enable: 1 for enable, 0 for disable
        Returns:
            Command string sent
        """
        try:
            # Publish torque command [motor_id, enable]
            msg = UInt8MultiArray()
            msg.data = [motor_id, enable]
            self.torque_command_publisher.publish(msg)
            return f"TORQUE {motor_id} {enable}"
        except Exception as e:
            self.get_logger().error(f"Failed to send torque command: {str(e)}")
            return None

    def wait_for_parameter_service(self, timeout_sec: float = 1.0) -> bool:
        if self.motion_planner_param_client.services_are_ready():
            return True
        return self.motion_planner_param_client.wait_for_services(
            timeout_sec=timeout_sec
        )

    def set_planner_orientation_config(
        self, object_center_offset_m: float, enable_axis_compensation: bool
    ):
        params = [
            Parameter(
                name="tool_tip_to_object_center_offset_m",
                value=float(object_center_offset_m),
            ),
            Parameter(
                name="enable_tilt_axis_compensation",
                value=bool(enable_axis_compensation),
            ),
        ]
        return self.motion_planner_param_client.set_parameters(params)

    def send_target(self, x_m: float, y_m: float, z_m: float):
        request = MoveToPoint.Request()
        request.target = Point(x=x_m * 1000.0, y=y_m * 1000.0, z=z_m * 1000.0)
        return self.client.call_async(request)

    def send_pose(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        tilt_rad: float,
        spin_rad: float,
        use_orientation: bool = True,
    ):
        request = MoveToPose.Request()
        request.target = Point(x=x_m * 1000.0, y=y_m * 1000.0, z=z_m * 1000.0)
        request.tilt = float(tilt_rad)
        request.spin = float(spin_rad)
        request.use_orientation = bool(use_orientation)
        return self.pose_client.call_async(request)

    def set_motion_mode(self, mode: int):
        """Set motion mode: 0 = TASK_MODE, 1 = LIVE_TEACH_MODE"""
        request = SetMotionMode.Request()
        request.mode = mode
        return self.set_motion_mode_client.call_async(request)

    def publish_live_target(self, x_m: float, y_m: float, z_m: float):
        self.live_target_publisher.publish(
            Point(x=x_m * 1000.0, y=y_m * 1000.0, z=z_m * 1000.0)
        )

    def publish_live_orientation(self, tilt_rad: float, spin_rad: float):
        msg = Float64MultiArray()
        msg.data = [float(tilt_rad), float(spin_rad)]
        self.live_orientation_publisher.publish(msg)

    def play_demo_trajectory(self, demo_name: str):
        request = PlayDemoTrajectory.Request()
        request.type.data = demo_name
        return self.play_demo_trajectory_client.call_async(request)

    def set_motion_demo(self, start: bool):
        request = MotionDemo.Request()
        request.start = start
        return self.motion_demo_client.call_async(request)


class LabeledSlider(QWidget):
    def __init__(self, spec: SliderSpec, parent=None):
        super().__init__(parent)
        self.scale = 1000.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 6, 16, 6)

        header = QHBoxLayout()
        self.label = QLabel(spec.label)
        self.label.setObjectName("sliderLabel")
        self.value_label = QLabel(self._format_value(spec.default_mm))
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setObjectName("valueLabel")
        header.addWidget(self.label)
        header.addStretch(1)
        header.addWidget(self.value_label)
        layout.addLayout(header)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(spec.minimum_mm, spec.maximum_mm)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(5)
        self.slider.setValue(spec.default_mm)
        self.slider.valueChanged.connect(self._update_value_label)
        layout.addWidget(self.slider)

    def _format_value(self, value_mm: int) -> str:
        return f"{value_mm / self.scale:.3f} m"

    def _update_value_label(self, value_mm: int):
        self.value_label.setText(self._format_value(value_mm))

    def value_m(self) -> float:
        return self.slider.value() / self.scale

    def set_mm(self, value_mm: int):
        self.slider.setValue(value_mm)


class DeltaRobotGui(QMainWindow):
    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = DeltaGuiNode()
        # Subscribe to motor init status messages (published by motor_control_node)
        try:
            self.node.create_subscription(String, "delta_motors/init_status", self._on_init_status_msg, 10)
        except Exception:
            # If subscription cannot be created now, ignore - ros_spin timer will pick it up when node ready
            pass
        self.pending_future = None
        self.pending_mode_future = None
        self.pending_mode_target = 0
        self.planner_mode = 0
        self.pending_demo_future = None
        self.pending_demo_name = None
        self.pending_param_future = None
        self.demo_loop_name = None
        self.pending_file_job = None
        self.active_job_type = None
        self.active_job_name = None
        self.stop_requested = False
        self.motor_angles_window = None

        self.setWindowTitle("Delta Robot Control Center")
        self.setMinimumSize(875, 1080)
        self.is_wayland = "wayland" in QGuiApplication.platformName().lower()
        self._drag_position = QPoint()
        self._drag_active = False

        if self.is_wayland:
            self.setAttribute(Qt.WA_TranslucentBackground, True)
            self.setAutoFillBackground(False)
            self.setWindowFlag(Qt.FramelessWindowHint, True)

        self._build_ui()
        self._apply_styles()
        self._connect_services()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_service_status)
        self.status_timer.start(500)

        self.result_timer = QTimer(self)
        self.result_timer.timeout.connect(self._poll_pending_future)
        self.result_timer.start(50)

        self.mode_timer = QTimer(self)
        self.mode_timer.timeout.connect(self._refresh_mode_service_status)
        self.mode_timer.start(500)

        self.demo_status_timer = QTimer(self)
        self.demo_status_timer.timeout.connect(self._refresh_demo_service_status)
        self.demo_status_timer.start(500)

        self.mode_future_timer = QTimer(self)
        self.mode_future_timer.timeout.connect(self._poll_mode_future)
        self.mode_future_timer.start(50)

        self.demo_future_timer = QTimer(self)
        self.demo_future_timer.timeout.connect(self._poll_demo_future)
        self.demo_future_timer.start(50)

        self.param_future_timer = QTimer(self)
        self.param_future_timer.timeout.connect(self._poll_param_future)
        self.param_future_timer.start(50)

        self.param_apply_timer = QTimer(self)
        self.param_apply_timer.setSingleShot(True)
        self.param_apply_timer.timeout.connect(self._apply_orientation_settings)

        self.demo_repeat_timer = QTimer(self)
        self.demo_repeat_timer.setSingleShot(True)
        self.demo_repeat_timer.timeout.connect(self._repeat_demo_if_needed)

        self.live_publish_timer = QTimer(self)
        self.live_publish_timer.setSingleShot(True)
        self.live_publish_timer.timeout.connect(self._publish_live_target)

        self.ros_spin_timer = QTimer(self)
        self.ros_spin_timer.timeout.connect(self._spin_ros)
        self.ros_spin_timer.start(10)

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_process_output)
        self.process.finished.connect(self._process_finished)
        self.process.errorOccurred.connect(self._process_error)

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        central.setObjectName("rootSurface")

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(0)

        shell = QFrame()
        shell.setObjectName("windowShell")
        shell_layout = QVBoxLayout(shell)
        shell_layout.setContentsMargins(10, 10, 10, 10)
        shell_layout.setSpacing(10)

        if self.is_wayland:
            self.window_header = QFrame()
            self.window_header.setObjectName("windowHeader")
            self.window_header.setFixedHeight(42)
            self.window_header.installEventFilter(self)

            header_layout = QHBoxLayout(self.window_header)
            header_layout.setContentsMargins(14, 8, 10, 8)
            header_layout.setSpacing(8)

            header_title = QLabel("Delta Robot Control Center")
            header_title.setObjectName("headerTitle")
            header_subtitle = QLabel("Wayland native shell")
            header_subtitle.setObjectName("headerSubtitle")

            title_col = QVBoxLayout()
            title_col.setContentsMargins(0, 0, 0, 0)
            title_col.setSpacing(0)
            title_col.addWidget(header_title)
            title_col.addWidget(header_subtitle)

            header_layout.addLayout(title_col)
            header_layout.addStretch(1)

            self.minimize_button = QPushButton("▁")
            self.minimize_button.setObjectName("windowControlButton")
            self.minimize_button.setFixedSize(30, 24)
            self.minimize_button.clicked.connect(self.showMinimized)

            self.maximize_button = QPushButton("□")
            self.maximize_button.setObjectName("windowControlButton")
            self.maximize_button.setFixedSize(30, 24)
            self.maximize_button.clicked.connect(self._toggle_maximize)

            self.close_button = QPushButton("✕")
            self.close_button.setObjectName("windowCloseButton")
            self.close_button.setFixedSize(30, 24)
            self.close_button.clicked.connect(self.close)

            header_layout.addWidget(self.minimize_button)
            header_layout.addWidget(self.maximize_button)
            header_layout.addWidget(self.close_button)
            shell_layout.addWidget(self.window_header)

        hero = QFrame()
        hero.setObjectName("heroCard")
        hero_layout = QVBoxLayout(hero)
        hero_layout.setContentsMargins(14, 12, 14, 12)
        hero_layout.setSpacing(6)

        title = QLabel("Delta Robot Control Center")
        title.setObjectName("titleLabel")
        subtitle = QLabel(
            "Live Cartesian control, G-code playback, and JSON task runs in one focused interface."
        )
        subtitle.setObjectName("subtitleLabel")
        hero_layout.addWidget(title)
        hero_layout.addWidget(subtitle)

        status_row = QHBoxLayout()
        self.service_indicator = QLabel("Service: checking...")
        self.service_indicator.setObjectName("serviceIndicator")
        self.service_indicator.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.activity_indicator = QLabel("Idle")
        self.activity_indicator.setObjectName("activityIndicator")
        self.activity_indicator.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.mode_indicator = QLabel("Mode: live Cartesian")
        self.mode_indicator.setObjectName("modeIndicator")
        self.mode_indicator.setAlignment(Qt.AlignCenter)

        status_row.addWidget(self.service_indicator)
        status_row.addWidget(self.mode_indicator)
        status_row.addWidget(self.activity_indicator)
        hero_layout.addLayout(status_row)

        self._add_shadow(hero, blur=34, y_offset=8)
        shell_layout.addWidget(hero)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_cartesian_tab(), "Cartesian")
        self.tabs.addTab(self._build_gcode_tab(), "G-code")
        self.tabs.addTab(self._build_json_tab(), "JSON Tasks")
        self.tabs.addTab(self._build_demo_tab(), "Demos")
        self.tabs.addTab(self._build_recording_tab(), "Recording")
        self.tabs.addTab(self._build_console_tab(), "Console")
        shell_layout.addWidget(self.tabs)

        root.addWidget(shell)

    def eventFilter(self, obj, event):
        if (
            self.is_wayland
            and hasattr(self, "window_header")
            and obj is self.window_header
        ):
            if (
                event.type() == QEvent.MouseButtonPress
                and event.button() == Qt.LeftButton
            ):
                window = self.windowHandle()
                if (
                    window is not None
                    and hasattr(window, "startSystemMove")
                    and window.startSystemMove()
                ):
                    return True
                self._drag_active = True
                self._drag_position = event.globalPos() - self.frameGeometry().topLeft()
                return True
            if (
                event.type() == QEvent.MouseMove
                and self._drag_active
                and event.buttons() & Qt.LeftButton
            ):
                self.move(event.globalPos() - self._drag_position)
                return True
            if event.type() == QEvent.MouseButtonRelease:
                self._drag_active = False
                return True
            if event.type() == QEvent.MouseButtonDblClick:
                self._toggle_maximize()
                return True
        return super().eventFilter(obj, event)

    def _toggle_maximize(self):
        if self.isMaximized():
            self.showNormal()
            if hasattr(self, "maximize_button"):
                self.maximize_button.setText("□")
        else:
            self.showMaximized()
            if hasattr(self, "maximize_button"):
                self.maximize_button.setText("❐")

    def _build_cartesian_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 8, 12, 12)
        layout.setSpacing(8)

        help_box = QFrame()
        help_box.setObjectName("infoCard")
        help_layout = QVBoxLayout(help_box)
        help_layout.setContentsMargins(12, 8, 12, 8)
        help_layout.setSpacing(6)

        hint = QLabel("Recommended range: x/y around +/-0.10 m, z around -0.18 m.")
        hint.setObjectName("hintLabel")
        help_layout.addWidget(hint)

        mode_control_row = QHBoxLayout()
        mode_label = QLabel("Control Mode:")
        mode_label.setStyleSheet("font-weight: 600;")
        self.mode_button = QPushButton("TASK MODE")
        self.mode_button.setObjectName("modeButton")
        self.mode_button.setMinimumWidth(180)
        self.mode_button.clicked.connect(self._toggle_planner_mode)
        mode_control_row.addWidget(mode_label)
        mode_control_row.addWidget(self.mode_button)
        mode_control_row.addStretch(1)
        help_layout.addLayout(mode_control_row)

        self.live_move_checkbox = QCheckBox(
            "Live move while sliders change (LIVE mode only)"
        )
        self.live_move_checkbox.setChecked(True)
        self.live_move_checkbox.toggled.connect(self._on_live_mode_changed)
        self.live_move_checkbox.setStyleSheet("padding: 4px 0;")
        help_layout.addWidget(self.live_move_checkbox)
        self._add_shadow(help_box, blur=24, y_offset=6)
        layout.addWidget(help_box)

        slider_box = QGroupBox("Cartesian Target")
        slider_box.setObjectName("cardBox")
        slider_box.setMinimumHeight(180)
        slider_layout = QVBoxLayout(slider_box)
        slider_layout.setSpacing(8)

        self.x_slider = LabeledSlider(SliderSpec("X", -120, 120, 0))
        self.y_slider = LabeledSlider(SliderSpec("Y", -120, 120, 0))
        self.z_slider = LabeledSlider(SliderSpec("Z", -450, -300, -375))
        self.tilt_slider = LabeledSlider(SliderSpec("Tilt", -90, 90, 0))
        self.spin_slider = LabeledSlider(SliderSpec("Spin", -180, 180, 0))

        self.x_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.y_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.z_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.tilt_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.spin_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.x_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.y_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.z_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.tilt_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.spin_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)

        slider_layout.addWidget(self.x_slider)
        slider_layout.addWidget(self.y_slider)
        slider_layout.addWidget(self.z_slider)
        slider_layout.addWidget(self.tilt_slider)
        slider_layout.addWidget(self.spin_slider)
        layout.addWidget(slider_box)

        orientation_box = QGroupBox("Orientation / Tool Center")
        orientation_box.setObjectName("cardBox")
        orientation_layout = QFormLayout(orientation_box)
        orientation_layout.setContentsMargins(10, 12, 10, 10)
        orientation_layout.setSpacing(6)

        self.object_center_offset_spin = QDoubleSpinBox()
        self.object_center_offset_spin.setDecimals(4)
        self.object_center_offset_spin.setRange(-0.2000, 0.2000)
        self.object_center_offset_spin.setSingleStep(0.0010)
        self.object_center_offset_spin.setValue(0.0)
        self.object_center_offset_spin.setSuffix(" m")
        self.object_center_offset_spin.setToolTip(
            "Offset from tool tip to object center (e.g., half object height for axial rotation)."
        )
        self.object_center_offset_spin.valueChanged.connect(
            self._on_orientation_setting_changed
        )

        self.axis_comp_checkbox = QCheckBox("Enable tilt X/Z offset (tool + object)")
        self.axis_comp_checkbox.setChecked(True)
        self.axis_comp_checkbox.toggled.connect(self._on_orientation_setting_changed)

        self.spin_enable_checkbox = QCheckBox("Enable orientation (tilt + spin)")
        self.spin_enable_checkbox.setChecked(True)
        self.spin_enable_checkbox.toggled.connect(self._on_orientation_enabled_toggled)

        orientation_layout.addRow(
            "Object center offset:", self.object_center_offset_spin
        )
        orientation_layout.addRow("", self.axis_comp_checkbox)
        orientation_layout.addRow("", self.spin_enable_checkbox)
        layout.addWidget(orientation_box)

        self.target_preview = QLabel(
            "Target: x=0.000 m, y=0.000 m, z=-0.180 m, tilt=0.0 deg, spin=0.0 deg, obj=0.000 m"
        )
        self.target_preview.setObjectName("previewLabel")
        self.target_preview.setStyleSheet(
            "color: #89bdf1; font-size: 12px; padding: 6px 0;"
        )
        layout.addWidget(self.target_preview)

        button_row = QHBoxLayout()
        button_row.setSpacing(10)
        self.send_button = QPushButton("Send Pos")
        self.send_button.setObjectName("primaryButton")
        self.send_button.setMinimumHeight(32)
        self.send_button.clicked.connect(self._send_target_from_sliders)

        self.home_button = QPushButton("Home Pose")
        self.home_button.setObjectName("secondaryButton")
        self.home_button.setMinimumHeight(32)
        self.home_button.clicked.connect(self._home_position)

        self.zero_xy_button = QPushButton("Zero X/Y")
        self.zero_xy_button.setObjectName("secondaryButton")
        self.zero_xy_button.setMinimumHeight(32)
        self.zero_xy_button.clicked.connect(self._zero_xy)

        self.see_motors_button = QPushButton("See Motors")
        self.see_motors_button.setObjectName("secondaryButton")
        self.see_motors_button.setMinimumHeight(32)
        self.see_motors_button.clicked.connect(self._open_motor_angles_window)

        button_row.addWidget(self.send_button)
        button_row.addWidget(self.home_button)
        button_row.addWidget(self.zero_xy_button)
        button_row.addWidget(self.see_motors_button)
        layout.addLayout(button_row)

        self.feedback_label = QLabel("Ready.")
        self.feedback_label.setObjectName("feedbackLabel")
        layout.addWidget(self.feedback_label)
        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        return tab

    def _build_gcode_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        description = QLabel(
            "Pick a G-code file and run it through delta_robot/gcode_parser.py."
        )
        description.setObjectName("hintLabel")
        description.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(description)

        file_box = QGroupBox("File Selection")
        file_box.setObjectName("cardBox")
        file_layout = QHBoxLayout(file_box)
        file_layout.setContentsMargins(14, 18, 14, 14)
        file_layout.setSpacing(10)

        self.gcode_path = QLineEdit()
        self.gcode_path.setPlaceholderText("Select a .gcode or .nc file")
        self.gcode_path.setMinimumHeight(36)
        gcode_browse = QPushButton("Browse")
        gcode_browse.setObjectName("secondaryButton")
        gcode_browse.setMaximumWidth(100)
        gcode_browse.setMinimumHeight(36)
        gcode_browse.clicked.connect(self._browse_gcode_file)

        file_layout.addWidget(self.gcode_path)
        file_layout.addWidget(gcode_browse)
        layout.addWidget(file_box)
        self._add_shadow(file_box, blur=20, y_offset=4)

        settings_box = QGroupBox("Execution Settings")
        settings_box.setObjectName("cardBox")
        settings_layout = QVBoxLayout(settings_box)
        settings_layout.setContentsMargins(14, 20, 14, 14)
        settings_layout.setSpacing(16)

        self.gcode_loop_checkbox = QCheckBox("Loop this file after it finishes")
        self.gcode_loop_checkbox.setChecked(False)
        self.gcode_loop_checkbox.toggled.connect(self._on_loop_toggled)
        settings_layout.addWidget(self.gcode_loop_checkbox)

        # Units setting
        units_row = QHBoxLayout()
        units_row.setContentsMargins(0, 0, 0, 0)
        units_row.setSpacing(12)
        units_label = QLabel("Units:")
        units_label.setMinimumWidth(100)
        units_label.setStyleSheet("font-weight: 600; padding: 4px 0;")
        self.gcode_units = QComboBox()
        self.gcode_units.addItems(["meters", "millimeters"])
        self.gcode_units.setMinimumHeight(36)
        self.gcode_units.setMaximumWidth(200)
        units_row.addWidget(units_label)
        units_row.addWidget(self.gcode_units)
        units_row.addStretch(1)
        settings_layout.addLayout(units_row)

        # Motion rate setting
        rate_row = QHBoxLayout()
        rate_row.setContentsMargins(0, 0, 0, 0)
        rate_row.setSpacing(12)
        rate_label = QLabel("Motion Rate:")
        rate_label.setMinimumWidth(100)
        rate_label.setStyleSheet("font-weight: 600; padding: 4px 0;")
        self.gcode_rate = QDoubleSpinBox()
        self.gcode_rate.setRange(1.0, 1000.0)
        self.gcode_rate.setValue(100.0)
        self.gcode_rate.setSuffix(" Hz")
        self.gcode_rate.setDecimals(1)
        self.gcode_rate.setMinimumHeight(36)
        self.gcode_rate.setMaximumWidth(200)
        rate_row.addWidget(rate_label)
        rate_row.addWidget(self.gcode_rate)
        rate_row.addStretch(1)
        settings_layout.addLayout(rate_row)

        layout.addWidget(settings_box)
        self._add_shadow(settings_box, blur=22, y_offset=5)

        run_row = QHBoxLayout()
        run_row.setSpacing(10)
        self.run_gcode_button = QPushButton("Run G-code")
        self.run_gcode_button.setObjectName("primaryButton")
        self.run_gcode_button.setMinimumHeight(36)
        self.run_gcode_button.clicked.connect(self._run_gcode_file)
        self.gcode_stop_button = QPushButton("Stop")
        self.gcode_stop_button.setObjectName("secondaryButton")
        self.gcode_stop_button.setMinimumHeight(36)
        self.gcode_stop_button.setMaximumWidth(120)
        self.gcode_stop_button.clicked.connect(self._stop_process)
        run_row.addWidget(self.run_gcode_button)
        run_row.addWidget(self.gcode_stop_button)
        run_row.addStretch(1)
        layout.addLayout(run_row)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _build_json_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        description = QLabel(
            "Pick a JSON task list and run it through delta_robot/json_task_sequencer.py."
        )
        description.setObjectName("hintLabel")
        description.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(description)

        file_box = QGroupBox("File Selection")
        file_box.setObjectName("cardBox")
        file_layout = QHBoxLayout(file_box)
        file_layout.setContentsMargins(14, 18, 14, 14)
        file_layout.setSpacing(10)

        self.json_path = QLineEdit()
        self.json_path.setPlaceholderText("Select a task .json file")
        self.json_path.setMinimumHeight(36)
        json_browse = QPushButton("Browse")
        json_browse.setObjectName("secondaryButton")
        json_browse.setMaximumWidth(100)
        json_browse.setMinimumHeight(36)
        json_browse.clicked.connect(self._browse_json_file)

        file_layout.addWidget(self.json_path)
        file_layout.addWidget(json_browse)
        layout.addWidget(file_box)
        self._add_shadow(file_box, blur=20, y_offset=4)

        settings_box = QGroupBox("Execution Settings")
        settings_box.setObjectName("cardBox")
        settings_layout = QVBoxLayout(settings_box)
        settings_layout.setContentsMargins(14, 20, 14, 14)
        settings_layout.setSpacing(16)

        self.json_loop_checkbox = QCheckBox("Loop this task list after it finishes")
        self.json_loop_checkbox.setChecked(False)
        self.json_loop_checkbox.toggled.connect(self._on_loop_toggled)
        settings_layout.addWidget(self.json_loop_checkbox)

        # Units setting
        units_row = QHBoxLayout()
        units_row.setContentsMargins(0, 0, 0, 0)
        units_row.setSpacing(12)
        units_label = QLabel("Units:")
        units_label.setMinimumWidth(100)
        units_label.setStyleSheet("font-weight: 600; padding: 4px 0;")
        self.json_units = QComboBox()
        self.json_units.addItems(["meters", "millimeters"])
        self.json_units.setMinimumHeight(36)
        self.json_units.setMaximumWidth(200)
        units_row.addWidget(units_label)
        units_row.addWidget(self.json_units)
        units_row.addStretch(1)
        settings_layout.addLayout(units_row)

        # Motion rate setting
        rate_row = QHBoxLayout()
        rate_row.setContentsMargins(0, 0, 0, 0)
        rate_row.setSpacing(12)
        rate_label = QLabel("Motion Rate:")
        rate_label.setMinimumWidth(100)
        rate_label.setStyleSheet("font-weight: 600; padding: 4px 0;")
        self.json_rate = QDoubleSpinBox()
        self.json_rate.setRange(1.0, 1000.0)
        self.json_rate.setValue(100.0)
        self.json_rate.setSuffix(" Hz")
        self.json_rate.setDecimals(1)
        self.json_rate.setMinimumHeight(36)
        self.json_rate.setMaximumWidth(200)
        rate_row.addWidget(rate_label)
        rate_row.addWidget(self.json_rate)
        rate_row.addStretch(1)
        settings_layout.addLayout(rate_row)

        layout.addWidget(settings_box)
        self._add_shadow(settings_box, blur=22, y_offset=5)

        run_row = QHBoxLayout()
        run_row.setSpacing(10)
        self.run_json_button = QPushButton("Run JSON")
        self.run_json_button.setObjectName("primaryButton")
        self.run_json_button.setMinimumHeight(36)
        self.run_json_button.clicked.connect(self._run_json_file)
        self.json_stop_button = QPushButton("Stop")
        self.json_stop_button.setObjectName("secondaryButton")
        self.json_stop_button.setMinimumHeight(36)
        self.json_stop_button.setMaximumWidth(120)
        self.json_stop_button.clicked.connect(self._stop_process)
        run_row.addWidget(self.run_json_button)
        run_row.addWidget(self.json_stop_button)
        run_row.addStretch(1)
        layout.addLayout(run_row)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _build_demo_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        description = QLabel(
            "Start a single demo trajectory or run the planner's built-in demo loop."
        )
        description.setObjectName("hintLabel")
        description.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(description)

        demo_box = QGroupBox("Demo Trajectory Buttons")
        demo_box.setObjectName("cardBox")
        demo_layout = QVBoxLayout(demo_box)
        demo_layout.setContentsMargins(14, 20, 14, 14)
        demo_layout.setSpacing(14)

        selector_row = QHBoxLayout()
        selector_row.setSpacing(10)
        selector_label = QLabel("Demo:")
        selector_label.setStyleSheet("font-weight: 600;")
        self.demo_selector = QComboBox()
        self.demo_selector.addItems(["circle", "pringle", "axes", "scan", "up_down"])
        self.demo_selector.setMinimumHeight(36)
        self.demo_selector.setMaximumWidth(180)

        self.demo_run_button = QPushButton("Run Selected Demo")
        self.demo_run_button.setObjectName("primaryButton")
        self.demo_run_button.setMinimumHeight(36)
        self.demo_run_button.clicked.connect(
            lambda: self._run_demo_trajectory(self.demo_selector.currentText())
        )

        selector_row.addWidget(selector_label)
        selector_row.addWidget(self.demo_selector)
        selector_row.addWidget(self.demo_run_button)
        selector_row.addStretch(1)
        demo_layout.addLayout(selector_row)

        self.demo_loop_checkbox = QCheckBox(
            "Use auto demo loop instead of a single trajectory"
        )
        self.demo_loop_checkbox.setChecked(False)
        self.demo_loop_checkbox.toggled.connect(self._on_loop_toggled)
        demo_layout.addWidget(self.demo_loop_checkbox)

        demo_grid = QGridLayout()
        demo_grid.setHorizontalSpacing(10)
        demo_grid.setVerticalSpacing(10)
        self.demo_circle_button = QPushButton("Circle")
        self.demo_pringle_button = QPushButton("Pringle")
        self.demo_axes_button = QPushButton("Axes")
        self.demo_scan_button = QPushButton("Scan")
        self.demo_up_down_button = QPushButton("Up/Down")

        for button in (
            self.demo_circle_button,
            self.demo_pringle_button,
            self.demo_axes_button,
            self.demo_scan_button,
            self.demo_up_down_button,
        ):
            button.setObjectName("secondaryButton")
            button.setMinimumHeight(32)

        self.demo_circle_button.clicked.connect(
            lambda: self._run_demo_trajectory("circle")
        )
        self.demo_pringle_button.clicked.connect(
            lambda: self._run_demo_trajectory("pringle")
        )
        self.demo_axes_button.clicked.connect(lambda: self._run_demo_trajectory("axes"))
        self.demo_scan_button.clicked.connect(lambda: self._run_demo_trajectory("scan"))
        self.demo_up_down_button.clicked.connect(
            lambda: self._run_demo_trajectory("up_down")
        )

        demo_grid.addWidget(self.demo_circle_button, 0, 0)
        demo_grid.addWidget(self.demo_pringle_button, 0, 1)
        demo_grid.addWidget(self.demo_axes_button, 0, 2)
        demo_grid.addWidget(self.demo_scan_button, 1, 0)
        demo_grid.addWidget(self.demo_up_down_button, 1, 1)
        demo_layout.addLayout(demo_grid)

        demo_mode_row = QHBoxLayout()
        demo_mode_row.setSpacing(10)
        self.demo_mode_start_button = QPushButton("Start Demo Loop")
        self.demo_mode_start_button.setObjectName("primaryButton")
        self.demo_mode_start_button.setMinimumHeight(36)
        self.demo_mode_start_button.clicked.connect(lambda: self._set_motion_demo(True))

        self.demo_mode_stop_button = QPushButton("Stop Demo Loop")
        self.demo_mode_stop_button.setObjectName("secondaryButton")
        self.demo_mode_stop_button.setMinimumHeight(36)
        self.demo_mode_stop_button.clicked.connect(lambda: self._set_motion_demo(False))

        demo_mode_row.addWidget(self.demo_mode_start_button)
        demo_mode_row.addWidget(self.demo_mode_stop_button)
        demo_mode_row.addStretch(1)
        demo_layout.addLayout(demo_mode_row)

        layout.addWidget(demo_box)
        self._add_shadow(demo_box, blur=22, y_offset=5)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _build_recording_tab(self) -> QWidget:
        """Build the recording mode tab for passive demo recording and playback."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        description = QLabel(
            "Record motor positions by moving the robot with torque off, then playback."
        )
        description.setObjectName("hintLabel")
        description.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(description)

        # Torque control box
        torque_box = QGroupBox("Torque Control")
        torque_box.setObjectName("cardBox")
        torque_layout = QHBoxLayout(torque_box)
        torque_layout.setContentsMargins(14, 20, 14, 14)
        torque_layout.setSpacing(10)

        self.torque_off_button = QPushButton("Torque OFF (Record Mode)")
        self.torque_off_button.setObjectName("primaryButton")
        self.torque_off_button.setMinimumHeight(40)
        self.torque_off_button.clicked.connect(self._on_torque_off_clicked)

        self.torque_on_button = QPushButton("Torque ON (Normal Mode)")
        self.torque_on_button.setObjectName("secondaryButton")
        self.torque_on_button.setMinimumHeight(40)
        self.torque_on_button.clicked.connect(self._on_torque_on_clicked)

        torque_layout.addWidget(self.torque_off_button)
        torque_layout.addWidget(self.torque_on_button)
        torque_layout.addStretch(1)
        layout.addWidget(torque_box)

        # Recording control box
        record_box = QGroupBox("Recording")
        record_box.setObjectName("cardBox")
        record_layout = QVBoxLayout(record_box)
        record_layout.setContentsMargins(14, 20, 14, 14)
        record_layout.setSpacing(10)

        control_row = QHBoxLayout()
        control_row.setSpacing(10)

        self.record_button = QPushButton("Start Recording")
        self.record_button.setObjectName("primaryButton")
        self.record_button.setMinimumHeight(40)
        self.record_button.setEnabled(False)
        self.record_button.clicked.connect(self._on_record_clicked)

        self.stop_record_button = QPushButton("Stop Recording")
        self.stop_record_button.setObjectName("secondaryButton")
        self.stop_record_button.setMinimumHeight(40)
        self.stop_record_button.setEnabled(False)
        self.stop_record_button.clicked.connect(self._on_stop_record_clicked)

        self.clear_recording_button = QPushButton("Clear Recording")
        self.clear_recording_button.setObjectName("secondaryButton")
        self.clear_recording_button.setMinimumHeight(40)
        self.clear_recording_button.clicked.connect(self._on_clear_recording_clicked)

        control_row.addWidget(self.record_button)
        control_row.addWidget(self.stop_record_button)
        control_row.addWidget(self.clear_recording_button)
        control_row.addStretch(1)
        record_layout.addLayout(control_row)

        # Recording status
        status_row = QHBoxLayout()
        status_row.setSpacing(10)
        status_row.addWidget(QLabel("Recording Status:"))
        self.recording_status_label = QLabel("Idle")
        self.recording_status_label.setStyleSheet("font-weight: 600; color: #7a8fa3;")
        status_row.addWidget(self.recording_status_label)
        status_row.addStretch(1)
        record_layout.addLayout(status_row)

        layout.addWidget(record_box)

        # Display mode box
        display_box = QGroupBox("Display Mode")
        display_box.setObjectName("cardBox")
        display_layout = QHBoxLayout(display_box)
        display_layout.setContentsMargins(14, 20, 14, 14)
        display_layout.setSpacing(10)

        display_layout.addWidget(QLabel("Show recorded positions as:"))
        self.display_mode_combo = QComboBox()
        self.display_mode_combo.addItems(
            ["Motor Ticks (Raw)", "Joint Angles (Radians)"]
        )
        self.display_mode_combo.setMinimumHeight(36)
        self.display_mode_combo.setMaximumWidth(220)
        self.display_mode_combo.currentTextChanged.connect(
            self._on_display_mode_changed
        )
        display_layout.addWidget(self.display_mode_combo)
        display_layout.addStretch(1)
        layout.addWidget(display_box)

        # Recorded data display
        data_box = QGroupBox("Recorded Data")
        data_box.setObjectName("cardBox")
        data_layout = QVBoxLayout(data_box)
        data_layout.setContentsMargins(14, 20, 14, 14)
        data_layout.setSpacing(10)

        self.recording_data_list = QPlainTextEdit()
        self.recording_data_list.setReadOnly(True)
        self.recording_data_list.setMaximumHeight(200)
        self.recording_data_list.setStyleSheet(
            "background-color: #1e1e1e; color: #d4d4d4; font-family: monospace; font-size: 9px;"
        )
        data_layout.addWidget(self.recording_data_list)

        layout.addWidget(data_box)

        # Playback control box
        playback_box = QGroupBox("Playback")
        playback_box.setObjectName("cardBox")
        playback_layout = QVBoxLayout(playback_box)
        playback_layout.setContentsMargins(14, 20, 14, 14)
        playback_layout.setSpacing(10)

        playback_control_row = QHBoxLayout()
        playback_control_row.setSpacing(10)

        self.playback_button = QPushButton("Play Recorded Trajectory")
        self.playback_button.setObjectName("primaryButton")
        self.playback_button.setMinimumHeight(40)
        self.playback_button.setEnabled(False)
        self.playback_button.clicked.connect(self._on_playback_clicked)

        playback_control_row.addWidget(self.playback_button)
        playback_control_row.addStretch(1)
        playback_layout.addLayout(playback_control_row)

        # Playback speed control
        speed_row = QHBoxLayout()
        speed_row.setSpacing(10)
        speed_row.addWidget(QLabel("Playback speed (ms between points):"))
        self.playback_speed_spinbox = QSpinBox()
        self.playback_speed_spinbox.setMinimum(50)
        self.playback_speed_spinbox.setMaximum(10000)
        self.playback_speed_spinbox.setValue(100)
        self.playback_speed_spinbox.setMaximumWidth(100)
        speed_row.addWidget(self.playback_speed_spinbox)
        # Playback smoothing control (EMA alpha 0..1)
        speed_row.addWidget(QLabel("Smoothing α (0.00-1.00):"))
        self.playback_smoothing_spinbox = QDoubleSpinBox()
        self.playback_smoothing_spinbox.setDecimals(2)
        self.playback_smoothing_spinbox.setRange(0.0, 1.0)
        self.playback_smoothing_spinbox.setSingleStep(0.05)
        self.playback_smoothing_spinbox.setValue(0.6)
        self.playback_smoothing_spinbox.setMaximumWidth(100)
        speed_row.addWidget(self.playback_smoothing_spinbox)
        speed_row.addStretch(1)
        playback_layout.addLayout(speed_row)

        # Save/Load buttons
        file_row = QHBoxLayout()
        file_row.setSpacing(10)

        self.save_recording_button = QPushButton("Save to File")
        self.save_recording_button.setObjectName("secondaryButton")
        self.save_recording_button.setMinimumHeight(36)
        self.save_recording_button.setEnabled(False)
        self.save_recording_button.clicked.connect(self._on_save_recording_clicked)

        self.load_recording_button = QPushButton("Load from File")
        self.load_recording_button.setObjectName("secondaryButton")
        self.load_recording_button.setMinimumHeight(36)
        self.load_recording_button.clicked.connect(self._on_load_recording_clicked)

        file_row.addWidget(self.save_recording_button)
        file_row.addWidget(self.load_recording_button)
        file_row.addStretch(1)
        playback_layout.addLayout(file_row)

        layout.addWidget(playback_box)
        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _on_torque_off_clicked(self):
        """Turn torque off for all motors (passive mode)."""
        MOTOR_IDS = [1, 2, 3, 4, 5]

        # Send TORQUE commands for each motor
        commands_sent = []
        try:
            for motor_id in MOTOR_IDS:
                cmd_result = self.node.send_torque_command(motor_id, 0)
                if cmd_result:
                    commands_sent.append(f"Motor {motor_id}: OFF")
                time.sleep(0.1)  # Small delay between commands
        except Exception as e:
            QMessageBox.warning(
                self, "Error", f"Failed to send torque commands: {str(e)}"
            )
            return

        self.node.recording_enabled = True
        self.torque_off_button.setEnabled(False)
        self.torque_on_button.setEnabled(True)
        self.record_button.setEnabled(True)

        msg_text = (
            "All motors are now in passive mode (torque OFF).\nYou can manually move the robot to record positions.\n\n"
            + "\n".join(commands_sent)
        )
        QMessageBox.information(self, "Torque OFF", msg_text)

    def _on_torque_on_clicked(self):
        """Turn torque on for all motors (normal mode)."""
        MOTOR_IDS = [1, 2, 3, 4, 5]

        # Send TORQUE commands for each motor
        commands_sent = []
        try:
            for motor_id in MOTOR_IDS:
                cmd_result = self.node.send_torque_command(motor_id, 1)
                if cmd_result:
                    commands_sent.append(f"Motor {motor_id}: ON")
                time.sleep(0.1)  # Small delay between commands
        except Exception as e:
            QMessageBox.warning(
                self, "Error", f"Failed to send torque commands: {str(e)}"
            )
            return

        self.node.recording_enabled = False
        self.node.is_recording = False
        self.torque_off_button.setEnabled(True)
        self.torque_on_button.setEnabled(False)
        self.record_button.setEnabled(False)
        self.stop_record_button.setEnabled(False)
        self.recording_status_label.setText("Idle - Torque ON")
        self.recording_status_label.setStyleSheet("font-weight: 600; color: #7a8fa3;")

        msg_text = "All motors are now in normal mode (torque ON).\n\n" + "\n".join(
            commands_sent
        )
        QMessageBox.information(self, "Torque ON", msg_text)

    def _on_record_clicked(self):
        """Start recording motor positions."""
        if not self.node.recording_enabled:
            QMessageBox.warning(
                self, "Error", "Turn torque OFF first to enable recording mode."
            )
            return

        self.node.is_recording = True
        self.node.recorded_positions = []
        self.record_button.setEnabled(False)
        self.stop_record_button.setEnabled(True)
        self.torque_off_button.setEnabled(False)
        self.recording_status_label.setText("RECORDING...")
        self.recording_status_label.setStyleSheet("font-weight: 600; color: #ff6b6b;")
        self.recording_data_list.clear()
        self.recording_data_list.appendPlainText("Recording started...")

    def _on_stop_record_clicked(self):
        """Stop recording motor positions."""
        self.node.is_recording = False
        self.record_button.setEnabled(True)
        self.stop_record_button.setEnabled(False)
        self.torque_off_button.setEnabled(True)
        self.recording_status_label.setText(
            f"Stopped - {len(self.node.recorded_positions)} points recorded"
        )
        self.recording_status_label.setStyleSheet("font-weight: 600; color: #51cf66;")
        self.save_recording_button.setEnabled(len(self.node.recorded_positions) > 0)
        self.playback_button.setEnabled(len(self.node.recorded_positions) > 0)
        self._update_recording_display()

    def _on_clear_recording_clicked(self):
        """Clear the recorded positions."""
        self.node.recorded_positions = []
        self.recording_status_label.setText("Cleared")
        self.recording_status_label.setStyleSheet("font-weight: 600; color: #7a8fa3;")
        self.recording_data_list.clear()
        self.save_recording_button.setEnabled(False)
        self.playback_button.setEnabled(False)

    def _on_display_mode_changed(self):
        """Update recording display when display mode changes."""
        self._update_recording_display()

    def _update_recording_display(self):
        """Update the recording data display."""
        self.recording_data_list.clear()
        if not self.node.recorded_positions:
            return

        is_radians = self.display_mode_combo.currentIndex() == 1
        for i, record in enumerate(self.node.recorded_positions):
            if is_radians:
                angles = record["radians"]
                line = f"[{i:3d}] θ1={angles[0]:+.4f} θ2={angles[1]:+.4f} θ3={angles[2]:+.4f} θ4={angles[3]:+.4f} θ5={angles[4]:+.4f}"
            else:
                ticks = record["ticks"]
                line = f"[{i:3d}] T1={ticks[0]:4d} T2={ticks[1]:4d} T3={ticks[2]:4d} T4={ticks[3]:4d} T5={ticks[4]:4d}"
            self.recording_data_list.appendPlainText(line)

    def _on_playback_clicked(self):
        """Playback the recorded trajectory."""
        if not self.node.recorded_positions:
            QMessageBox.warning(self, "Error", "No recording to playback.")
            return

        if not self.node.recording_enabled:
            QMessageBox.warning(
                self, "Error", "Turn torque OFF first (to unlock motors for playback)."
            )
            return

        speed_ms = self.playback_speed_spinbox.value()
        smoothing_alpha = float(self.playback_smoothing_spinbox.value()) if hasattr(self, 'playback_smoothing_spinbox') else 0.0
        thread = threading.Thread(target=self._playback_thread, args=(speed_ms, smoothing_alpha))
        thread.daemon = True
        thread.start()

    def _playback_thread(self, speed_ms, smoothing_alpha=0.0):
        """Playback recorded trajectory in a separate thread with spline interpolation for smooth motion.
        Applies an exponential moving average (EMA) to the interpolated joint values for smoother playback.
        Qt-safe."""
        import numpy as np
        from scipy.interpolate import CubicSpline
        from PyQt5.QtCore import QTimer

        # Disable button in main thread
        QTimer.singleShot(0, lambda: self.playback_button.setEnabled(False))
        try:
            positions = self.node.recorded_positions
            if len(positions) < 2:
                return
            t = np.arange(len(positions))
            thetas = np.array([rec["radians"] for rec in positions])  # shape: (N, 5)
            n_interp = 10  # number of interpolated points between each pair
            t_interp = np.linspace(
                0, len(positions) - 1, num=(len(positions) - 1) * n_interp + 1
            )
            splines = [CubicSpline(t, thetas[:, i]) for i in range(5)]
            theta_interp = np.stack([spl(t_interp) for spl in splines], axis=1)

            # EMA smoothing state (start at first interpolated sample)
            ema = np.array(theta_interp[0], dtype=float)
            alpha = float(smoothing_alpha)
            if alpha < 0.0:
                alpha = 0.0
            if alpha > 1.0:
                alpha = 1.0

            for joint_vals in theta_interp:
                arr = np.array(joint_vals, dtype=float)
                if alpha > 0.0:
                    smoothed = alpha * arr + (1.0 - alpha) * ema
                else:
                    smoothed = arr
                ema = smoothed

                msg = DeltaJoints()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.theta1 = float(smoothed[0])
                msg.theta2 = float(smoothed[1])
                msg.theta3 = float(smoothed[2])
                msg.theta4 = float(smoothed[3])
                msg.theta5 = float(smoothed[4])
                self.node.joint_command_publisher.publish(msg)
                time.sleep((speed_ms / n_interp) / 1000.0)
        finally:
            # Re-enable button in main thread
            QTimer.singleShot(0, lambda: self.playback_button.setEnabled(True))

    def _on_save_recording_clicked(self):
        """Save recorded trajectory to a JSON file."""
        if not self.node.recorded_positions:
            QMessageBox.warning(self, "Error", "No recording to save.")
            return

        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Recording", "", "JSON Files (*.json)"
        )
        if not filepath:
            return

        try:
            data = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "num_points": len(self.node.recorded_positions),
                "points": self.node.recorded_positions,
            }
            with open(filepath, "w") as f:
                json.dump(data, f, indent=2)
            QMessageBox.information(self, "Saved", f"Recording saved to:\n{filepath}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save:\n{str(e)}")

    def _on_load_recording_clicked(self):
        """Load recorded trajectory from a JSON file."""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Recording", "", "JSON Files (*.json)"
        )
        if not filepath:
            return

        try:
            with open(filepath, "r") as f:
                data = json.load(f)

            self.node.recorded_positions = data["points"]
            self.recording_status_label.setText(
                f"Loaded - {len(self.node.recorded_positions)} points"
            )
            self.recording_status_label.setStyleSheet(
                "font-weight: 600; color: #51cf66;"
            )
            self.save_recording_button.setEnabled(True)
            self.playback_button.setEnabled(True)
            self._update_recording_display()
            QMessageBox.information(
                self,
                "Loaded",
                f"Loaded {len(self.node.recorded_positions)} recorded points.",
            )
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load:\n{str(e)}")

    def _build_console_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        info = QLabel("Process output and errors appear here.")
        info.setObjectName("hintLabel")
        info.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(info)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setObjectName("consoleBox")
        self.console.setMinimumHeight(300)
        layout.addWidget(self.console)

        clear_row = QHBoxLayout()
        clear_row.setSpacing(8)
        clear_row.addStretch(1)
        clear_button = QPushButton("Clear Console")
        clear_button.setObjectName("secondaryButton")
        clear_button.setMinimumHeight(36)
        clear_button.setMaximumWidth(140)
        clear_button.clicked.connect(self.console.clear)
        clear_row.addWidget(clear_button)
        layout.addLayout(clear_row)

        return tab

    def _apply_styles(self):
        self.setStyleSheet(
            """
            QWidget {
                background: transparent;
                color: #e7eef7;
                font-family: "SF Pro Display Nerd Font", "SF Pro Text Nerd Font", "SF Pro Display", "SF Pro Text", "Noto Sans", "DejaVu Sans", sans-serif;
                font-size: 13px;
            }
            QWidget#rootSurface {
                background: transparent;
            }
            QFrame#windowShell {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #0a1017, stop:0.55 #101b27, stop:1 #0b1320);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 30px;
            }
            QFrame#windowHeader {
                background: rgba(255, 255, 255, 0.03);
                border: 1px solid rgba(255, 255, 255, 0.05);
                border-radius: 18px;
            }
            QLabel#headerTitle {
                color: #f4f8fc;
                font-size: 14px;
                font-weight: 700;
                letter-spacing: 0.2px;
                font-family: "SF Pro Display Nerd Font", "SF Pro Display", "SF Pro Text Nerd Font", "Noto Sans", sans-serif;
            }
            QLabel#headerSubtitle {
                color: #90a2b5;
                font-size: 11px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QPushButton#windowControlButton,
            QPushButton#windowCloseButton {
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
                padding: 0;
                font-weight: 700;
                background: rgba(255, 255, 255, 0.05);
                color: #e7eef7;
            }
            QPushButton#windowControlButton:hover {
                background: rgba(255, 255, 255, 0.12);
            }
            QPushButton#windowCloseButton {
                background: rgba(186, 74, 74, 0.18);
                color: #ffd9d9;
                border: 1px solid rgba(186, 74, 74, 0.28);
            }
            QPushButton#windowCloseButton:hover {
                background: rgba(220, 92, 92, 0.30);
            }
            QFrame#heroCard,
            QFrame#infoCard,
            QGroupBox#cardBox,
            QGroupBox {
                background: rgba(16, 24, 35, 0.92);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 10px;
                margin-top: 8px;
            }
            QFrame#heroCard:hover,
            QFrame#infoCard:hover,
            QGroupBox#cardBox:hover {
                border-color: rgba(255, 255, 255, 0.14);
                background: rgba(16, 24, 35, 0.96);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                top: 2px;
                padding: 0 8px;
                color: #e8f0f8;
                font-weight: 700;
                font-size: 13px;
            }
            QLabel#titleLabel {
                font-size: 27px;
                font-weight: 800;
                letter-spacing: 0.2px;
                color: #f4f8fc;
                font-family: "SF Pro Display Nerd Font", "SF Pro Display", "SF Pro Text Nerd Font", "Noto Sans", sans-serif;
            }
            QLabel#subtitleLabel {
                color: #98a9ba;
                margin-bottom: 2px;
                font-size: 13px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QLabel#hintLabel {
                color: #9aa9b9;
                font-size: 12px;
                padding: 10px 8px;
                line-height: 1.5;
                background: transparent;
            }
            QLabel#serviceIndicator,
            QLabel#activityIndicator,
            QLabel#modeIndicator {
                color: #d7e2ec;
                padding: 7px 12px;
                background: rgba(255, 255, 255, 0.04);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 999px;
                font-size: 12px;
                font-weight: 500;
            }
            QTabWidget::pane {
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 10px;
                top: -1px;
                background: rgba(16, 24, 35, 0.88);
            }
            QTabBar::tab {
                background: rgba(255, 255, 255, 0.05);
                color: #b8c6d6;
                padding: 11px 18px;
                margin-right: 2px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                min-width: 100px;
                font-weight: 500;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QTabBar::tab:hover {
                background: rgba(255, 255, 255, 0.08);
                color: #d4dfe9;
            }
            QTabBar::tab:selected {
                background: rgba(58, 141, 222, 0.35);
                color: #f7fbff;
                font-weight: 600;
            }
            QLabel#sliderLabel {
                font-weight: 700;
                color: #eef5fb;
            }
            QLabel#valueLabel {
                color: #d7ecff;
                min-width: 104px;
                min-height: 22px;
                padding: 2px 7px;
                margin-right: 2px;
                background: rgba(42, 108, 176, 0.42);
                border: 1px solid rgba(96, 169, 238, 0.60);
                border-radius: 8px;
                font-size: 11px;
                font-weight: 600;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", "DejaVu Sans", sans-serif;
            }
            QSlider::groove:horizontal {
                border: none;
                height: 11px;
                background: #16202c;
                border-radius: 6px;
            }
            QSlider::sub-page:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                border-radius: 6px;
            }
            QSlider::handle:horizontal {
                background: #f7fbff;
                border: 2px solid #5ba3ec;
                width: 20px;
                margin: -6px 0;
                border-radius: 10px;
            }
            QLineEdit, QComboBox, QDoubleSpinBox, QTextEdit {
                background: rgba(8, 14, 21, 0.85);
                border: 1px solid rgba(255, 255, 255, 0.12);
                border-radius: 8px;
                padding: 10px 12px;
                color: #e7eef7;
                selection-background-color: rgba(58, 141, 222, 0.40);
                font-size: 13px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QTextEdit:focus {
                background: rgba(8, 14, 21, 0.95);
                border: 2px solid rgba(58, 141, 222, 0.50);
            }
            QComboBox::drop-down {
                border: none;
                padding-right: 8px;
                subcontrol-position: center right;
                subcontrol-origin: padding;
                width: 24px;
            }
            QComboBox::down-arrow {
                image: none;
                color: #89bdf1;
                width: 8px;
                height: 8px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 24px;
                border: none;
                background: rgba(58, 141, 222, 0.12);
                border-left: 1px solid rgba(255, 255, 255, 0.08);
            }
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
                background: rgba(58, 141, 222, 0.20);
            }
            QPlainTextEdit#consoleBox {
                background: rgba(6, 11, 18, 0.95);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
                font-family: "FiraCode Nerd Font", "Fira Code", "SFMono Nerd Font", "DejaVu Sans Mono", "Courier New", monospace;
                font-size: 11px;
                color: #a0b5c7;
                padding: 8px;
            }
            QPlainTextEdit#consoleBox:focus {
                border: 1px solid rgba(58, 141, 222, 0.30);
                background: rgba(6, 11, 18, 0.98);
            }
            QPushButton {
                border: none;
                border-radius: 14px;
                padding: 12px 16px;
                font-weight: 700;
                color: #e9f1f8;
                background: rgba(255, 255, 255, 0.08);
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QPushButton:hover {
                background: rgba(255, 255, 255, 0.14);
            }
            QPushButton:pressed {
                background: rgba(255, 255, 255, 0.18);
            }
            QPushButton#primaryButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                color: white;
                padding: 14px 18px;
                font-weight: 600;
            }
            QPushButton#primaryButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #60adef, stop:1 #3b8ae0);
            }
            QPushButton#secondaryButton {
                background: rgba(32, 49, 66, 0.70);
                padding: 12px 16px;
                font-weight: 500;
            }
            QPushButton#secondaryButton:hover {
                background: rgba(32, 49, 66, 0.95);
            }
            QPushButton#modeButton {
                background: rgba(255, 152, 0, 0.30);
                color: #ffc857;
                border: 1px solid rgba(255, 152, 0, 0.50);
                font-weight: 700;
                padding: 10px 18px;
            }
            QPushButton#modeButton:hover {
                background: rgba(255, 152, 0, 0.45);
                border: 1px solid rgba(255, 152, 0, 0.70);
            }
            QLabel#feedbackLabel {
                color: #c1cfdb;
                padding: 10px 2px;
                line-height: 1.4;
                font-size: 12px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QCheckBox {
                color: #d6e1eb;
                spacing: 10px;
                font-weight: 500;
                padding: 4px 0;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
            QCheckBox::indicator:unchecked {
                background: rgba(255, 255, 255, 0.08);
                border: 1px solid rgba(255, 255, 255, 0.15);
                border-radius: 4px;
            }
            QCheckBox::indicator:checked {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                border: 1px solid rgba(58, 141, 222, 0.6);
                border-radius: 4px;
            }
            QCheckBox#loopCheckbox {
                margin-top: 4px;
            }
            """
        )

    def _connect_services(self):
        self._refresh_service_status()
        self._refresh_mode_service_status()
        self._refresh_demo_service_status()
        self._sync_mode_controls()
        self._apply_orientation_settings()

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def _on_init_status_msg(self, msg):
        """ROS callback for initialization status messages from motor_control_node.
        Schedule a GUI warning dialog on the main thread.
        """
        text = getattr(msg, "data", str(msg))
        QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Motor Init Warning", text))

    def _refresh_service_status(self):
        available = self.node.wait_for_service(timeout_sec=0.0)
        pose_available = self.node.wait_for_pose_service(timeout_sec=0.0)
        if available:
            service_label = "move_to_point"
            if pose_available:
                service_label += " + move_to_pose"
            self.service_indicator.setText(f"Service: connected ({service_label})")
            self.service_indicator.setStyleSheet(
                "color: #bdf3c8; background: rgba(61, 156, 85, 0.16); border: 1px solid rgba(61, 156, 85, 0.28);"
            )
            self.send_button.setEnabled(self.planner_mode == 0)
        else:
            self.service_indicator.setText(
                "Service: waiting for delta_motion_planner/move_to_point"
            )
            self.service_indicator.setStyleSheet(
                "color: #ffd7a8; background: rgba(159, 108, 29, 0.18); border: 1px solid rgba(159, 108, 29, 0.26);"
            )
            self.send_button.setEnabled(False)

    def _refresh_mode_service_status(self):
        available = self.node.wait_for_mode_service(timeout_sec=0.0)
        self.mode_button.setEnabled(available)
        self._refresh_demo_service_status()

    def _refresh_demo_service_status(self):
        demo_available = self.node.wait_for_demo_trajectory_service(timeout_sec=0.0)
        motion_demo_available = self.node.wait_for_motion_demo_service(timeout_sec=0.0)
        if hasattr(self, "demo_circle_button"):
            for button in (
                self.demo_circle_button,
                self.demo_pringle_button,
                self.demo_axes_button,
                self.demo_scan_button,
                self.demo_up_down_button,
            ):
                button.setEnabled(demo_available)
        if hasattr(self, "demo_mode_start_button"):
            self.demo_mode_start_button.setEnabled(motion_demo_available)
        if hasattr(self, "demo_mode_stop_button"):
            self.demo_mode_stop_button.setEnabled(motion_demo_available)

    def _on_loop_toggled(self, checked: bool):
        sender = self.sender()
        if sender is getattr(self, "gcode_loop_checkbox", None):
            self.feedback_label.setText(
                "G-code loop enabled." if checked else "G-code loop disabled."
            )
            return
        if sender is getattr(self, "json_loop_checkbox", None):
            self.feedback_label.setText(
                "JSON loop enabled." if checked else "JSON loop disabled."
            )
            return
        if sender is getattr(self, "demo_loop_checkbox", None):
            if checked:
                self.feedback_label.setText(
                    "Demo loop enabled: selected demo will repeat."
                )
            else:
                self.demo_repeat_timer.stop()
                self.demo_loop_name = None
                self.feedback_label.setText("Demo loop disabled.")
            return
        self.feedback_label.setText("Loop setting updated.")

    def _run_demo_trajectory(self, demo_name: str):
        if not isinstance(demo_name, str) or not demo_name.strip():
            self.feedback_label.setText("Invalid demo selection.")
            return

        if not self.node.wait_for_demo_trajectory_service(timeout_sec=0.1):
            QMessageBox.warning(
                self,
                "Service unavailable",
                "play_demo_trajectory service is not available yet.",
            )
            return

        if self.pending_demo_future is not None:
            self.feedback_label.setText("A demo command is already in progress.")
            return

        self.pending_demo_name = demo_name
        self.pending_demo_future = self.node.play_demo_trajectory(demo_name)
        self.active_job_type = "demo"
        self.active_job_name = demo_name
        self.demo_loop_name = demo_name if self.demo_loop_checkbox.isChecked() else None
        self.feedback_label.setText(f"Starting demo trajectory: {demo_name}")

    def _set_motion_demo(self, start: bool):
        if not self.node.wait_for_motion_demo_service(timeout_sec=0.1):
            QMessageBox.warning(
                self, "Service unavailable", "motion_demo service is not available yet."
            )
            return

        future = self.node.set_motion_demo(start)
        self.feedback_label.setText(
            "Demo loop started." if start else "Demo loop stopped."
        )
        self.pending_demo_future = future
        self.pending_demo_name = "motion_demo"
        if not start:
            self.demo_repeat_timer.stop()
            self.demo_loop_name = None
            if (
                hasattr(self, "demo_loop_checkbox")
                and self.demo_loop_checkbox.isChecked()
            ):
                self.demo_loop_checkbox.setChecked(False)

    def _repeat_demo_if_needed(self):
        if not self.demo_loop_checkbox.isChecked() or not self.demo_loop_name:
            return
        if self.pending_demo_future is not None:
            self.demo_repeat_timer.start(500)
            return
        self._run_demo_trajectory(self.demo_loop_name)

    def _poll_demo_future(self):
        if self.pending_demo_future is None or not self.pending_demo_future.done():
            return

        future = self.pending_demo_future
        self.pending_demo_future = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.feedback_label.setText(f"Demo request failed: {exc}")
            return

        response_success = (
            True if not hasattr(response, "success") else bool(response.success)
        )
        if response_success:
            if self.pending_demo_name == "motion_demo":
                self.feedback_label.setText("Demo loop command accepted.")
            elif self.pending_demo_name:
                self.feedback_label.setText(
                    f"Demo trajectory queued: {self.pending_demo_name}"
                )
                if self.demo_loop_checkbox.isChecked() and self.demo_loop_name:
                    self.demo_repeat_timer.start(700)
        else:
            self.feedback_label.setText("Demo request rejected.")
            if self.demo_loop_checkbox.isChecked() and self.demo_loop_name:
                self.demo_repeat_timer.start(1000)

    def _sync_mode_controls(self):
        if self.planner_mode == 0:
            self.mode_button.setText("Switch to LIVE MODE")
            self.mode_indicator.setText("Mode: TASK MODE")
            self.send_button.setEnabled(self.node.wait_for_service(timeout_sec=0.0))
        else:
            self.mode_button.setText("Switch to TASK MODE")
            self.mode_indicator.setText("Mode: LIVE TEACH MODE")
            self.send_button.setEnabled(False)

    def _toggle_planner_mode(self):
        """Toggle between TASK_MODE (0) and LIVE_TEACH_MODE (1)"""
        if not self.node.wait_for_mode_service(timeout_sec=0.1):
            QMessageBox.warning(
                self, "Service unavailable", "set_motion_mode service is not available."
            )
            return

        if self.pending_mode_future is not None:
            self.feedback_label.setText("Mode switch already in progress.")
            return

        self.pending_mode_target = 1 if self.planner_mode == 0 else 0
        self.pending_mode_future = self.node.set_motion_mode(self.pending_mode_target)
        self.feedback_label.setText("Switching mode...")

    def _poll_mode_future(self):
        if self.pending_mode_future is None or not self.pending_mode_future.done():
            return

        future = self.pending_mode_future
        self.pending_mode_future = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.feedback_label.setText(f"Mode switch failed: {exc}")
            return

        if response.success:
            self.planner_mode = self.pending_mode_target
            self._sync_mode_controls()
            mode_name = "LIVE TEACH MODE" if self.planner_mode == 1 else "TASK MODE"
            if self.planner_mode == 1 and self.live_move_checkbox.isChecked():
                self._schedule_live_publish(immediate=True)
            elif self.planner_mode == 0:
                self.live_publish_timer.stop()
            self.feedback_label.setText(
                f"Mode switched to {mode_name}. {response.message}"
            )
        else:
            self.feedback_label.setText(f"Mode switch rejected: {response.message}")

        self._refresh_service_status()

    def _on_live_mode_changed(self, checked: bool):
        if checked:
            if self.planner_mode == 1:
                self.feedback_label.setText(
                    "Live move enabled: slider changes publish immediately."
                )
                self._schedule_live_publish()
            else:
                self.feedback_label.setText(
                    "Live move enabled: switch the planner to LIVE TEACH MODE to publish."
                )
        else:
            self.live_publish_timer.stop()
            self.feedback_label.setText(
                "Live move disabled: use Send Pos to move in TASK MODE."
            )

    def _on_cartesian_slider_pressed(self):
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish(immediate=True)

    def _on_orientation_setting_changed(self, _value=None):
        self.param_apply_timer.start(120)
        if self.live_move_checkbox.isChecked() and self.planner_mode == 1:
            self._schedule_live_publish(immediate=True)

    def _on_orientation_enabled_toggled(self, checked: bool):
        self.tilt_slider.setEnabled(checked)
        self.spin_slider.setEnabled(checked)
        if not checked:
            self.feedback_label.setText(
                "Orientation disabled: tilt and spin are forced to 0."
            )
        self._on_orientation_setting_changed(checked)

    def _effective_orientation_rad(self):
        if not self.spin_enable_checkbox.isChecked():
            return 0.0, 0.0
        tilt_rad = math.radians(self.tilt_slider.slider.value())
        spin_rad = math.radians(self.spin_slider.slider.value())
        return tilt_rad, spin_rad

    def _apply_orientation_settings(self):
        if not self.node.wait_for_parameter_service(timeout_sec=0.0):
            return
        if (
            self.pending_param_future is not None
            and not self.pending_param_future.done()
        ):
            self.param_apply_timer.start(80)
            return

        self.pending_param_future = self.node.set_planner_orientation_config(
            self.object_center_offset_spin.value(),
            self.axis_comp_checkbox.isChecked(),
        )

    def _poll_param_future(self):
        if self.pending_param_future is None or not self.pending_param_future.done():
            return

        future = self.pending_param_future
        self.pending_param_future = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.feedback_label.setText(f"Planner parameter update failed: {exc}")
            return

        # AsyncParameterClient.set_parameters returns SetParameters.Response,
        # where individual outcomes are in the `results` field.
        if hasattr(response, "results"):
            result_items = response.results
        elif hasattr(response, "result"):
            # Fallback for atomic-style responses.
            result_items = [response.result]
        elif isinstance(response, list):
            result_items = response
        else:
            result_items = [response]

        if all(getattr(res, "successful", False) for res in result_items):
            return
        self.feedback_label.setText("Planner parameter update rejected.")

    def _cartesian_target(self):
        x_m = self.x_slider.value_m()
        y_m = self.y_slider.value_m()
        z_m = self.z_slider.value_m()
        tilt_rad, spin_rad = self._effective_orientation_rad()
        return x_m, y_m, z_m, tilt_rad, spin_rad

    def _on_cartesian_slider_changed(self, _value: int):
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish(immediate=True)

    def _schedule_live_publish(self, immediate: bool = False):
        if self.planner_mode != 1:
            self.feedback_label.setText(
                "Live publish is only active in LIVE TEACH MODE."
            )
            return

        self.live_publish_timer.stop()
        self._publish_live_target()

    def _publish_live_target(self):
        if self.planner_mode != 1:
            return

        x, y, z, tilt, spin = self._cartesian_target()
        point_available = self.node.wait_for_service(timeout_sec=0.0)
        if not point_available:
            self.feedback_label.setText("Waiting for move_to_point service...")
            return

        self.target_preview.setText(
            f"Target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg, "
            f"obj={self.object_center_offset_spin.value():.3f} m"
        )
        self.feedback_label.setText(
            f"Publishing live pose stream: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg"
        )
        self.node.publish_live_target(x, y, z)
        self.node.publish_live_orientation(tilt, spin)

    def _send_target_from_sliders(self, silent: bool = False):
        self._apply_orientation_settings()
        pose_available = self.node.wait_for_pose_service(timeout_sec=0.1)
        point_available = self.node.wait_for_service(timeout_sec=0.1)
        if not pose_available and not point_available:
            if silent:
                self.feedback_label.setText(
                    "Waiting for move_to_point / move_to_pose service..."
                )
            else:
                QMessageBox.warning(
                    self,
                    "Service unavailable",
                    "move_to_point / move_to_pose is not available yet.",
                )
            return

        x, y, z, tilt, spin = self._cartesian_target()
        self.target_preview.setText(
            f"Target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg, "
            f"obj={self.object_center_offset_spin.value():.3f} m"
        )
        if pose_available:
            self.feedback_label.setText(
                f"Sending pose: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
                f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg"
            )
            self.pending_future = self.node.send_pose(
                x,
                y,
                z,
                tilt,
                spin,
                use_orientation=self.spin_enable_checkbox.isChecked(),
            )
        else:
            self.feedback_label.setText(
                f"Sending XYZ: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m"
            )
            self.pending_future = self.node.send_target(x, y, z)

    def _poll_pending_future(self):
        if self.pending_future is None or not self.pending_future.done():
            return

        future = self.pending_future
        self.pending_future = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.feedback_label.setText(f"Request failed: {exc}")
            return

        if response.success:
            self.feedback_label.setText("Target accepted by kinematics.")
        else:
            self.feedback_label.setText("Target rejected by kinematics.")

    def _home_position(self):
        self.x_slider.set_mm(0)
        self.y_slider.set_mm(0)
        self.z_slider.set_mm(-375)
        self.tilt_slider.set_mm(0)
        self.spin_slider.set_mm(0)
        self._send_target_from_sliders(silent=False)

    def _zero_xy(self):
        self.x_slider.set_mm(0)
        self.y_slider.set_mm(0)
        self.feedback_label.setText("X/Y reset to zero.")

    def _browse_gcode_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select G-code file",
            "",
            "G-code Files (*.gcode *.nc *.tap *.txt);;All Files (*)",
        )
        if file_path:
            self.gcode_path.setText(file_path)

    def _browse_json_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select JSON task file",
            "",
            "JSON Files (*.json);;All Files (*)",
        )
        if file_path:
            self.json_path.setText(file_path)

    def _run_gcode_file(self):
        file_path = self.gcode_path.text().strip()
        if not file_path:
            QMessageBox.warning(self, "Missing file", "Select a G-code file first.")
            return

        args = [
            "run",
            "delta_robot",
            "gcode_parser.py",
            file_path,
            "--ros-args",
            "-p",
            f"default_units:={self.gcode_units.currentText()}",
            "-p",
            f"motion_rate_hz:={self.gcode_rate.value():.1f}",
        ]
        self._run_external_process("G-code", "ros2", args)

    def _run_json_file(self):
        file_path = self.json_path.text().strip()
        if not file_path:
            QMessageBox.warning(self, "Missing file", "Select a JSON task file first.")
            return

        args = [
            "run",
            "delta_robot",
            "json_task_sequencer.py",
            file_path,
            "--ros-args",
            "-p",
            f"json_units:={self.json_units.currentText()}",
            "-p",
            f"motion_rate_hz:={self.json_rate.value():.1f}",
        ]
        self._run_external_process("JSON", "ros2", args)

    def _run_external_process(self, label: str, program: str, args: list[str]):
        if self.process.state() != QProcess.NotRunning:
            QMessageBox.information(self, "Busy", "Another command is already running.")
            return

        self.pending_file_job = (label, program, args)
        self.stop_requested = False
        self.active_job_type = "file"
        self.active_job_name = label
        self.console.appendPlainText(f"$ {program} {' '.join(args)}")
        self.activity_indicator.setText(f"Running {label}...")
        self._append_console_line(f"Starting {label} command")
        self.process.start(program, args)
        if not self.process.waitForStarted(3000):
            self.activity_indicator.setText("Idle")
            self._append_console_line("Failed to start command")
            self.pending_file_job = None

    def _stop_process(self):
        self.stop_requested = True
        self.pending_file_job = None
        if hasattr(self, "gcode_loop_checkbox"):
            self.gcode_loop_checkbox.setChecked(False)
        if hasattr(self, "json_loop_checkbox"):
            self.json_loop_checkbox.setChecked(False)
        if self.process.state() == QProcess.NotRunning:
            return
        self._append_console_line("Stopping command")
        self.process.terminate()
        if not self.process.waitForFinished(1000):
            self.process.kill()

    def _read_process_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if data:
            self._append_console_line(data.rstrip())

    def _process_finished(self, exit_code: int, exit_status):
        status_text = "finished" if exit_code == 0 else f"exited with code {exit_code}"
        self.activity_indicator.setText(f"Idle ({status_text})")
        self._append_console_line(f"Command {status_text}")
        loop_enabled = False
        if self.pending_file_job is not None:
            label = self.pending_file_job[0]
            if label == "G-code" and hasattr(self, "gcode_loop_checkbox"):
                loop_enabled = self.gcode_loop_checkbox.isChecked()
            elif label == "JSON" and hasattr(self, "json_loop_checkbox"):
                loop_enabled = self.json_loop_checkbox.isChecked()

        if (
            loop_enabled
            and not self.stop_requested
            and self.pending_file_job is not None
        ):
            label, program, args = self.pending_file_job
            QTimer.singleShot(
                100, lambda: self._run_external_process(label, program, args)
            )

    def _process_error(self, error):
        self.activity_indicator.setText("Idle")
        self._append_console_line(f"Process error: {error}")

    def _append_console_line(self, text: str):
        for line in text.splitlines():
            if line.strip():
                self.console.appendPlainText(line)

    def _add_shadow(self, widget, blur: int = 28, y_offset: int = 6):
        effect = QGraphicsDropShadowEffect(widget)
        effect.setBlurRadius(blur)
        effect.setOffset(0, y_offset)
        effect.setColor(Qt.black)
        widget.setGraphicsEffect(effect)

    def _open_motor_angles_window(self):
        if self.motor_angles_window is None:
            self.motor_angles_window = MotorAnglesWindow(self)
            self.motor_angles_window.setAttribute(Qt.WA_DeleteOnClose, True)
            self.motor_angles_window.destroyed.connect(
                lambda *_: setattr(self, "motor_angles_window", None)
            )

        self.motor_angles_window.show()
        self.motor_angles_window.raise_()
        self.motor_angles_window.activateWindow()

    def closeEvent(self, event):
        self.status_timer.stop()
        self.result_timer.stop()
        self.mode_timer.stop()
        self.demo_status_timer.stop()
        self.mode_future_timer.stop()
        self.demo_future_timer.stop()
        self.param_future_timer.stop()
        self.demo_repeat_timer.stop()
        self.param_apply_timer.stop()
        self.live_publish_timer.stop()
        self.ros_spin_timer.stop()
        if self.process.state() != QProcess.NotRunning:
            self.process.kill()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    if (
        os.environ.get("WAYLAND_DISPLAY")
        and os.environ.get("QT_QPA_PLATFORM", "").strip() == ""
    ):
        os.environ["QT_QPA_PLATFORM"] = "wayland"
    app = QApplication(sys.argv)
    gui = DeltaRobotGui()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
