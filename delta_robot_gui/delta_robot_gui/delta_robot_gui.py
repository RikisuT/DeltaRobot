#!/usr/bin/env python3
"""Versatile delta robot GUI for Cartesian moves, G-code, and JSON tasks."""

import sys
from dataclasses import dataclass

import rclpy
from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
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
    QSlider,
    QSpacerItem,
    QSizePolicy,
    QTabWidget,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
    QGraphicsDropShadowEffect,
)
from geometry_msgs.msg import Point
from rclpy.node import Node

from deltarobot_interfaces.srv import MoveToPoint, SetMotionMode


@dataclass
class SliderSpec:
    label: str
    minimum_mm: int
    maximum_mm: int
    default_mm: int


class DeltaGuiNode(Node):
    def __init__(self):
        super().__init__("delta_robot_gui")
        self.declare_parameter("move_to_point_service", "delta_motion_planner/move_to_point")
        self.declare_parameter("set_motion_mode_service", "delta_motion_planner/set_motion_mode")
        self.declare_parameter("live_target_topic", "delta_motion_planner/live_target")
        self.move_to_point_service = (
            self.get_parameter("move_to_point_service").get_parameter_value().string_value
        )
        self.set_motion_mode_service = (
            self.get_parameter("set_motion_mode_service").get_parameter_value().string_value
        )
        self.live_target_topic = (
            self.get_parameter("live_target_topic").get_parameter_value().string_value
        )
        self.client = self.create_client(MoveToPoint, self.move_to_point_service)
        self.set_motion_mode_client = self.create_client(SetMotionMode, self.set_motion_mode_service)
        self.live_target_publisher = self.create_publisher(Point, self.live_target_topic, 10)

    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def wait_for_mode_service(self, timeout_sec: float = 5.0) -> bool:
        return self.set_motion_mode_client.wait_for_service(timeout_sec=timeout_sec)

    def send_target(self, x_m: float, y_m: float, z_m: float):
        request = MoveToPoint.Request()
        request.target = Point(x=x_m * 1000.0, y=y_m * 1000.0, z=z_m * 1000.0)
        return self.client.call_async(request)

    def set_motion_mode(self, mode: int):
        """Set motion mode: 0 = TASK_MODE, 1 = LIVE_TEACH_MODE"""
        request = SetMotionMode.Request()
        request.mode = mode
        return self.set_motion_mode_client.call_async(request)

    def publish_live_target(self, x_m: float, y_m: float, z_m: float):
        self.live_target_publisher.publish(Point(x=x_m * 1000.0, y=y_m * 1000.0, z=z_m * 1000.0))


class LabeledSlider(QWidget):
    def __init__(self, spec: SliderSpec, parent=None):
        super().__init__(parent)
        self.scale = 1000.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

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
        self.pending_future = None
        self.pending_mode_future = None
        self.pending_mode_target = 0
        self.planner_mode = 0

        self.setWindowTitle("Delta Robot Control Center")
        self.setMinimumSize(980, 680)

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

        self.mode_future_timer = QTimer(self)
        self.mode_future_timer.timeout.connect(self._poll_mode_future)
        self.mode_future_timer.start(50)

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

        root = QVBoxLayout(central)
        root.setContentsMargins(26, 26, 26, 26)
        root.setSpacing(18)

        hero = QFrame()
        hero.setObjectName("heroCard")
        hero_layout = QVBoxLayout(hero)
        hero_layout.setContentsMargins(22, 20, 22, 20)
        hero_layout.setSpacing(10)

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
        root.addWidget(hero)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_cartesian_tab(), "Cartesian")
        self.tabs.addTab(self._build_gcode_tab(), "G-code")
        self.tabs.addTab(self._build_json_tab(), "JSON Tasks")
        self.tabs.addTab(self._build_console_tab(), "Console")
        root.addWidget(self.tabs)

    def _build_cartesian_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        help_box = QFrame()
        help_box.setObjectName("infoCard")
        help_layout = QHBoxLayout(help_box)
        help_layout.setContentsMargins(18, 14, 18, 14)
        help_layout.setSpacing(12)

        hint = QLabel("Recommended range: x/y around +/-0.10 m, z around -0.18 m.")
        hint.setObjectName("hintLabel")
        help_layout.addWidget(hint)
        help_layout.addStretch(1)
        
        mode_control = QHBoxLayout()
        mode_label = QLabel("Control Mode:")
        self.mode_button = QPushButton("TASK MODE")
        self.mode_button.setObjectName("modeButton")
        self.mode_button.setMaximumWidth(150)
        self.mode_button.clicked.connect(self._toggle_planner_mode)
        mode_control.addWidget(mode_label)
        mode_control.addWidget(self.mode_button)
        mode_control.addStretch(1)
        
        self.live_move_checkbox = QCheckBox("Live move while sliders change (LIVE mode only)")
        self.live_move_checkbox.setChecked(True)
        self.live_move_checkbox.toggled.connect(self._on_live_mode_changed)
        
        help_layout.addLayout(mode_control)
        help_layout.addWidget(self.live_move_checkbox)
        self._add_shadow(help_box, blur=24, y_offset=6)
        layout.addWidget(help_box)

        slider_box = QGroupBox("Cartesian Target")
        slider_box.setObjectName("cardBox")
        slider_layout = QVBoxLayout(slider_box)
        slider_layout.setSpacing(12)

        self.x_slider = LabeledSlider(SliderSpec("X", -120, 120, 0))
        self.y_slider = LabeledSlider(SliderSpec("Y", -120, 120, 0))
        self.z_slider = LabeledSlider(SliderSpec("Z", -260, -120, -180))

        self.x_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.y_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.z_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.x_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.y_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.z_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)

        slider_layout.addWidget(self.x_slider)
        slider_layout.addWidget(self.y_slider)
        slider_layout.addWidget(self.z_slider)
        layout.addWidget(slider_box)

        self.target_preview = QLabel("Target: x=0.000 m, y=0.000 m, z=-0.180 m")
        self.target_preview.setObjectName("previewLabel")
        layout.addWidget(self.target_preview)

        button_row = QHBoxLayout()
        self.send_button = QPushButton("Send Pos")
        self.send_button.setObjectName("primaryButton")
        self.send_button.clicked.connect(self._send_target_from_sliders)

        self.home_button = QPushButton("Home Pose")
        self.home_button.setObjectName("secondaryButton")
        self.home_button.clicked.connect(self._home_position)

        self.zero_xy_button = QPushButton("Zero X/Y")
        self.zero_xy_button.clicked.connect(self._zero_xy)

        button_row.addWidget(self.send_button)
        button_row.addWidget(self.home_button)
        button_row.addWidget(self.zero_xy_button)
        layout.addLayout(button_row)

        self.feedback_label = QLabel("Ready.")
        self.feedback_label.setObjectName("feedbackLabel")
        layout.addWidget(self.feedback_label)
        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        return tab

    def _build_gcode_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        description = QLabel("Pick a G-code file and run it through delta_robot/gcode_parser.py.")
        description.setObjectName("hintLabel")
        layout.addWidget(description)

        self.gcode_path = QLineEdit()
        self.gcode_path.setPlaceholderText("Select a .gcode or .nc file")
        gcode_browse = QPushButton("Browse")
        gcode_browse.clicked.connect(self._browse_gcode_file)

        path_row = QHBoxLayout()
        path_row.addWidget(self.gcode_path)
        path_row.addWidget(gcode_browse)
        layout.addLayout(path_row)

        settings_box = QGroupBox("Execution Settings")
        settings_layout = QFormLayout(settings_box)
        self.gcode_units = QComboBox()
        self.gcode_units.addItems(["meters", "millimeters"])
        self.gcode_rate = QDoubleSpinBox()
        self.gcode_rate.setRange(1.0, 1000.0)
        self.gcode_rate.setValue(100.0)
        self.gcode_rate.setSuffix(" Hz")
        self.gcode_rate.setDecimals(1)
        settings_layout.addRow("Units", self.gcode_units)
        settings_layout.addRow("Motion rate", self.gcode_rate)
        layout.addWidget(settings_box)
        self._add_shadow(settings_box, blur=22, y_offset=5)

        run_row = QHBoxLayout()
        self.run_gcode_button = QPushButton("Run G-code")
        self.run_gcode_button.setObjectName("primaryButton")
        self.run_gcode_button.clicked.connect(self._run_gcode_file)
        self.gcode_stop_button = QPushButton("Stop")
        self.gcode_stop_button.clicked.connect(self._stop_process)
        run_row.addWidget(self.run_gcode_button)
        run_row.addWidget(self.gcode_stop_button)
        layout.addLayout(run_row)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _build_json_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        description = QLabel("Pick a JSON task list and run it through delta_robot/json_task_sequencer.py.")
        description.setObjectName("hintLabel")
        layout.addWidget(description)

        self.json_path = QLineEdit()
        self.json_path.setPlaceholderText("Select a task .json file")
        json_browse = QPushButton("Browse")
        json_browse.clicked.connect(self._browse_json_file)

        path_row = QHBoxLayout()
        path_row.addWidget(self.json_path)
        path_row.addWidget(json_browse)
        layout.addLayout(path_row)

        settings_box = QGroupBox("Execution Settings")
        settings_layout = QFormLayout(settings_box)
        self.json_units = QComboBox()
        self.json_units.addItems(["meters", "millimeters"])
        self.json_rate = QDoubleSpinBox()
        self.json_rate.setRange(1.0, 1000.0)
        self.json_rate.setValue(100.0)
        self.json_rate.setSuffix(" Hz")
        self.json_rate.setDecimals(1)
        settings_layout.addRow("Units", self.json_units)
        settings_layout.addRow("Motion rate", self.json_rate)
        layout.addWidget(settings_box)
        self._add_shadow(settings_box, blur=22, y_offset=5)

        run_row = QHBoxLayout()
        self.run_json_button = QPushButton("Run JSON")
        self.run_json_button.setObjectName("primaryButton")
        self.run_json_button.clicked.connect(self._run_json_file)
        self.json_stop_button = QPushButton("Stop")
        self.json_stop_button.clicked.connect(self._stop_process)
        run_row.addWidget(self.run_json_button)
        run_row.addWidget(self.json_stop_button)
        layout.addLayout(run_row)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _build_console_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setObjectName("consoleBox")
        layout.addWidget(self.console)

        clear_row = QHBoxLayout()
        clear_row.addStretch(1)
        clear_button = QPushButton("Clear Console")
        clear_button.clicked.connect(self.console.clear)
        clear_row.addWidget(clear_button)
        layout.addLayout(clear_row)

        return tab

    def _apply_styles(self):
        self.setStyleSheet(
            """
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #091019, stop:0.55 #101b27, stop:1 #0b1320);
                color: #e7eef7;
                font-family: "Segoe UI", "Noto Sans", "DejaVu Sans", sans-serif;
                font-size: 13px;
            }
            QFrame#heroCard,
            QFrame#infoCard,
            QGroupBox#cardBox,
            QGroupBox {
                background: rgba(16, 24, 35, 0.90);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 20px;
            }
            QLabel#titleLabel {
                font-size: 27px;
                font-weight: 800;
                letter-spacing: 0.2px;
                color: #f4f8fc;
            }
            QLabel#subtitleLabel {
                color: #98a9ba;
                margin-bottom: 2px;
                font-size: 13px;
            }
            QLabel#hintLabel {
                color: #9aa9b9;
            }
            QLabel#serviceIndicator,
            QLabel#activityIndicator,
            QLabel#modeIndicator {
                color: #d7e2ec;
                padding: 7px 12px;
                background: rgba(255, 255, 255, 0.04);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 999px;
            }
            QTabWidget::pane {
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 20px;
                top: -1px;
                background: rgba(16, 24, 35, 0.82);
            }
            QTabBar::tab {
                background: rgba(255, 255, 255, 0.03);
                color: #b8c6d6;
                padding: 11px 18px;
                margin-right: 6px;
                border-top-left-radius: 12px;
                border-top-right-radius: 12px;
                min-width: 120px;
            }
            QTabBar::tab:selected {
                background: rgba(58, 141, 222, 0.30);
                color: #f7fbff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 6px;
                color: #d8e6f2;
                font-weight: 700;
            }
            QLabel#sliderLabel {
                font-weight: 700;
                color: #eef5fb;
            }
            QLabel#valueLabel {
                color: #89bdf1;
                min-width: 88px;
                padding: 3px 8px;
                background: rgba(58, 141, 222, 0.14);
                border: 1px solid rgba(58, 141, 222, 0.18);
                border-radius: 999px;
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
                background: rgba(8, 14, 21, 0.78);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 12px;
                padding: 8px 10px;
                color: #e7eef7;
            }
            QPlainTextEdit#consoleBox {
                background: rgba(6, 11, 18, 0.88);
                font-family: "DejaVu Sans Mono", monospace;
                font-size: 12px;
            }
            QPushButton {
                border: none;
                border-radius: 14px;
                padding: 12px 16px;
                font-weight: 700;
                color: #e9f1f8;
                background: rgba(255, 255, 255, 0.08);
            }
            QPushButton:hover {
                background: rgba(255, 255, 255, 0.13);
            }
            QPushButton:pressed {
                background: rgba(255, 255, 255, 0.16);
            }
            QPushButton#primaryButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                color: white;
            }
            QPushButton#primaryButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #60adef, stop:1 #3b8ae0);
            }
            QPushButton#secondaryButton {
                background: #203142;
            }
            QPushButton#modeButton {
                background: rgba(255, 152, 0, 0.30);
                color: #ffc857;
                border: 1px solid rgba(255, 152, 0, 0.50);
                font-weight: 700;
            }
            QPushButton#modeButton:hover {
                background: rgba(255, 152, 0, 0.45);
                border: 1px solid rgba(255, 152, 0, 0.70);
            }
            QLabel#feedbackLabel {
                color: #c1cfdb;
                padding: 8px 2px;
            }
            QCheckBox {
                color: #d6e1eb;
                spacing: 10px;
                font-weight: 600;
            }
            """
        )

    def _connect_services(self):
        self._refresh_service_status()
        self._refresh_mode_service_status()
        self._sync_mode_controls()

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def _refresh_service_status(self):
        available = self.node.wait_for_service(timeout_sec=0.0)
        if available:
            self.service_indicator.setText("Service: connected")
            self.service_indicator.setStyleSheet(
                "color: #bdf3c8; background: rgba(61, 156, 85, 0.16); border: 1px solid rgba(61, 156, 85, 0.28);"
            )
            self.send_button.setEnabled(self.planner_mode == 0)
        else:
            self.service_indicator.setText("Service: waiting for delta_motion_planner/move_to_point")
            self.service_indicator.setStyleSheet(
                "color: #ffd7a8; background: rgba(159, 108, 29, 0.18); border: 1px solid rgba(159, 108, 29, 0.26);"
            )
            self.send_button.setEnabled(False)

    def _refresh_mode_service_status(self):
        available = self.node.wait_for_mode_service(timeout_sec=0.0)
        self.mode_button.setEnabled(available)

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
            QMessageBox.warning(self, "Service unavailable", "set_motion_mode service is not available.")
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
            self.feedback_label.setText(f"Mode switched to {mode_name}. {response.message}")
        else:
            self.feedback_label.setText(f"Mode switch rejected: {response.message}")

        self._refresh_service_status()

    def _on_live_mode_changed(self, checked: bool):
        if checked:
            if self.planner_mode == 1:
                self.feedback_label.setText("Live move enabled: slider changes publish after a short debounce.")
                self._schedule_live_publish()
            else:
                self.feedback_label.setText("Live move enabled: switch the planner to LIVE TEACH MODE to publish.")
        else:
            self.live_publish_timer.stop()
            self.feedback_label.setText("Live move disabled: use Send Pos to move in TASK MODE.")

    def _on_cartesian_slider_pressed(self):
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish(immediate=True)

    def _cartesian_target(self):
        return self.x_slider.value_m(), self.y_slider.value_m(), self.z_slider.value_m()

    def _on_cartesian_slider_changed(self, _value: int):
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish()

    def _schedule_live_publish(self, immediate: bool = False):
        if self.planner_mode != 1:
            self.feedback_label.setText("Live publish is only active in LIVE TEACH MODE.")
            return

        if immediate:
            self.live_publish_timer.stop()
            self._publish_live_target()
            return

        self.live_publish_timer.start(40)

    def _publish_live_target(self):
        if self.planner_mode != 1:
            return

        x, y, z = self._cartesian_target()
        self.target_preview.setText(f"Target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
        self.feedback_label.setText(f"Publishing live target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
        self.node.publish_live_target(x, y, z)

    def _send_target_from_sliders(self, silent: bool = False):
        if not self.node.wait_for_service(timeout_sec=0.1):
            if silent:
                self.feedback_label.setText("Waiting for move_to_point service...")
            else:
                QMessageBox.warning(self, "Service unavailable", "move_to_point is not available yet.")
            return

        x, y, z = self._cartesian_target()
        self.target_preview.setText(f"Target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
        self.feedback_label.setText(f"Sending target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
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
        self.z_slider.set_mm(-180)
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

        self.console.appendPlainText(f"$ {program} {' '.join(args)}")
        self.activity_indicator.setText(f"Running {label}...")
        self._append_console_line(f"Starting {label} command")
        self.process.start(program, args)
        if not self.process.waitForStarted(3000):
            self.activity_indicator.setText("Idle")
            self._append_console_line("Failed to start command")

    def _stop_process(self):
        if self.process.state() == QProcess.NotRunning:
            return
        self._append_console_line("Stopping command")
        self.process.terminate()
        if not self.process.waitForFinished(1000):
            self.process.kill()

    def _read_process_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if data:
            self._append_console_line(data.rstrip())

    def _process_finished(self, exit_code: int, exit_status):
        status_text = "finished" if exit_code == 0 else f"exited with code {exit_code}"
        self.activity_indicator.setText(f"Idle ({status_text})")
        self._append_console_line(f"Command {status_text}")

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

    def closeEvent(self, event):
        self.status_timer.stop()
        self.result_timer.stop()
        self.mode_timer.stop()
        self.mode_future_timer.stop()
        self.live_publish_timer.stop()
        self.ros_spin_timer.stop()
        if self.process.state() != QProcess.NotRunning:
            self.process.kill()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = DeltaRobotGui()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
