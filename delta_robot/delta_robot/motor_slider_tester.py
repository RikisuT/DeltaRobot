#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node

#!/usr/bin/env python3

import threading
import time

import serial
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


MOTOR_IDS = [1, 2, 3, 4, 5]
MOTOR_MIN = 0
MOTOR_MAX = 4095
MOTOR_CENTER = 2048
DEFAULT_BAUD = 500000
DEFAULT_SPEED = 0
DEFAULT_ACCEL = 0


class Esp32Bridge:
    def __init__(self, port="/dev/ttyUSB0", baudrate=DEFAULT_BAUD, timeout=0.08):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.lock = threading.Lock()

    def connect(self):
        self.close()
        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(0.2)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

    def close(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None

    def is_open(self):
        return self.serial_port is not None and self.serial_port.is_open

    def send(self, command):
        if not self.is_open():
            raise RuntimeError("serial port is not open")

        with self.lock:
            self.serial_port.write((command.strip() + "\n").encode("ascii", errors="ignore"))
            self.serial_port.flush()
            return self._read_lines()

    def _read_lines(self, settle_s=0.05, max_lines=8):
        deadline = time.monotonic() + settle_s
        lines = []
        while True:
            if self.serial_port.in_waiting > 0:
                raw = self.serial_port.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
                if len(lines) >= max_lines:
                    break
            else:
                if lines or time.monotonic() >= deadline:
                    break
                time.sleep(0.01)
        return lines


class MotorRow(QFrame):
    def __init__(self, motor_id, on_change):
        super().__init__()
        self.motor_id = motor_id
        self.on_change = on_change

        self.setObjectName("motorRow")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)

        self.label = QLabel(f"Motor {motor_id}")
        self.label.setMinimumWidth(80)

        self.command_label = QLabel(str(MOTOR_CENTER))
        self.command_label.setMinimumWidth(70)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(MOTOR_MIN, MOTOR_MAX)
        self.slider.setValue(MOTOR_CENTER)
        self.slider.valueChanged.connect(self._slider_changed)

        self.spin = QSpinBox()
        self.spin.setRange(MOTOR_MIN, MOTOR_MAX)
        self.spin.setValue(MOTOR_CENTER)
        self.spin.valueChanged.connect(self._spin_changed)

        self.feedback_label = QLabel("fb: --")
        self.feedback_label.setMinimumWidth(110)

        layout.addWidget(self.label)
        layout.addWidget(self.command_label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)
        layout.addWidget(self.feedback_label)

    def _set_value(self, value, source):
        value = max(MOTOR_MIN, min(MOTOR_MAX, int(value)))
        self.command_label.setText(str(value))

        slider_blocked = self.slider.blockSignals(True)
        spin_blocked = self.spin.blockSignals(True)
        try:
            if source != "slider":
                self.slider.setValue(value)
            if source != "spin":
                self.spin.setValue(value)
        finally:
            self.slider.blockSignals(slider_blocked)
            self.spin.blockSignals(spin_blocked)

        self.on_change(self.motor_id, value)

    def _slider_changed(self, value):
        self._set_value(value, "slider")

    def _spin_changed(self, value):
        self._set_value(value, "spin")

    def value(self):
        return int(self.spin.value())

    def set_feedback(self, text):
        self.feedback_label.setText(text)

    def center(self):
        self._set_value(MOTOR_CENTER, "program")


class MotorSliderTester(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bridge = Esp32Bridge()
        self.rows = {}

        self.setWindowTitle("ESP32 Motor Tester")
        self.setMinimumSize(1120, 680)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        title = QLabel("Raw Motor Tester")
        title.setStyleSheet("font-size: 22px; font-weight: 700;")
        subtitle = QLabel(
            "Direct ESP32 bridge control using raw encoder ticks. 2048 is midpoint; 0..4095 is the full range."
        )
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color: #b8c4d0;")
        layout.addWidget(title)
        layout.addWidget(subtitle)

        connection_row = QHBoxLayout()
        self.port_edit = QLineEdit("/dev/ttyUSB0")
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 4000000)
        self.baud_spin.setValue(DEFAULT_BAUD)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_bridge)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_bridge)
        self.status_label = QLabel("Disconnected")

        connection_row.addWidget(QLabel("Port"))
        connection_row.addWidget(self.port_edit)
        connection_row.addWidget(QLabel("Baud"))
        connection_row.addWidget(self.baud_spin)
        connection_row.addWidget(self.connect_button)
        connection_row.addWidget(self.disconnect_button)
        connection_row.addWidget(self.status_label, 1)
        layout.addLayout(connection_row)

        control_row = QHBoxLayout()
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(0, 7500)
        self.speed_spin.setValue(DEFAULT_SPEED)
        self.acc_spin = QSpinBox()
        self.acc_spin.setRange(0, 254)
        self.acc_spin.setValue(DEFAULT_ACCEL)
        self.live_publish_checkbox = QCheckBox("Send while dragging")
        self.live_publish_checkbox.setChecked(True)
        self.torque_checkbox = QCheckBox("Enable torque on connect")
        self.torque_checkbox.setChecked(True)
        self.feedback_checkbox = QCheckBox("Poll feedback")
        self.feedback_checkbox.setChecked(True)

        control_row.addWidget(QLabel("Speed"))
        control_row.addWidget(self.speed_spin)
        control_row.addWidget(QLabel("Accel"))
        control_row.addWidget(self.acc_spin)
        control_row.addWidget(self.live_publish_checkbox)
        control_row.addWidget(self.torque_checkbox)
        control_row.addWidget(self.feedback_checkbox)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        grid.addWidget(QLabel("Motor"), 0, 0)
        grid.addWidget(QLabel("Raw Tick"), 0, 1)
        grid.addWidget(QLabel("Adjust"), 0, 2)
        grid.addWidget(QLabel("Feedback"), 0, 3)
        layout.addLayout(grid)

        for index, motor_id in enumerate(MOTOR_IDS, start=1):
            row = MotorRow(motor_id, self._on_motor_changed)
            grid.addWidget(row, index, 0, 1, 4)
            self.rows[motor_id] = row

        button_row = QHBoxLayout()
        self.center_all_button = QPushButton("Center All (2048)")
        self.center_all_button.clicked.connect(self.center_all)
        self.all_min_button = QPushButton("All to 0")
        self.all_min_button.clicked.connect(lambda: self.set_all_to(MOTOR_MIN))
        self.all_max_button = QPushButton("All to 4095")
        self.all_max_button.clicked.connect(lambda: self.set_all_to(MOTOR_MAX))
        self.send_button = QPushButton("Send Now")
        self.send_button.clicked.connect(self.send_current)
        self.refresh_button = QPushButton("Refresh Feedback")
        self.refresh_button.clicked.connect(self.refresh_feedback)

        button_row.addWidget(self.center_all_button)
        button_row.addWidget(self.all_min_button)
        button_row.addWidget(self.all_max_button)
        button_row.addWidget(self.send_button)
        button_row.addWidget(self.refresh_button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        self.console = QLabel("Ready.")
        self.console.setWordWrap(True)
        layout.addWidget(self.console)

        self.send_timer = QTimer(self)
        self.send_timer.setSingleShot(True)
        self.send_timer.timeout.connect(self.send_current)

        self.feedback_timer = QTimer(self)
        self.feedback_timer.timeout.connect(self.refresh_feedback)
        self.feedback_timer.start(150)

        self._apply_dark_style()
        QTimer.singleShot(0, self.center_all)

    def _apply_dark_style(self):
        self.setStyleSheet(
            """
            QWidget {
                background: #0b1118;
                color: #e7eef7;
                font-family: sans-serif;
                font-size: 13px;
            }
            QFrame#motorRow {
                background: #111b26;
                border: 1px solid #223244;
                border-radius: 10px;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #2a3b4d;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                width: 18px;
                margin: -7px 0;
                border-radius: 9px;
                background: #7cc7ff;
            }
            QPushButton {
                background: #223244;
                border: 1px solid #30465d;
                padding: 6px 10px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: #2b3f55;
            }
            QLineEdit, QSpinBox {
                background: #0f1720;
                border: 1px solid #30465d;
                padding: 4px 6px;
                border-radius: 6px;
            }
            """
        )

    def connect_bridge(self):
        try:
            self.bridge.port = self.port_edit.text().strip()
            self.bridge.baudrate = int(self.baud_spin.value())
            self.bridge.connect()
            self.status_label.setText(f"Connected to {self.bridge.port}")
            self.console.setText("Connected. Sending torque / center commands...")

            if self.torque_checkbox.isChecked():
                for motor_id in MOTOR_IDS:
                    self._send_command(f"TORQUE {motor_id} 1")

            self.center_all(send=False)
            self.refresh_feedback()
        except Exception as exc:  # noqa: BLE001
            self.status_label.setText("Disconnected")
            QMessageBox.critical(self, "Connection failed", str(exc))

    def disconnect_bridge(self):
        self.bridge.close()
        self.status_label.setText("Disconnected")

    def _send_command(self, command):
        if not self.bridge.is_open():
            raise RuntimeError("Connect to the ESP32 first")
        lines = self.bridge.send(command)
        if lines:
            self.console.setText("\n".join(lines[-6:]))
        return lines

    def _on_motor_changed(self, motor_id, value):
        if self.live_publish_checkbox.isChecked():
            self.send_timer.start(30)

    def current_values(self):
        return {motor_id: self.rows[motor_id].value() for motor_id in MOTOR_IDS}

    def send_current(self):
        if not self.bridge.is_open():
            return

        values = self.current_values()
        speed = int(self.speed_spin.value())
        acc = int(self.acc_spin.value())
        for motor_id in MOTOR_IDS:
            self._send_command(f"SET {motor_id} {values[motor_id]} {speed} {acc}")

        self.status_label.setText(
            "Sent: "
            + ", ".join(f"ID{motor_id}={values[motor_id]}" for motor_id in MOTOR_IDS)
        )

    def center_all(self, send=True):
        for motor_id in MOTOR_IDS:
            self.rows[motor_id].center()
        if send:
            self.send_current()

    def set_all_to(self, value):
        for motor_id in MOTOR_IDS:
            self.rows[motor_id]._set_value(value, "program")
        self.send_current()

    def refresh_feedback(self):
        if not self.bridge.is_open() or not self.feedback_checkbox.isChecked():
            return

        for motor_id in MOTOR_IDS:
            try:
                lines = self._send_command(f"GETP {motor_id}")
            except Exception as exc:  # noqa: BLE001
                self.status_label.setText(f"Feedback error: {exc}")
                return

            position = None
            for line in lines:
                if line.startswith("FB id=") or line.startswith("AREAD id="):
                    tokens = line.replace(",", " ").split()
                    for token in tokens:
                        if token.startswith("pos="):
                            try:
                                position = int(token.split("=", 1)[1])
                            except ValueError:
                                position = None
                            break
                if position is not None:
                    break

            if position is None:
                self.rows[motor_id].set_feedback("fb: --")
            else:
                self.rows[motor_id].set_feedback(f"fb: {position}")

    def closeEvent(self, event):
        self.bridge.close()
        super().closeEvent(event)


def main():
    app = QApplication([])
    window = MotorSliderTester()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()