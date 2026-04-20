#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import re
import math
import sys


def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]


class EETfBroadcaster(Node):
    def __init__(self):
        super().__init__("ee_tf_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.port_name = "/dev/ttyACM0"
        self.baudrate = 115200

        try:
            self.serial_port = serial.Serial(
                self.port_name, self.baudrate, timeout=0.01
            )
            self.get_logger().info(
                f"Connected to {self.port_name} at {self.baudrate} baud."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)

        self.timer = self.create_timer(0.005, self.timer_callback)
        self.regex = re.compile(
            r"R:\s*([-+]?\d*\.?\d+)\s*P:\s*([-+]?\d*\.?\d+)\s*Y:\s*([-+]?\d*\.?\d+)\s*\|\s*dX:\s*([-+]?\d*\.?\d+)\s*\|\s*dY:\s*([-+]?\d*\.?\d+)\s*\|\s*dZ:\s*([-+]?\d*\.?\d+)"
        )

        self.get_logger().info(
            "Ready to parse sensor data and broadcast TF over 'world_link' -> 'ee_link'"
        )

    def timer_callback(self):
        try:
            # Read all available lines rapidly
            while self.serial_port.in_waiting > 0:
                line = (
                    self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                )
                if not line:
                    continue

                match = self.regex.search(line)
                if match:
                    roll = float(match.group(1))
                    pitch = float(match.group(2))
                    yaw = float(match.group(3))
                    dx = float(match.group(4))
                    dy = float(match.group(5))
                    dz = float(match.group(6))

                    self.publish_tf(roll, pitch, yaw, dx, dy, dz)
        except Exception as e:
            pass

    def publish_tf(self, roll, pitch, yaw, dx, dy, dz):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # Set to world link
        t.header.frame_id = "delta_robot/world_link"
        t.child_frame_id = "ee_link"

        # User requested: swap + and - for x and y
        # User requested: x and y is 0 but z will be -300
        x_m = dy / 1000.0
        y_m = -dx / 1000.0
        z_m = (dz - 375.0) / 1000.0

        t.transform.translation.x = x_m
        t.transform.translation.y = y_m
        t.transform.translation.z = z_m

        q = euler_to_quaternion(roll, pitch, -yaw)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EETfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "serial_port") and node.serial_port.is_open:
            node.serial_port.close()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
