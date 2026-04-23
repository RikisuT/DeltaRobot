#!/usr/bin/env python3
"""Realtime TF plot for end-effector: XY, XZ, YZ planes.

Defaults assume frames published by the motion planner and controller.
Run with e.g.:

  source /opt/ros/<distro>/setup.bash
  python3 src/delta_robot/delta_robot/plot_ee_tf.py \
    --parent-frame world_link \
    --commanded delta_robot/commanded_end_effector_pin \
    --calculated delta_robot/calculated_fk_end_effector_pin \
    --actual delta_robot/actual_fk_end_effector_pin

Controls:
  --max-points N   keep last N points (default 1000)
  --interval-ms N  animation update interval in ms (default 50)

"""

import argparse
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class PlotEeTf(Node):
    def __init__(
        self,
        parent_frame,
        commanded_frame,
        ee_frame,
        calculated_frame,
        actual_frame,
        max_points=1000,
        interval_ms=20,
        half_range=0.12,
    ):
        super().__init__("plot_ee_tf")
        self.parent_frame = parent_frame
        # draw traces in the requested order: commanded, ee (sensor actual), ik-fk calc, motorfeedback fk calc
        self.frames = {
            "commanded": commanded_frame,
            "ee": ee_frame,
            "calculated": calculated_frame,
            "actual": actual_frame,
        }
        self.max_points = max_points
        self.interval_ms = interval_ms
        self.half_range = half_range

        # storage for recent translations (meters)
        self.data = {}
        for k in self.frames:
            self.data[k] = {
                "x": deque(maxlen=self.max_points),
                "y": deque(maxlen=self.max_points),
                "z": deque(maxlen=self.max_points),
                "t": deque(maxlen=self.max_points),
            }

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Start a background ROS spinner so TF callbacks keep running
        # independently of the Matplotlib GUI event loop.
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        # Lock to protect data deques between the ROS timer thread and
        # the Matplotlib GUI thread.
        self._data_lock = threading.Lock()


        # determine start pose to center the fixed windows
        start_x, start_y, start_z = self._determine_start_position(timeout=2.0)
        hr = self.half_range

        # matplotlib setup: create three separate figure windows (XY, YZ, XZ)
        plt.style.use("seaborn-darkgrid")

        # individual figures for clearer windows
        self.fig_xy, self.ax_xy = plt.subplots(figsize=(6, 6))
        self.fig_yz, self.ax_yz = plt.subplots(figsize=(6, 6))
        self.fig_xz, self.ax_xz = plt.subplots(figsize=(6, 6))

        colors = {
            "commanded": (1.0, 0.45, 0.0),
            "ee": (0.0, 1.0, 0.0),
            "calculated": (0.1, 0.7, 1.0),
            "actual": (1.0, 0.2, 0.4),
        }

        self.lines = {"xy": {}, "yz": {}, "xz": {}}
        for key in self.frames:
            self.lines["xy"][key] = self.ax_xy.plot([], [], color=colors.get(key, "k"), lw=1.0, label=key)[0]
            self.lines["yz"][key] = self.ax_yz.plot([], [], color=colors.get(key, "k"), lw=1.0, label=key)[0]
            self.lines["xz"][key] = self.ax_xz.plot([], [], color=colors.get(key, "k"), lw=1.0, label=key)[0]

        # set titles and axis labels; include slice coordinate in title
        self.ax_xy.set_title(f"XY plane (Z = {start_z:.3f} m)")
        self.ax_xy.set_xlabel("X (m)")
        self.ax_xy.set_ylabel("Y (m)")

        self.ax_yz.set_title(f"YZ plane (X = {start_x:.3f} m)")
        self.ax_yz.set_xlabel("Y (m)")
        self.ax_yz.set_ylabel("Z (m)")

        self.ax_xz.set_title(f"XZ plane (Y = {start_y:.3f} m)")
        self.ax_xz.set_xlabel("X (m)")
        self.ax_xz.set_ylabel("Z (m)")

        # show legends on each window
        self.ax_xy.legend(loc="upper right")
        self.ax_yz.legend(loc="upper right")
        self.ax_xz.legend(loc="upper right")

        # fixed windows centered on start pose with requested ranges:
        # XY: X,Y = ±0.075
        # YZ/XZ: Y or X = ±0.075, Z = [-0.300, 0.400]
        xy_range = 0.1
        z_min, z_max = -0.450, -0.350

        self.ax_xy.set_xlim(- xy_range,  xy_range)
        self.ax_xy.set_ylim(- xy_range,  xy_range)

        self.ax_yz.set_xlim(- xy_range,  xy_range)
        self.ax_yz.set_ylim(z_min, z_max)

        self.ax_xz.set_xlim(- xy_range,  xy_range)
        self.ax_xz.set_ylim(z_min, z_max)

        # Create a ROS timer to sample TFs at a steady rate. Sampling is
        # independent of Matplotlib's draw loop which reduces bursty
        # updates and ordering issues.
        self.sample_rate_hz = max(1.0, 1000.0 / float(self.interval_ms))
        self.sample_timer = self.create_timer(1.0 / self.sample_rate_hz, self._sample_tf_timer)

        # Create one FuncAnimation per figure so each window gets regular updates.
        self.anim_xy = FuncAnimation(self.fig_xy, self.update_plot, interval=self.interval_ms, blit=False)
        self.anim_yz = FuncAnimation(self.fig_yz, self.update_plot, interval=self.interval_ms, blit=False)
        self.anim_xz = FuncAnimation(self.fig_xz, self.update_plot, interval=self.interval_ms, blit=False)

    def lookup_and_store(self, key):
        frame = self.frames[key]
        try:
            # Use current time to get latest transform
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(self.parent_frame, frame, now)
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            # TF header stamp as float seconds
            ts = float(t.header.stamp.sec) + float(t.header.stamp.nanosec) * 1e-9
            d = self.data[key]
            d["x"].append(x)
            d["y"].append(y)
            d["z"].append(z)
            d["t"].append(ts)
            return True
        except Exception:
            # lookup may fail until TFs are published; ignore silently
            return False

    def _fetch_transform(self, key):
        """Return (x,y,z,ts) for a given child frame or None on failure."""
        frame = self.frames[key]
        try:
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(self.parent_frame, frame, now)
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            ts = float(t.header.stamp.sec) + float(t.header.stamp.nanosec) * 1e-9
            return x, y, z, ts
        except Exception:
            return None

    def update_plot(self, frame_index):
        # Snapshot data under lock so the GUI thread can render without
        # being affected by concurrent writes from the ROS timer.
        snapshot = {}
        with self._data_lock:
            for key in self.frames:
                d = self.data[key]
                snapshot[key] = (list(d["t"]), list(d["x"]), list(d["y"]), list(d["z"]))

        # update plane plots from the snapshot (preserve sampling order)
        artists = []
        for key in self.frames:
            ts, xs, ys, zs = snapshot.get(key, ([], [], [], []))
            sx, sy, sz = xs, ys, zs

            if sx and sy:
                self.lines["xy"][key].set_data(sx, sy)
            else:
                self.lines["xy"][key].set_data([], [])

            if sx and sz:
                self.lines["xz"][key].set_data(sx, sz)
            else:
                self.lines["xz"][key].set_data([], [])

            if sy and sz:
                self.lines["yz"][key].set_data(sy, sz)
            else:
                self.lines["yz"][key].set_data([], [])

        for plane in ("xy", "yz", "xz"):
            for key in self.frames:
                artists.append(self.lines[plane][key])
        return artists

    def _sample_tf_timer(self):
        """ROS timer callback that samples TFs at a steady rate and appends
        points into the per-frame deques under a lock."""
        for key in self.frames:
            result = self._fetch_transform(key)
            if result is None:
                continue
            x, y, z, ts = result
            with self._data_lock:
                d = self.data[key]
                d["x"].append(x)
                d["y"].append(y)
                d["z"].append(z)
                d["t"].append(ts)

    def _determine_start_position(self, timeout: float = 2.0):
        """Try to obtain a valid start transform to center the plots.
        Tries frames in order: actual, commanded, calculated. Falls back to 0,0,0."""
        frames_try = ["ee", "actual", "commanded", "calculated"]
        end_time = time.time() + timeout
        while time.time() < end_time:
            # spinner thread handles TF callbacks; wait briefly between attempts
            time.sleep(0.05)
            for key in frames_try:
                child = self.frames.get(key)
                if not child:
                    continue
                try:
                    trans = self.tf_buffer.lookup_transform(self.parent_frame, child, rclpy.time.Time())
                    x = trans.transform.translation.x
                    y = trans.transform.translation.y
                    z = trans.transform.translation.z
                    self.get_logger().info(f"Start pose from frame {child}: x={x:.4f}, y={y:.4f}, z={z:.4f}")
                    return x, y, z
                except Exception:
                    continue
            time.sleep(0.05)
        self.get_logger().warning("Could not obtain start transform; defaulting to 0,0,0")
        return 0.0, 0.0, 0.0


def main():
    parser = argparse.ArgumentParser(description="Realtime plot of end-effector TFs (XY/XZ/YZ)")
    parser.add_argument("--parent-frame", default="delta_robot/world_link", help="TF parent frame (default: delta_robot/world_link)")
    parser.add_argument("--commanded", default="delta_robot/commanded_end_effector_pin", help="commanded TF child frame")
    parser.add_argument("--ee", default="ee_link", help="sensor EE frame (ee_link)")
    parser.add_argument("--calculated", default="delta_robot/calculated_fk_end_effector_pin", help="calculated FK TF child frame")
    parser.add_argument("--actual", default="delta_robot/actual_fk_end_effector_pin", help="actual FK TF child frame (from motors)")
    parser.add_argument("--max-points", type=int, default=1000, help="history length")
    parser.add_argument("--interval-ms", type=int, default=50, help="plot update interval (ms)")
    parser.add_argument("--half-range", type=float, default=0.12, help="half range for fixed y-limits (meters)")
    args = parser.parse_args()

    rclpy.init()
    node = PlotEeTf(
        args.parent_frame,
        args.commanded,
        args.ee,
        args.calculated,
        args.actual,
        max_points=args.max_points,
        interval_ms=args.interval_ms,
        half_range=args.half_range,
    )

    try:
        # Apply tight layout to each figure and show all windows
        try:
            node.fig_xy.tight_layout()
            node.fig_yz.tight_layout()
            node.fig_xz.tight_layout()
        except Exception:
            pass
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
