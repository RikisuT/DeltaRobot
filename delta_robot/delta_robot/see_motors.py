#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time
import numpy as np

# --- Configuration ---
WINDOW_SIZE = 200
UPDATE_INTERVAL = 50  # ms
SERVO_IDS = [1, 2, 3]
SAMPLE_INTERVAL = UPDATE_INTERVAL / 1000.0  # seconds per sample


class ServoPlotterNode(Node):
    def __init__(self):
        super().__init__("servo_plotter_node")
        self.lock = threading.Lock()
        self.create_subscription(Float32MultiArray, "/servo/target", self.target_cb, 10)
        self.create_subscription(Float32MultiArray, "/servo/actual", self.actual_cb, 10)
        self.latest_targets = {mid: 2048.0 for mid in SERVO_IDS}
        self.latest_actuals = {mid: 2048.0 for mid in SERVO_IDS}

    def target_cb(self, msg):
        if not msg.data:
            return

        with self.lock:
            # Support both [id1,id2,id3] and [id1..id5] payloads.
            for i, mid in enumerate(SERVO_IDS):
                source_idx = mid - 1 if len(msg.data) > i and len(msg.data) >= max(SERVO_IDS) else i
                if source_idx < len(msg.data):
                    self.latest_targets[mid] = msg.data[source_idx]

    def actual_cb(self, msg):
        if not msg.data:
            return

        with self.lock:
            # Support both [id1,id2,id3] and [id1..id5] payloads.
            for i, mid in enumerate(SERVO_IDS):
                source_idx = mid - 1 if len(msg.data) > i and len(msg.data) >= max(SERVO_IDS) else i
                if source_idx < len(msg.data):
                    self.latest_actuals[mid] = msg.data[source_idx]


def estimate_lag_ms(target_arr, actual_arr, sample_interval_s):
    """
    Cross-correlation lag estimator.
    Returns lag in milliseconds — how far behind actual is relative to target.
    Positive = actual is lagging (normal).
    Negative = actual is leading (unusual).
    """
    if len(target_arr) < 20:
        return 0.0

    t = np.array(target_arr, dtype=float)
    a = np.array(actual_arr, dtype=float)

    # Normalise both signals to zero mean and unit variance
    t -= t.mean()
    std = t.std()
    if std < 1e-6:
        return 0.0
    t /= std

    a -= a.mean()
    std = a.std()
    if std < 1e-6:
        return 0.0
    a /= std

    # Full cross-correlation
    corr = np.correlate(a, t, mode="full")
    lags = np.arange(-(len(t) - 1), len(t))
    lag_samples = lags[np.argmax(corr)]

    return float(lag_samples * sample_interval_s * 1000.0)


def compute_rmse(target_arr, actual_arr):
    """RMS tracking error — how far actual deviates from target on average."""
    if len(target_arr) < 2:
        return 0.0
    t = np.array(target_arr, dtype=float)
    a = np.array(actual_arr, dtype=float)
    return float(np.sqrt(np.mean((t - a) ** 2)))


def main():
    rclpy.init()
    node = ServoPlotterNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plot_time = deque(maxlen=WINDOW_SIZE)
    plot_targets = {mid: deque(maxlen=WINDOW_SIZE) for mid in SERVO_IDS}
    plot_actuals = {mid: deque(maxlen=WINDOW_SIZE) for mid in SERVO_IDS}

    # Dictionary to keep track of the max RPM seen so far
    max_rpms = {mid: 0.0 for mid in SERVO_IDS}

    start_time = time.perf_counter()

    # --- Figure layout: servo subplots + one stats panel ---
    n = len(SERVO_IDS)
    fig = plt.figure(figsize=(10, 10))
    gs = fig.add_gridspec(n + 1, 1, height_ratios=[3] * n + [1.5], hspace=0.4)

    axes = [fig.add_subplot(gs[i]) for i in range(n)]
    ax_stats = fig.add_subplot(gs[n])
    ax_stats.axis("off")

    lines = {}
    for i, mid in enumerate(SERVO_IDS):
        axes[i].set_ylabel(f"Joint {mid} (Ticks)", fontsize=9)
        axes[i].set_ylim(500, 3600)
        (ln_t,) = axes[i].plot(
            [], [], "r--", label="Ref (Sent)", alpha=0.7, linewidth=1.5
        )
        (ln_a,) = axes[i].plot([], [], "b-", label="Actual", linewidth=1.0)
        lines[mid] = (ln_t, ln_a)
        axes[i].legend(loc="upper right", fontsize=8)
        axes[i].grid(True, linestyle=":", alpha=0.6)

    axes[-1].set_xlabel("Time (seconds)")
    fig.suptitle("Live Servo Feedback Comparison", fontsize=14, fontweight="bold")

    # Stats text — updated each frame
    stats_text = ax_stats.text(
        0.01,
        0.95,
        "",
        transform=ax_stats.transAxes,
        fontsize=9,
        verticalalignment="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8),
    )

    def update(frame):
        t = time.perf_counter() - start_time
        plot_time.append(t)

        with node.lock:
            targets_now = node.latest_targets.copy()
            actuals_now = node.latest_actuals.copy()

        for mid in SERVO_IDS:
            plot_targets[mid].append(targets_now[mid])
            plot_actuals[mid].append(actuals_now[mid])

        t_snap = list(plot_time)
        t_now = t_snap[-1] if t_snap else 0

        # --- RPM Calculation ---
        if len(t_snap) >= 2:
            dt = t_snap[-1] - t_snap[-2]
            if dt > 0:
                for mid in SERVO_IDS:
                    # Calculate change in ticks
                    d_ticks = abs(plot_actuals[mid][-1] - plot_actuals[mid][-2])
                    # Convert to RPM: (ticks / dt) * (60 seconds / 4096 ticks per rev)
                    current_rpm = (d_ticks / dt) * (60.0 / 4096.0)
                    if current_rpm > max_rpms[mid]:
                        max_rpms[mid] = current_rpm

        for mid in SERVO_IDS:
            lines[mid][0].set_data(t_snap, list(plot_targets[mid]))
            lines[mid][1].set_data(t_snap, list(plot_actuals[mid]))

        for ax in axes:
            ax.set_xlim(max(0, t_now - 2.0), t_now + 0.5)

        # --- Compute response metrics ---
        stats_lines = [
            "  Joint │   Sent  │  Actual │ Lag (ms) │ RMSE (cnts) │ Peak Err │ Max RPM "
        ]
        stats_lines.append(
            " ───────┼─────────┼─────────┼──────────┼─────────────┼──────────┼─────────"
        )

        for mid in SERVO_IDS:
            tgt = list(plot_targets[mid])
            act = list(plot_actuals[mid])

            cur_tgt = targets_now[mid]
            cur_act = actuals_now[mid]

            if len(tgt) < 20:
                stats_lines.append(
                    f"  {mid:>5} │ {cur_tgt:>7.1f} │ {cur_act:>7.1f} │ collecting data..."
                )
                continue

            lag_ms = estimate_lag_ms(tgt, act, SAMPLE_INTERVAL)
            rmse = compute_rmse(tgt, act)
            peak = float(np.max(np.abs(np.array(tgt) - np.array(act))))

            stats_lines.append(
                f"  {mid:>5} │ {cur_tgt:>7.1f} │ {cur_act:>7.1f} │ {lag_ms:>+8.1f} │ {rmse:>11.1f} │ {peak:>8.1f} │ {max_rpms[mid]:>7.1f}"
            )

        stats_text.set_text("\n".join(stats_lines))

    ani = animation.FuncAnimation(
        fig, update, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down plotter...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
