#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class TfPlotter(Node):
    def __init__(self):
        super().__init__('tf_plotter')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.times = []
        self.cmd_x = []
        self.cmd_y = []
        self.cmd_z = []
        self.ee_x = []
        self.ee_y = []
        self.ee_z = []
        
        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.capture_tfs) # 20Hz polling
        self.get_logger().info('Initialized TF plotter node. Listening for TFs...')

    def capture_tfs(self):
        t = time.time() - self.start_time
        
        c_x, c_y, c_z = None, None, None
        e_x, e_y, e_z = None, None, None
        
        has_changes = False
        
        try:
            trans_cmd = self.tf_buffer.lookup_transform('delta_robot/world_link', 'delta_robot/commanded_end_effector_pin', rclpy.time.Time())
            c_x = trans_cmd.transform.translation.x * 1000.0
            c_y = trans_cmd.transform.translation.y * 1000.0
            c_z = trans_cmd.transform.translation.z * 1000.0
            has_changes = True
        except Exception as e:
            if self.cmd_x:
                c_x, c_y, c_z = self.cmd_x[-1], self.cmd_y[-1], self.cmd_z[-1]
            else:
                c_x, c_y, c_z = 0.0, 0.0, 0.0

        try:
            trans_ee = self.tf_buffer.lookup_transform('delta_robot/world_link', 'ee_link', rclpy.time.Time())
            e_x = trans_ee.transform.translation.x * 1000.0
            e_y = trans_ee.transform.translation.y * 1000.0
            e_z = trans_ee.transform.translation.z * 1000.0
            has_changes = True
        except Exception as e:
            if self.ee_x:
                e_x, e_y, e_z = self.ee_x[-1], self.ee_y[-1], self.ee_z[-1]
            else:
                e_x, e_y, e_z = 0.0, 0.0, 0.0

        # Output even if one is missing, to keep graph moving
        self.times.append(t)
        self.cmd_x.append(c_x)
        self.cmd_y.append(c_y)
        self.cmd_z.append(c_z)
        self.ee_x.append(e_x)
        self.ee_y.append(e_y)
        self.ee_z.append(e_z)
        
        # Retain last 200 points (~10 seconds at 20Hz)
        if len(self.times) > 200:
            self.times.pop(0)
            self.cmd_x.pop(0)
            self.cmd_y.pop(0)
            self.cmd_z.pop(0)
            self.ee_x.pop(0)
            self.ee_y.pop(0)
            self.ee_z.pop(0)

def update_plot(frame, node, axs):
    if len(node.times) < 2:
        return
    
    axs[0].clear()
    axs[1].clear()
    axs[2].clear()
    
    # X Plot
    axs[0].plot(node.times, node.cmd_x, label='Commanded X', color='cyan', linestyle='--', linewidth=2)
    axs[0].plot(node.times, node.ee_x, label='Measured X', color='red', alpha=0.9, linewidth=2)
    axs[0].legend(loc='upper right', fontsize='small')
    axs[0].set_ylabel('X (mm)')
    axs[0].grid(True, linestyle=':', alpha=0.4)
    axs[0].set_title('Realtime Trajectory: Commanded vs Measured', fontweight='bold')
    
    # Y Plot
    axs[1].plot(node.times, node.cmd_y, label='Commanded Y', color='magenta', linestyle='--', linewidth=2)
    axs[1].plot(node.times, node.ee_y, label='Measured Y', color='green', alpha=0.9, linewidth=2)
    axs[1].legend(loc='upper right', fontsize='small')
    axs[1].set_ylabel('Y (mm)')
    axs[1].grid(True, linestyle=':', alpha=0.4)
    
    # Z Plot
    axs[2].plot(node.times, node.cmd_z, label='Commanded Z', color='yellow', linestyle='--', linewidth=2)
    axs[2].plot(node.times, node.ee_z, label='Measured Z', color='dodgerblue', alpha=0.9, linewidth=2)
    axs[2].legend(loc='upper right', fontsize='small')
    axs[2].set_ylabel('Z (mm)')
    axs[2].set_xlabel('Time (sec)')
    axs[2].grid(True, linestyle=':', alpha=0.4)

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TfPlotter()
    
    # Run ROS spin loop in background
    thread = threading.Thread(target=ros_spin_thread, args=(node,))
    thread.daemon = True
    thread.start()
    
    # Setup matplotlib window
    plt.style.use('dark_background')
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    fig.tight_layout(pad=3.0)
    
    # Animate at constant rate
    ani = FuncAnimation(fig, update_plot, fargs=(node, axs), interval=50, cache_frame_data=False)
    
    plt.show() # Blocks until X out
    
    node.get_logger().info('Shutting down plotter...')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
