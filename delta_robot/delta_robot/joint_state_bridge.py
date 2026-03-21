#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from deltarobot_interfaces.msg import DeltaJoints, DeltaJointVels
import math

ACTUATED_JOINT_NAMES = ["jbf1", "jbf2", "jbf3", "Bevelj1", "Bevelj2"]


def correct_joint_angle(raw: float) -> float:
    """
    Gazebo can report ~-π for joints physically at 0 rad (arms horizontal)
    due to joint angle reference ambiguity on init.
    Valid range is [-0.1745, 1.5708] so anything below -0.5 is a wrap artifact.
    Adding π maps -π → 0 correctly.
    """
    if raw < -0.5:
        return raw + math.pi
    return raw


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")

        self.pos_pub = self.create_publisher(
            DeltaJoints, "delta_motors/motor_position_feedback", 10
        )
        self.vel_pub = self.create_publisher(
            DeltaJointVels, "delta_motors/motor_velocity_feedback", 10
        )

        self.create_subscription(JointState, "/joint_states", self.cb, 10)

    def cb(self, msg: JointState):
        try:
            indices = [msg.name.index(name) for name in ACTUATED_JOINT_NAMES]
        except ValueError:
            return

        corrected = [
            correct_joint_angle(msg.position[i]) if j < 3 else msg.position[i]
            for j, i in enumerate(indices)
        ]
        vels = [msg.velocity[i] if msg.velocity else 0.0 for i in indices]

        pos_msg = DeltaJoints()
        if hasattr(pos_msg, "theta1"):
            pos_msg.theta1 = corrected[0]
        if hasattr(pos_msg, "theta2"):
            pos_msg.theta2 = corrected[1]
        if hasattr(pos_msg, "theta3"):
            pos_msg.theta3 = corrected[2]
        if hasattr(pos_msg, "theta4"):
            pos_msg.theta4 = corrected[3]
        if hasattr(pos_msg, "theta5"):
            pos_msg.theta5 = corrected[4]
        self.pos_pub.publish(pos_msg)

        vel_msg = DeltaJointVels()
        if hasattr(vel_msg, "theta1_vel"):
            vel_msg.theta1_vel = vels[0]
        if hasattr(vel_msg, "theta2_vel"):
            vel_msg.theta2_vel = vels[1]
        if hasattr(vel_msg, "theta3_vel"):
            vel_msg.theta3_vel = vels[2]
        if hasattr(vel_msg, "theta4_vel"):
            vel_msg.theta4_vel = vels[3]
        if hasattr(vel_msg, "theta5_vel"):
            vel_msg.theta5_vel = vels[4]
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
