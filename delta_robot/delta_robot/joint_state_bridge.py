#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from deltarobot_interfaces.msg import DeltaJoints, DeltaJointVels
import math

JOINT_NAMES = ["jbf1", "jbf2", "jbf3"]


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
    def cb(self, msg: JointState):
        try:
            indices = [msg.name.index(name) for name in JOINT_NAMES]
        except ValueError:
            return

        corrected = [correct_joint_angle(msg.position[i]) for i in indices]
        vels = [msg.velocity[i] if msg.velocity else 0.0 for i in indices]

        pos_msg = DeltaJoints()
        pos_msg.theta1, pos_msg.theta2, pos_msg.theta3 = corrected
        self.pos_pub.publish(pos_msg)

        vel_msg = DeltaJointVels()
        vel_msg.theta1_vel, vel_msg.theta2_vel, vel_msg.theta3_vel = vels
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
