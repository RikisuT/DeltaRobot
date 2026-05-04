#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from deltarobot_interfaces.msg import DeltaJoints, DeltaJointVels
import math

DEFAULT_JOINT_NAMES = ["motor_joint_1", "motor_joint_2", "motor_joint_3"]


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
        super().__init__('joint_state_bridge')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.pos_pub = self.create_publisher(DeltaJoints, 'delta_motors/motor_position_feedback', 10)
        self.vel_pub = self.create_publisher(DeltaJointVels, 'delta_motors/motor_velocity_feedback', 10)
        self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg: JointState):
        try:
            indices = [msg.name.index(name) for name in self.joint_names]
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
