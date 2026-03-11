#!/usr/bin/env python3
# joint_state_bridge.py
# Bridges /joint_states -> /delta_motors/motor_position_feedback
#                       -> /delta_motors/motor_velocity_feedback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from deltarobot_interfaces.msg import DeltaJoints, DeltaJointVels

# Must match joint names in your ros2_controllers.yaml
JOINT_NAMES = ["jbf1", "jbf2", "jbf3"]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")
        self.pos_pub = self.create_publisher(
            DeltaJoints, "delta_motors/motor_position_feedback", 10
        )
        self.vel_pub = self.create_publisher(
            DeltaJointVels, "delta_motors/motor_velocity_feedback", 10
        )
        self.sub = self.create_subscription(JointState, "joint_states", self.cb, 10)
        self.get_logger().info("JointStateBridge started")

    def cb(self, msg: JointState):
        # Build index map in case joint_states order varies
        try:
            idx = [msg.name.index(j) for j in JOINT_NAMES]
        except ValueError as e:
            self.get_logger().warn(f"Joint not found: {e}", throttle_duration_sec=2.0)
            return

        pos = DeltaJoints()
        pos.theta1 = msg.position[idx[0]]
        pos.theta2 = msg.position[idx[1]]
        pos.theta3 = msg.position[idx[2]]
        self.pos_pub.publish(pos)

        vel = DeltaJointVels()
        vel.theta1_vel = msg.velocity[idx[0]] if msg.velocity else 0.0
        vel.theta2_vel = msg.velocity[idx[1]] if msg.velocity else 0.0
        vel.theta3_vel = msg.velocity[idx[2]] if msg.velocity else 0.0
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
