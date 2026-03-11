#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import DeltaJoints
from deltarobot_interfaces.msg import DeltaJointVels
from deltarobot_interfaces.srv import SetJointLimits

from stservo.sdk import PortHandler, GroupSyncRead, COMM_SUCCESS, SMS_STS_PRESENT_POSITION_L, SMS_STS_PRESENT_SPEED_L
from stservo.sdk.sts import sts as sms_sts

# ST Servo Constants
BAUDRATE = 1000000
DEVICE_NAME = "/dev/ttyUSB0"

# Converting from radians to motor position mapping
# Motor Position 0-4095 corresponds to 360 degrees (2*pi radians)
# 2047 is the middle position
MOTOR_MID_POS = 2047
MOTOR_MAX_POS = 4095
RAD_TO_MOTOR_TICKS = 4095.0 / (2.0 * math.pi)

# Up pos is theta = 0, Down pos is theta = pi/2
UP_POS = 3073.0  # From old C++ code
DOWN_POS = 2048.0  # From old C++ code
RAD_TO_TICKS = (DOWN_POS - UP_POS) / (math.pi / 2.0)

# Velocity limit range is 0~4095. 50 steps/sec ≈ 0.732 RPM.
VEL_UNIT_RPM = 0.732 / 50.0  # rpm per step/sec
RAD_S_TO_REV_MIN = 60.0 / (2.0 * math.pi)


class DeltaMotorControl(Node):
    def __init__(self):
        super().__init__("delta_motor_control")
        self.get_logger().info("Python DeltaMotorControl Node Started")

        self.qos_depth = self.declare_parameter("qos_depth", 10).value

        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = sms_sts(self.portHandler)

        # Open port
        if self.portHandler.openPort():
            self.get_logger().info(f"Succeeded to open the port {DEVICE_NAME}")
        else:
            self.get_logger().error(f"Failed to open the port {DEVICE_NAME}")
            sys.exit(1)

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info(f"Succeeded to change the baudrate to {BAUDRATE}")
        else:
            self.get_logger().error(f"Failed to change the baudrate to {BAUDRATE}")
            sys.exit(1)

        self.groupSyncRead = GroupSyncRead(
            self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11
        )

        # Subscribers
        self.delta_joints_sub = self.create_subscription(
            DeltaJoints,
            "delta_motors/set_joints",
            self.set_joints_callback,
            self.qos_depth,
        )

        self.delta_joint_vels_sub = self.create_subscription(
            DeltaJointVels,
            "delta_motors/set_joint_vels",
            self.set_joint_vels_callback,
            self.qos_depth,
        )

        # Service
        self.set_joint_limits_server = self.create_service(
            SetJointLimits,
            "delta_motors/set_joint_limits",
            self.set_joint_limits_callback,
        )

        # Publishers
        self.motor_positions_pub = self.create_publisher(
            DeltaJoints, "delta_motors/motor_position_feedback", 10
        )
        self.motor_velocities_pub = self.create_publisher(
            DeltaJointVels, "delta_motors/motor_velocity_feedback", 10
        )

        # Timer (25Hz)
        self.timer = self.create_timer(1.0 / 25.0, self.timer_callback)

    def convert_to_radians(self, motor_pos):
        return (motor_pos - UP_POS) / RAD_TO_TICKS

    def convert_to_motor_position(self, theta):
        motor_pos = RAD_TO_TICKS * theta + UP_POS
        return int(max(0, min(MOTOR_MAX_POS, motor_pos)))

    def convert_to_motor_velocity(self, theta_vel):
        rpm = RAD_S_TO_REV_MIN * theta_vel
        # Convert rpm to step/sec
        return int(
            abs(rpm / VEL_UNIT_RPM)
        )  # Velocity command is absolute, direction based on position target

    def set_joints_callback(self, msg):
        motor_positions = [
            self.convert_to_motor_position(msg.theta1),
            self.convert_to_motor_position(msg.theta2),
            self.convert_to_motor_position(msg.theta3),
        ]

        # 2400 is the default speed from the example, 50 is acc
        scs_moving_speed = 2400
        scs_moving_acc = 50

        # Adding parameters to sync write
        for i, pos in enumerate(motor_positions):
            scs_id = i + 1
            result = self.packetHandler.SyncWritePosEx(
                scs_id, pos, scs_moving_speed, scs_moving_acc
            )
            if not result:
                self.get_logger().error(
                    f"[ID:{scs_id:03d}] groupSyncWrite addparam failed"
                )

        # Syncwrite goal position
        scs_comm_result = self.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            self.get_logger().error(self.packetHandler.getTxRxResult(scs_comm_result))

        self.packetHandler.groupSyncWrite.clearParam()
        self.get_logger().debug(f"Motor Positions Set: {motor_positions} [motor ticks]")

    def set_joint_vels_callback(self, msg):
        # NOTE: Using SyncWritePosEx for velocity control as wheel mode (continuous rotation)
        # is typically not used for Delta robot arms which have strict position limits.
        # This implementation matches the old dynamixel logic conceptually.
        motor_vels = [
            self.convert_to_motor_velocity(msg.theta1_vel),
            self.convert_to_motor_velocity(msg.theta2_vel),
            self.convert_to_motor_velocity(msg.theta3_vel),
        ]

        # For true velocity control we'd need to use WheelMode, but for now we issue speed commands
        # with a very far position target.
        # If true wheel mode is needed, self.packetHandler.WriteSpe() can be used per servo.
        for i, vel in enumerate(motor_vels):
            scs_id = i + 1
            # We set wheel mode (speed control) for this servo
            self.packetHandler.WheelMode(scs_id)
            # Write speed (Note: WriteSpe is not part of groupSyncWrite in the provided python, but we can do consecutive writes)
            # In ST_Servo speed control, bit 15 determines direction.
            # Convert signed velocity to magnitude + direction bit
            target_vel = msg.__getattribute__(f"theta{scs_id}_vel")
            speed = vel
            if target_vel < 0:
                speed |= 1 << 15

            self.packetHandler.WriteSpe(scs_id, speed, 50)

        self.get_logger().debug(f"Motor Velocities Set: {motor_vels} [ticks/s]")

    def set_joint_limits_callback(self, request, response):
        motor_min = self.convert_to_motor_position(request.min_rad)
        motor_max = self.convert_to_motor_position(request.max_rad)

        for scs_id in range(1, 4):
            # The python SDK doesn't expose a direct "write 2 bytes to register" for arbitrary registers
            # easily in the high-level class outside of WritePosEx. We can use write2ByteTxRx from PacketHandler
            # Min/Max angle limits are registers 9, 10 and 11, 12.
            # MIN_ANGLE_LIMIT_L = 9
            # MAX_ANGLE_LIMIT_L = 11

            scs_comm_result, scs_error = (
                self.packetHandler.packetHandler.write2ByteTxRx(
                    self.portHandler, scs_id, 9, motor_min
                )
            )

            scs_comm_result, scs_error = (
                self.packetHandler.packetHandler.write2ByteTxRx(
                    self.portHandler, scs_id, 11, motor_max
                )
            )

        self.get_logger().info(
            f"Joint Limits Set: Position [{request.min_rad}, {request.max_rad}] [rad]"
        )
        response.success = True
        return response

    def timer_callback(self):
        # Add parameter storage for SC Servo#1~3 present position value
        for scs_id in range(1, 4):
            scs_addparam_result = self.groupSyncRead.addParam(scs_id)
            if not scs_addparam_result:
                self.get_logger().error(
                    f"[ID:{scs_id:03d}] groupSyncRead addparam failed"
                )

        scs_comm_result = self.groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            # Just debug log to not spam if bus is busy
            self.get_logger().debug(self.packetHandler.getTxRxResult(scs_comm_result))

        motor_positions = [0, 0, 0]
        motor_velocities = [0, 0, 0]

        for scs_id in range(1, 4):
            scs_data_result, scs_error = self.groupSyncRead.isAvailable(
                scs_id, SMS_STS_PRESENT_POSITION_L, 11
            )
            if scs_data_result:
                pres_pos = self.groupSyncRead.getData(
                    scs_id, SMS_STS_PRESENT_POSITION_L, 2
                )
                pres_spd = self.groupSyncRead.getData(
                    scs_id, SMS_STS_PRESENT_SPEED_L, 2
                )

                # Handling signed speed (15th bit is direction)
                spd_val = self.packetHandler.scs_tohost(pres_spd, 15)

                motor_positions[scs_id - 1] = pres_pos
                motor_velocities[scs_id - 1] = spd_val
            else:
                self.get_logger().debug(
                    f"[ID:{scs_id:03d}] groupSyncRead getdata failed"
                )

        self.groupSyncRead.clearParam()

        # Convert and Publish
        pos_msg = DeltaJoints()
        vel_msg = DeltaJointVels()

        pos_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.stamp = self.get_clock().now().to_msg()

        pos_msg.theta1 = self.convert_to_radians(motor_positions[0])
        pos_msg.theta2 = self.convert_to_radians(motor_positions[1])
        pos_msg.theta3 = self.convert_to_radians(motor_positions[2])

        # ticks/sec -> rpm -> rad/s
        vel_msg.theta1_vel = motor_velocities[0] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
        vel_msg.theta2_vel = motor_velocities[1] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
        vel_msg.theta3_vel = motor_velocities[2] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN

        self.motor_positions_pub.publish(pos_msg)
        self.motor_velocities_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
