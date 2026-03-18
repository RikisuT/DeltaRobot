#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import DeltaJoints
from deltarobot_interfaces.msg import DeltaJointVels
from deltarobot_interfaces.srv import SetJointLimits
from std_msgs.msg import Float32MultiArray

from scservo_sdk.port_handler import PortHandler
from scservo_sdk.group_sync_read import GroupSyncRead
from scservo_sdk.scservo_def import COMM_SUCCESS
from scservo_sdk.sms_sts import (
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_SPEED_L,
    sms_sts as SMS_STS,
)

# ST Servo Constants
BAUDRATE = 1000000
DEVICE_NAME = "/dev/ttyUSB0"

# Converting from radians to motor position mapping
# Motor Position 0-4095 corresponds to 360 degrees (2*pi radians)
# 2048 is the middle position (0 radians)
MOTOR_MID_POS = 2048
MOTOR_MAX_POS = 4095
RAD_TO_TICKS = 4096.0 / (2.0 * math.pi)
UP_POS = 2048.0  # Center position corresponds to 0 radians

# Velocity limit range is 0~4095. ST3215-HS uses 50 steps/sec ≈ 0.732 RPM.
VEL_UNIT_RPM = 0.732 / 50.0  # units in RPM per (step/sec)
RAD_S_TO_REV_MIN = 60.0 / (2.0 * math.pi)


class DeltaMotorControl(Node):
    def __init__(self):
        super().__init__("delta_motor_control")
        self.get_logger().info("Python DeltaMotorControl Node Started")

        self.qos_depth = self.declare_parameter("qos_depth", 10).value

        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = SMS_STS(self.portHandler)

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

        # Enable Torque for ID 1, 2, 3
        for st_id in range(1, 4):
            self.packetHandler.write1ByteTxRx(
                st_id, 40, 1
            )  # Register 40 is Torque Enable
            self.get_logger().info(f"Torque enabled for Motor ID: {st_id}")

        self.groupSyncRead = GroupSyncRead(
            self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11
        )

        # Startup Servo Connectivity Check
        self.get_logger().info("Checking servo connectivity...")
        for st_id in range(1, 4):
            self.groupSyncRead.addParam(st_id)

        st_comm_result = self.groupSyncRead.txRxPacket()
        if st_comm_result == COMM_SUCCESS:
            for st_id in range(1, 4):
                st_data_result, _ = self.groupSyncRead.isAvailable(
                    st_id, SMS_STS_PRESENT_POSITION_L, 2
                )
                if st_data_result:
                    self.get_logger().info(f"Servo ID:{st_id:03d} Found")
                else:
                    self.get_logger().warn(f"Servo ID:{st_id:03d} NOT Found")
        else:
            self.get_logger().error(
                f"Startup GroupSyncRead failed: {self.packetHandler.getTxRxResult(st_comm_result)}"
            )

        self.groupSyncRead.clearParam()

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
        self.servo_target_pub = self.create_publisher(
            Float32MultiArray, "/servo/target", 10
        )
        self.servo_actual_pub = self.create_publisher(
            Float32MultiArray, "/servo/actual", 10
        )

        # Timer (50Hz - increased for better control and plotter resolution)
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)

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

        # Max speed settings for ST3215-HS
        # Based on Waveshare/Feetech specs: max speed is 4000.
        # Software unit: 0-4095 is position range. 0-4000 is speed range.
        # Acceleration: 0-254. 254 is maximum snappiness.
        st_moving_speed = 8000  # Increased from 4000 to allow 100+ RPM
        st_moving_acc = 0  # Added slight acceleration for smoothness

        # Adding parameters to sync write
        for i, pos in enumerate(motor_positions):
            st_id = i + 1
            result = self.packetHandler.SyncWritePosEx(
                st_id, pos, st_moving_speed, st_moving_acc
            )
            if not result:
                self.get_logger().error(
                    f"[ID:{st_id:03d}] groupSyncWrite addparam failed"
                )

        # Syncwrite goal position
        st_comm_result = self.packetHandler.groupSyncWrite.txPacket()
        if st_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f"SyncWrite failed: {self.packetHandler.getTxRxResult(st_comm_result)}"
            )
        else:
            # self.get_logger().info(f"SyncWrite sent: {motor_positions}") # Removed for performance at 100Hz
            # Publish to /servo/target for plotter
            target_msg = Float32MultiArray()
            target_msg.data = [float(p) for p in motor_positions]
            self.servo_target_pub.publish(target_msg)

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
            st_id = i + 1
            # We set wheel mode (speed control) for this servo
            self.packetHandler.WheelMode(st_id)
            # Write speed (Note: WriteSpe is not part of groupSyncWrite in the provided python, but we can do consecutive writes)
            # In ST_Servo speed control, bit 15 determines direction.
            # Convert signed velocity to magnitude + direction bit
            target_vel = msg.__getattribute__(f"theta{st_id}_vel")
            speed = vel
            if target_vel < 0:
                speed |= 1 << 15

            self.packetHandler.WriteSpec(st_id, speed, 50)

        self.get_logger().debug(f"Motor Velocities Set: {motor_vels} [ticks/s]")

    def set_joint_limits_callback(self, request, response):
        motor_min = self.convert_to_motor_position(request.min_rad)
        motor_max = self.convert_to_motor_position(request.max_rad)

        for st_id in range(1, 4):
            # The python SDK doesn't expose a direct "write 2 bytes to register" for arbitrary registers
            # easily in the high-level class outside of WritePosEx. We can use write2ByteTxRx from PacketHandler
            # Min/Max angle limits are registers 9, 10 and 11, 12.
            # MIN_ANGLE_LIMIT_L = 9
            # MAX_ANGLE_LIMIT_L = 11

            st_comm_result, st_error = self.packetHandler.write2ByteTxRx(
                st_id, 9, motor_min
            )

            st_comm_result, st_error = self.packetHandler.write2ByteTxRx(
                st_id, 11, motor_max
            )

        self.get_logger().info(
            f"Joint Limits Set: Position [{request.min_rad}, {request.max_rad}] [rad]"
        )
        response.success = True
        return response

    def timer_callback(self):
        # Add parameter storage for ST Servo#1~3 present position value
        for st_id in range(1, 4):
            st_addparam_result = self.groupSyncRead.addParam(st_id)
            if not st_addparam_result:
                self.get_logger().error(
                    f"[ID:{st_id:03d}] groupSyncRead addparam failed"
                )

        st_comm_result = self.groupSyncRead.txRxPacket()
        if st_comm_result != COMM_SUCCESS:
            # Just debug log to not spam if bus is busy
            self.get_logger().debug(self.packetHandler.getTxRxResult(st_comm_result))

        motor_positions = [0, 0, 0]
        motor_velocities = [0, 0, 0]

        for st_id in range(1, 4):
            st_data_result, st_error = self.groupSyncRead.isAvailable(
                st_id, SMS_STS_PRESENT_POSITION_L, 11
            )
            if st_data_result:
                pres_pos = self.groupSyncRead.getData(
                    st_id, SMS_STS_PRESENT_POSITION_L, 2
                )
                pres_spd = self.groupSyncRead.getData(st_id, SMS_STS_PRESENT_SPEED_L, 2)

                # Handling signed speed (15th bit is direction)
                spd_val = self.packetHandler.scs_tohost(pres_spd, 15)

                # Safety check: If position is 0, it usually means a read failure or uninitialized ID
                # 0 ticks corresponds to -PI which is out of bounds for DeltaRobot kinematics
                if pres_pos == 0:
                    self.get_logger().warn(
                        f"[ID:{st_id:03d}] Reported position 0, ignoring to prevent kinematics crash."
                    )
                    continue

                motor_positions[st_id - 1] = pres_pos
                motor_velocities[st_id - 1] = spd_val
            else:
                self.get_logger().debug(
                    f"[ID:{st_id:03d}] groupSyncRead getdata failed"
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

        # Publish to /servo/actual for plotter
        actual_msg = Float32MultiArray()
        actual_msg.data = [float(p) for p in motor_positions]
        self.servo_actual_pub.publish(actual_msg)


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
