#!/usr/bin/env python3

import glob
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
BAUDRATE = 500000
DEVICE_NAME = "/dev/ttyUSB0"
MAX_ST_MOVING_SPEED = 7500
MAX_ST_MOVING_ACC = 254

# Converting from radians to motor position mapping
# Motor Position 0-4095 corresponds to 360 degrees (2*pi radians)
# 2048 is the center position (0 radians)
MOTOR_MAX_POS = 4095
RAD_TO_TICKS = 4096.0 / (2.0 * math.pi)
UP_POS = 2048.0

# Velocity limit range is 0~7500. ST3215-HS uses 50 steps/sec ≈ 0.732 RPM.
VEL_UNIT_RPM = 0.732 / 50.0  # units in RPM per (step/sec)
RAD_S_TO_REV_MIN = 60.0 / (2.0 * math.pi)

BICEP_IDS = [1, 2, 3]
EE_IDS = [4, 5]
ALL_MOTOR_IDS = BICEP_IDS + EE_IDS


class DeltaMotorControl(Node):
    def __init__(self):
        super().__init__("delta_motor_control")
        self.get_logger().info("Python DeltaMotorControl Node Started")

        self.qos_depth = self.declare_parameter("qos_depth", 10).value
        self.device_name = self.declare_parameter("device_name", DEVICE_NAME).value
        self.baudrate = int(self.declare_parameter("baudrate", BAUDRATE).value)
        self.bicep_moving_speed = int(self.declare_parameter("moving_speed", 0).value)
        self.bicep_moving_acc = int(self.declare_parameter("moving_acc", 0).value)
        self.ee_moving_speed = int(self.declare_parameter("ee_moving_speed", 0).value)
        self.ee_moving_acc = int(self.declare_parameter("ee_moving_acc", 0).value)
        self.max_write_retries = int(self.declare_parameter("max_write_retries", 3).value)
        self.read_fail_watchdog_limit = int(self.declare_parameter("read_fail_watchdog_limit", 5).value)

        self.bicep_moving_speed = max(0, min(MAX_ST_MOVING_SPEED, self.bicep_moving_speed))
        self.bicep_moving_acc = max(0, min(MAX_ST_MOVING_ACC, self.bicep_moving_acc))
        self.ee_moving_speed = max(0, min(MAX_ST_MOVING_SPEED, self.ee_moving_speed))
        self.ee_moving_acc = max(0, min(MAX_ST_MOVING_ACC, self.ee_moving_acc))
        self.max_write_retries = max(1, self.max_write_retries)
        self.read_fail_watchdog_limit = max(1, self.read_fail_watchdog_limit)
        self.consecutive_read_failures = 0

        # Initialize SDK handlers after probing serial candidates.
        self.portHandler = None
        self.packetHandler = None
        self.groupSyncRead = None
        self.hardware_available = False
        self.visible_motor_ids = []
        self.missing_motor_ids = list(ALL_MOTOR_IDS)

        self._initialize_hardware()

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

    def _get_candidate_ports(self, preferred_port):
        """Return likely serial ports with preferred first when present."""
        detected_ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))

        candidates = []
        if preferred_port:
            candidates.append(preferred_port)

        for port in detected_ports:
            if port not in candidates:
                candidates.append(port)

        return candidates

    def _ping_servo(self, servo_id):
        """Check if a servo responds to ping."""
        try:
            _, scs_comm_result, _ = self.packetHandler.ping(servo_id)
            return scs_comm_result == COMM_SUCCESS
        except Exception:
            return False

    def _initialize_hardware(self):
        candidate_ports = self._get_candidate_ports(self.device_name)
        if not candidate_ports:
            self.get_logger().warning(
                "No serial ports found (expected /dev/ttyUSB* or /dev/ttyACM*); running in no-hardware mode"
            )
            self.hardware_available = False
            return

        self.get_logger().info(f"Trying serial ports: {candidate_ports}")
        port_errors = []

        for port_path in candidate_ports:
            try:
                trial_port = PortHandler(port_path)
                trial_packet = SMS_STS(trial_port)

                if not trial_port.openPort():
                    port_errors.append(f"{port_path}: openPort() returned False")
                    continue

                if not trial_port.setBaudRate(self.baudrate):
                    port_errors.append(f"{port_path}: failed to set baudrate {self.baudrate}")
                    trial_port.closePort()
                    continue

                self.portHandler = trial_port
                self.packetHandler = trial_packet
                self.device_name = port_path
                self.get_logger().info(f"Connected on {port_path} @ {self.baudrate}")
                break

            except Exception as exc:
                port_errors.append(f"{port_path}: {exc}")

        if self.portHandler is None or self.packetHandler is None:
            self.get_logger().warning("Could not open any serial port; running in no-hardware mode")
            for err in port_errors:
                self.get_logger().warning(f"  - {err}")
            self.hardware_available = False
            return

        self.get_logger().info(f"Scanning servo IDs: {ALL_MOTOR_IDS}")
        self.visible_motor_ids = []
        self.missing_motor_ids = []

        for st_id in ALL_MOTOR_IDS:
            if self._ping_servo(st_id):
                self.visible_motor_ids.append(st_id)
                self.get_logger().info(f"Servo ID:{st_id:03d} Found")
            else:
                self.missing_motor_ids.append(st_id)
                self.get_logger().warning(f"Servo ID:{st_id:03d} NOT Found")

        self.get_logger().info(
            f"Visible motors ({len(self.visible_motor_ids)}/{len(ALL_MOTOR_IDS)}): {self.visible_motor_ids}"
        )
        if self.missing_motor_ids:
            self.get_logger().warning(f"Missing motors: {self.missing_motor_ids}")

        if not self.visible_motor_ids:
            self.get_logger().warning("No servos detected; running in no-hardware mode")
            self.portHandler.closePort()
            self.hardware_available = False
            return

        for st_id in self.visible_motor_ids:
            self.packetHandler.write1ByteTxRx(st_id, 40, 1)
            self.get_logger().info(f"Torque enabled for Motor ID: {st_id}")

        self.groupSyncRead = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 11)
        self.hardware_available = True

    def convert_to_radians(self, motor_pos):
        return (motor_pos - UP_POS) / RAD_TO_TICKS

    def convert_to_motor_position(self, theta):
        motor_pos = RAD_TO_TICKS * theta + UP_POS
        return int(max(0, min(MOTOR_MAX_POS, motor_pos)))

    def convert_to_motor_velocity(self, theta_vel):
        rpm = RAD_S_TO_REV_MIN * theta_vel
        # Convert rpm to step/sec
        raw = int(abs(rpm / VEL_UNIT_RPM))
        return max(0, min(MAX_ST_MOVING_SPEED, raw))

    def speed_acc_for_id(self, st_id):
        if st_id in EE_IDS:
            return self.ee_moving_speed, self.ee_moving_acc
        return self.bicep_moving_speed, self.bicep_moving_acc

    def _get_joint_value(self, msg, index, velocity=False):
        """Return theta/theta_vel value from message, defaulting to 0 if absent."""
        field = f"theta{index}_vel" if velocity else f"theta{index}"
        return getattr(msg, field, 0.0)

    def set_joints_callback(self, msg):
        if not self.hardware_available:
            self.get_logger().debug("Ignoring set_joints: hardware unavailable")
            return

        motor_positions = [
            self.convert_to_motor_position(self._get_joint_value(msg, 1)),
            self.convert_to_motor_position(self._get_joint_value(msg, 2)),
            self.convert_to_motor_position(self._get_joint_value(msg, 3)),
            self.convert_to_motor_position(self._get_joint_value(msg, 4)),
            self.convert_to_motor_position(self._get_joint_value(msg, 5)),
        ]

        # For this setup, speed=0 is intentionally used as the fastest mode.
        for i, pos in enumerate(motor_positions):
            st_id = i + 1
            if st_id not in self.visible_motor_ids:
                continue
            st_moving_speed, st_moving_acc = self.speed_acc_for_id(st_id)
            result = self.packetHandler.SyncWritePosEx(
                st_id, pos, st_moving_speed, st_moving_acc
            )
            if not result:
                self.get_logger().error(
                    f"[ID:{st_id:03d}] groupSyncWrite addparam failed"
                )

        # Syncwrite goal position
        st_comm_result = COMM_SUCCESS
        for _ in range(self.max_write_retries):
            st_comm_result = self.packetHandler.groupSyncWrite.txPacket()
            if st_comm_result == COMM_SUCCESS:
                break
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
        if not self.hardware_available:
            self.get_logger().debug("Ignoring set_joint_vels: hardware unavailable")
            return

        # NOTE: Using SyncWritePosEx for velocity control as wheel mode (continuous rotation)
        # is typically not used for Delta robot arms which have strict position limits.
        # This implementation matches the old dynamixel logic conceptually.
        motor_vels = [
            self.convert_to_motor_velocity(self._get_joint_value(msg, 1, velocity=True)),
            self.convert_to_motor_velocity(self._get_joint_value(msg, 2, velocity=True)),
            self.convert_to_motor_velocity(self._get_joint_value(msg, 3, velocity=True)),
            self.convert_to_motor_velocity(self._get_joint_value(msg, 4, velocity=True)),
            self.convert_to_motor_velocity(self._get_joint_value(msg, 5, velocity=True)),
        ]

        # For true velocity control we'd need to use WheelMode, but for now we issue speed commands
        # with a very far position target.
        # If true wheel mode is needed, self.packetHandler.WriteSpe() can be used per servo.
        for i, vel in enumerate(motor_vels):
            st_id = i + 1
            if st_id not in self.visible_motor_ids:
                continue
            # We set wheel mode (speed control) for this servo
            self.packetHandler.WheelMode(st_id)
            # Write speed (Note: WriteSpe is not part of groupSyncWrite in the provided python, but we can do consecutive writes)
            # In ST_Servo speed control, bit 15 determines direction.
            # Convert signed velocity to magnitude + direction bit
            target_vel = self._get_joint_value(msg, st_id, velocity=True)
            speed = vel
            if target_vel < 0:
                speed |= 1 << 15

            self.packetHandler.WriteSpec(st_id, speed, 50)

        self.get_logger().debug(f"Motor Velocities Set: {motor_vels} [ticks/s]")

    def set_joint_limits_callback(self, request, response):
        if not self.hardware_available:
            self.get_logger().warning("set_joint_limits requested but hardware is unavailable")
            response.success = True
            return response

        motor_min = self.convert_to_motor_position(request.min_rad)
        motor_max = self.convert_to_motor_position(request.max_rad)

        for st_id in self.visible_motor_ids:
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
        if not self.hardware_available or self.groupSyncRead is None:
            return

        # Add parameter storage for configured servo IDs present position value
        for st_id in self.visible_motor_ids:
            st_addparam_result = self.groupSyncRead.addParam(st_id)
            if not st_addparam_result:
                self.get_logger().error(
                    f"[ID:{st_id:03d}] groupSyncRead addparam failed"
                )

        st_comm_result = self.groupSyncRead.txRxPacket()
        if st_comm_result != COMM_SUCCESS:
            self.consecutive_read_failures += 1
            self.get_logger().debug(self.packetHandler.getTxRxResult(st_comm_result))
            if self.consecutive_read_failures >= self.read_fail_watchdog_limit:
                self.get_logger().warning(
                    "Read watchdog tripped after %d failures; disabling hardware I/O"
                    % self.consecutive_read_failures
                )
                self.hardware_available = False
            self.groupSyncRead.clearParam()
            return
        self.consecutive_read_failures = 0

        motor_positions = [0] * len(ALL_MOTOR_IDS)
        motor_velocities = [0] * len(ALL_MOTOR_IDS)

        for st_id in self.visible_motor_ids:
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

        if hasattr(pos_msg, "theta1"):
            pos_msg.theta1 = self.convert_to_radians(motor_positions[0])
        if hasattr(pos_msg, "theta2"):
            pos_msg.theta2 = self.convert_to_radians(motor_positions[1])
        if hasattr(pos_msg, "theta3"):
            pos_msg.theta3 = self.convert_to_radians(motor_positions[2])
        if hasattr(pos_msg, "theta4"):
            pos_msg.theta4 = self.convert_to_radians(motor_positions[3])
        if hasattr(pos_msg, "theta5"):
            pos_msg.theta5 = self.convert_to_radians(motor_positions[4])

        # ticks/sec -> rpm -> rad/s
        if hasattr(vel_msg, "theta1_vel"):
            vel_msg.theta1_vel = motor_velocities[0] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
        if hasattr(vel_msg, "theta2_vel"):
            vel_msg.theta2_vel = motor_velocities[1] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
        if hasattr(vel_msg, "theta3_vel"):
            vel_msg.theta3_vel = motor_velocities[2] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
        if hasattr(vel_msg, "theta4_vel"):
            vel_msg.theta4_vel = motor_velocities[3] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
        if hasattr(vel_msg, "theta5_vel"):
            vel_msg.theta5_vel = motor_velocities[4] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN

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
        if hasattr(node, "portHandler"):
            node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
