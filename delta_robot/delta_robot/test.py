#!/usr/bin/env python3

import re
import time
import serial


PORT = "/dev/ttyUSB0"
BAUD = 921600
TIMEOUT_S = 0.2

MOTOR_IDS = (4, 5)

APPLY_NEW_LIMITS = True
NEW_LIMIT_MIN = 0
NEW_LIMIT_MAX = 4095

SWING_CYCLES = 6
SWING_SPEED = 800
SWING_ACC = 20
DWELL_S = 1.0

LOG_COMMANDS = True


def send_cmd(ser, command, wait_s=0.08, log=True):
    ser.write((command + "\n").encode("ascii", errors="ignore"))
    ser.flush()
    time.sleep(wait_s)
    raw = ser.read_all().decode(errors="replace").strip()
    lines = [ln for ln in raw.splitlines() if ln.strip()]
    if log:
        print(f"> {command}")
        if lines:
            for ln in lines:
                print(ln)
        else:
            print("(no response)")
    return lines


def mode_ack_ok(lines, motor_id, expected_mode):
    expected = f"OK mode id={motor_id} value={expected_mode}"
    return any(expected in line for line in lines)


def middle_ack_ok(lines, motor_id):
    expected = f"OK middle id={motor_id}"
    return any(expected in line for line in lines)


def extract_limits(lines):
    for line in lines:
        match = re.search(r"LIMITS\s+id=(\d+)\s+min=(\d+)\s+max=(\d+)", line)
        if match:
            return int(match.group(1)), int(match.group(2)), int(match.group(3))
    return None


def force_servo_mode(ser, motor_id):
    mode_lines = send_cmd(ser, f"MODE {motor_id} 0", log=LOG_COMMANDS)
    if not mode_ack_ok(mode_lines, motor_id, 0):
        raise RuntimeError(f"ID{motor_id} MODE command was not acknowledged")


def set_all(ser, pos, speed, acc):
    for motor_id in MOTOR_IDS:
        send_cmd(ser, f"SET {motor_id} {pos} {speed} {acc}", log=LOG_COMMANDS)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
    time.sleep(0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    try:
        send_cmd(ser, "STREAM 0", log=LOG_COMMANDS)
        tmode_lines = send_cmd(ser, "TMODE POS", log=LOG_COMMANDS)
        if not any("OK tmode=POS" in line for line in tmode_lines):
            raise RuntimeError("Failed to set telemetry mode to POS")

        limits_by_id = {}
        print("\nPreparing motors (servo mode + torque + limits)")
        for motor_id in MOTOR_IDS:
            force_servo_mode(ser, motor_id)
            send_cmd(ser, f"TORQUE {motor_id} 1", log=LOG_COMMANDS)

            if APPLY_NEW_LIMITS:
                send_cmd(
                    ser,
                    f"LIMITS {motor_id} {NEW_LIMIT_MIN} {NEW_LIMIT_MAX}",
                    log=LOG_COMMANDS,
                )

            limit_lines = send_cmd(ser, f"LIMITS {motor_id}", log=LOG_COMMANDS)
            limits = extract_limits(limit_lines)
            if limits is None:
                raise RuntimeError(f"Failed reading limits for ID{motor_id}")
            limits_by_id[motor_id] = (limits[1], limits[2])
            print(f"ID{motor_id} limits: min={limits[1]} max={limits[2]}")

        swing_min = min(lim[0] for lim in limits_by_id.values())
        swing_max = max(lim[1] for lim in limits_by_id.values())

        print(
            f"\nSwinging all motors between {swing_min} and {swing_max} "
            f"for {SWING_CYCLES} cycles"
        )

        for cycle in range(1, SWING_CYCLES + 1):
            print(f"\nCycle {cycle}/{SWING_CYCLES}: -> MIN")
            set_all(ser, swing_min, SWING_SPEED, SWING_ACC)
            time.sleep(DWELL_S)

            print(f"Cycle {cycle}/{SWING_CYCLES}: -> MAX")
            set_all(ser, swing_max, SWING_SPEED, SWING_ACC)
            time.sleep(DWELL_S)

        print("\nMoving each motor to midpoint command")
        for motor_id in MOTOR_IDS:
            middle_lines = send_cmd(ser, f"MIDDLE {motor_id}", log=LOG_COMMANDS)
            if middle_ack_ok(middle_lines, motor_id):
                continue

            # Fallback: half of each motor's configured max limit.
            midpoint = limits_by_id[motor_id][1] // 2
            print(f"ID{motor_id}: MIDDLE not acknowledged, fallback midpoint={midpoint}")
            send_cmd(ser, f"SET {motor_id} {midpoint} {SWING_SPEED} {SWING_ACC}", log=LOG_COMMANDS)

        print("\nDone.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()