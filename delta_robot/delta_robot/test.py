#!/usr/bin/env python3

import re
import time
import serial


PORT = "/dev/ttyUSB0"
BAUD = 500000
TIMEOUT_S = 0.2

# EEPROM angle limit update (disabled by default for safety).
APPLY_NEW_LIMITS = True
NEW_LIMIT_MIN = 0
NEW_LIMIT_MAX = 4090

# Read robustness / pass-fail reporting
GETP_RETRIES = 3
GETP_RETRY_WAIT_S = 0.05
TARGET_TOLERANCE_TICKS = 30
SETTLE_TIMEOUT_S = 2.5
SETTLE_SAMPLE_S = 0.12

# Logging controls
LOG_COMMANDS = True
LOG_GETP = False

LIMIT_IDS = (1, 2, 3, 4, 5)


def extract_pos(lines):
    for line in lines:
        match = re.search(r"pos=(\d+)", line)
        if match:
            return int(match.group(1))
    return None


def extract_limits(lines):
    for line in lines:
        match = re.search(r"LIMITS\s+id=(\d+)\s+min=(\d+)\s+max=(\d+)", line)
        if match:
            return int(match.group(1)), int(match.group(2)), int(match.group(3))
    return None


def last_nonempty(lines):
    if not lines:
        return ""
    return lines[-1]


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


def poll_pos(ser, motor_id, n=3, pause_s=0.08):
    values = []
    for _ in range(n):
        lines = []
        pos = None
        for attempt in range(GETP_RETRIES):
            lines = send_cmd(ser, f"GETP {motor_id}", wait_s=pause_s, log=LOG_GETP)
            pos = extract_pos(lines)
            if pos is not None:
                break
            if attempt < (GETP_RETRIES - 1):
                time.sleep(GETP_RETRY_WAIT_S)
        if pos is not None:
            values.append(pos)
        else:
            print(f"WARN: GETP {motor_id} failed after {GETP_RETRIES} retries ({last_nonempty(lines)})")
    return values


def compact_values(values):
    if not values:
        return "[]"
    if len(values) <= 4:
        return str(values)
    return f"[{values[0]} .. {values[-1]}] (n={len(values)})"


def summarize_step(target, id4_values, id5_values):
    def eval_motor(values):
        if not values:
            return "NO_DATA"
        err = abs(values[-1] - target)
        return f"OK err={err}" if err <= TARGET_TOLERANCE_TICKS else f"MISS err={err}"

    s4 = eval_motor(id4_values)
    s5 = eval_motor(id5_values)
    print(
        f"SUMMARY target={target} tol={TARGET_TOLERANCE_TICKS} | "
        f"ID4={s4} last={id4_values[-1] if id4_values else 'NA'} | "
        f"ID5={s5} last={id5_values[-1] if id5_values else 'NA'}"
    )


def wait_for_target(ser, motor_id, target, timeout_s=SETTLE_TIMEOUT_S):
    start = time.monotonic()
    samples = []
    while (time.monotonic() - start) < timeout_s:
        vals = poll_pos(ser, motor_id, n=1, pause_s=0.06)
        if vals:
            pos = vals[-1]
            samples.append(pos)
            if abs(pos - target) <= TARGET_TOLERANCE_TICKS:
                break
        time.sleep(SETTLE_SAMPLE_S)
    return samples


def target_in_range(target, lim):
    if lim is None:
        return True
    _id, min_v, max_v = lim
    return min_v <= target <= max_v


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
    time.sleep(0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    try:
        # Disable stream spam so command responses are easy to read.
        send_cmd(ser, "STREAM 0", log=LOG_COMMANDS)
        send_cmd(ser, "TMODE POS", log=LOG_COMMANDS)

        limits_by_id = {motor_id: None for motor_id in LIMIT_IDS}
        for motor_id in LIMIT_IDS:
            send_cmd(ser, f"MODE {motor_id} 0", log=LOG_COMMANDS)
            send_cmd(ser, f"TORQUE {motor_id} 1", log=LOG_COMMANDS)
            send_cmd(ser, f"TORQUE_LIMIT {motor_id}", log=LOG_COMMANDS)
            limit_lines = send_cmd(ser, f"LIMITS {motor_id}", log=LOG_COMMANDS)
            limits_by_id[motor_id] = extract_limits(limit_lines)

        if APPLY_NEW_LIMITS:
            print(
                f"\nApplying new limits to IDs {LIMIT_IDS}: min={NEW_LIMIT_MIN}, max={NEW_LIMIT_MAX}"
            )
            for motor_id in LIMIT_IDS:
                send_cmd(ser, f"LIMITS {motor_id} {NEW_LIMIT_MIN} {NEW_LIMIT_MAX}", log=LOG_COMMANDS)
                limit_lines = send_cmd(ser, f"LIMITS {motor_id}", log=LOG_COMMANDS)
                limits_by_id[motor_id] = extract_limits(limit_lines)

        print("\nInitial positions")
        p4 = poll_pos(ser, 4)
        p5 = poll_pos(ser, 5)
        print(f"ID4: {compact_values(p4)}")
        print(f"ID5: {compact_values(p5)}")

        tests = [
            (1500, 800, 20),
            (2048, 800, 20),
            (2500, 800, 20),
            (2800, 800, 20),
        ]

        for pos, speed, acc in tests:
            print(f"\n--- Move both motors to pos={pos}, speed={speed}, acc={acc} ---")
            if not target_in_range(pos, limits_by_id[4]):
                print(
                    f"WARN: target {pos} is outside ID4 limits "
                    f"[{limits_by_id[4][1]}, {limits_by_id[4][2]}]"
                )
            if not target_in_range(pos, limits_by_id[5]):
                print(
                    f"WARN: target {pos} is outside ID5 limits "
                    f"[{limits_by_id[5][1]}, {limits_by_id[5][2]}]"
                )
            send_cmd(ser, f"SET 4 {pos} {speed} {acc}", log=LOG_COMMANDS)
            send_cmd(ser, f"SET 5 {pos} {speed} {acc}", log=LOG_COMMANDS)
            p4 = wait_for_target(ser, 4, pos)
            p5 = wait_for_target(ser, 5, pos)
            print(f"ID4: {compact_values(p4)}")
            print(f"ID5: {compact_values(p5)}")
            summarize_step(pos, p4, p5)

        print("\nDone.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()