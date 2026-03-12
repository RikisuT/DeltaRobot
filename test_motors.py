import time
import sys
from stservo.sdk import PortHandler, COMM_SUCCESS, STS_PRESENT_POSITION_L
from stservo.sdk.sts import sts

BAUDRATE = 1000000
DEVICE_NAME = "/dev/ttyUSB0"


def main():
    portHandler = PortHandler(DEVICE_NAME)
    packetHandler = sts(portHandler)

    if portHandler.openPort():
        print(f"Succeeded to open the port {DEVICE_NAME}")
    else:
        print(f"Failed to open the port {DEVICE_NAME}")
        sys.exit(1)

    if portHandler.setBaudRate(BAUDRATE):
        print(f"Succeeded to change the baudrate to {BAUDRATE}")
    else:
        print(f"Failed to change the baudrate to {BAUDRATE}")
        sys.exit(1)

    # Ping motor 1
    model_number, result, error = packetHandler.ping(1)
    if result == COMM_SUCCESS:
        print(f"[ID:1] ping succeeded. Model Number: {model_number}")
    else:
        print(f"[ID:1] ping failed. Result: {result}, Error: {error}")

    # Ping motor 2
    model_number, result, error = packetHandler.ping(2)
    if result == COMM_SUCCESS:
        print(f"[ID:2] ping succeeded. Model Number: {model_number}")
    else:
        print(f"[ID:2] ping failed. Result: {result}, Error: {error}")

    print("Trying to read present position of ID 1:")
    pos1_result = packetHandler.ReadPos(1)
    if pos1_result[0] != -1:
        print(f"[ID:1] Present Position: {pos1_result}")
    else:
        print(f"[ID:1] Failed to read present position.")

    print("Trying to read present position of ID 2:")
    pos2_result = packetHandler.ReadPos(2)
    if pos2_result[0] != -1:
        print(f"[ID:2] Present Position: {pos2_result}")
    else:
        print(f"[ID:2] Failed to read present position.")

    print("\nAttempting to move motor 1 slightly...")
    if pos1_result[0] != -1:
        target1 = pos1_result[0] + 100
        print(f"[ID:1] Moving to {target1} at speed 2400, acc 50...")
        packetHandler.WritePosEx(1, target1, 2400, 50)

    print("\nAttempting to move motor 2 slightly...")
    if pos2_result[0] != -1:
        target2 = pos2_result[0] + 100
        print(f"[ID:2] Moving to {target2} at speed 2400, acc 50...")
        packetHandler.WritePosEx(2, target2, 2400, 50)

    time.sleep(1)

    portHandler.closePort()
    print("Test complete.")


if __name__ == "__main__":
    main()
