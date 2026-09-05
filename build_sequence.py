"""Standalone, hardware-tested RoboGame arm build entry point."""

import serial
import time

from arm_sequence import ArmSequence


PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = 115200


def run_menu(arm):
    print("1 = run group 1")
    print("2 = run group 2")
    print("3 = run group 3")
    print("a = run ALL groups")
    print("s = STOP")
    print("q = quit")

    while True:
        choice = input("\nSELECT > ").strip().lower()

        if choice == "1":
            arm.group1()
        elif choice == "2":
            arm.group2()
        elif choice == "3":
            arm.group3()
        elif choice == "a":
            confirm = input("Run ALL build actions? (yes/no) > ").strip().lower()

            if confirm == "yes":
                arm.run_all()
            else:
                print("Cancelled.")
        elif choice == "s":
            arm.stop()
        elif choice == "q":
            arm.stop()
            break
        else:
            print("Unknown selection.")


def main():
    ser = None
    arm = None

    try:
        # The standalone entry point owns its one STM32 serial connection.
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        arm = ArmSequence(ser)

        print()
        print("========================================")
        print("       RoboGame Build Sequence")
        print("========================================")
        print(f"STM32: {PORT} @ {BAUD}")
        print()

        time.sleep(0.5)

        print("Connecting STM32...")
        arm.hello()
        print("STM32 connected.")

        # Standalone operation always keeps the verified startup raise.
        arm.startup_raise()
        run_menu(arm)

    except KeyboardInterrupt:
        print()
        print("[CTRL+C] Emergency STOP")

        if arm is not None:
            try:
                arm.stop()
            except Exception:
                pass

    except Exception as exc:
        print()
        print(f"[ERROR] {exc}")
        print("[SAFETY] Sending STOP")

        if arm is not None:
            try:
                arm.stop()
            except Exception:
                pass

    finally:
        if ser is not None:
            ser.close()
            print("Serial closed.")


if __name__ == "__main__":
    main()
