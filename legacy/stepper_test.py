import serial
import time

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=0.2)

def send(cmd):
    print(f"[TX] {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()

    end = time.time() + 2.0

    while time.time() < end:
        line = ser.readline().decode(errors="ignore").strip()

        if line:
            print(f"[RX] {line}")

            if "DONE" in line:
                break

try:
    time.sleep(0.5)

    # 先确保停止
    send("STOP")

    while True:
        print()
        print("1 = +5 steps, slow")
        print("2 = -5 steps, slow")
        print("3 = +20 steps, slow")
        print("4 = -20 steps, slow")
        print("5 = +20 steps, 800pps")
        print("6 = -20 steps, 800pps")
        print("s = STOP")
        print("q = quit")

        choice = input("> ").strip().lower()

        if choice == "1":
            send("STEPPER_SLOW,5")

        elif choice == "2":
            send("STEPPER_SLOW,-5")

        elif choice == "3":
            send("STEPPER_SLOW,20")

        elif choice == "4":
            send("STEPPER_SLOW,-20")

        elif choice == "5":
            send("STEPPER_NUDGE,20")

        elif choice == "6":
            send("STEPPER_NUDGE,-20")

        elif choice == "s":
            send("STOP")

        elif choice == "q":
            send("STOP")
            break

finally:
    ser.close()
    