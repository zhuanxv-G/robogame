import serial
import time


STM32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STM32_BAUD = 115200


stm32 = serial.Serial(
    STM32_PORT,
    STM32_BAUD,
    timeout=0.2
)

time.sleep(1)

stm32.reset_input_buffer()

print("STM32 connected")


def read_reply():

    time.sleep(0.05)

    while stm32.in_waiting:

        data = stm32.readline()

        if data:
            print(
                "RX:",
                data.decode(errors="replace").strip()
            )


def send_once(cmd):

    stm32.write((cmd + "\n").encode())

    print("TX:", cmd)

    read_reply()


def run_vel(x, y, omega, duration=2.0):

    cmd = f"VEL,{x},{y},{omega}"

    print()
    print("========================")
    print("Running:", cmd)
    print("========================")

    start = time.time()

    # 约5Hz持续刷新，避免REMOTE 1秒watchdog停车
    while time.time() - start < duration:

        stm32.write((cmd + "\n").encode())

        read_reply()

        time.sleep(0.15)

    send_once("STOP")


try:

    while True:

        print()
        print("1  Forward")
        print("2  Backward")
        print("3  Right strafe")
        print("4  Left strafe")
        print("5  Right-forward diagonal")
        print("6  Left-forward diagonal")
        print("7  Rotate left")
        print("8  Rotate right")
        print("9  Watchdog test")
        print("0  STOP")
        print("q  Quit")

        choice = input("\nSelect> ").strip()

        if choice == "1":

            run_vel(0.0, 0.2, 0.0)

        elif choice == "2":

            run_vel(0.0, -0.2, 0.0)

        elif choice == "3":

            run_vel(0.2, 0.0, 0.0)

        elif choice == "4":

            run_vel(-0.2, 0.0, 0.0)

        elif choice == "5":

            run_vel(0.15, 0.15, 0.0)

        elif choice == "6":

            run_vel(-0.15, 0.15, 0.0)

        elif choice == "7":

            run_vel(0.0, 0.0, 0.8)

        elif choice == "8":

            run_vel(0.0, 0.0, -0.8)

        elif choice == "9":

            print("Sending ONE VEL command.")
            print("Robot should automatically stop after about 1 second.")

            send_once("VEL,0.0,0.2,0.0")

        elif choice == "0":

            send_once("STOP")

        elif choice.lower() == "q":

            break


except KeyboardInterrupt:

    print("\nCtrl+C")


finally:

    try:
        send_once("STOP")
    except Exception:
        pass

    stm32.close()

    print("Stopped")