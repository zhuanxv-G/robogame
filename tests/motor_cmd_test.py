import serial
import time

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

ser = serial.Serial(
    PORT,
    115200,
    timeout=0.2
)

time.sleep(1)

def send_cmd(cmd, duration=2):
    print(f"\n===== {cmd} =====")

    end = time.time() + duration

    while time.time() < end:
        data = (cmd + "\n").encode("ascii")
        ser.write(data)
        ser.flush()

        print("TX :", repr(data))

        time.sleep(0.1)

        while ser.in_waiting:
            rx = ser.readline()
            print("RX :", repr(rx))

        time.sleep(0.1)

    # 每个动作后停止
    ser.write(b"STOP\n")
    ser.flush()
    print("TX :", repr(b"STOP\n"))

    time.sleep(1)

    while ser.in_waiting:
        print("RX :", repr(ser.readline()))


print("Motor command test start")

send_cmd("FORWARD", 2)
send_cmd("LEFT", 2)
send_cmd("RIGHT", 2)
send_cmd("BACKWARD", 2)

print("\nTest finished.")