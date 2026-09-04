import serial
import time


PORT = "/dev/serial0"
BAUDRATE = 115200


def connect():
    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=1
    )

    print("STM32 serial connected")
    return ser


def send_command(ser, cmd):
    data = cmd + "\n"
    ser.write(data.encode())
    print("send:", cmd)


if __name__ == "__main__":

    uart = connect()

    while True:
        send_command(uart, "test")
        time.sleep(1)
