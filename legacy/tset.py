import serial
import time

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

ser = serial.Serial(
    PORT,
    115200,
    timeout=1
)

time.sleep(1)

while True:
    data = b"HELLO_STM32,1\n"

    ser.write(data)
    ser.flush()

    print("TX :", repr(data))

    reply = ser.readline()

    if reply:
        print("RX :", repr(reply))
    else:
        print("RX : no data")

    time.sleep(1)