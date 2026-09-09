import serial
import time

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

try:
    while True:
        cmd = input("CMD > ").strip()

        if cmd.lower() in ("q", "quit", "exit"):
            break

        ser.write((cmd + "\n").encode())
        ser.flush()

        time.sleep(0.05)

        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("STM32 >", line)

finally:
    ser.close()
    