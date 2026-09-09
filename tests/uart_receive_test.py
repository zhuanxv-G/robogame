import serial

PORT = "/dev/serial0"
BAUDRATE = 115200

ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

print("Waiting for STM32...")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        data = ser.readline()

        if data:
            text = data.decode("utf-8", errors="replace").strip()
            print(f"[RX] {text}")

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()
