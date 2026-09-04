import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

print("=" * 50)
print("Raspberry Pi <-> STM32 UART Test")
print(f"Port: {PORT}")
print(f"Baudrate: {BAUDRATE}")
print("=" * 50)

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.5
    )

    print("Serial port opened successfully.")
    print("Press Ctrl+C to stop.\n")

    count = 0

    while True:
        count += 1

        message = f"HELLO_STM32,{count}\n"

        ser.write(message.encode("utf-8"))
        ser.flush()

        print(f"[TX] {message.strip()}")

        time.sleep(0.1)

        if ser.in_waiting > 0:
            data = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if data:
                print(f"[RX] {data}")

        time.sleep(0.9)

except KeyboardInterrupt:
    print("\nTest stopped by user.")

except serial.SerialException as e:
    print(f"\nSerial error: {e}")

except Exception as e:
    print(f"\nError: {e}")

finally:
    try:
        ser.close()
        print("Serial port closed.")
    except:
        pass
