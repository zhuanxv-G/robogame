import serial
import time

PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

print("=" * 55)
print("Raspberry Pi <-> STM32 UART Link Test")
print(f"Port     : {PORT}")
print(f"Baudrate : {BAUDRATE}")
print("=" * 55)

try:
    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
    ) as ser:

        print("[OK] UART opened successfully.")
        print("[INFO] Sending PING once per second.")
        print("[INFO] Expected STM32 response: PONG")
        print("[INFO] Press Ctrl+C to stop.\n")

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        seq = 1

        while True:
            message = "PING"

            ser.write((message + "\n").encode("utf-8"))
            ser.flush()

            print(f"[{seq:04d}] TX: {message}")

            raw = ser.readline()

            if raw:
                received = raw.decode("utf-8", errors="replace").strip()

                print(f"[{seq:04d}] RX: {received}")

                if received == "PONG":
                    print(f"[{seq:04d}] PASS\n")
                else:
                    print(f"[{seq:04d}] WARNING: unexpected response\n")

            else:
                print(f"[{seq:04d}] TIMEOUT: no response\n")

            seq += 1
            time.sleep(1)

except serial.SerialException as e:
    print(f"[ERROR] Cannot open UART: {e}")

except KeyboardInterrupt:
    print("\n[INFO] Test stopped by user.")
