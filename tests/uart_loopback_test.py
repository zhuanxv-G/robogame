import serial
import time

PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

print("=" * 50)
print("Raspberry Pi UART Loopback Test")
print(f"Port     : {PORT}")
print(f"Baudrate : {BAUDRATE}")
print("=" * 50)

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
    )

    print("[OK] Serial port opened.")
    print("[INFO] Pin 8 (TX) should be connected to Pin 10 (RX).\n")

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    success = 0
    total = 10

    for i in range(1, total + 1):
        message = f"PING_{i}"

        ser.write((message + "\n").encode("utf-8"))
        ser.flush()

        print(f"[TX] {message}")

        raw = ser.readline()

        if raw:
            received = raw.decode("utf-8", errors="replace").strip()
            print(f"[RX] {received}")

            if received == message:
                print("[PASS]\n")
                success += 1
            else:
                print("[FAIL] Data mismatch.\n")
        else:
            print("[FAIL] No data received.\n")

        time.sleep(0.5)

    print("=" * 50)
    print(f"Result: {success}/{total} passed")

    if success == total:
        print("[SUCCESS] UART loopback is working.")
    else:
        print("[WARNING] UART loopback test failed.")

except serial.SerialException as e:
    print(f"[ERROR] Serial error: {e}")

except KeyboardInterrupt:
    print("\n[INFO] Test stopped.")

finally:
    try:
        ser.close()
    except Exception:
        pass
