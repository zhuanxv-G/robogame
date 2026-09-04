import serial
import time

PORT = "/dev/ttyUSB0"

BAUDRATES = [
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
    460800,
]

for baud in BAUDRATES:
    print()
    print("=" * 50)
    print(f"Testing baudrate: {baud}")

    try:
        stm = serial.Serial(
            PORT,
            baud,
            timeout=0.3
        )

        time.sleep(0.3)

        stm.reset_input_buffer()
        stm.reset_output_buffer()

        data = b"HELLO_STM32,1\n"

        stm.write(data)
        stm.flush()

        print(f"[TX] {data!r}")

        time.sleep(0.3)

        reply = stm.read(stm.in_waiting or 64)

        if reply:
            print(f"[RX RAW] {reply!r}")
            print(f"[RX HEX] {reply.hex(' ')}")

            try:
                print(
                    "[RX TEXT]",
                    reply.decode("ascii")
                )
            except UnicodeDecodeError:
                print("[RX TEXT] <not valid ASCII>")
        else:
            print("[RX] no data")

        stm.close()

    except Exception as e:
        print(f"[ERROR] {e}")

    time.sleep(0.5)

print()
print("Test finished.")