import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=1
)

time.sleep(1)

def send_command(command):
    """
    向 STM32 发送一条命令，并打印回复。
    """
    # 清掉之前残留的数据
    ser.reset_input_buffer()

    message = command + "\n"

    ser.write(message.encode("utf-8"))
    ser.flush()

    print(f"[TX] {command}")

    response = ser.readline().decode(
        "utf-8",
        errors="replace"
    ).strip()

    if response:
        print(f"[RX] {response}")
    else:
        print("[RX] No response")

    return response


print("=" * 50)
print("Raspberry Pi -> STM32 Motor Test")
print("=" * 50)

try:
    while True:

        print()
        print("请选择命令：")
        print("  w = FORWARD")
        print("  s = BACKWARD")
        print("  a = LEFT")
        print("  d = RIGHT")
        print("  x = STOP")
        print("  q = QUIT")

        key = input("> ").strip().lower()

        if key == "w":
            send_command("FORWARD")

        elif key == "s":
            send_command("BACKWARD")

        elif key == "a":
            send_command("LEFT")

        elif key == "d":
            send_command("RIGHT")

        elif key == "x":
            send_command("STOP")

        elif key == "q":
            # 退出前一定先停车
            send_command("STOP")
            break

        else:
            print("未知输入")

except KeyboardInterrupt:
    print("\nCtrl+C detected.")

    try:
        send_command("STOP")
    except:
        pass

finally:
    ser.close()
    print("Serial port closed.")
