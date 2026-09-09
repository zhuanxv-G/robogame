import serial
import time


STM32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STM32_BAUD = 115200


print("Connecting STM32...")

stm32 = serial.Serial(
    STM32_PORT,
    STM32_BAUD,
    timeout=0.2
)

time.sleep(1)

# 清掉启动时可能残留的数据
stm32.reset_input_buffer()

print("STM32 connected")
print()
print("Available commands:")
print("  HELLO_STM32,1")
print("  FORWARD")
print("  BACKWARD")
print("  LEFT")
print("  RIGHT")
print("  STOP")
print("  LINE_START")
print("  LINE_STOP")
print()
print("Type q to quit.")
print()


def send_cmd(cmd):

    # 发送
    msg = cmd + "\n"
    stm32.write(msg.encode())

    print("TX:", cmd)

    # 给STM32一点处理时间
    time.sleep(0.05)

    # 读取当前所有返回
    while stm32.in_waiting:

        data = stm32.readline()

        if data:
            print(
                "RX:",
                data.decode(errors="replace").strip()
            )


try:

    while True:

        cmd = input("\nCMD> ").strip()

        if not cmd:
            continue

        if cmd.lower() in ["q", "quit", "exit"]:
            break

        send_cmd(cmd)


except KeyboardInterrupt:

    print("\nCtrl+C")


finally:

    print("Sending STOP...")

    try:
        stm32.write(b"STOP\n")
        time.sleep(0.1)
    except Exception:
        pass

    stm32.close()

    print("Serial closed")