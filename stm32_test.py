import serial
import time

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200

stm = serial.Serial(
    PORT,
    BAUDRATE,
    timeout=0.2
)

time.sleep(1)

print(f"STM32 connected: {PORT} @ {BAUDRATE}")

while True:
    # 清除之前残留的数据
    stm.reset_input_buffer()

    # 发送握手
    data = b"HELLO_STM32,1\n"
    stm.write(data)
    stm.flush()

    print(f"[TX] {data!r}")

    # 等待 STM32 回复
    time.sleep(0.1)

    if stm.in_waiting:
        reply = stm.read(stm.in_waiting)
        print(f"[RX] {reply!r}")
        print(f"[HEX] {reply.hex(' ')}")
    else:
        print("[RX] no data")

    time.sleep(1)
