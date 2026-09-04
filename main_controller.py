import serial
import time


# ==========================
# 串口配置
# ==========================

# K230D
K230_PORT = "/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00"
K230_BAUD = 115200

# STM32
STM32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STM32_BAUD = 115200


# ==========================
# 打开串口
# ==========================

print("Connecting K230D...")

k230 = serial.Serial(
    K230_PORT,
    K230_BAUD,
    timeout=1
)

print("K230D connected")


print("Connecting STM32...")

stm32 = serial.Serial(
    STM32_PORT,
    STM32_BAUD,
    timeout=1
)

print("STM32 connected")


# ==========================
# 发送给STM32
# ==========================

def send_cmd(cmd):
    msg = cmd + "\n"
    stm32.write(msg.encode())
    print("CMD:", cmd)

# ==========================
# 解析K230数据
# ==========================

def parse_k230(data):

    try:
        msg = data.decode().strip()
        msg = msg.replace("[UART TX] ", "")
        # 示例:
        # TARGET,390,31,70,10,10341

        if "TARGET" in msg:

            value = msg.split(",")

            cx = int(value[1])
            cy = int(value[2])
            error_x = int(value[3])
            steering = int(value[4])

            return {
                "target": True,
                "cx": cx,
                "cy": cy,
                "error_x": error_x,
                "steering": steering
            }


        elif msg.startswith("NO_TARGET"):

            return {
                "target": False
            }


    except Exception:
        pass


    return None



# ==========================
# 主循环
# ==========================

print("Controller start")

print("Start after 3 seconds...")
time.sleep(3)

while True:

    data = None

    while k230.in_waiting:
        data = k230.readline()

    if data:

        result = parse_k230(data)


        if result:

            print(result)


            if result["target"]:

                steering = result["steering"]


                # 根据视觉方向控制

                if steering > 40:

                    cmd = "RIGHT"


                elif steering < -40:

                    cmd = "LEFT"


                else:

                    cmd = "FORWARD"


            else:

                cmd = "STOP"


            send_cmd(cmd)


    # STM32要求约5Hz刷新
    time.sleep(0.2)
