import serial
import time


# ==========================
# 串口配置
# ==========================

K230_PORT = "/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00"
K230_BAUD = 115200

STM32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STM32_BAUD = 115200


# ==========================
# 控制参数
# ==========================

STEERING_THRESHOLD = 40

# 允许目标短暂丢失
TARGET_LOST_TIMEOUT = 0.3

# K230完全无数据
K230_TIMEOUT = 0.6


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
# 发送命令
# ==========================

def send_cmd(cmd):
    stm32.write((cmd + "\n").encode())
    print("CMD:", cmd)


# ==========================
# 解析K230数据
# ==========================

def parse_k230(data):

    try:

        msg = data.decode().strip()
        msg = msg.replace("[UART TX] ", "")

        if msg.startswith("NO_TARGET"):

            return {
                "target": False
            }

        elif msg.startswith("TARGET,"):

            value = msg.split(",")

            if len(value) < 5:
                return None

            return {
                "target": True,
                "cx": int(value[1]),
                "cy": int(value[2]),
                "error_x": int(value[3]),
                "steering": int(value[4])
            }

    except Exception as e:

        print("K230 parse error:", e, "raw:", data)

    return None


# ==========================
# 主循环
# ==========================

print("Controller start")
print("Start after 3 seconds...")

time.sleep(3)


last_k230_time = time.time()
last_target_time = time.time()

current_cmd = "STOP"

# 防止超时时不停重复发STOP
stopped_by_timeout = False


try:

    while True:

        data = None

        # 只取最新视觉数据
        while k230.in_waiting:
            data = k230.readline()

        if data:

            result = parse_k230(data)

            if result:

                now = time.time()

                last_k230_time = now

                print(result)


                # ==========================
                # 看见目标
                # ==========================

                if result["target"]:

                    last_target_time = now
                    stopped_by_timeout = False

                    steering = result["steering"]

                    if steering > STEERING_THRESHOLD:

                        cmd = "RIGHT"

                    elif steering < -STEERING_THRESHOLD:

                        cmd = "LEFT"

                    else:

                        cmd = "FORWARD"


                    current_cmd = cmd

                    # 只有收到新的视觉结果时才发送
                    send_cmd(cmd)


                # ==========================
                # NO_TARGET
                # ==========================

                else:

                    # 此处不立即发送STOP
                    # 等下面0.3秒超时判断
                    pass


        now = time.time()


        # ==========================
        # 目标持续消失
        # ==========================

        if (
            now - last_target_time > TARGET_LOST_TIMEOUT
            and current_cmd != "STOP"
        ):

            print("Target lost -> STOP")

            send_cmd("STOP")

            current_cmd = "STOP"
            stopped_by_timeout = True


        # ==========================
        # K230完全失联
        # ==========================

        if (
            now - last_k230_time > K230_TIMEOUT
            and current_cmd != "STOP"
        ):

            print("K230 timeout -> STOP")

            send_cmd("STOP")

            current_cmd = "STOP"
            stopped_by_timeout = True


        # 这里只控制主循环速度
        # 不再持续发送current_cmd
        time.sleep(0.02)


except KeyboardInterrupt:

    print("\nCtrl+C -> STOP")


finally:

    try:
        send_cmd("STOP")
    except Exception:
        pass

    k230.close()
    stm32.close()

    print("Controller stopped")