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

# 目标短暂丢失时允许保持原动作
TARGET_LOST_TIMEOUT = 0.3

# K230完全无有效数据
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
# 发送STM32命令
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

    except Exception as e:

        print("K230 parse error:", e, "raw:", data)

    return None


# ==========================
# 主循环
# ==========================

print("Controller start")
print("Start after 3 seconds...")

time.sleep(3)


current_cmd = "STOP"

last_k230_time = time.time()
last_target_time = time.time()


try:

    while True:

        data = None

        # 读取最新的一条K230数据
        while k230.in_waiting:
            data = k230.readline()

        if data:

            result = parse_k230(data)

            if result:

                now = time.time()

                # 说明K230还活着
                last_k230_time = now

                print(result)

                # ==========================
                # 有目标
                # ==========================

                if result["target"]:

                    last_target_time = now

                    steering = result["steering"]

                    # 恢复旧版已经验证顺滑的控制逻辑
                    if steering > STEERING_THRESHOLD:

                        current_cmd = "RIGHT"

                    elif steering < -STEERING_THRESHOLD:

                        current_cmd = "LEFT"

                    else:

                        current_cmd = "FORWARD"

                # ==========================
                # 没目标
                # ==========================

                else:

                    # 不立即STOP
                    # 让下面的超时逻辑判断
                    pass


        now = time.time()


        # ==========================
        # 目标持续丢失
        # ==========================

        if now - last_target_time > TARGET_LOST_TIMEOUT:

            current_cmd = "STOP"


        # ==========================
        # K230完全失联
        # ==========================

        if now - last_k230_time > K230_TIMEOUT:

            current_cmd = "STOP"


        # ==========================
        # 持续刷新STM32
        # ==========================

        send_cmd(current_cmd)

        time.sleep(0.2)


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