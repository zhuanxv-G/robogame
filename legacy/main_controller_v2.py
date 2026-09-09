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

        # 无目标
        if msg.startswith("NO_TARGET"):
            return {
                "target": False
            }

        # 有目标
        elif msg.startswith("TARGET,"):

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

    except Exception as e:
        print("K230 parse error:", e, "raw:", data)

    return None


# ==========================
# 主循环
# ==========================

# ==========================
# 视觉控制参数
# ==========================

# 从直行进入转向，需要超过这个值
TURN_ENTER = 50

# 已经在转向时，回到直行需要进入更小的范围
TURN_EXIT = 30

# 连续看到多少次 TARGET 才确认目标出现
TARGET_CONFIRM_COUNT = 2

# 短暂丢失目标时先保持原动作
TARGET_LOST_TIMEOUT = 0.3

# K230D 整体无任何有效数据，强制停车
K230_TIMEOUT = 0.6


def decide_motion(steering, current_cmd):

    # 当前正在右转
    if current_cmd == "RIGHT":

        # 目标突然跑到了很左边，允许直接切 LEFT
        if steering < -TURN_ENTER:
            return "LEFT"

        # 回到中心区域
        elif steering < TURN_EXIT:
            return "FORWARD"

        else:
            return "RIGHT"

    # 当前正在左转
    elif current_cmd == "LEFT":

        # 目标突然跑到了很右边
        if steering > TURN_ENTER:
            return "RIGHT"

        # 回到中心区域
        elif steering > -TURN_EXIT:
            return "FORWARD"

        else:
            return "LEFT"

    # 当前是 FORWARD 或 STOP
    else:

        if steering > TURN_ENTER:
            return "RIGHT"

        elif steering < -TURN_ENTER:
            return "LEFT"

        else:
            return "FORWARD"

print("Controller start")
print("Start after 3 seconds...")
time.sleep(3)

current_cmd = "STOP"

# 最近一次收到有效K230数据的时间
last_k230_time = time.time()

# 最近一次真正看到TARGET的时间
last_target_time = 0

# 连续TARGET计数
target_confirm_count = 0

# 当前是否已经确认存在目标
target_confirmed = False


try:

    while True:

        data = None

        # 只处理最新的一条视觉数据
        while k230.in_waiting:
            data = k230.readline()

        if data:

            result = parse_k230(data)

            if result:

                now = time.time()

                # K230正常工作
                last_k230_time = now

                print(result)

                # ==========================
                # 检测到目标
                # ==========================

                if result["target"]:

                    last_target_time = now

                    # 尚未确认目标
                    if not target_confirmed:

                        target_confirm_count += 1

                        print(
                            "TARGET confirm:",
                            target_confirm_count,
                            "/",
                            TARGET_CONFIRM_COUNT
                        )

                        # 连续看到足够次数才允许启动
                        if target_confirm_count >= TARGET_CONFIRM_COUNT:
                            target_confirmed = True

                    # 已经确认有目标
                    if target_confirmed:

                        steering = result["steering"]

                        current_cmd = decide_motion(
                            steering,
                            current_cmd
                        )

                # ==========================
                # 暂时没有目标
                # ==========================

                else:

                    # 出现NO_TARGET后，
                    # 不马上把确认计数清零，
                    # 等真正超时后再停车
                    pass

        now = time.time()

        # ==========================
        # 目标持续丢失
        # ==========================

        if target_confirmed:

            if now - last_target_time > TARGET_LOST_TIMEOUT:

                print("Target lost -> STOP")

                current_cmd = "STOP"

                target_confirmed = False
                target_confirm_count = 0

        # ==========================
        # K230D完全没有有效数据
        # ==========================

        if now - last_k230_time > K230_TIMEOUT:

            if current_cmd != "STOP":
                print("K230 timeout -> STOP")

            current_cmd = "STOP"

            target_confirmed = False
            target_confirm_count = 0

        # STM32约5Hz持续刷新
        send_cmd(current_cmd)

        time.sleep(0.2)


except KeyboardInterrupt:

    print("\nCtrl+C -> STOP")

    send_cmd("STOP")


finally:

    try:
        send_cmd("STOP")
    except Exception:
        pass

    k230.close()
    stm32.close()

    print("Controller stopped")