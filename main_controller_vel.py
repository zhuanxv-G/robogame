import serial
import time


# ============================================================
# 串口配置
# ============================================================

K230_PORT = "/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00"
K230_BAUD = 115200

STM32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STM32_BAUD = 115200


# ============================================================
# 图像参数
# ============================================================

IMAGE_CENTER_X = 320


# ============================================================
# 横向控制参数
# ============================================================

# 真正认为已经对准的中心死区
X_DEAD_ZONE = 30

# 偏差超过80像素：
# 停止前进，重新进入“横向对准”模式
ALIGN_ENTER_ERROR = 80

# 已经处于横向对准模式时：
# 必须回到45像素以内，才重新开始向前
ALIGN_EXIT_ERROR = 45


# 横向对准阶段最大横移速度
ALIGN_MAX_X_SPEED = 0.12

# 前进阶段只允许很小的横向微调
APPROACH_MAX_X_SPEED = 0.05


# ============================================================
# 前进速度参数
# ============================================================

MAX_Y_SPEED = 0.10

# pixels小于这个值时，认为目标还比较远，
# 使用最大前进速度
AREA_FULL_SPEED = 8000

# 到这里以后前进速度降到0
# 当前只是临时值，摄像头固定后重新标定
APPROACH_STOP_AREA = 20000


# ============================================================
# 临时抓取区域
# ============================================================

GRAB_AREA_MIN = 20000
GRAB_AREA_TARGET = 25000
GRAB_AREA_MAX = 30000


# ============================================================
# 平滑参数
# ============================================================

# pixels EMA
AREA_ALPHA = 0.20

# 最终VEL速度EMA
#
# 越小越平滑，但响应越慢
# 建议先用0.35
VEL_ALPHA = 0.35


# ============================================================
# VEL发送参数
# ============================================================

# 最多约10Hz发送新的VEL
VEL_SEND_INTERVAL = 0.10

# 速度量化
#
# 比如：
# 0.036 -> 0.04
# 0.003 -> 0.00
#
# 避免每一帧都因为0.001级别变化修改轮速
VEL_STEP = 0.01


# ============================================================
# 安全超时
# ============================================================

TARGET_LOST_TIMEOUT = 0.30
K230_TIMEOUT = 0.60


# ============================================================
# 状态打印频率
# ============================================================

STATUS_PRINT_INTERVAL = 0.25


# ============================================================
# 打开串口
# ============================================================

print("Connecting K230D...")

k230 = serial.Serial(
    K230_PORT,
    K230_BAUD,
    timeout=0.2
)

print("K230D connected")


print("Connecting STM32...")

stm32 = serial.Serial(
    STM32_PORT,
    STM32_BAUD,
    timeout=0.2
)

print("STM32 connected")


k230.reset_input_buffer()
stm32.reset_input_buffer()


# ============================================================
# 工具函数
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def quantize(value, step):
    """
    将速度量化。

    例如step=0.01：

    0.034 -> 0.03
    0.036 -> 0.04
    0.003 -> 0.00
    """
    return round(value / step) * step


# ============================================================
# STM32通信
# ============================================================

def send_cmd(cmd):
    stm32.write((cmd + "\n").encode())
    print("CMD:", cmd)


def send_vel(x, y, omega=0.0):

    # 最终安全限幅
    x = clamp(x, -0.30, 0.30)
    y = clamp(y, -0.30, 0.30)
    omega = clamp(omega, -1.50, 1.50)

    # 量化，过滤非常小的视觉速度变化
    x = quantize(x, VEL_STEP)
    y = quantize(y, VEL_STEP)
    omega = quantize(omega, 0.01)

    cmd = f"VEL,{x:.2f},{y:.2f},{omega:.2f}"

    stm32.write((cmd + "\n").encode())

    return x, y, omega


def stop_robot():

    stm32.write(b"STOP\n")

    print("CMD: STOP")


def drain_stm32_rx():
    """
    STM32会返回ACK,VEL等信息。

    控制程序不需要逐条打印ACK，
    但应该定期清掉接收缓冲区。

    如果出现ERR则打印出来。
    """

    while stm32.in_waiting:

        data = stm32.readline()

        if not data:
            continue

        msg = data.decode(
            errors="ignore"
        ).strip()

        if msg.startswith("ERR"):
            print("STM32:", msg)


# ============================================================
# K230数据解析
# ============================================================

def parse_k230(data):

    try:

        msg = data.decode(
            errors="ignore"
        ).strip()

        msg = msg.replace(
            "[UART TX] ",
            ""
        )


        # ----------------------------
        # 无目标
        # ----------------------------

        if msg.startswith("NO_TARGET"):

            return {
                "target": False
            }


        # ----------------------------
        # 有目标
        # ----------------------------

        if msg.startswith("TARGET,"):

            values = msg.split(",")

            # TARGET,cx,cy,error_x,steering,pixels
            if len(values) < 6:

                print(
                    "Invalid TARGET:",
                    msg
                )

                return None


            return {
                "target": True,
                "cx": int(values[1]),
                "cy": int(values[2]),
                "error_x": int(values[3]),
                "steering": int(values[4]),
                "pixels": int(values[5])
            }


    except Exception as e:

        print(
            "K230 parse error:",
            e,
            "raw:",
            data
        )


    return None


# ============================================================
# 横向速度计算
# ============================================================

def calculate_x_speed(
    error_x,
    max_speed,
    dead_zone
):
    """
    error_x > 0
        → 目标在右边
        → X > 0
        → 向右横移

    error_x < 0
        → 向左横移
    """


    abs_error = abs(error_x)


    # 中心死区
    if abs_error <= dead_zone:

        return 0.0


    # --------------------------------------------
    # 把死区以外的部分重新映射
    #
    # 这样刚离开死区时速度从0开始，
    # 不会突然产生较大的横移速度。
    # --------------------------------------------

    effective_error = (
        abs_error - dead_zone
    )

    usable_range = (
        IMAGE_CENTER_X - dead_zone
    )


    ratio = (
        effective_error
        / usable_range
    )

    ratio = clamp(
        ratio,
        0.0,
        1.0
    )


    speed = (
        max_speed
        * ratio
    )


    if error_x < 0:

        speed = -speed


    return speed


# ============================================================
# 前进速度计算
# ============================================================

def calculate_y_speed(area):
    """
    根据pixels连续控制前进速度。

    与之前不同：

    不再存在：
        19999 -> 0.04
        20000 -> 0

    而是连续减速。
    """


    # --------------------------------------------
    # 很远：
    # 最大前进速度
    # --------------------------------------------

    if area <= AREA_FULL_SPEED:

        return MAX_Y_SPEED


    # --------------------------------------------
    # 已经到目标距离：
    # 停止向前
    # --------------------------------------------

    if area >= APPROACH_STOP_AREA:

        return 0.0


    # --------------------------------------------
    # 中间区域：
    # 连续线性减速
    # --------------------------------------------

    ratio = (
        APPROACH_STOP_AREA - area
    ) / (
        APPROACH_STOP_AREA
        - AREA_FULL_SPEED
    )


    ratio = clamp(
        ratio,
        0.0,
        1.0
    )


    return (
        MAX_Y_SPEED
        * ratio
    )


# ============================================================
# 主程序
# ============================================================

print()
print("======================================")
print("Visual VEL Controller V2")
print("Align first -> Approach second")
print("======================================")
print()

print("Start after 3 seconds...")

time.sleep(3)


# ============================================================
# 状态变量
# ============================================================

now = time.monotonic()

last_k230_time = now
last_target_time = now

last_vel_send_time = 0.0
last_status_print_time = 0.0


# pixels EMA
area_ema = None


# VEL EMA
x_filtered = 0.0
y_filtered = 0.0


# 最开始先进行横向对准
aligning = True


# STOP状态
stopped = True


# READY状态
last_ready_state = False


# ============================================================
# 主循环
# ============================================================

try:

    while True:

        data = None


        # ====================================================
        # 只处理K230最新视觉数据
        # ====================================================

        while k230.in_waiting:

            data = k230.readline()


        # 清理STM32 ACK
        drain_stm32_rx()


        # ====================================================
        # 收到K230数据
        # ====================================================

        if data:

            result = parse_k230(data)


            if result:

                now = time.monotonic()

                last_k230_time = now


                # =================================================
                # TARGET
                # =================================================

                if result["target"]:

                    last_target_time = now
                    stopped = False


                    error_x = result["error_x"]
                    pixels = result["pixels"]


                    # =================================================
                    # pixels EMA
                    # =================================================

                    if area_ema is None:

                        area_ema = float(
                            pixels
                        )

                    else:

                        area_ema = (
                            AREA_ALPHA
                            * pixels

                            +

                            (1.0 - AREA_ALPHA)
                            * area_ema
                        )


                    # =================================================
                    # 对准状态机
                    # =================================================

                    previous_aligning = aligning


                    # -----------------------------
                    # 当前正在横向对准
                    # -----------------------------

                    if aligning:

                        # 必须真正回到45以内
                        # 才重新前进
                        if (
                            abs(error_x)
                            <= ALIGN_EXIT_ERROR
                        ):

                            aligning = False


                    # -----------------------------
                    # 当前正在向前接近
                    # -----------------------------

                    else:

                        # 偏差重新大于80
                        # 才停止前进重新对准
                        if (
                            abs(error_x)
                            >= ALIGN_ENTER_ERROR
                        ):

                            aligning = True


                    # =================================================
                    # 如果刚刚重新进入ALIGN模式
                    #
                    # 立即把前进滤波速度清零。
                    #
                    # 这样目标已经明显偏出去以后，
                    # 不会因为EMA残留继续往前冲。
                    # =================================================

                    if (
                        aligning
                        and
                        not previous_aligning
                    ):

                        y_filtered = 0.0


                    # =================================================
                    # 计算目标速度
                    # =================================================

                    if aligning:

                        # ---------------------------------------------
                        # ALIGN模式：
                        #
                        # 只横移，不向前。
                        # ---------------------------------------------

                        phase = "ALIGN"

                        x_target = calculate_x_speed(
                            error_x,
                            ALIGN_MAX_X_SPEED,
                            X_DEAD_ZONE
                        )

                        y_target = 0.0


                    else:

                        # ---------------------------------------------
                        # APPROACH模式：
                        #
                        # 主要向前，
                        # 只允许很小的横向修正。
                        # ---------------------------------------------

                        phase = "APPROACH"

                        x_target = calculate_x_speed(
                            error_x,
                            APPROACH_MAX_X_SPEED,
                            X_DEAD_ZONE
                        )

                        y_target = calculate_y_speed(
                            area_ema
                        )


                    # 第一版依然不转动车头
                    omega = 0.0


                    # =================================================
                    # READY TO GRAB
                    # =================================================

                    ready_to_grab = (

                        GRAB_AREA_MIN
                        <= area_ema
                        <= GRAB_AREA_MAX

                        and

                        abs(error_x)
                        <= X_DEAD_ZONE
                    )


                    # =================================================
                    # 已经进入抓取位置
                    #
                    # 不使用EMA慢慢减速，
                    # 而是直接归零。
                    # =================================================

                    if ready_to_grab:

                        phase = "READY"

                        x_target = 0.0
                        y_target = 0.0

                        x_filtered = 0.0
                        y_filtered = 0.0


                    # =================================================
                    # 非READY：
                    # 对VEL进行EMA低通
                    # =================================================

                    else:

                        x_filtered = (
                            VEL_ALPHA
                            * x_target

                            +

                            (1.0 - VEL_ALPHA)
                            * x_filtered
                        )


                        y_filtered = (
                            VEL_ALPHA
                            * y_target

                            +

                            (1.0 - VEL_ALPHA)
                            * y_filtered
                        )


                    # =================================================
                    # 状态打印
                    # =================================================

                    if (
                        now
                        - last_status_print_time
                        >= STATUS_PRINT_INTERVAL
                    ):

                        print(
                            {
                                "phase": phase,
                                "error_x": error_x,
                                "pixels": pixels,
                                "area_ema": round(area_ema),

                                "X_target":
                                    round(x_target, 3),

                                "Y_target":
                                    round(y_target, 3),

                                "X_filtered":
                                    round(x_filtered, 3),

                                "Y_filtered":
                                    round(y_filtered, 3),

                                "ready":
                                    ready_to_grab
                            }
                        )

                        last_status_print_time = now


                    # =================================================
                    # READY提示
                    # =================================================

                    if (
                        ready_to_grab
                        and
                        not last_ready_state
                    ):

                        print()
                        print(
                            "======================"
                        )
                        print(
                            "READY TO GRAB"
                        )
                        print(
                            "======================"
                        )
                        print()


                    last_ready_state = (
                        ready_to_grab
                    )


                    # =================================================
                    # 发送VEL
                    #
                    # 最多10Hz。
                    # =================================================

                    if ready_to_grab:

                        # READY时立即发送0速度
                        # 不等待发送周期

                        send_vel(
                            0.0,
                            0.0,
                            0.0
                        )

                        last_vel_send_time = now


                    elif (
                        now
                        - last_vel_send_time
                        >= VEL_SEND_INTERVAL
                    ):

                        sent_x, sent_y, _ = (
                            send_vel(
                                x_filtered,
                                y_filtered,
                                omega
                            )
                        )


                        last_vel_send_time = now


                # =================================================
                # NO_TARGET
                # =================================================

                else:

                    # 暂时不立即停车。
                    #
                    # 下面使用0.3秒超时处理。
                    pass


        # ========================================================
        # 安全超时
        # ========================================================

        now = time.monotonic()


        # ========================================================
        # TARGET持续丢失
        # ========================================================

        if (
            now - last_target_time
            > TARGET_LOST_TIMEOUT
        ):

            if not stopped:

                print()
                print(
                    "Target lost -> STOP"
                )

                stop_robot()


                stopped = True

                area_ema = None

                x_filtered = 0.0
                y_filtered = 0.0

                aligning = True

                last_ready_state = False


        # ========================================================
        # K230彻底无数据
        # ========================================================

        if (
            now - last_k230_time
            > K230_TIMEOUT
        ):

            if not stopped:

                print()
                print(
                    "K230 timeout -> STOP"
                )

                stop_robot()


                stopped = True

                area_ema = None

                x_filtered = 0.0
                y_filtered = 0.0

                aligning = True

                last_ready_state = False


        # ========================================================
        # 主循环快速检查
        # ========================================================

        time.sleep(0.01)


# ============================================================
# Ctrl+C
# ============================================================

except KeyboardInterrupt:

    print()
    print("Ctrl+C -> STOP")


# ============================================================
# 最终安全退出
# ============================================================

finally:

    try:

        stop_robot()

        time.sleep(0.1)

    except Exception:

        pass


    try:

        k230.close()

    except Exception:

        pass


    try:

        stm32.close()

    except Exception:

        pass


    print("Controller stopped")