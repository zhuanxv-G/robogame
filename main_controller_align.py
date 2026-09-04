import serial
import time


# ============================================================
# 串口配置
# ============================================================

# K230D
K230_PORT = "/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00"
K230_BAUD = 115200

# STM32
STM32_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STM32_BAUD = 115200


# ============================================================
# 视觉控制参数
# ============================================================

# ALIGN状态：
# 误差进入这个范围后，认为已经基本对准，可以开始前进
ALIGN_EXIT_ERROR = 30

# APPROACH状态：
# 前进过程中误差重新超过这个值，停止前进并重新对准
ALIGN_ENTER_ERROR = 70


# ============================================================
# 运动速度
#
# 注意：
# 这里只使用纯运动：
#
# ALIGN:
#   X = 0
#   Y = 0
#   Omega != 0
#
# APPROACH:
#   X = 0
#   Y > 0
#   Omega = 0
#
# 不再出现 X 和 Y 同时非零的斜向运动。
# ============================================================

# 原地转向速度 rad/s
ALIGN_OMEGA = 0.45

# 向前接近速度 m/s
APPROACH_SPEED = 0.20


# ============================================================
# 抓取距离
#
# 当前摄像头还没有最终固定，
# 所以这里只使用临时值。
#
# 后续摄像头固定后重新标定。
# ============================================================

GRAB_AREA_MIN = 20000

# 连续多少帧都满足抓取条件，
# 才真正进入 READY
GRAB_CONFIRM_COUNT = 3


# ============================================================
# pixels 平滑
# ============================================================

AREA_ALPHA = 0.20


# ============================================================
# 安全超时
# ============================================================

# 暂时丢失目标时允许保持上一动作一小会儿
TARGET_LOST_TIMEOUT = 0.30

# K230完全无有效数据
K230_TIMEOUT = 0.60


# ============================================================
# 状态打印频率
# ============================================================

STATUS_PRINT_INTERVAL = 0.25


# ============================================================
# 控制状态
# ============================================================

STATE_ALIGN = "ALIGN"
STATE_APPROACH = "APPROACH"
STATE_READY = "READY"
STATE_STOP = "STOP"


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


# 清理旧数据
k230.reset_input_buffer()
stm32.reset_input_buffer()


# ============================================================
# STM32通信
# ============================================================

def send_cmd(cmd):
    stm32.write((cmd + "\n").encode())


def send_vel(x, y, omega):
    """
    STM32当前VEL定义：

    X:
        + 向右横移
        - 向左横移

    Y:
        + 向前
        - 向后

    Omega:
        + 左转
        - 右转
    """

    cmd = (
        f"VEL,"
        f"{x:.2f},"
        f"{y:.2f},"
        f"{omega:.2f}"
    )

    send_cmd(cmd)

    return cmd


def stop_robot():

    send_cmd("STOP")

    print("CMD: STOP")


def drain_stm32_rx():
    """
    STM32会回复ACK,VEL等内容。

    平时不打印大量ACK，
    只打印ERR。
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
# K230解析
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
        #
        # TARGET,cx,cy,error_x,steering,pixels
        # ----------------------------

        if msg.startswith("TARGET,"):

            values = msg.split(",")

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
# 启动
# ============================================================

print()
print("========================================")
print("RoboGame Visual Align Controller")
print("========================================")
print()
print("Control strategy:")
print("  ALIGN    -> rotate only")
print("  APPROACH -> forward only")
print("  READY    -> STOP")
print()
print("No diagonal X+Y motion.")
print()
print("Start after 3 seconds...")


# 启动前先确保停车
try:
    stop_robot()
except Exception:
    pass


time.sleep(3)


# ============================================================
# 状态变量
# ============================================================

state = STATE_STOP

area_ema = None

grab_confirm_count = 0


now = time.monotonic()

last_k230_time = now
last_target_time = now

last_status_print_time = 0.0


# 是否已经因为超时停车
stopped = True


# ============================================================
# 主循环
# ============================================================

try:

    while True:

        data = None


        # ====================================================
        # 只取K230最新的一条数据
        # ====================================================

        while k230.in_waiting:

            data = k230.readline()


        # 清掉STM32 ACK
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
                # 看到了目标
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

                        area_ema = float(pixels)

                    else:

                        area_ema = (
                            AREA_ALPHA * pixels
                            +
                            (1.0 - AREA_ALPHA)
                            * area_ema
                        )


                    # =================================================
                    # 抓取条件
                    #
                    # 必须：
                    # 1. 足够近
                    # 2. 基本正对目标
                    # 3. 连续多帧成立
                    # =================================================

                    grab_candidate = (

                        area_ema >= GRAB_AREA_MIN

                        and

                        abs(error_x)
                        <= ALIGN_EXIT_ERROR
                    )


                    if grab_candidate:

                        grab_confirm_count += 1

                    else:

                        grab_confirm_count = 0


                    # =================================================
                    # READY优先级最高
                    # =================================================

                    if (
                        grab_confirm_count
                        >= GRAB_CONFIRM_COUNT
                    ):

                        if state != STATE_READY:

                            print()
                            print(
                                "=========================="
                            )
                            print(
                                "READY TO GRAB"
                            )
                            print(
                                "=========================="
                            )
                            print()


                        state = STATE_READY


                    # =================================================
                    # 非READY时执行状态切换
                    # =================================================

                    elif state != STATE_READY:


                        # ---------------------------------------------
                        # 初次看到目标 / STOP状态
                        # ---------------------------------------------

                        if state == STATE_STOP:

                            if (
                                abs(error_x)
                                <= ALIGN_EXIT_ERROR
                            ):

                                state = STATE_APPROACH

                            else:

                                state = STATE_ALIGN


                        # ---------------------------------------------
                        # ALIGN
                        #
                        # 必须真正进入±30以后才开始前进。
                        # ---------------------------------------------

                        elif state == STATE_ALIGN:

                            if (
                                abs(error_x)
                                <= ALIGN_EXIT_ERROR
                            ):

                                state = STATE_APPROACH


                        # ---------------------------------------------
                        # APPROACH
                        #
                        # 允许目标有一定偏差。
                        #
                        # 只有重新超过±70，
                        # 才停止前进，重新转向。
                        # ---------------------------------------------

                        elif state == STATE_APPROACH:

                            if (
                                abs(error_x)
                                >= ALIGN_ENTER_ERROR
                            ):

                                state = STATE_ALIGN


                    # =================================================
                    # 根据状态产生运动命令
                    # =================================================

                    if state == STATE_ALIGN:

                        # ---------------------------------------------
                        # 只原地旋转
                        # ---------------------------------------------

                        if error_x > 0:

                            # 目标在右边
                            # Omega < 0 -> 右转

                            cmd = send_vel(
                                0.0,
                                0.0,
                                -ALIGN_OMEGA
                            )

                            motion = "ROTATE_RIGHT"


                        else:

                            # 目标在左边
                            # Omega > 0 -> 左转

                            cmd = send_vel(
                                0.0,
                                0.0,
                                ALIGN_OMEGA
                            )

                            motion = "ROTATE_LEFT"


                    elif state == STATE_APPROACH:

                        # ---------------------------------------------
                        # 只向前
                        #
                        # X = 0
                        # Omega = 0
                        #
                        # 完全没有斜向运动。
                        # ---------------------------------------------

                        cmd = send_vel(
                            0.0,
                            APPROACH_SPEED,
                            0.0
                        )

                        motion = "FORWARD"


                    elif state == STATE_READY:

                        # ---------------------------------------------
                        # 已达到抓取位置
                        #
                        # 使用真正STOP，不继续发VEL。
                        # ---------------------------------------------

                        if not stopped:

                            stop_robot()

                            stopped = True

                        cmd = "STOP"

                        motion = "READY"


                    else:

                        if not stopped:

                            stop_robot()

                            stopped = True

                        cmd = "STOP"

                        motion = "STOP"


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
                                "state":
                                    state,

                                "motion":
                                    motion,

                                "error_x":
                                    error_x,

                                "pixels":
                                    pixels,

                                "area_ema":
                                    round(area_ema),

                                "grab_confirm":
                                    grab_confirm_count,

                                "cmd":
                                    cmd
                            }
                        )

                        last_status_print_time = now


                # =================================================
                # NO_TARGET
                # =================================================

                else:

                    # 不马上停车。
                    #
                    # 允许偶尔漏检。
                    # 真正停车在下面的timeout里处理。
                    pass


        # ========================================================
        # 安全超时
        # ========================================================

        now = time.monotonic()


        # ========================================================
        # 目标持续丢失
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

                state = STATE_STOP

                area_ema = None

                grab_confirm_count = 0


        # ========================================================
        # K230彻底失联
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

                state = STATE_STOP

                area_ema = None

                grab_confirm_count = 0


        # ========================================================
        # 主循环
        # ========================================================

        time.sleep(0.01)


# ============================================================
# Ctrl+C
# ============================================================

except KeyboardInterrupt:

    print()
    print("Ctrl+C -> STOP")


# ============================================================
# 安全退出
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