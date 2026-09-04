import serial
import time


# =========================
# 串口配置
# =========================

# K230D
VISION_PORT = "/dev/ttyACM1"

# STM32
STM32_PORT = "/dev/ttyUSB0"

BAUDRATE = 115200


# =========================
# 视觉参数
# =========================

# K230D 图像中心大约为 x = 320
IMAGE_CENTER_X = 320

# 死区，避免目标在中心附近轻微抖动时
# LEFT / RIGHT / CENTER 疯狂切换
DEAD_ZONE = 30


def parse_vision_message(message):
    """
    K230D 当前数据格式：

    NO_TARGET

    或：

    TARGET,390,31,70,10,10341

    目前我们只使用：
    cx      = 第2项
    cy      = 第3项
    error_x = 第4项
    """

    if message == "NO_TARGET":
        return {
            "detected": False,
            "state": "NO_TARGET"
        }

    if not message.startswith("TARGET,"):
        return None

    parts = message.split(",")

    if len(parts) < 4:
        return None

    try:
        cx = int(parts[1])
        cy = int(parts[2])
        error_x = int(parts[3])

    except ValueError:
        return None

    # 根据水平偏差判断方向
    if error_x < -DEAD_ZONE:
        direction = "LEFT"

    elif error_x > DEAD_ZONE:
        direction = "RIGHT"

    else:
        direction = "CENTER"

    return {
        "detected": True,
        "cx": cx,
        "cy": cy,
        "error_x": error_x,
        "state": direction
    }


def main():

    print("=" * 60)
    print("K230D -> Raspberry Pi -> STM32 Vision Test")
    print("=" * 60)

    # 打开 K230D
    try:
        vision_serial = serial.Serial(
            VISION_PORT,
            BAUDRATE,
            timeout=0.2
        )

        print(f"[OK] K230D opened: {VISION_PORT}")

    except Exception as e:

        print(f"[ERROR] Cannot open K230D port: {e}")
        return


    # 打开 STM32
    try:
        stm32_serial = serial.Serial(
            STM32_PORT,
            BAUDRATE,
            timeout=0.2
        )

        print(f"[OK] STM32 opened: {STM32_PORT}")

    except Exception as e:

        print(f"[ERROR] Cannot open STM32 port: {e}")

        vision_serial.close()
        return


    print()
    print("Waiting for vision data...")
    print("Press Ctrl+C to stop.")
    print()


    last_state = None


    try:

        while True:

            # =========================
            # 读取 K230D
            # =========================

            raw = vision_serial.readline()

            if not raw:
                continue

            message = raw.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not message:
                continue


            # =========================
            # 解析视觉信息
            # =========================

            result = parse_vision_message(message)

            if result is None:
                continue


            # =========================
            # 没检测到目标
            # =========================

            if not result["detected"]:

                print("[VISION] NO_TARGET")

                state = "NO_TARGET"

            else:

                state = result["state"]

                print(
                    f"[VISION] "
                    f"cx={result['cx']:3d} "
                    f"cy={result['cy']:3d} "
                    f"error_x={result['error_x']:4d} "
                    f"=> {state}"
                )


            # =========================
            # 状态变化时才发送 STM32
            # =========================

            if state != last_state:

                stm32_message = f"VISION_{state}\n"

                stm32_serial.write(
                    stm32_message.encode("utf-8")
                )

                stm32_serial.flush()

                print(
                    f"[TX -> STM32] "
                    f"{stm32_message.strip()}"
                )

                last_state = state


            # =========================
            # 看看 STM32 有没有回复
            # =========================

            while stm32_serial.in_waiting > 0:

                response = stm32_serial.readline()

                response_text = response.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if response_text:

                    print(
                        f"[RX <- STM32] "
                        f"{response_text}"
                    )


    except KeyboardInterrupt:

        print()
        print("Stopping...")


    finally:

        vision_serial.close()
        stm32_serial.close()

        print("Serial ports closed.")


if __name__ == "__main__":
    main()
