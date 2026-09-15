"""Manual experiment: wall approach + K230D-assisted grab positioning.

The robot must be placed at the grab area facing the wall.  The script first
replays the verified fixed-position flow of ``tests/test_old_grab_sequence.py``
(wall approach, backoff), then hands chassis positioning to the K230D:

    Servo0=0 + 旧步骤1-5 固定初始姿态 -> 撞墙 0.50 m -> 后撤 0.05 m 到视觉抓取位置
    -> SEARCH -> ALIGN -> READY (STOP ACK)
    -> 恢复 Servo0=95 -> PREPARE_ARM -> PICK -> 后撤 0.30 m -> PLACE -> 静止姿态

Before the wall approach the initial arm pose is fixed: Servo0 at 0 deg (the
base turned away from the K230D) plus the old steps 1-5 angles
(Servo1/2/4 = 0/0/135).  Once READY is confirmed, Servo0 returns to the
verified grab baseline (95 deg) and the previously used arm process
(PREPARE_ARM -> PICK -> PLACE) runs unchanged.

This experiment only checks whether K230D visual positioning can replace the
fixed grab position.  It does NOT touch the competition state machine,
``arm_sequence.py``, ``competition/task_controller.py``,
``hardware/stm32_link.py``, ``vision/k230_link.py`` or the STM32 protocol.

Serial ownership: one ``Stm32Link`` owns the STM32 CH340 and one ``K230Link``
owns the K230D USB serial.  Both are created through the existing links and
closed in ``finally``.  Importing this module has no hardware side effects.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Sequence
from enum import Enum
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    Stm32Link,
    Stm32ProtocolError,
)
from vision.k230_link import K230Link  # noqa: E402
from competition.grab_task import (  # noqa: E402
    GRAB_REST_SERVO_ANGLES,
    PICK_ARM_STEPS,
    PLACE_ARM_STEPS,
    PREPARE_ARM_STEPS,
    PRE_PICK_SETTLE_STEPS,
    PRE_PLACE_SETTLE_STEPS,
    REST_ARM_STEPS,
    execute_grab_steps,
)


# ============================================================
# 固定位置阶段：沿用 tests/test_old_grab_sequence.py 已验证距离
# ============================================================

WALL_APPROACH_DISTANCE = 0.50      # 旧步骤6：前进撞墙定位
VISUAL_BACKOFF_DISTANCE = 0.05     # 旧步骤7：后撤到视觉抓取位置
POST_PICK_RETREAT_DISTANCE = 0.30  # 旧步骤17：抓起后后撤

VISUAL_SERVO0_ANGLE = 0  # 视觉阶段：机械臂底座转离 K230D 视野
GRAB_SERVO0_ANGLE = GRAB_REST_SERVO_ANGLES[0]  # 抓取前恢复已验证抓取基线(95)


# ============================================================
# 底盘开环速度：命令值与实测速度基线分离（与旧测试相同）
# ============================================================

DEFAULT_COMMAND_SPEED_M_S = 0.80
MAX_COMMAND_SPEED_M_S = 0.80
DEFAULT_FORWARD_ACTUAL_SPEED_M_S = 0.118
DEFAULT_BACKWARD_ACTUAL_SPEED_M_S = 0.118
VELOCITY_REFRESH_S = 0.10  # REMOTE watchdog 为 1000 ms，必须持续刷新


# ============================================================
# K230D 视觉定位参数
# ============================================================

SEARCH_SPEED = 0.10        # SEARCH：向左横移命令速度 m/s（低速）
ALIGN_SPEED = 0.10         # ALIGN：左右横移命令速度 m/s（低速）
ALIGN_ERROR_PIXELS = 30    # 对准条件：abs(error_x) < 30 像素
READY_CONFIRM_FRAMES = 5   # 连续满足帧数才进入 READY
PIXELS_VALID_MIN = 1       # pixels 有效：必须 > 0，排除空检测
K230_TIMEOUT_S = 0.60      # K230D 无有效数据超时（与已验证主控一致）
DEFAULT_SEARCH_TIMEOUT_S = 10.0  # SEARCH 未发现目标超时
STATUS_PRINT_INTERVAL = 0.25


# ============================================================
# 串口与协议超时
# ============================================================

ACK_TIMEOUT_S = 2.0
VELOCITY_ACK_TIMEOUT_S = 0.5
HELLO_TIMEOUT_S = 2.0
SERIAL_POLL_TIMEOUT_S = 0.01
K230_READ_TIMEOUT_S = 0.2


class VisualPhase(str, Enum):
    """K230D-assisted positioning states."""

    SEARCH = "SEARCH"
    ALIGN = "ALIGN"
    READY = "READY"


# ============================================================
# 参数解析
# ============================================================

def _positive_speed(value: str) -> float:
    try:
        speed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("速度必须是数字") from exc
    if not math.isfinite(speed) or speed <= 0.0:
        raise argparse.ArgumentTypeError("速度必须是有限正数")
    return speed


def _command_speed(value: str) -> float:
    speed = _positive_speed(value)
    if speed > MAX_COMMAND_SPEED_M_S:
        raise argparse.ArgumentTypeError(
            f"命令速度必须在 (0, {MAX_COMMAND_SPEED_M_S:.2f}] 范围内"
        )
    return speed


def _positive_time(value: str) -> float:
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("时间必须是数字") from exc
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise argparse.ArgumentTypeError("时间必须是有限正数")
    return seconds


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "K230D 视觉辅助单次抓取实验：撞墙、后撤、视觉定位、"
            "抓取并放入中间盒。"
        )
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印流程和参数，不打开串口",
    )
    parser.add_argument(
        "--command-speed",
        type=_command_speed,
        default=DEFAULT_COMMAND_SPEED_M_S,
        metavar="M_S",
        help=(
            "VEL.Y 命令速度绝对值 "
            f"(default: %(default).2f, max: {MAX_COMMAND_SPEED_M_S:.2f})"
        ),
    )
    parser.add_argument(
        "--forward-actual-speed",
        type=_positive_speed,
        default=DEFAULT_FORWARD_ACTUAL_SPEED_M_S,
        metavar="M_S",
        help="前进实测速度，用于撞墙计时",
    )
    parser.add_argument(
        "--backward-actual-speed",
        type=_positive_speed,
        default=DEFAULT_BACKWARD_ACTUAL_SPEED_M_S,
        metavar="M_S",
        help="后退实测速度，用于后撤计时",
    )
    parser.add_argument(
        "--search-timeout",
        type=_positive_time,
        default=DEFAULT_SEARCH_TIMEOUT_S,
        metavar="S",
        help=(
            "SEARCH 未发现目标的安全超时 "
            f"(default: %(default).1f s)"
        ),
    )
    return parser.parse_args(argv)


# ============================================================
# STM32 辅助函数
# ============================================================

def _guarded_delay(
    stm32: Stm32Link,
    seconds: float,
    *,
    label: str,
    output: Callable[[str], object],
    clock: Callable[[], float],
) -> None:
    """Wait while watching the STM32 stream for asynchronous ERR lines."""
    deadline = clock() + seconds
    while clock() < deadline:
        raw_line = stm32.readline()
        if not raw_line:
            continue
        line = raw_line.decode(errors="replace").strip()
        if not line:
            continue
        output(f"[{label}] 收到异步消息: {line}")
        if line.startswith("ERR,"):
            raise Stm32ProtocolError(f"STM32 error: {line}")


def _drive_timed_distance(
    stm32: Stm32Link,
    velocity_y: float,
    duration_s: float,
    *,
    label: str,
    output: Callable[[str], object],
    clock: Callable[[], float],
) -> None:
    """Drive one VEL.Y for a timed distance, then STOP with ACK.

    Follows the verified pattern of tests/test_old_grab_sequence.py:
    refresh VEL every 100 ms for the REMOTE watchdog, then a confirmed STOP.
    """
    deadline = clock() + duration_s
    while clock() < deadline:
        refresh_due = min(deadline, clock() + VELOCITY_REFRESH_S)
        command = f"VEL,0.00,{velocity_y:.2f},0.00"
        output(f"[{label}] TX: {command}")
        stm32.send_command(command)
        ack = stm32.wait_ack("VEL", VELOCITY_ACK_TIMEOUT_S)
        output(f"[{label}] RX: {ack}")
        remaining_s = refresh_due - clock()
        if remaining_s > 0.0:
            _guarded_delay(
                stm32,
                min(VELOCITY_REFRESH_S, remaining_s),
                label=label,
                output=output,
                clock=clock,
            )
    output(f"[{label}] TX: STOP")
    stop_ack = stm32.stop(
        require_ack=True,
        timeout=ACK_TIMEOUT_S,
        debug_label=f"{label}_end",
    )
    output(f"[{label}] RX: {stop_ack}")


def _emergency_stop(
    stm32: Stm32Link | None,
    *,
    reason: str,
    output: Callable[[str], object],
) -> None:
    """Best-effort global STOP that never masks the original failure."""
    if stm32 is None or not stm32.is_connected:
        return
    output(f"危险保护 STOP ({reason})")
    try:
        stm32.stop(require_ack=False, timeout=ACK_TIMEOUT_S)
    except Exception as exc:
        output(f"危险保护 STOP 发送失败: {type(exc).__name__}: {exc}")


# ============================================================
# K230D 视觉定位阶段：SEARCH -> ALIGN -> READY
# ============================================================

def run_visual_phase(
    stm32: Stm32Link,
    k230: K230Link,
    *,
    search_timeout_s: float,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Run SEARCH -> ALIGN -> READY and finish with a confirmed STOP.

    SEARCH strafes slowly to the left until a TARGET appears; ALIGN strafes
    left/right on error_x; READY needs abs(error_x)<30 and pixels>0 for five
    consecutive frames, then sends STOP and waits for ACK,STOP before the
    grab phase.  Raises on STM32 ERR lines, K230D silence or search timeout;
    the caller's safety wrapper performs the emergency STOP.
    """
    phase = VisualPhase.SEARCH
    ready_frames = 0
    last_target: bool | None = None
    last_error_x = 0
    last_pixels = 0
    last_motion_x: float | None = None
    last_vel_sent_at = 0.0
    last_status_print_at = 0.0
    last_k230_data_at = clock()
    search_started_at = last_k230_data_at

    output("")
    output("=== VISUAL POSITIONING (SEARCH -> ALIGN -> READY) ===")
    output("SEARCH: 缓慢向左横移，扩大摄像头搜索范围")

    while True:
        now = clock()

        # --- 1. 观察 STM32 异步消息；ERR 立即失败 ---
        raw_line = stm32.readline()
        if raw_line:
            line = raw_line.decode(errors="replace").strip()
            if line:
                output(f"STM32 async RX: {line}")
                if line.startswith("ERR,"):
                    raise Stm32ProtocolError(f"STM32 error: {line}")

        # --- 2. 读取 K230D 最新一条消息 ---
        result = k230.read_latest()
        if result is not None:
            last_k230_data_at = now
            last_target = result["target"]
            if result["target"]:
                last_error_x = result["error_x"]
                last_pixels = result["pixels"]
                if phase is VisualPhase.SEARCH:
                    phase = VisualPhase.ALIGN
                    ready_frames = 0
                    output("TARGET 出现 -> ALIGN")
                aligned = (
                    abs(last_error_x) < ALIGN_ERROR_PIXELS
                    and last_pixels > PIXELS_VALID_MIN
                )
                ready_frames = ready_frames + 1 if aligned else 0
                if ready_frames >= READY_CONFIRM_FRAMES:
                    phase = VisualPhase.READY
                    output("")
                    output("==========================")
                    output("READY TO GRAB")
                    output("==========================")
                    output("")
            elif phase is not VisualPhase.SEARCH:
                # 目标丢失：回到 SEARCH 重新向左搜索
                phase = VisualPhase.SEARCH
                ready_frames = 0
                search_started_at = now
                output("NO_TARGET -> SEARCH")

        # --- 3. 安全超时 ---
        if now - last_k230_data_at > K230_TIMEOUT_S:
            raise TimeoutError("K230D 无有效数据超时，进入安全停车")
        if (
            phase is VisualPhase.SEARCH
            and now - search_started_at > search_timeout_s
        ):
            raise TimeoutError(
                f"SEARCH 超过 {search_timeout_s:.1f} s 未发现目标"
            )

        # --- 4. READY：发送 STOP 并等待 ACK，之后才能进入抓取阶段 ---
        if phase is VisualPhase.READY:
            output("TX: STOP")
            stop_ack = stm32.stop(
                require_ack=True,
                timeout=ACK_TIMEOUT_S,
                debug_label="visual_ready",
            )
            output(f"RX: {stop_ack}")
            output("视觉定位完成：已停车，可以进入抓取阶段。")
            return

        # --- 5. 按状态刷新 VEL（100 ms 一次，满足 REMOTE watchdog）---
        if now - last_vel_sent_at >= VELOCITY_REFRESH_S:
            if phase is VisualPhase.SEARCH:
                motion_x = -SEARCH_SPEED  # 缓慢向左横移
            elif abs(last_error_x) < ALIGN_ERROR_PIXELS:
                motion_x = 0.0  # 已对准：保持静止等待连续帧确认
            else:
                # 目标偏右(error_x>0)向右移，目标偏左(error_x<0)向左移
                motion_x = (
                    ALIGN_SPEED if last_error_x > 0 else -ALIGN_SPEED
                )
            if motion_x != last_motion_x:
                output(f"TX: VEL,{motion_x:.2f},0.00,0.00")
                last_motion_x = motion_x
            stm32.send_command(f"VEL,{motion_x:.2f},0.00,0.00")
            stm32.wait_ack("VEL", VELOCITY_ACK_TIMEOUT_S)
            last_vel_sent_at = now

        # --- 6. 状态打印 ---
        if now - last_status_print_at >= STATUS_PRINT_INTERVAL:
            output(
                f"state={phase.value} target={last_target} "
                f"error_x={last_error_x} pixels={last_pixels} "
                f"ready_frames={ready_frames}"
            )
            last_status_print_at = now


# ============================================================
# 主流程
# ============================================================

def _default_stm32_factory() -> Stm32Link:
    return Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)


def _default_k230_factory() -> K230Link:
    return K230Link(timeout=K230_READ_TIMEOUT_S)


def _print_plan(
    output: Callable[[str], object],
    args: argparse.Namespace,
) -> None:
    wall_time = WALL_APPROACH_DISTANCE / args.forward_actual_speed
    backoff_time = VISUAL_BACKOFF_DISTANCE / args.backward_actual_speed
    retreat_time = POST_PICK_RETREAT_DISTANCE / args.backward_actual_speed
    output("K230D 视觉辅助单次抓取实验")
    output("验证目标: K230D 视觉定位能否替代固定抓取位置")
    output(f"command_speed={args.command_speed:.2f} m/s")
    output(f"forward_actual_speed={args.forward_actual_speed:.3f} m/s")
    output(
        f"backward_actual_speed={args.backward_actual_speed:.3f} m/s "
        "(暂沿用前进基线，待单独标定)"
    )
    output(
        f"wall_approach={WALL_APPROACH_DISTANCE:.2f} m "
        f"({wall_time:.2f} s)"
    )
    output(
        f"visual_backoff={VISUAL_BACKOFF_DISTANCE:.2f} m "
        f"({backoff_time:.2f} s)"
    )
    output(f"search_speed={SEARCH_SPEED:.2f} m/s 向左横移")
    output(f"align_speed={ALIGN_SPEED:.2f} m/s 左右横移")
    output(
        f"align_error<{ALIGN_ERROR_PIXELS} px, "
        f"ready_frames={READY_CONFIRM_FRAMES}"
    )
    output(f"search_timeout={args.search_timeout:.1f} s")
    output(
        f"visual_servo0_angle={VISUAL_SERVO0_ANGLE} deg "
        "（视觉阶段机械臂转离视野）"
    )
    output(
        f"grab_servo0_angle={GRAB_SERVO0_ANGLE} deg "
        "（抓取前恢复已验证基线）"
    )
    output(
        f"post_pick_retreat={POST_PICK_RETREAT_DISTANCE:.2f} m "
        f"({retreat_time:.2f} s)"
    )
    output(
        "流程: 初始姿态(Servo0=0+旧步骤1-5) -> 撞墙 -> 后撤到视觉位置 -> "
        "SEARCH -> ALIGN -> READY(STOP) -> Servo0=95恢复基线 -> "
        "PREPARE_ARM -> PICK -> 后撤0.30m -> PLACE -> 静止姿态 -> STOP"
    )


def run_test(
    args: argparse.Namespace,
    *,
    input_fn: Callable[[str], str] = input,
    stm32_factory: Callable[[], Stm32Link] = _default_stm32_factory,
    k230_factory: Callable[[], K230Link] = _default_k230_factory,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Run the manually confirmed visual grab experiment."""
    output("RoboGame K230D 视觉辅助单次抓取实验")
    output("WARNING: 本入口会驱动底盘、Servo 和 Stepper。")
    output("WARNING: 请清空场地、确认升降轴剩余行程并准备人工急停。")
    output("WARNING: 距离由 VEL 时间估算，没有编码器反馈，必须实车测量。")
    _print_plan(output, args)

    if args.dry_run:
        output("DRY RUN完成：未打开串口，未发送任何命令。")
        return 0

    if input_fn("Press ENTER to start (输入其它字符将取消): ") != "":
        output("已取消：未执行任何动作。")
        return 2

    stm32: Stm32Link | None = None
    k230: K230Link | None = None
    try:
        output("连接 STM32 ...")
        stm32 = stm32_factory()
        stm32.connect()
        output("STM32 connected")
        output("TX: HELLO_STM32,1")
        hello_reply = stm32.hello(version=1, timeout=HELLO_TIMEOUT_S)
        output(f"RX: {hello_reply}")

        output("连接 K230D ...")
        k230 = k230_factory()
        k230.connect()
        output("K230D connected")

        # 启动保护 STOP：第一个运动相关命令前必须先确认停车
        output("")
        output("启动保护 STOP")
        stop_ack = stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label="visual_grab_startup",
        )
        output(f"RX: {stop_ack}")

        # --- 视觉阶段前：固定初始舵机姿态，机械臂转离 K230D 视野 ---
        # Servo0=0 使底座转离摄像头；Servo1/2/4 沿用旧序列开头
        # （旧步骤1-5）的角度 0/0/135。抓取开始前恢复 Servo0 并重新
        # 执行 PREPARE_ARM，确认已验证抓取姿态。
        output("")
        output(
            f"=== 初始舵机姿态：Servo0 -> {VISUAL_SERVO0_ANGLE} 度 ==="
        )
        ack = stm32.servo_set(0, VISUAL_SERVO0_ANGLE, timeout=ACK_TIMEOUT_S)
        output(f"RX: {ack}")

        output("")
        output("=== 初始舵机姿态：Servo1/2/4 = 0/0/135（旧步骤1-5） ===")
        execute_grab_steps(
            stm32,
            PREPARE_ARM_STEPS,
            output=output,
            clock=clock,
        )

        output("")
        output(f"=== 撞墙定位 ({WALL_APPROACH_DISTANCE:.2f} m) ===")
        wall_time = (
            WALL_APPROACH_DISTANCE / args.forward_actual_speed
        )
        _drive_timed_distance(
            stm32,
            args.command_speed,
            wall_time,
            label="撞墙0.50m",
            output=output,
            clock=clock,
        )

        output("")
        output(
            f"=== 后撤到视觉抓取位置 ({VISUAL_BACKOFF_DISTANCE:.2f} m) ==="
        )
        backoff_time = (
            VISUAL_BACKOFF_DISTANCE / args.backward_actual_speed
        )
        _drive_timed_distance(
            stm32,
            -args.command_speed,
            backoff_time,
            label="后撤0.05m",
            output=output,
            clock=clock,
        )

        execute_grab_steps(
            stm32,
            PRE_PICK_SETTLE_STEPS,
            output=output,
            clock=clock,
        )

        # --- K230D 视觉定位阶段 ---
        run_visual_phase(
            stm32,
            k230,
            search_timeout_s=args.search_timeout,
            output=output,
            clock=clock,
        )

        # --- 抓取与放盒（复用已验证机械臂动作，不重新实现）---
        # 抓取开始前恢复 Servo0 到已验证抓取基线，再执行此前一直使用
        # 的机械臂准备与抓取过程（PREPARE_ARM -> PICK）。
        output("")
        output(
            f"=== 恢复 Servo0 -> {GRAB_SERVO0_ANGLE} 度（抓取基线） ==="
        )
        ack = stm32.servo_set(0, GRAB_SERVO0_ANGLE, timeout=ACK_TIMEOUT_S)
        output(f"RX: {ack}")

        output("")
        output("=== PREPARE ARM (旧步骤1-5) ===")
        execute_grab_steps(
            stm32,
            PREPARE_ARM_STEPS,
            output=output,
            clock=clock,
        )

        output("")
        output("=== GRAB: 执行已验证抓取动作 ===")
        execute_grab_steps(
            stm32,
            PICK_ARM_STEPS,
            output=output,
            clock=clock,
        )

        output("")
        output(f"=== 抓起后后撤 ({POST_PICK_RETREAT_DISTANCE:.2f} m) ===")
        retreat_time = (
            POST_PICK_RETREAT_DISTANCE / args.backward_actual_speed
        )
        _drive_timed_distance(
            stm32,
            -args.command_speed,
            retreat_time,
            label="后撤0.30m",
            output=output,
            clock=clock,
        )

        execute_grab_steps(
            stm32,
            PRE_PLACE_SETTLE_STEPS,
            output=output,
            clock=clock,
        )

        output("")
        output("=== PLACE: 放入中间盒 ===")
        execute_grab_steps(
            stm32,
            PLACE_ARM_STEPS,
            output=output,
            clock=clock,
        )
        execute_grab_steps(
            stm32,
            REST_ARM_STEPS,
            output=output,
            clock=clock,
        )

        output("")
        output("完成保护 STOP")
        stop_ack = stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label="visual_grab_end",
        )
        output(f"RX: {stop_ack}")
        output("视觉辅助抓取实验完成：机械臂已进入静止姿态。")
        return 0
    except KeyboardInterrupt:
        _emergency_stop(stm32, reason="Ctrl+C", output=output)
        output("Ctrl+C：已请求立即STOP。")
        return 130
    except Exception as exc:
        _emergency_stop(stm32, reason=type(exc).__name__, output=output)
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        for link_name, link in (("STM32", stm32), ("K230D", k230)):
            if link is not None:
                try:
                    link.close()
                except Exception as exc:
                    output(
                        f"FAULT: {link_name}关闭失败: "
                        f"{type(exc).__name__}: {exc}"
                    )


def main(argv: Sequence[str] | None = None) -> int:
    return run_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
