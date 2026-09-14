"""Manual replay of the old grab/release candidate's arm actions.

The source candidate also contains three blocking chassis ``Move_Distance``
calls. The current allowed UART commands cannot reproduce those distance
moves, so this tool records them in their original positions but never
substitutes an open-loop ``VEL`` command. Importing this module has no hardware
side effects.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    Stm32Link,
    Stm32ProtocolError,
)


ACK_TIMEOUT_S = 2.0
SERIAL_POLL_TIMEOUT_S = 0.01
STEPPER_PPS = 800
SWEEP_STEP_DELAY_S = 0.01


class ActionKind(str, Enum):
    """Kinds found in the old ``main.c:214-248`` candidate."""

    DELAY = "delay"
    SERVO = "servo"
    STEPPER = "stepper"
    SERVO_SWEEP = "servo_sweep"
    CHASSIS_DISTANCE = "chassis_distance"


@dataclass(frozen=True, slots=True)
class OldAction:
    """One source-level action retained in its original order."""

    step: int
    kind: ActionKind
    values: tuple[int | float, ...]
    description: str


OLD_SEQUENCE: tuple[OldAction, ...] = (
    OldAction(1, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(2, ActionKind.SERVO, (1, 0), "Servo[1].Set_Angle(0)"),
    OldAction(3, ActionKind.SERVO, (2, 0), "Servo[2].Set_Angle(0)"),
    OldAction(4, ActionKind.SERVO, (4, 135), "Servo[4].Set_Angle(135)"),
    OldAction(5, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(
        6,
        ActionKind.CHASSIS_DISTANCE,
        (0.50, 2.0),
        "Robot.Move_Distance(0.50f, 2.0f)",
    ),
    OldAction(
        7,
        ActionKind.CHASSIS_DISTANCE,
        (-0.05, 1.0),
        "Robot.Move_Distance(-0.05f, 1.0f)",
    ),
    OldAction(8, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(
        9,
        ActionKind.STEPPER,
        (3000, STEPPER_PPS),
        "Stepper.Move_Blocking(3000, 800)",
    ),
    OldAction(10, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(11, ActionKind.SERVO, (4, 105), "Servo[4].Set_Angle(105)"),
    OldAction(12, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(13, ActionKind.SERVO, (2, 15), "Servo[2].Set_Angle(15)"),
    OldAction(14, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(
        15,
        ActionKind.STEPPER,
        (-3000, STEPPER_PPS),
        "Stepper.Move_Blocking(-3000, 800)",
    ),
    OldAction(16, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(
        17,
        ActionKind.CHASSIS_DISTANCE,
        (-0.30, 1.5),
        "Robot.Move_Distance(-0.30f, 1.5f)",
    ),
    OldAction(18, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(19, ActionKind.SERVO, (2, 0), "Servo[2].Set_Angle(0)"),
    OldAction(
        20,
        ActionKind.SERVO_SWEEP,
        (1, 0, 93, 10),
        "Servo[1].Sweep_To(93, 10)",
    ),
    OldAction(21, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(
        22,
        ActionKind.STEPPER,
        (5000, STEPPER_PPS),
        "Stepper.Move_Blocking(5000, 800)",
    ),
    OldAction(23, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(24, ActionKind.SERVO, (4, 135), "Servo[4].Set_Angle(135)"),
    OldAction(25, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
    OldAction(
        26,
        ActionKind.STEPPER,
        (-5000, STEPPER_PPS),
        "Stepper.Move_Blocking(-5000, 800)",
    ),
    OldAction(27, ActionKind.DELAY, (0.5,), "HAL_Delay(500)"),
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay the Servo/Stepper portion of old main.c:214-248 "
            "using the current Stm32Link protocol."
        )
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the retained 27-step source plan without opening UART",
    )
    return parser.parse_args(argv)


def _step_label(step: int, substep: int | None = None) -> str:
    if substep is None:
        return f"旧步骤 {step:02d}"
    return f"旧步骤 {step:02d}.{substep:03d}"


def _print_plan(output: Callable[[str], object]) -> None:
    output("旧版 main.c:214-248 动作表：")
    for action in OLD_SEQUENCE:
        suffix = ""
        if action.kind is ActionKind.CHASSIS_DISTANCE:
            suffix = " [仅记录：当前协议不能忠实执行]"
        output(
            f"  [{_step_label(action.step)}] {action.description}{suffix}"
        )


def _raise_on_err(line: str) -> None:
    if line.startswith("ERR,"):
        raise Stm32ProtocolError(f"STM32 error: {line}")


def _guarded_delay(
    stm32: Stm32Link,
    seconds: float,
    *,
    label: str,
    output: Callable[[str], object],
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Preserve an old delay while watching the serial stream for ERR."""
    output(f"[{label}] 等待: {seconds * 1000:.0f} ms")
    deadline = clock() + seconds
    while clock() < deadline:
        raw_line = stm32.readline()
        if not raw_line:
            continue
        line = raw_line.decode(errors="replace").strip()
        if not line:
            continue
        output(f"[{label}] 收到异步消息: {line}")
        _raise_on_err(line)
    output(f"[{label}] 等待完成")


def _send_ack_only(
    stm32: Stm32Link,
    command: str,
    *,
    label: str,
    output: Callable[[str], object],
) -> None:
    output(f"[{label}] 发送命令: {command}")
    stm32.send_command(command)
    ack = stm32.wait_ack(command, ACK_TIMEOUT_S)
    output(f"[{label}] 收到ACK: {ack}")
    output(f"[{label}] 收到DONE: 不适用（该命令协议只定义ACK）")


def _send_stepper(
    stm32: Stm32Link,
    steps: int,
    pps: int,
    *,
    label: str,
    output: Callable[[str], object],
) -> None:
    command = f"STEPPER_MOVE,{steps},{pps}"
    output(f"[{label}] 发送命令: {command}")
    stm32.send_command(command)
    ack = stm32.wait_ack(command, ACK_TIMEOUT_S)
    output(f"[{label}] 收到ACK: {ack}")

    nominal_seconds = abs(steps) / float(pps)
    done_timeout = nominal_seconds * 2.0 + 8.0
    done = stm32.wait_done(
        f"STEPPER_MOVE_DONE,{steps}",
        done_timeout,
    )
    output(f"[{label}] 收到DONE: {done}")


def _log_unsupported_chassis_action(
    action: OldAction,
    output: Callable[[str], object],
) -> None:
    label = _step_label(action.step)
    output(f"[{label}] 原动作: {action.description}")
    output(
        f"[{label}] 发送命令: 无（现有协议没有MOVE_DISTANCE，禁止用VEL替代）"
    )
    output(f"[{label}] 收到ACK: 不适用（未发送命令）")
    output(f"[{label}] 收到DONE: 不适用（未发送命令）")


def _execute_sweep(
    stm32: Stm32Link,
    action: OldAction,
    *,
    output: Callable[[str], object],
) -> None:
    index, start, target, step_delay_ms = (
        int(value) for value in action.values
    )
    direction = 1 if target > start else -1
    for substep, angle in enumerate(
        range(start + direction, target + direction, direction),
        start=1,
    ):
        label = _step_label(action.step, substep)
        _send_ack_only(
            stm32,
            f"SERVO_SET,{index},{angle}",
            label=label,
            output=output,
        )
        _guarded_delay(
            stm32,
            step_delay_ms / 1000.0,
            label=label,
            output=output,
        )


def execute_old_sequence(
    stm32: Stm32Link,
    *,
    output: Callable[[str], object] = print,
) -> None:
    """Execute every representable old action in exact source order."""
    for action in OLD_SEQUENCE:
        label = _step_label(action.step)
        output(f"[{label}] 原动作: {action.description}")

        if action.kind is ActionKind.DELAY:
            _guarded_delay(
                stm32,
                float(action.values[0]),
                label=label,
                output=output,
            )
        elif action.kind is ActionKind.SERVO:
            index, angle = (int(value) for value in action.values)
            _send_ack_only(
                stm32,
                f"SERVO_SET,{index},{angle}",
                label=label,
                output=output,
            )
        elif action.kind is ActionKind.STEPPER:
            steps, pps = (int(value) for value in action.values)
            _send_stepper(
                stm32,
                steps,
                pps,
                label=label,
                output=output,
            )
        elif action.kind is ActionKind.SERVO_SWEEP:
            _execute_sweep(stm32, action, output=output)
        elif action.kind is ActionKind.CHASSIS_DISTANCE:
            _log_unsupported_chassis_action(action, output)
        else:  # pragma: no cover - protects future action-table edits
            raise RuntimeError(f"unsupported action kind: {action.kind}")


def _request_stop(
    stm32: Stm32Link,
    *,
    label: str,
    output: Callable[[str], object],
    require_ack: bool,
) -> bool:
    """Write STOP first, then report its ACK without hiding failures."""
    output(f"[{label}] 发送命令: STOP")
    try:
        stm32.send_command("STOP")
    except Exception as exc:
        output(f"[{label}] STOP发送失败: {type(exc).__name__}: {exc}")
        if require_ack:
            raise
        return False

    try:
        ack = stm32.wait_ack("STOP", ACK_TIMEOUT_S)
    except Exception as exc:
        output(f"[{label}] STOP ACK失败: {type(exc).__name__}: {exc}")
        if require_ack:
            raise
        return True

    output(f"[{label}] 收到ACK: {ack}")
    output(f"[{label}] 收到DONE: 不适用（STOP协议只定义ACK）")
    return True


def _emergency_stop(
    stm32: Stm32Link | None,
    *,
    reason: str,
    output: Callable[[str], object],
) -> None:
    if stm32 is None or not stm32.is_connected:
        return
    _request_stop(
        stm32,
        label=f"危险保护/{reason}",
        output=output,
        require_ack=False,
    )


def _default_stm32_factory() -> Stm32Link:
    return Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)


def run_test(
    args: argparse.Namespace,
    *,
    input_fn: Callable[[str], str] = input,
    stm32_factory: Callable[[], Stm32Link] = _default_stm32_factory,
    output: Callable[[str], object] = print,
) -> int:
    """Run the manually confirmed live sequence."""
    output("RoboGame 旧版抓放候选机械臂测试")
    output("WARNING: 本入口会驱动 Servo 和 Stepper，Stepper 没有回零/限位语义。")
    output("WARNING: 请固定底盘、清空机构行程，并准备人工急停。")
    output("WARNING: 三段旧版底盘定距动作只记录，不发送，也不用VEL替代。")
    _print_plan(output)

    if args.dry_run:
        output("DRY RUN完成：未打开串口，未发送任何命令。")
        return 0

    if input_fn("输入 START 才会连接并执行: ").strip() != "START":
        output("已取消：未打开串口，未发送任何命令。")
        return 2

    stm32: Stm32Link | None = None
    try:
        stm32 = stm32_factory()
        stm32.connect()

        # This must remain the first command sent after opening the UART.
        _request_stop(
            stm32,
            label="启动保护",
            output=output,
            require_ack=True,
        )
        execute_old_sequence(stm32, output=output)
        _request_stop(
            stm32,
            label="完成保护",
            output=output,
            require_ack=True,
        )
        output("旧版候选的可表达机械臂动作执行完成。")
        return 0
    except KeyboardInterrupt:
        _emergency_stop(stm32, reason="Ctrl+C", output=output)
        output("Ctrl+C：已请求立即STOP。")
        return 130
    except Stm32ProtocolError as exc:
        _emergency_stop(stm32, reason="STM32_ERR", output=output)
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    except Exception as exc:
        _emergency_stop(stm32, reason="异常", output=output)
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None:
            try:
                stm32.close()
            except Exception as exc:
                output(f"FAULT: STM32关闭失败: {type(exc).__name__}: {exc}")


def main(argv: Sequence[str] | None = None) -> int:
    return run_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
