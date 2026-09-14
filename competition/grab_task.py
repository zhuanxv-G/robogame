"""Formal Pi-side orange grab task wrapping the verified old arm sequence.

This module packages the arm-only actions of the sequence retained in
``tests/test_old_grab_sequence.py`` (itself a replay of the old
``main.c:214-248`` candidate) into one callable task for the future mission
controller.

Scope of the packaging:

* The arm actions were verified on the real robot with
  ``tests/test_old_grab_sequence.py``.
* Servo angles, Stepper steps/pps, delays and their order are kept exactly
  as verified; do not tune parameters here.
* The three chassis ``Robot.Move_Distance`` steps of the old candidate are
  intentionally excluded: in the test they are open-loop VEL time estimates
  rather than verified arm motion, and chassis positioning belongs to the
  line-follow navigation (``test_line_to_wall.py``), not to the grab task.

Importing this module has no hardware side effects.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path


if __package__ in (None, ""):
    project_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(project_root))

from hardware.stm32_link import (  # noqa: E402
    Stm32Link,
    Stm32NotConnectedError,
    Stm32ProtocolError,
)


# Verified action parameters retained from tests/test_old_grab_sequence.py.
ACK_TIMEOUT_S = 2.0
STEPPER_PPS = 800
ACTION_DELAY_S = 0.5
SWEEP_STEP_DELAY_S = 0.01
SERIAL_POLL_TIMEOUT_S = 0.01


class ArmStepKind(str, Enum):
    """Kinds of arm-only actions retained from the verified test."""

    DELAY = "delay"
    SERVO = "servo"
    STEPPER = "stepper"
    SERVO_SWEEP = "servo_sweep"


@dataclass(frozen=True, slots=True)
class ArmStep:
    """One verified arm action in its original order."""

    phase: str
    kind: ArmStepKind
    values: tuple[int | float, ...]
    source: str


GRAB_SEQUENCE: tuple[ArmStep, ...] = (
    # --- Prepare arm (old candidate: Servo[1..3] setup + HAL_Delay) ---
    ArmStep("Prepare arm", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Prepare arm", ArmStepKind.SERVO, (1, 0),
            "Servo[1].Set_Angle(0)"),
    ArmStep("Prepare arm", ArmStepKind.SERVO, (2, 0),
            "Servo[2].Set_Angle(0)"),
    ArmStep("Prepare arm", ArmStepKind.SERVO, (4, 135),
            "Servo[4].Set_Angle(135)"),
    ArmStep("Prepare arm", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    # --- Lower arm ---
    ArmStep("Lower arm", ArmStepKind.STEPPER, (3000, STEPPER_PPS),
            "Stepper.Move_Blocking(3000, 800)"),
    ArmStep("Lower arm", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    # --- Close gripper ---
    ArmStep("Close gripper", ArmStepKind.SERVO, (4, 105),
            "Servo[4].Set_Angle(105)"),
    ArmStep("Close gripper", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Close gripper", ArmStepKind.SERVO, (2, 15),
            "Servo[2].Set_Angle(15)"),
    ArmStep("Close gripper", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    # --- Lift arm ---
    ArmStep("Lift arm", ArmStepKind.STEPPER, (-3000, STEPPER_PPS),
            "Stepper.Move_Blocking(-3000, 800)"),
    ArmStep("Lift arm", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    # --- Move to box (placement + release + retract) ---
    ArmStep("Move to box", ArmStepKind.SERVO, (2, 0),
            "Servo[2].Set_Angle(0)"),
    ArmStep("Move to box", ArmStepKind.SERVO_SWEEP, (1, 0, 93, 10),
            "Servo[1].Sweep_To(93, 10)"),
    ArmStep("Move to box", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Move to box", ArmStepKind.STEPPER, (5000, STEPPER_PPS),
            "Stepper.Move_Blocking(5000, 800)"),
    ArmStep("Move to box", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Move to box", ArmStepKind.SERVO, (4, 135),
            "Servo[4].Set_Angle(135)"),
    ArmStep("Move to box", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Move to box", ArmStepKind.STEPPER, (-5000, STEPPER_PPS),
            "Stepper.Move_Blocking(-5000, 800)"),
    ArmStep("Move to box", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
)


def _guarded_delay(
    stm32: Stm32Link,
    seconds: float,
    *,
    output: Callable[[str], object],
) -> None:
    """Wait while watching the serial stream for asynchronous ERR lines."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw_line = stm32.readline()
        if not raw_line:
            continue
        line = raw_line.decode(errors="replace").strip()
        if not line:
            continue
        output(f"      async RX: {line}")
        if line.startswith("ERR,"):
            raise Stm32ProtocolError(f"STM32 error: {line}")


def _emergency_stop(
    stm32: Stm32Link,
    *,
    reason: str,
    output: Callable[[str], object],
) -> None:
    """Best-effort global STOP that never masks the original failure."""
    if not stm32.is_connected:
        return
    output(f"    Emergency STOP ({reason})")
    try:
        stm32.stop(require_ack=False, timeout=ACK_TIMEOUT_S)
    except Exception as exc:
        output(f"    Emergency STOP send failed: {type(exc).__name__}: {exc}")


def _execute_step(
    stm32: Stm32Link,
    step: ArmStep,
    *,
    output: Callable[[str], object],
) -> None:
    """Execute one verified arm step with the current transactional protocol."""
    if step.kind is ArmStepKind.DELAY:
        seconds = float(step.values[0])
        output(f"    Wait {seconds * 1000:.0f} ms")
        _guarded_delay(stm32, seconds, output=output)
    elif step.kind is ArmStepKind.SERVO:
        index, angle = (int(value) for value in step.values)
        output(f"    Servo{index} -> {angle}")
        stm32.servo_set(index, angle, timeout=ACK_TIMEOUT_S)
    elif step.kind is ArmStepKind.STEPPER:
        steps, pps = (int(value) for value in step.values)
        sign = "+" if steps >= 0 else "-"
        output(f"    Stepper {sign}{abs(steps)} @ {pps} pps")
        stm32.stepper_move(steps, pps, ack_timeout=ACK_TIMEOUT_S)
    elif step.kind is ArmStepKind.SERVO_SWEEP:
        index, start, target, step_delay_ms = (
            int(value) for value in step.values
        )
        output(
            f"    Servo{index} sweep {start} -> {target} "
            f"({step_delay_ms} ms/deg)"
        )
        direction = 1 if target > start else -1
        for angle in range(start + direction, target + direction, direction):
            stm32.servo_set(index, angle, timeout=ACK_TIMEOUT_S)
            _guarded_delay(
                stm32,
                step_delay_ms / 1000.0,
                output=output,
            )
    else:  # pragma: no cover - protects future table edits
        raise RuntimeError(f"unsupported arm step kind: {step.kind}")


def grab_orange(
    stm32: Stm32Link | None = None,
    *,
    output: Callable[[str], object] = print,
) -> None:
    """Execute one complete orange grab-and-place with the verified arm.

    Without ``stm32`` the task opens its own serial link and closes it in
    ``finally``.  A caller-supplied link must already be connected and is
    left open afterwards, so the future mission controller can share the
    navigation connection.

    Safety: a global STOP is requested before the first arm action and after
    the last one.  Any serial failure, STM32 ``ERR`` reply or Ctrl+C triggers
    an immediate best-effort STOP and re-raises the original exception.
    """
    owns_link = stm32 is None
    if owns_link:
        stm32 = Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)

    output("")
    output("=== GRAB ORANGE START ===")

    try:
        if owns_link:
            stm32.connect()
        elif not stm32.is_connected:
            raise Stm32NotConnectedError(
                "supplied Stm32Link is not connected; call connect() first"
            )

        output("")
        output("Safety stop before start")
        stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label="grab_start",
        )
        output("    ACK,STOP")

        last_phase: str | None = None
        for step in GRAB_SEQUENCE:
            if step.phase != last_phase:
                last_phase = step.phase
                output("")
                output(step.phase)
                output("")
            _execute_step(stm32, step, output=output)

        output("")
        output("Safety stop after sequence")
        stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label="grab_end",
        )
        output("    ACK,STOP")
        output("")
        output("=== GRAB COMPLETE ===")
    except KeyboardInterrupt:
        _emergency_stop(stm32, reason="Ctrl+C", output=output)
        output("=== GRAB ABORTED (Ctrl+C) ===")
        raise
    except Exception as exc:
        _emergency_stop(stm32, reason=type(exc).__name__, output=output)
        output(f"=== GRAB ABORTED: {type(exc).__name__}: {exc} ===")
        raise
    finally:
        if owns_link:
            try:
                stm32.close()
            except Exception as exc:
                output(
                    f"STM32 close failed: {type(exc).__name__}: {exc}"
                )


def _print_plan(output: Callable[[str], object]) -> None:
    output("Verified grab plan (arm only):")
    for step in GRAB_SEQUENCE:
        output(f"  [{step.phase}] {step.source}")


def main(argv: Sequence[str] | None = None) -> int:
    """Field entry: connect, execute one grab_orange() run, release."""
    parser = argparse.ArgumentParser(
        description="Run the verified orange grab-and-place arm task once."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the retained plan without opening UART",
    )
    args = parser.parse_args(argv)

    _print_plan(print)

    if args.dry_run:
        print("DRY RUN complete: no serial port opened, no commands sent.")
        return 0

    if input("输入 START 才会连接并执行: ").strip() != "START":
        print("已取消：未打开串口，未发送任何命令。")
        return 2

    try:
        grab_orange()
    except KeyboardInterrupt:
        print("Ctrl+C：已请求立即STOP。")
        return 130
    except Exception as exc:
        print(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
