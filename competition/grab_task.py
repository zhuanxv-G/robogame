"""Formal Pi-side orange grab task wrapping the verified old arm sequence.

This module packages the arm-only actions of the sequence retained in
``tests/test_old_grab_sequence.py`` (itself a replay of the old
``main.c:214-248`` candidate) into one callable task for the future mission
controller.

Scope of the packaging:

* The arm actions were verified on the real robot with
  ``tests/test_old_grab_sequence.py``.
* Servo angles, Stepper steps/pps, delays and their order are kept exactly
  as verified. After those actions, the arm is moved to the configured
  grab-task rest pose requested for Servo0..4.
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


# Verified arm-action parameters retained from tests/test_old_grab_sequence.py.
ACK_TIMEOUT_S = 2.0
STEPPER_PPS = 800
ACTION_DELAY_S = 0.5
SWEEP_STEP_DELAY_S = 0.01
SERIAL_POLL_TIMEOUT_S = 0.01
REST_SETTLE_S = 0.5
GRAB_REST_SERVO_ANGLES: tuple[int, ...] = (95, 135, 90, 95, 90)


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


PREPARE_ARM_STEPS: tuple[ArmStep, ...] = (
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
)

# These two waits were old steps 8 and 18.  They remain explicit even though
# the chassis moves immediately before them are owned by grab_mission.py.
PRE_PICK_SETTLE_STEPS: tuple[ArmStep, ...] = (
    ArmStep("Settle after backoff", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
)

PICK_ARM_STEPS: tuple[ArmStep, ...] = (
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
)

PRE_PLACE_SETTLE_STEPS: tuple[ArmStep, ...] = (
    ArmStep("Settle after retreat", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
)

PLACE_ARM_STEPS: tuple[ArmStep, ...] = (
    # --- Placement + release + retract ---
    ArmStep("Place object", ArmStepKind.SERVO, (2, 0),
            "Servo[2].Set_Angle(0)"),
    ArmStep("Place object", ArmStepKind.SERVO_SWEEP, (1, 0, 93, 10),
            "Servo[1].Sweep_To(93, 10)"),
    ArmStep("Place object", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Place object", ArmStepKind.STEPPER, (5000, STEPPER_PPS),
            "Stepper.Move_Blocking(5000, 800)"),
    ArmStep("Place object", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Place object", ArmStepKind.SERVO, (4, 135),
            "Servo[4].Set_Angle(135)"),
    ArmStep("Place object", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
    ArmStep("Place object", ArmStepKind.STEPPER, (-5000, STEPPER_PPS),
            "Stepper.Move_Blocking(-5000, 800)"),
    ArmStep("Place object", ArmStepKind.DELAY, (ACTION_DELAY_S,),
            "HAL_Delay(500)"),
)

REST_ARM_STEPS: tuple[ArmStep, ...] = (
    ArmStep("Return arm to rest", ArmStepKind.SERVO, (0, 95),
            "Grab rest pose: Servo[0].Set_Angle(95)"),
    ArmStep("Return arm to rest", ArmStepKind.SERVO, (1, 135),
            "Grab rest pose: Servo[1].Set_Angle(135)"),
    ArmStep("Return arm to rest", ArmStepKind.SERVO, (2, 90),
            "Grab rest pose: Servo[2].Set_Angle(90)"),
    ArmStep("Return arm to rest", ArmStepKind.SERVO, (3, 95),
            "Grab rest pose: Servo[3].Set_Angle(95)"),
    ArmStep("Return arm to rest", ArmStepKind.SERVO, (4, 90),
            "Grab rest pose: Servo[4].Set_Angle(90)"),
    ArmStep("Return arm to rest", ArmStepKind.DELAY, (REST_SETTLE_S,),
            "Grab rest pose settle (500 ms)"),
)

# Every non-chassis source action, including the waits immediately following
# the two omitted chassis moves, is represented in its original relative
# order. The configured rest pose is appended only after all old actions.
# The integrated mission inserts its chassis moves at those boundaries.
GRAB_SEQUENCE: tuple[ArmStep, ...] = (
    PREPARE_ARM_STEPS
    + PRE_PICK_SETTLE_STEPS
    + PICK_ARM_STEPS
    + PRE_PLACE_SETTLE_STEPS
    + PLACE_ARM_STEPS
    + REST_ARM_STEPS
)


def _guarded_delay(
    stm32: Stm32Link,
    seconds: float,
    *,
    output: Callable[[str], object],
    clock: Callable[[], float],
) -> None:
    """Wait while watching the serial stream for asynchronous ERR lines."""
    deadline = clock() + seconds
    while clock() < deadline:
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
    clock: Callable[[], float],
) -> None:
    """Execute one verified arm step with the current transactional protocol."""
    if step.kind is ArmStepKind.DELAY:
        seconds = float(step.values[0])
        output(f"    Wait {seconds * 1000:.0f} ms")
        _guarded_delay(stm32, seconds, output=output, clock=clock)
    elif step.kind is ArmStepKind.SERVO:
        index, angle = (int(value) for value in step.values)
        command = f"SERVO_SET,{index},{angle}"
        output(f"    TX: {command}")
        ack = stm32.servo_set(index, angle, timeout=ACK_TIMEOUT_S)
        output(f"    RX ACK: {ack}")
        output("    RX DONE: N/A (SERVO_SET defines ACK only)")
    elif step.kind is ArmStepKind.STEPPER:
        steps, pps = (int(value) for value in step.values)
        command = f"STEPPER_MOVE,{steps},{pps}"
        output(f"    TX: {command}")
        stm32.send_command(command)
        ack = stm32.wait_ack(command, ACK_TIMEOUT_S)
        output(f"    RX ACK: {ack}")
        nominal_seconds = abs(steps) / float(pps)
        done = stm32.wait_done(
            f"STEPPER_MOVE_DONE,{steps}",
            nominal_seconds * 2.0 + 8.0,
        )
        output(f"    RX DONE: {done}")
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
            command = f"SERVO_SET,{index},{angle}"
            output(f"    TX: {command}")
            ack = stm32.servo_set(index, angle, timeout=ACK_TIMEOUT_S)
            output(f"    RX ACK: {ack}")
            output("    RX DONE: N/A (SERVO_SET defines ACK only)")
            _guarded_delay(
                stm32,
                step_delay_ms / 1000.0,
                output=output,
                clock=clock,
            )
    else:  # pragma: no cover - protects future table edits
        raise RuntimeError(f"unsupported arm step kind: {step.kind}")


def execute_grab_steps(
    stm32: Stm32Link,
    steps: Sequence[ArmStep],
    *,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Execute a selected, ordered arm segment on an existing connection.

    This lower-level entry does not send lifecycle STOP commands.  It lets the
    mission insert the old 5 cm and 30 cm chassis moves at the exact boundaries
    between the verified arm segments.  Callers retain responsibility for the
    same exception-to-STOP safety wrapper used by :func:`grab_orange`.
    """
    if not stm32.is_connected:
        raise Stm32NotConnectedError(
            "supplied Stm32Link is not connected; call connect() first"
        )

    last_phase: str | None = None
    for step in steps:
        if step.phase != last_phase:
            last_phase = step.phase
            output("")
            output(step.phase)
            output("")
        _execute_step(stm32, step, output=output, clock=clock)


def grab_orange(
    stm32: Stm32Link | None = None,
    *,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
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

        execute_grab_steps(
            stm32,
            GRAB_SEQUENCE,
            output=output,
            clock=clock,
        )

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
