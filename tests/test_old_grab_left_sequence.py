#!/usr/bin/env python3
"""Manually test placing an already-grabbed block into the left box."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from arm_sequence import ArmSequence
from hardware.stm32_link import Stm32Link


SERVO_2_PLACE_ANGLE = 180
SERVO_0_LEFT_ANGLE = 0
SERVO_1_PLACE_ANGLE = 170
SERVO_1_SWEEP_STEP_DELAY_MS = 10
STEPPER_PLACE_STEPS = 5000
STEPPER_PPS = 800
SERVO_4_RELEASE_ANGLE = 135
ACTION_DELAY_S = 0.5


def run_left_placement(
    arm: ArmSequence,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """Run the STM32-derived left-box placement sequence."""
    print("\nLEFT BOX PLACEMENT")

    arm.servo_set(2, SERVO_2_PLACE_ANGLE)
    arm.servo_set(0, SERVO_0_LEFT_ANGLE)
    arm.servo_sweep(
        1,
        SERVO_1_PLACE_ANGLE,
        SERVO_1_SWEEP_STEP_DELAY_MS,
    )
    sleep_fn(ACTION_DELAY_S)

    arm.stepper_move(STEPPER_PLACE_STEPS, STEPPER_PPS)
    sleep_fn(ACTION_DELAY_S)

    arm.servo_set(4, SERVO_4_RELEASE_ANGLE)
    sleep_fn(ACTION_DELAY_S)

    arm.stepper_move(-STEPPER_PLACE_STEPS, STEPPER_PPS)
    sleep_fn(ACTION_DELAY_S)


def _try_stop(arm: ArmSequence | None) -> None:
    if arm is None:
        return
    try:
        print("\nEmergency STOP")
        arm.stop(require_ack=False)
    except Exception as exc:
        print(f"STOP failed: {exc}", file=sys.stderr)


def main() -> int:
    stm32: Stm32Link | None = None
    arm: ArmSequence | None = None

    print("Old grab left-box placement test")
    print("WARNING: the block must already be gripped and raised.")

    try:
        print("Connecting STM32...")
        stm32 = Stm32Link(timeout=0.1)
        stm32.connect()
        print("STM32 connected")

        arm = ArmSequence(stm32)
        arm.hello()

        confirmation = input("Press ENTER to start")
        if confirmation:
            print("Cancelled: ENTER was not pressed alone.")
            return 2

        # Clear any chassis motion before starting the arm-only test.
        arm.stop(require_ack=True)
        run_left_placement(arm)
        arm.stop(require_ack=True)

        print("\nLEFT BOX TEST FINISHED")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by Ctrl+C.", file=sys.stderr)
        _try_stop(arm)
        return 130
    except Exception as exc:
        print(f"\nTEST FAILED: {exc}", file=sys.stderr)
        _try_stop(arm)
        return 1
    finally:
        if stm32 is not None:
            stm32.close()
            print("STM32 serial closed")


if __name__ == "__main__":
    raise SystemExit(main())
