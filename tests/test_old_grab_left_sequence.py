#!/usr/bin/env python3
"""Run the complete old grab flow and place the block in the left box.

The first 18 source actions are reused from ``test_old_grab_sequence.py``:
prepare the arm, drive to the wall, back off, pick, retract, and retreat. The
old middle-box placement is replaced with the provided left-box placement.
Importing this module never opens the UART or moves hardware.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TESTS_DIR = Path(__file__).resolve().parent
for import_path in (PROJECT_ROOT, TESTS_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from arm_sequence import ArmSequence
from hardware.stm32_link import Stm32Link
import test_old_grab_sequence as old_grab


# Preserve preparation, wall approach, pick and retreat (old steps 1-18).
GRAB_AND_RETREAT_ACTIONS = old_grab.OLD_SEQUENCE[:18]

SERVO_2_PLACE_ANGLE = 180
SERVO_0_LEFT_ANGLE = 0
SERVO_1_PLACE_ANGLE = 170
SERVO_1_SWEEP_STEP_DELAY_MS = 10
STEPPER_PLACE_STEPS = 5000
STEPPER_PPS = 800
SERVO_4_RELEASE_ANGLE = 135
ACTION_DELAY_S = 0.5
REST_SETTLE_S = 0.5
GRAB_REST_SERVO_ANGLES: tuple[tuple[int, int], ...] = (
    (0, 95),
    (1, 135),
    (2, 90),
    (3, 95),
    (4, 90),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Complete old grab flow with left-box placement."
    )
    parser.add_argument(
        "--command-speed",
        type=old_grab._command_speed,
        default=old_grab.DEFAULT_COMMAND_SPEED_M_S,
        metavar="M_S",
        help="absolute VEL.Y command used for chassis moves",
    )
    parser.add_argument(
        "--forward-actual-speed",
        type=old_grab._positive_speed,
        default=old_grab.DEFAULT_FORWARD_ACTUAL_SPEED_M_S,
        metavar="M_S",
        help="measured forward physical speed used for distance timing",
    )
    parser.add_argument(
        "--backward-actual-speed",
        type=old_grab._positive_speed,
        default=old_grab.DEFAULT_BACKWARD_ACTUAL_SPEED_M_S,
        metavar="M_S",
        help="measured backward physical speed used for distance timing",
    )
    return parser.parse_args()


def execute_grab_and_retreat(
    stm32: Stm32Link,
    arm: ArmSequence,
    args: argparse.Namespace,
) -> None:
    """Execute old source steps 1-18 in their original order."""
    for action in GRAB_AND_RETREAT_ACTIONS:
        label = old_grab._step_label(action.step)
        print(f"[{label}] 原动作: {action.description}")

        if action.kind is old_grab.ActionKind.DELAY:
            old_grab._guarded_delay(
                stm32,
                float(action.values[0]),
                label=label,
                output=print,
            )
        elif action.kind is old_grab.ActionKind.SERVO:
            index, angle = (int(value) for value in action.values)
            arm.servo_set(index, angle)
        elif action.kind is old_grab.ActionKind.STEPPER:
            steps, pps = (int(value) for value in action.values)
            arm.stepper_move(steps, pps)
        elif action.kind is old_grab.ActionKind.CHASSIS_DISTANCE:
            old_grab._drive_distance_open_loop(
                stm32,
                action,
                command_speed=args.command_speed,
                forward_actual_speed=args.forward_actual_speed,
                backward_actual_speed=args.backward_actual_speed,
                output=print,
                clock=time.monotonic,
            )
        else:
            raise RuntimeError(
                f"unexpected action in grab prefix: {action.kind}"
            )


def execute_left_placement(
    stm32: Stm32Link,
    arm: ArmSequence,
) -> None:
    """Place the held block in the left box in the required order."""
    print("\nLEFT BOX PLACEMENT")

    arm.servo_set(2, SERVO_2_PLACE_ANGLE)
    arm.servo_set(0, SERVO_0_LEFT_ANGLE)
    arm.servo_sweep(
        1,
        SERVO_1_PLACE_ANGLE,
        SERVO_1_SWEEP_STEP_DELAY_MS,
    )
    old_grab._guarded_delay(
        stm32,
        ACTION_DELAY_S,
        label="左盒放置/Servo就位",
        output=print,
    )

    arm.stepper_move(STEPPER_PLACE_STEPS, STEPPER_PPS)
    old_grab._guarded_delay(
        stm32,
        ACTION_DELAY_S,
        label="左盒放置/下降完成",
        output=print,
    )
    arm.servo_set(4, SERVO_4_RELEASE_ANGLE)
    old_grab._guarded_delay(
        stm32,
        ACTION_DELAY_S,
        label="左盒放置/释放完成",
        output=print,
    )
    arm.stepper_move(-STEPPER_PLACE_STEPS, STEPPER_PPS)
    old_grab._guarded_delay(
        stm32,
        ACTION_DELAY_S,
        label="左盒放置/回升完成",
        output=print,
    )


def execute_grab_rest_pose(
    stm32: Stm32Link,
    arm: ArmSequence,
) -> None:
    """Move every Servo to the configured post-grab resting angle."""
    print("\nRETURN ARM TO REST: Servo0..4 = 95,135,90,95,90")
    for index, angle in GRAB_REST_SERVO_ANGLES:
        arm.servo_set(index, angle)
    old_grab._guarded_delay(
        stm32,
        REST_SETTLE_S,
        label="左盒抓取结束/静止姿态稳定等待",
        output=print,
    )


def _try_stop(arm: ArmSequence | None) -> None:
    if arm is None:
        return
    try:
        print("\nEmergency STOP")
        arm.stop(require_ack=False)
    except Exception as exc:
        print(f"STOP failed: {exc}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    stm32: Stm32Link | None = None
    arm: ArmSequence | None = None

    print("Complete old grab and left-box placement test")
    print("WARNING: this test moves the chassis, Servo and Stepper.")
    print("WARNING: clear the route to the wall and prepare emergency stop.")
    print(f"command_speed={args.command_speed:.2f}")
    print(f"forward_actual_speed={args.forward_actual_speed:.3f} m/s")
    print(f"backward_actual_speed={args.backward_actual_speed:.3f} m/s")

    try:
        print("Connecting STM32...")
        stm32 = Stm32Link(timeout=old_grab.SERIAL_POLL_TIMEOUT_S)
        stm32.connect()
        print("STM32 connected")

        arm = ArmSequence(stm32)
        arm.hello()

        if input("Press ENTER to start"):
            print("Cancelled: ENTER was not pressed alone.")
            return 2

        arm.stop(require_ack=True)
        execute_grab_and_retreat(stm32, arm, args)
        execute_left_placement(stm32, arm)
        execute_grab_rest_pose(stm32, arm)
        arm.stop(require_ack=True)

        print("\nLEFT BOX COMPLETE TEST FINISHED")
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
            try:
                stm32.close()
                print("STM32 serial closed")
            except Exception as exc:
                print(f"STM32 close failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
