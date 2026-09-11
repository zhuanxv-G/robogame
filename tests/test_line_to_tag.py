"""Manual STM32 line-follow plus AprilTag-arrival field test.

Importing this module has no hardware side effects. Run it explicitly and
confirm the prompt before it opens the camera/serial link or sends LINE_START.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from competition.navigation_controller import (  # noqa: E402
    NavigationController,
    NavigationState,
)
from hardware.stm32_link import Stm32Link  # noqa: E402
from vision.apriltag_service import AprilTagService  # noqa: E402


def positive_int(value: str) -> int:
    """Parse a positive integer command-line value."""
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if result <= 0:
        raise argparse.ArgumentTypeError("must be greater than 0")
    return result


def positive_area(value: str) -> float:
    """Parse a finite positive AprilTag pixel-area threshold."""
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError(
            "must be a finite number greater than 0"
        )
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Start STM32 line following and stop after the requested "
            "AprilTag reaches a caller-supplied area threshold."
        )
    )
    parser.add_argument("--tag-id", type=positive_int, required=True)
    parser.add_argument(
        "--arrival-area",
        type=positive_area,
        required=True,
        help="field-calibrated target Tag pixel area (no built-in default)",
    )
    parser.add_argument(
        "--confirm-frames",
        type=positive_int,
        default=3,
        help="consecutive qualifying frames required (default: %(default)s)",
    )
    return parser.parse_args(argv)


def print_status(controller: NavigationController) -> None:
    status = controller.get_navigation_status()
    state = status["state"].value
    error_x = status["error_x"]
    area = status["area"]
    print(
        f"state={state} target={status['target_tag_id']} "
        f"visible={int(status['tag_visible'])} "
        f"error_x={error_x if error_x is not None else '-'} "
        f"area={area if area is not None else '-'} "
        f"confirm={status['confirm_count']}/{status['confirm_frames']}"
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    print("RoboGame line-to-Tag manual test")
    print(f"target tag: {args.tag_id}")
    print(f"arrival area: {args.arrival_area}")
    print(f"confirm frames: {args.confirm_frames}")
    print("WARNING: confirming will send LINE_START and move the chassis.")

    confirmation = input("Type START to continue: ").strip()
    if confirmation != "START":
        print("Cancelled; no hardware was opened and no command was sent.")
        return 0

    stm32 = Stm32Link()
    apriltag = AprilTagService()
    controller = NavigationController(stm32, apriltag)

    try:
        # Make vision available before opening the chassis link. Neither
        # constructor above opens hardware.
        apriltag.connect()
        stm32.connect()
        controller.start_line_to_tag(
            args.tag_id,
            args.arrival_area,
            args.confirm_frames,
        )
        print("LINE_START sent; press Ctrl+C at any time to STOP.")

        while True:
            state = controller.update()
            print_status(controller)
            if state is NavigationState.ARRIVED:
                print("ARRIVED: STOP acknowledged by STM32.")
                return 0
            if state is NavigationState.FAULT:
                print("FAULT: navigation stopped.")
                return 1
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping navigation.")
        return 130
    except Exception as exc:
        print(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        arrived = controller.current_state is NavigationState.ARRIVED
        if stm32.is_connected and not arrived:
            try:
                stm32.stop(require_ack=True)
                print("STOP acknowledged by STM32.")
            except Exception as exc:
                print(f"STOP failed: {type(exc).__name__}: {exc}")
        try:
            apriltag.close()
        except Exception as exc:
            print(f"Camera close failed: {type(exc).__name__}: {exc}")
        try:
            stm32.close()
        except Exception as exc:
            print(f"STM32 close failed: {type(exc).__name__}: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
