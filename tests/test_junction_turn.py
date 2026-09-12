"""Operator-confirmed live test for one STM32 junction action.

This module is safe to import. It connects to hardware and sends ``LINE_START``
only after the operator types the exact uppercase word ``START``.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    NextTurn,
    Stm32Link,
    Stm32ProtocolError,
    parse_line_navigation_event,
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manually test one confirmed-junction action",
    )
    parser.add_argument(
        "--turn",
        required=True,
        choices=("LEFT", "RIGHT", "STRAIGHT"),
        help="pending action to execute at the next real junction",
    )
    return parser.parse_args(argv)


def run_test(
    args: argparse.Namespace,
    *,
    input_fn: Callable[[str], str] = input,
    stm32_factory: Callable[[], Stm32Link] = Stm32Link,
    output: Callable[[str], object] = print,
) -> int:
    """Run the live test after an exact operator confirmation."""
    output("RoboGame junction turn live test")
    output(f"turn={args.turn}")
    if args.turn == "STRAIGHT":
        output("STRAIGHT is permitted only where a real straight exit exists.")
    output("Expected: normal line -> junction -> turn -> reacquire -> line follow")
    output("The robot may move after confirmation. Keep emergency stop ready.")

    if input_fn("Type START to connect and begin: ") != "START":
        output("Cancelled: exact START confirmation not received.")
        return 2

    stm32: Stm32Link | None = None
    try:
        stm32 = stm32_factory()
        stm32.connect()

        turn_ack = stm32.set_next_turn(NextTurn(args.turn))
        output(turn_ack)

        line_command = stm32.send_command("LINE_START")
        line_ack = stm32.wait_ack(line_command)
        output(line_ack)
        output("LineNav active; press Ctrl+C to STOP and exit.")

        while True:
            raw_line = stm32.readline()
            if not raw_line:
                continue
            line = raw_line.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("ERR,"):
                raise Stm32ProtocolError(f"STM32 error: {line}")

            event = parse_line_navigation_event(line)
            if event is not None:
                output(line)
            elif line.startswith(("LINE_", "JUNCTION_", "TURN_")):
                output(line)
    except KeyboardInterrupt:
        output("Ctrl+C: STOP requested.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None:
            if stm32.is_connected:
                try:
                    stm32.stop(timeout=0.5, debug_label="junction_test_cleanup")
                except Exception as exc:
                    output(f"FAULT: STOP failed: {type(exc).__name__}: {exc}")
            try:
                stm32.close()
            except Exception as exc:
                output(f"FAULT: STM32 close failed: {type(exc).__name__}: {exc}")


def main(argv: Sequence[str] | None = None) -> int:
    return run_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
