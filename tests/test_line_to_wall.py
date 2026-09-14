"""Operator-confirmed line-follow-to-wall live test.

The test presets one RIGHT junction action, starts the existing STM32 LineNav,
waits for the first confirmed junction and ``LINE_REACQUIRED``, then leaves
LineNav active for a distance-derived duration before sending ``LINE_STOP``.
Importing this module is hardware-safe.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    NextTurn,
    Stm32Link,
    Stm32ProtocolError,
)


# Field calibration: keep command value and measured physical speed separate.
TARGET_DISTANCE = 0.38
REAL_SPEED = 0.118
LINE_DURATION = TARGET_DISTANCE / REAL_SPEED

JUNCTION_TIMEOUT_S = 30.0
REACQUIRE_TIMEOUT_S = 25.0
ACK_TIMEOUT_S = 2.0
SERIAL_POLL_TIMEOUT_S = 0.05
EXPECTED_TURN = NextTurn.RIGHT


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Test line follow -> RIGHT T turn -> line reacquire -> timed "
            "line follow -> LINE_STOP"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the test plan without opening the STM32 serial port",
    )
    return parser.parse_args(argv)


def _default_stm32_factory() -> Stm32Link:
    return Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)


def _print_plan(output: Callable[[str], object]) -> None:
    output("RoboGame line-to-wall test")
    output(f"target_distance={TARGET_DISTANCE:.3f} m")
    output(f"real_speed={REAL_SPEED:.3f} m/s")
    output(f"line_duration={LINE_DURATION:.2f} s")
    output(f"junction_action={EXPECTED_TURN.value}")
    output(
        "Flow: STOP -> NEXT_TURN,RIGHT -> LINE_START -> "
        "JUNCTION_DETECTED -> LINE_REACQUIRED -> timed LINE_FOLLOW -> "
        "LINE_STOP"
    )


def _send_with_ack(
    stm32: Stm32Link,
    command: str,
    output: Callable[[str], object],
) -> str:
    output(f"TX: {command}")
    sent_command = stm32.send_command(command)
    ack = stm32.wait_ack(sent_command, timeout=ACK_TIMEOUT_S)
    output(f"RX: {ack}")
    return ack


def _read_protocol_line(
    stm32: Stm32Link,
    output: Callable[[str], object],
) -> str | None:
    raw_line = stm32.readline()
    if not raw_line:
        return None
    line = raw_line.decode(errors="replace").strip()
    if not line:
        return None
    output(f"RX: {line}")
    if line.startswith("ERR,"):
        raise Stm32ProtocolError(f"STM32 error: {line}")
    return line


def _validate_turn_direction(line: str) -> None:
    for prefix in ("TURN_STARTED,", "TURN_DONE,"):
        if not line.startswith(prefix):
            continue
        fields = line.split(",")
        if len(fields) != 2 or fields[1] != EXPECTED_TURN.value:
            raise Stm32ProtocolError(
                f"Expected {prefix}{EXPECTED_TURN.value}, received {line}"
            )


def _wait_for_event(
    stm32: Stm32Link,
    expected: str,
    timeout_s: float,
    *,
    output: Callable[[str], object],
    clock: Callable[[], float],
    junction_seen: bool,
) -> float:
    deadline = clock() + timeout_s
    while clock() < deadline:
        line = _read_protocol_line(stm32, output)
        if line is None:
            continue
        _validate_turn_direction(line)
        if line == expected:
            return clock()
        if not junction_seen and (
            line == "LINE_REACQUIRED"
            or line.startswith(("TURN_STARTED,", "TURN_DONE,"))
        ):
            raise Stm32ProtocolError(
                f"Out-of-order event before JUNCTION_DETECTED: {line}"
            )
    raise TimeoutError(f"Timeout waiting for: {expected}")


def _continue_line_follow(
    stm32: Stm32Link,
    started_at: float,
    *,
    output: Callable[[str], object],
    clock: Callable[[], float],
) -> None:
    deadline = started_at + LINE_DURATION
    output("Continue line following")
    output(f"Duration: {LINE_DURATION:.2f}s")
    while clock() < deadline:
        line = _read_protocol_line(stm32, output)
        if line is None:
            continue
        _validate_turn_direction(line)
        if line == "JUNCTION_DETECTED":
            raise Stm32ProtocolError(
                "Unexpected second JUNCTION_DETECTED during wall approach"
            )


def execute_test_flow(
    stm32: Stm32Link,
    *,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Execute the single test flow on an already-connected STM32 link."""
    output("SAFETY STOP")
    output("TX: STOP")
    stop_ack = stm32.stop(
        require_ack=True,
        timeout=ACK_TIMEOUT_S,
        debug_label="line_to_wall_startup",
    )
    output(f"RX: {stop_ack}")

    _send_with_ack(stm32, f"NEXT_TURN,{EXPECTED_TURN.value}", output)

    output("START LINE FOLLOW")
    _send_with_ack(stm32, "LINE_START", output)

    _wait_for_event(
        stm32,
        "JUNCTION_DETECTED",
        JUNCTION_TIMEOUT_S,
        output=output,
        clock=clock,
        junction_seen=False,
    )
    output("Waiting turn and line reacquisition...")
    reacquired_at = _wait_for_event(
        stm32,
        "LINE_REACQUIRED",
        REACQUIRE_TIMEOUT_S,
        output=output,
        clock=clock,
        junction_seen=True,
    )

    # STM32 is already back in LINE_FOLLOW here. Do not send VEL or LINE_START.
    _continue_line_follow(
        stm32,
        reacquired_at,
        output=output,
        clock=clock,
    )

    output("LINE_STOP")
    _send_with_ack(stm32, "LINE_STOP", output)
    output("GRAB PREPARATION READY (no arm action in this test)")
    output("TEST FINISHED")


def run_test(
    args: argparse.Namespace,
    *,
    input_fn: Callable[[str], str] = input,
    stm32_factory: Callable[[], Stm32Link] = _default_stm32_factory,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Run once after exact operator confirmation; never loop automatically."""
    _print_plan(output)
    if args.dry_run:
        output("DRY RUN: no serial connection and no motion command sent.")
        return 0

    output("WARNING: this test moves the chassis. Clear the route first.")
    output("Keep a physical emergency stop ready; do not run another UART reader.")
    if input_fn("Type START to connect and begin: ") != "START":
        output("Cancelled: exact START confirmation not received.")
        return 2

    stm32: Stm32Link | None = None
    line_stopped = False
    try:
        output("Connecting STM32...")
        stm32 = stm32_factory()
        stm32.connect()
        output("STM32 connected")
        execute_test_flow(stm32, output=output, clock=clock)
        line_stopped = True
        return 0
    except KeyboardInterrupt:
        output("Ctrl+C: immediate STOP requested.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None and stm32.is_connected and not line_stopped:
            try:
                output("TX: STOP (emergency cleanup)")
                stop_ack = stm32.stop(
                    require_ack=False,
                    timeout=0.5,
                    debug_label="line_to_wall_cleanup",
                )
                if stop_ack is not None:
                    output(f"RX: {stop_ack}")
            except Exception as exc:
                output(f"FAULT: STOP failed: {type(exc).__name__}: {exc}")
        if stm32 is not None:
            try:
                stm32.close()
            except Exception as exc:
                output(f"FAULT: STM32 close failed: {type(exc).__name__}: {exc}")


def main(argv: Sequence[str] | None = None) -> int:
    return run_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
