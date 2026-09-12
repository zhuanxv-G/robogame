"""Read-only manual capture of the STM32 four-channel line sensors.

This tool sends only ``LINE_STATUS``. It never starts LineNav, sends a
chassis command, opens a camera, or touches Servo/Stepper hardware.
"""

from __future__ import annotations

import argparse
import csv
import select
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Protocol, TextIO


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    Stm32Link,
    Stm32ProtocolError,
)


LINE_STATUS_COMMAND = "LINE_STATUS"
LINE_STATUS_PREFIX = "LINE_STATUS,"
DEFAULT_RATE_HZ = 10.0
RESPONSE_TIMEOUT_S = 1.0
CSV_FIELDS = (
    "timestamp",
    "sensor_ok",
    "s1",
    "s2",
    "s3",
    "s4",
    "active",
    "mark",
)
ALLOWED_MARKS = frozenset(
    {
        "STRAIGHT",
        "LEFT_OFFSET",
        "RIGHT_OFFSET",
        "T_APPROACH",
        "T_ENTRY",
        "T_CENTER",
        "T_EXIT",
        "ALL_WHITE",
        "ALL_BLACK",
    }
)
QUIT_WORDS = frozenset({"Q", "QUIT", "EXIT"})


class LineStatusLink(Protocol):
    """The read-only subset of Stm32Link used by this capture tool."""

    def send_command(self, command: str) -> str: ...

    def readline(self) -> bytes: ...


class LineStatusParseError(ValueError):
    """Raised when a LINE_STATUS response is malformed."""


@dataclass(frozen=True, slots=True)
class LineSensorStatus:
    sensor_ok: int
    s1: int
    s2: int
    s3: int
    s4: int
    active: int

    @property
    def sensor_bits(self) -> str:
        return f"{self.s1}{self.s2}{self.s3}{self.s4}"


def parse_line_status(line: str) -> LineSensorStatus:
    """Parse the existing seven-field LINE_STATUS response."""
    fields = line.strip().split(",")
    if len(fields) != 7 or fields[0] != "LINE_STATUS":
        raise LineStatusParseError(f"invalid LINE_STATUS response: {line!r}")

    try:
        values = tuple(int(field) for field in fields[1:])
    except ValueError as exc:
        raise LineStatusParseError(
            f"non-integer LINE_STATUS field: {line!r}"
        ) from exc

    if any(value not in (0, 1) for value in values):
        raise LineStatusParseError(
            f"LINE_STATUS fields must be 0 or 1: {line!r}"
        )
    return LineSensorStatus(*values)


def request_line_status(
    stm32: LineStatusLink,
    *,
    timeout: float = RESPONSE_TIMEOUT_S,
    clock: Callable[[], float] = time.monotonic,
) -> LineSensorStatus:
    """Send the sole permitted query and wait for its matching response."""
    if timeout <= 0:
        raise ValueError("timeout must be greater than 0")

    stm32.send_command(LINE_STATUS_COMMAND)
    deadline = clock() + timeout
    while clock() < deadline:
        raw_line = stm32.readline()
        if not raw_line:
            continue
        line = raw_line.decode(errors="replace").strip()
        if line.startswith("ERR,"):
            raise Stm32ProtocolError(f"STM32 error: {line}")
        if line.startswith(LINE_STATUS_PREFIX):
            return parse_line_status(line)

    raise TimeoutError("Timeout waiting for: LINE_STATUS")


def create_csv_writer(stream: TextIO) -> csv.DictWriter:
    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
    writer.writeheader()
    return writer


def write_csv_sample(
    writer: csv.DictWriter,
    *,
    timestamp: float,
    status: LineSensorStatus,
    mark: str = "",
) -> None:
    writer.writerow(
        {
            "timestamp": f"{timestamp:.3f}",
            "sensor_ok": status.sensor_ok,
            "s1": status.s1,
            "s2": status.s2,
            "s3": status.s3,
            "s4": status.s4,
            "active": status.active,
            "mark": mark,
        }
    )


def default_output_path(now: datetime | None = None) -> Path:
    capture_time = now or datetime.now()
    filename = (
        "line_sensor_capture_"
        f"{capture_time.strftime('%Y%m%d_%H%M%S')}.csv"
    )
    return PROJECT_ROOT / "logs" / filename


def low_capture_rate(value: str) -> float:
    try:
        rate = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not 0 < rate <= 20:
        raise argparse.ArgumentTypeError("must be greater than 0 and at most 20")
    return rate


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture four line-sensor channels using read-only LINE_STATUS"
        )
    )
    parser.add_argument(
        "--rate-hz",
        type=low_capture_rate,
        default=DEFAULT_RATE_HZ,
        help="sampling rate, limited to 20 Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV path (default: timestamped file under logs/)",
    )
    return parser.parse_args(argv)


def wait_for_terminal_line(timeout: float) -> str | None:
    """Wait for one completed terminal line without a reader thread."""
    readable, _, _ = select.select([sys.stdin], [], [], max(0.0, timeout))
    if not readable:
        return None
    line = sys.stdin.readline()
    return "QUIT" if line == "" else line.strip()


def normalize_operator_input(value: str) -> tuple[bool, str | None]:
    """Return (quit_requested, mark) for one terminal line."""
    normalized = value.strip().upper()
    if not normalized:
        return False, None
    if normalized in QUIT_WORDS:
        return True, None
    if normalized.startswith("MARK,"):
        normalized = normalized.removeprefix("MARK,").strip()
    if normalized not in ALLOWED_MARKS:
        raise ValueError(f"unknown mark: {value!r}")
    return False, normalized


def run_capture(
    args: argparse.Namespace,
    *,
    stm32_factory: Callable[[], Stm32Link] = Stm32Link,
    input_waiter: Callable[[float], str | None] = wait_for_terminal_line,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Run the manual, read-only sensor capture loop."""
    csv_path = args.output or default_output_path()
    sample_period = 1.0 / args.rate_hz
    pending_marks: list[str] = []
    stm32 = stm32_factory()

    output("RoboGame read-only line sensor capture")
    output(f"command={LINE_STATUS_COMMAND}")
    output(f"rate_hz={args.rate_hz:g}")
    output(f"csv={csv_path}")
    output("No chassis, LineNav, Servo, Stepper, Camera, or K230D action.")
    output("Type a mark name and press Enter; type Q or press Ctrl+C to quit.")
    output("Marks: " + " ".join(sorted(ALLOWED_MARKS)))

    try:
        stm32.connect()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        started = clock()
        next_sample = started

        with csv_path.open("x", encoding="utf-8", newline="") as stream:
            writer = create_csv_writer(stream)
            while True:
                now = clock()
                if now < next_sample:
                    operator_input = input_waiter(next_sample - now)
                    if operator_input is not None:
                        try:
                            quit_requested, mark = normalize_operator_input(
                                operator_input
                            )
                        except ValueError as exc:
                            output(f"INPUT_ERROR: {exc}")
                            continue
                        if quit_requested:
                            output("Quit requested.")
                            return 0
                        if mark is not None:
                            pending_marks.append(mark)
                            output(f"MARK,{mark}")
                    continue

                sample_timestamp = now - started
                try:
                    status = request_line_status(stm32)
                except Exception as exc:
                    output(
                        f"{sample_timestamp:.3f}  "
                        f"ERROR={type(exc).__name__}: {exc}"
                    )
                else:
                    mark_value = "|".join(pending_marks)
                    pending_marks.clear()
                    output(
                        f"{sample_timestamp:.3f}  ok={status.sensor_ok}  "
                        f"sensors={status.sensor_bits}  "
                        f"active={status.active}"
                        + (f"  mark={mark_value}" if mark_value else "")
                    )
                    write_csv_sample(
                        writer,
                        timestamp=sample_timestamp,
                        status=status,
                        mark=mark_value,
                    )
                    stream.flush()

                next_sample += sample_period
                if next_sample < clock():
                    next_sample = clock()
    except KeyboardInterrupt:
        output("Ctrl+C: capture stopped.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        try:
            stm32.close()
        except Exception as exc:
            output(f"STM32 close failed: {type(exc).__name__}: {exc}")


def main(argv: Sequence[str] | None = None) -> int:
    return run_capture(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
