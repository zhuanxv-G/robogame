"""STM32 serial communication abstraction for the RoboGame Pi software.

Constructing :class:`Stm32Link` has no hardware side effects.  The serial port
is opened only by an explicit :meth:`connect` call.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

from config import STM32_BAUDRATE, STM32_PORT


class SerialConnection(Protocol):
    """Subset of the pyserial interface used by this module."""

    def write(self, data: bytes) -> int | None: ...

    def flush(self) -> None: ...

    def readline(self) -> bytes: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


SerialFactory = Callable[..., SerialConnection]


class Stm32NotConnectedError(RuntimeError):
    """Raised when an operation requires an unopened STM32 link."""


class Stm32ProtocolError(RuntimeError):
    """Raised when STM32 reports an ``ERR,...`` response."""


class NextTurn(str, Enum):
    """Direction that STM32 will execute at the next confirmed junction."""

    NONE = "NONE"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STRAIGHT = "STRAIGHT"


class LineNavigationEventType(str, Enum):
    """Asynchronous LineNav events emitted by STM32."""

    JUNCTION_DETECTED = "JUNCTION_DETECTED"
    TURN_STARTED = "TURN_STARTED"
    TURN_DONE = "TURN_DONE"


@dataclass(frozen=True, slots=True)
class LineNavigationEvent:
    """Parsed asynchronous junction/turn event."""

    event_type: LineNavigationEventType
    turn: NextTurn | None = None


def parse_line_navigation_event(line: str) -> LineNavigationEvent | None:
    """Parse a LineNav event, returning ``None`` for unrelated protocol lines."""
    normalized = line.strip()
    if normalized == LineNavigationEventType.JUNCTION_DETECTED.value:
        return LineNavigationEvent(LineNavigationEventType.JUNCTION_DETECTED)

    fields = normalized.split(",")
    if not fields or fields[0] not in {
        LineNavigationEventType.TURN_STARTED.value,
        LineNavigationEventType.TURN_DONE.value,
    }:
        return None
    if len(fields) != 2:
        raise Stm32ProtocolError(f"Invalid LineNav event: {line!r}")

    try:
        turn = NextTurn(fields[1])
        event_type = LineNavigationEventType(fields[0])
    except ValueError as exc:
        raise Stm32ProtocolError(f"Invalid LineNav event: {line!r}") from exc
    if turn is NextTurn.NONE:
        raise Stm32ProtocolError(f"Invalid LineNav event: {line!r}")
    return LineNavigationEvent(event_type, turn)


class Stm32Link:
    """Own the single Pi-to-STM32 serial connection and text protocol."""

    def __init__(
        self,
        port: str = STM32_PORT,
        baudrate: int = STM32_BAUDRATE,
        timeout: float = 0.2,
        *,
        serial_factory: SerialFactory | None = None,
        stop_debug: bool = False,
        diagnostic_output: Callable[[str], object] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial_factory = serial_factory
        self._serial: SerialConnection | None = None
        self._stop_debug = stop_debug
        self._diagnostic_output = diagnostic_output or print
        self._stop_request_count = 0

    @property
    def is_connected(self) -> bool:
        """Return whether this object currently owns an open serial link."""
        if self._serial is None:
            return False
        return bool(getattr(self._serial, "is_open", True))

    def connect(self) -> None:
        """Open the configured STM32 serial port.

        Importing pyserial is delayed until this explicit hardware operation.
        If opening or initialization fails, the partial connection is closed
        and the original exception is re-raised.
        """
        if self.is_connected:
            return

        serial_connection: SerialConnection | None = None
        try:
            factory = self._serial_factory or self._load_serial_factory()
            serial_connection = factory(
                self.port,
                self.baudrate,
                timeout=self.timeout,
            )
            serial_connection.reset_input_buffer()
        except Exception:
            if serial_connection is not None:
                try:
                    serial_connection.close()
                except Exception:
                    pass
            raise

        self._serial = serial_connection

    @staticmethod
    def _load_serial_factory() -> SerialFactory:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required to connect to STM32"
            ) from exc
        return serial.Serial

    def close(self) -> None:
        """Close the serial port if it is open, preserving close errors."""
        if self._serial is None:
            return

        serial_connection = self._serial
        self._serial = None
        serial_connection.close()

    def _require_connection(self) -> SerialConnection:
        if not self.is_connected or self._serial is None:
            raise Stm32NotConnectedError(
                "STM32 link is not connected; call connect() first"
            )
        return self._serial

    @staticmethod
    def _validate_command(command: str) -> None:
        if not command:
            raise ValueError("STM32 command cannot be empty")
        if "\n" in command or "\r" in command:
            raise ValueError("STM32 command cannot contain a line ending")

    def send_command(self, command: str) -> str:
        """Send one unchanged protocol command followed by the required LF."""
        self._validate_command(command)
        serial_connection = self._require_connection()
        serial_connection.write((command + "\n").encode("ascii"))
        serial_connection.flush()
        return command

    def write(self, data: bytes) -> int | None:
        """Forward bytes for compatibility with the existing ArmSequence.

        New protocol code should use :meth:`send_command`.  This narrow serial
        interface lets the verified action module reuse this link without
        exposing or owning the underlying ``serial.Serial`` instance.
        """
        return self._require_connection().write(data)

    def flush(self) -> None:
        """Flush pending output for the existing ArmSequence interface."""
        self._require_connection().flush()

    def readline(self) -> bytes:
        """Read one response line for the existing ArmSequence interface."""
        return self._require_connection().readline()

    def _wait_for(
        self,
        expected_prefix: str,
        timeout: float,
        *,
        stop_debug_context: tuple[int, str, int] | None = None,
    ) -> str:
        """Wait for a response prefix and fail immediately on STM32 ERR."""
        serial_connection = self._require_connection()
        wait_started_ns = time.monotonic_ns()
        deadline_ns = wait_started_ns + int(timeout * 1_000_000_000)

        while time.monotonic_ns() < deadline_ns:
            raw_line = serial_connection.readline()
            if stop_debug_context is not None and raw_line:
                self._emit_stop_debug(
                    stop_debug_context,
                    f"RX {raw_line!r}",
                )
            line = raw_line.decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("ERR,"):
                raise Stm32ProtocolError(f"STM32 error: {line}")
            if line.startswith(expected_prefix):
                if stop_debug_context is not None:
                    self._emit_stop_debug(
                        stop_debug_context,
                        f"matched {expected_prefix}",
                    )
                return line

        if stop_debug_context is not None:
            self._emit_stop_debug(
                stop_debug_context,
                f"timeout waiting_for={expected_prefix!r}",
            )
        raise TimeoutError(f"Timeout waiting for: {expected_prefix}")

    def _emit_stop_debug(
        self,
        context: tuple[int, str, int],
        message: str,
    ) -> None:
        """Emit best-effort STOP diagnostics without affecting control flow."""
        if not self._stop_debug:
            return
        request_id, phase, request_started_ns = context
        elapsed_ms = (
            time.monotonic_ns() - request_started_ns
        ) / 1_000_000.0
        try:
            self._diagnostic_output(
                f"[STOPDBG] request={request_id} phase={phase} "
                f"elapsed_ms={elapsed_ms:.1f} {message}"
            )
        except Exception:
            # Diagnostics must never prevent or delay a safety STOP.
            pass

    @staticmethod
    def _safe_in_waiting(serial_connection: SerialConnection) -> str:
        try:
            value = getattr(serial_connection, "in_waiting")
        except Exception as exc:
            return f"unavailable:{type(exc).__name__}"
        if value is None:
            return "unavailable"
        try:
            return str(int(value))
        except (TypeError, ValueError):
            return f"unavailable:{type(value).__name__}"

    def wait_ack(self, command: str, timeout: float = 2.0) -> str:
        """Wait for ``ACK,<command>`` using the current prefix semantics."""
        self._validate_command(command)
        return self._wait_for(f"ACK,{command}", timeout)

    def wait_done(self, expected_prefix: str, timeout: float = 10.0) -> str:
        """Wait for a command-specific completion response prefix."""
        if not expected_prefix:
            raise ValueError("DONE response prefix cannot be empty")
        return self._wait_for(expected_prefix, timeout)

    def hello(self, version: int = 1, timeout: float = 2.0) -> str:
        """Perform the existing ``HELLO_STM32`` protocol handshake."""
        command = f"HELLO_STM32,{version}"
        self.send_command(command)
        return self._wait_for(f"STM32_OK,{version}", timeout)

    def set_velocity(self, x: float, y: float, omega: float) -> str:
        """Send the existing two-decimal ``VEL`` command without blocking."""
        command = f"VEL,{x:.2f},{y:.2f},{omega:.2f}"
        return self.send_command(command)

    def set_next_turn(
        self,
        turn: NextTurn | str,
        timeout: float = 2.0,
    ) -> str:
        """Preset one junction action and wait for its echoed ACK.

        This command does not start LineNav and does not execute the action.
        """
        try:
            normalized_turn = turn if isinstance(turn, NextTurn) else NextTurn(turn)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "turn must be NONE, LEFT, RIGHT, or STRAIGHT"
            ) from exc

        command = f"NEXT_TURN,{normalized_turn.value}"
        self.send_command(command)
        return self.wait_ack(command, timeout)

    def servo_get(self, index: int, timeout: float = 2.0) -> int:
        """Read a Servo angle with the existing ``SERVO_GET`` exchange."""
        command = f"SERVO_GET,{index}"
        self.send_command(command)
        line = self._wait_for(f"SERVO_POS,{index},", timeout)
        try:
            return int(line.split(",")[2])
        except (IndexError, ValueError) as exc:
            raise Stm32ProtocolError(
                f"Invalid SERVO_POS response: {line}"
            ) from exc

    def servo_set(
        self,
        index: int,
        angle: int,
        timeout: float = 2.0,
    ) -> str:
        """Set a Servo angle and wait for its echoed ACK."""
        command = f"SERVO_SET,{index},{angle}"
        self.send_command(command)
        return self.wait_ack(command, timeout)

    def stepper_move(
        self,
        steps: int,
        pps: int = 800,
        *,
        ack_timeout: float = 2.0,
        done_timeout: float | None = None,
    ) -> str:
        """Move the Stepper and wait for both its ACK and MOVE_DONE."""
        command = f"STEPPER_MOVE,{steps},{pps}"
        self.send_command(command)
        self.wait_ack(command, ack_timeout)

        if done_timeout is None:
            done_timeout = abs(steps) / float(pps) * 2.0 + 8.0
        return self.wait_done(f"STEPPER_MOVE_DONE,{steps}", done_timeout)

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
        debug_label: str = "stop",
    ) -> str | None:
        """Send global ``STOP``, optionally requiring its existing ACK."""
        serial_connection = self._require_connection()
        self._stop_request_count += 1
        request_started_ns = time.monotonic_ns()
        debug_context = (
            self._stop_request_count,
            debug_label,
            request_started_ns,
        )
        if self._stop_debug:
            in_waiting = self._safe_in_waiting(serial_connection)
            self._emit_stop_debug(
                debug_context,
                f"STOP_REQUEST monotonic_ns={request_started_ns} "
                f"in_waiting={in_waiting}",
            )

        try:
            self.send_command("STOP")
        except Exception as exc:
            self._emit_stop_debug(
                debug_context,
                f"STOP_WRITE_FAILED {type(exc).__name__}: {exc}",
            )
            raise
        self._emit_stop_debug(debug_context, "STOP_WRITE_DONE")

        try:
            return self._wait_for(
                "ACK,STOP",
                timeout,
                stop_debug_context=(
                    debug_context if self._stop_debug else None
                ),
            )
        except Exception:
            if require_ack:
                raise
            return None

    def __enter__(self) -> Stm32Link:
        self.connect()
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()
