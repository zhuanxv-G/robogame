"""STM32 serial communication abstraction for the RoboGame Pi software.

Constructing :class:`Stm32Link` has no hardware side effects.  The serial port
is opened only by an explicit :meth:`connect` call.
"""

from __future__ import annotations

import time
from collections.abc import Callable
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


class Stm32Link:
    """Own the single Pi-to-STM32 serial connection and text protocol."""

    def __init__(
        self,
        port: str = STM32_PORT,
        baudrate: int = STM32_BAUDRATE,
        timeout: float = 0.2,
        *,
        serial_factory: SerialFactory | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial_factory = serial_factory
        self._serial: SerialConnection | None = None

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

    def _wait_for(self, expected_prefix: str, timeout: float) -> str:
        """Wait for a response prefix and fail immediately on STM32 ERR."""
        serial_connection = self._require_connection()
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            line = serial_connection.readline().decode(
                errors="ignore"
            ).strip()
            if not line:
                continue
            if line.startswith("ERR,"):
                raise Stm32ProtocolError(f"STM32 error: {line}")
            if line.startswith(expected_prefix):
                return line

        raise TimeoutError(f"Timeout waiting for: {expected_prefix}")

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
    ) -> str | None:
        """Send global ``STOP``, optionally requiring its existing ACK."""
        self.send_command("STOP")
        try:
            return self.wait_ack("STOP", timeout)
        except Exception:
            if require_ack:
                raise
            return None

    def __enter__(self) -> Stm32Link:
        self.connect()
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()
