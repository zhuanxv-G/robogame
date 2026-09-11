"""K230D serial input and TARGET/NO_TARGET protocol parser."""

from __future__ import annotations

from collections.abc import Callable
from typing import Literal, Protocol, TypedDict

from config import K230_BAUDRATE, K230_PORT


class SerialConnection(Protocol):
    """Subset of the pyserial interface used by the K230D link."""

    @property
    def in_waiting(self) -> int: ...

    def readline(self) -> bytes: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


SerialFactory = Callable[..., SerialConnection]


class NoTargetResult(TypedDict):
    target: Literal[False]


class TargetResult(TypedDict):
    target: Literal[True]
    cx: int
    cy: int
    error_x: int
    steering: int
    pixels: int


K230Result = NoTargetResult | TargetResult


class K230NotConnectedError(RuntimeError):
    """Raised when reading before the K230D serial link is opened."""


def parse_k230_message(data: bytes | str) -> K230Result | None:
    """Parse the existing K230D ``TARGET`` and ``NO_TARGET`` messages."""
    try:
        if isinstance(data, bytes):
            message = data.decode(errors="ignore").strip()
        else:
            message = data.strip()

        message = message.replace("[UART TX] ", "")

        if message.startswith("NO_TARGET"):
            return {"target": False}

        if not message.startswith("TARGET,"):
            return None

        values = message.split(",")
        if len(values) < 6:
            return None

        return {
            "target": True,
            "cx": int(values[1]),
            "cy": int(values[2]),
            "error_x": int(values[3]),
            "steering": int(values[4]),
            "pixels": int(values[5]),
        }
    except (AttributeError, TypeError, ValueError):
        return None


class K230Link:
    """Own the K230D input link without opening it during construction."""

    def __init__(
        self,
        port: str = K230_PORT,
        baudrate: int = K230_BAUDRATE,
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
        if self._serial is None:
            return False
        return bool(getattr(self._serial, "is_open", True))

    def connect(self) -> None:
        """Explicitly open the configured K230D serial port."""
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
                "pyserial is required to connect to K230D"
            ) from exc
        return serial.Serial

    def read(self) -> K230Result | None:
        """Read and parse one K230D protocol line."""
        if not self.is_connected or self._serial is None:
            raise K230NotConnectedError(
                "K230D link is not connected; call connect() first"
            )
        return parse_k230_message(self._serial.readline())

    def read_latest(self) -> K230Result | None:
        """Read the newest queued message, matching the verified controller."""
        if not self.is_connected or self._serial is None:
            raise K230NotConnectedError(
                "K230D link is not connected; call connect() first"
            )

        latest_data: bytes | None = None
        while self._serial.in_waiting:
            latest_data = self._serial.readline()

        if latest_data is None:
            return None
        return parse_k230_message(latest_data)

    def close(self) -> None:
        """Close the K230D serial port if it is open."""
        if self._serial is None:
            return
        serial_connection = self._serial
        self._serial = None
        serial_connection.close()
