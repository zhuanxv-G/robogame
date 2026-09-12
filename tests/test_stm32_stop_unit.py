"""Pure-software timing and matching tests for Stm32Link STOP."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    Stm32Link,
    Stm32ProtocolError,
)


NANOSECONDS_PER_MILLISECOND = 1_000_000


class FakeMonotonicClock:
    def __init__(self) -> None:
        self.now_ns = 0

    def __call__(self) -> int:
        return self.now_ns

    def advance_to(self, target_ns: int) -> None:
        if target_ns > self.now_ns:
            self.now_ns = target_ns


class ScheduledFakeSerial:
    """A serial fake whose readline advances a deterministic fake clock."""

    def __init__(self, clock: FakeMonotonicClock) -> None:
        self._clock = clock
        self._responses: list[tuple[int, bytes]] = []
        self.timeout = 0.2
        self.is_open = True
        self.writes: list[tuple[int, bytes]] = []
        self.read_calls = 0

    def queue_after(self, delay_ms: float, raw_line: bytes) -> None:
        available_ns = self._clock() + int(
            delay_ms * NANOSECONDS_PER_MILLISECOND
        )
        self._responses.append((available_ns, raw_line))
        self._responses.sort(key=lambda response: response[0])

    @property
    def in_waiting(self) -> int:
        return sum(
            len(raw_line)
            for available_ns, raw_line in self._responses
            if available_ns <= self._clock()
        )

    def write(self, data: bytes) -> int:
        self.writes.append((self._clock(), data))
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        self.read_calls += 1
        if self._responses and self._responses[0][0] <= self._clock():
            return self._responses.pop(0)[1]

        read_deadline_ns = self._clock() + int(
            self.timeout * 1_000_000_000
        )
        if self._responses and self._responses[0][0] <= read_deadline_ns:
            self._clock.advance_to(self._responses[0][0])
            return self._responses.pop(0)[1]

        self._clock.advance_to(read_deadline_ns)
        return b""

    def reset_input_buffer(self) -> None:
        self._responses = [
            response
            for response in self._responses
            if response[0] > self._clock()
        ]

    def close(self) -> None:
        self.is_open = False


class Stm32StopTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeMonotonicClock()
        self.serial = ScheduledFakeSerial(self.clock)
        self.debug_lines: list[str] = []
        self.clock_patch = mock.patch(
            "hardware.stm32_link.time.monotonic_ns",
            side_effect=self.clock,
        )
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)

        def serial_factory(
            _port: str,
            _baudrate: int,
            *,
            timeout: float,
        ) -> ScheduledFakeSerial:
            self.serial.timeout = timeout
            return self.serial

        self.link = Stm32Link(
            port="fake",
            serial_factory=serial_factory,
            stop_debug=True,
            diagnostic_output=self.debug_lines.append,
        )
        self.link.connect()

    def test_immediate_stop_ack_succeeds_and_logs_timing(self) -> None:
        self.serial.queue_after(10, b"ACK,STOP\r\n")

        result = self.link.stop(require_ack=True)

        self.assertEqual(result, "ACK,STOP")
        debug = "\n".join(self.debug_lines)
        self.assertIn("STOP_REQUEST", debug)
        self.assertIn("in_waiting=0", debug)
        self.assertIn("STOP_WRITE_DONE", debug)
        self.assertIn("RX b'ACK,STOP\\r\\n'", debug)
        self.assertIn("matched ACK,STOP", debug)

    def test_unrelated_replies_are_discarded_until_stop_ack(self) -> None:
        self.serial.queue_after(5, b"ACK,LINE_START\r\n")
        self.serial.queue_after(10, b"LINE_STATUS,1,0,1,1,0,1\r\n")
        self.serial.queue_after(15, b"ACK,STOP\r\n")

        result = self.link.stop(require_ack=True)

        self.assertEqual(result, "ACK,STOP")
        debug = "\n".join(self.debug_lines)
        self.assertIn("RX b'ACK,LINE_START\\r\\n'", debug)
        self.assertIn("RX b'LINE_STATUS,1,0,1,1,0,1\\r\\n'", debug)

    def test_cleanup_can_match_first_stop_late_ack(self) -> None:
        # This response is scheduled as STOP #1's ACK, just after its 2 s
        # deadline. STOP #2 has no response of its own.
        self.serial.queue_after(2100, b"ACK,STOP\r\n")

        with self.assertRaisesRegex(TimeoutError, "ACK,STOP"):
            self.link.stop(require_ack=True)

        result = self.link.stop(
            require_ack=True,
            debug_label="cleanup",
        )

        self.assertEqual(result, "ACK,STOP")
        self.assertEqual(
            [data for _timestamp, data in self.serial.writes],
            [b"STOP\n", b"STOP\n"],
        )
        debug = "\n".join(self.debug_lines)
        self.assertIn(
            "request=1 phase=stop elapsed_ms=2000.0 "
            "timeout waiting_for='ACK,STOP'",
            debug,
        )
        self.assertIn(
            "request=2 phase=cleanup elapsed_ms=100.0 "
            "matched ACK,STOP",
            debug,
        )

    def test_missing_stop_ack_times_out(self) -> None:
        with self.assertRaisesRegex(TimeoutError, "ACK,STOP"):
            self.link.stop(require_ack=True)

        self.assertEqual(self.clock(), 2_000_000_000)
        self.assertEqual(self.serial.read_calls, 10)
        self.assertIn(
            "timeout waiting_for='ACK,STOP'",
            "\n".join(self.debug_lines),
        )

    def test_err_reply_fails_immediately(self) -> None:
        self.serial.queue_after(10, b"ERR,LINE_SENSOR_IO\r\n")

        with self.assertRaisesRegex(
            Stm32ProtocolError,
            "ERR,LINE_SENSOR_IO",
        ):
            self.link.stop(require_ack=True)

        self.assertEqual(self.clock(), 10_000_000)


if __name__ == "__main__":
    unittest.main()
