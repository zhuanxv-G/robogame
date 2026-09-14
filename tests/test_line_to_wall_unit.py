"""Hardware-free coverage for the line-to-wall live test."""

from __future__ import annotations

import argparse
import unittest
from collections import deque

from tests.test_line_to_wall import (
    LINE_DURATION,
    REAL_SPEED,
    TARGET_DISTANCE,
    run_test,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeStm32Link:
    def __init__(
        self,
        clock: FakeClock,
        lines: list[bytes],
        *,
        interrupt_when_empty: bool = False,
    ) -> None:
        self.clock = clock
        self.lines = deque(lines)
        self.interrupt_when_empty = interrupt_when_empty
        self.is_connected = False
        self.calls: list[object] = []

    def connect(self) -> None:
        self.is_connected = True
        self.calls.append("connect")

    def close(self) -> None:
        self.is_connected = False
        self.calls.append("close")

    def send_command(self, command: str) -> str:
        self.calls.append(("send", command))
        return command

    def wait_ack(self, command: str, timeout: float = 2.0) -> str:
        self.calls.append(("wait_ack", command, timeout))
        return f"ACK,{command}"

    def stop(self, **kwargs: object) -> str:
        self.calls.append(("stop", kwargs.get("require_ack")))
        return "ACK,STOP"

    def readline(self) -> bytes:
        self.clock.advance(0.25)
        if self.lines:
            return self.lines.popleft()
        if self.interrupt_when_empty:
            raise KeyboardInterrupt
        return b""


class LineToWallTests(unittest.TestCase):
    def test_duration_comes_from_distance_and_real_speed(self) -> None:
        self.assertEqual(LINE_DURATION, TARGET_DISTANCE / REAL_SPEED)
        self.assertAlmostEqual(LINE_DURATION, 3.220338983, places=6)

    def test_dry_run_never_constructs_hardware(self) -> None:
        calls = 0

        def forbidden_factory() -> FakeStm32Link:
            nonlocal calls
            calls += 1
            raise AssertionError("hardware must not be constructed")

        result = run_test(
            argparse.Namespace(dry_run=True),
            stm32_factory=forbidden_factory,  # type: ignore[arg-type]
            output=lambda line: None,
        )

        self.assertEqual(result, 0)
        self.assertEqual(calls, 0)

    def test_without_exact_start_never_constructs_hardware(self) -> None:
        calls = 0

        def forbidden_factory() -> FakeStm32Link:
            nonlocal calls
            calls += 1
            raise AssertionError("hardware must not be constructed")

        result = run_test(
            argparse.Namespace(dry_run=False),
            input_fn=lambda prompt: "start",
            stm32_factory=forbidden_factory,  # type: ignore[arg-type]
            output=lambda line: None,
        )

        self.assertEqual(result, 2)
        self.assertEqual(calls, 0)

    def test_success_keeps_line_follow_and_uses_line_stop(self) -> None:
        clock = FakeClock()
        output: list[str] = []
        link = FakeStm32Link(
            clock,
            [
                b"JUNCTION_ARMED\r\n",
                b"JUNCTION_DETECTED\r\n",
                b"TURN_STARTED,RIGHT\r\n",
                b"SEARCH_LINE\r\n",
                b"LINE_REACQUIRED\r\n",
                b"TURN_DONE,RIGHT\r\n",
            ],
        )

        result = run_test(
            argparse.Namespace(dry_run=False),
            input_fn=lambda prompt: "START",
            stm32_factory=lambda: link,  # type: ignore[arg-type]
            output=output.append,
            clock=clock,
        )

        self.assertEqual(result, 0)
        sent = [call for call in link.calls if isinstance(call, tuple)]
        self.assertIn(("stop", True), sent)
        self.assertIn(("send", "NEXT_TURN,RIGHT"), sent)
        self.assertIn(("send", "LINE_START"), sent)
        self.assertIn(("send", "LINE_STOP"), sent)
        self.assertFalse(
            any(
                isinstance(call, tuple)
                and len(call) > 1
                and str(call[1]).startswith("VEL")
                for call in sent
            )
        )
        self.assertEqual(sent.count(("send", "LINE_START")), 1)
        self.assertNotIn(("stop", False), sent)
        self.assertGreaterEqual(clock(), LINE_DURATION)
        self.assertTrue(any("GRAB PREPARATION READY" in line for line in output))
        self.assertEqual(link.calls[-1], "close")

    def test_wrong_turn_direction_immediately_stops(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            [b"JUNCTION_DETECTED\r\n", b"TURN_STARTED,LEFT\r\n"],
        )
        output: list[str] = []

        result = run_test(
            argparse.Namespace(dry_run=False),
            input_fn=lambda prompt: "START",
            stm32_factory=lambda: link,  # type: ignore[arg-type]
            output=output.append,
            clock=clock,
        )

        self.assertEqual(result, 1)
        self.assertIn(("stop", False), link.calls)
        self.assertTrue(
            any("Expected TURN_STARTED,RIGHT" in line for line in output)
        )

    def test_err_and_ctrl_c_both_request_cleanup_stop(self) -> None:
        scenarios = (
            ([b"ERR,LINE_SENSOR_IO\r\n"], False, 1),
            ([], True, 130),
        )
        for lines, interrupt_when_empty, expected_result in scenarios:
            with self.subTest(expected_result=expected_result):
                clock = FakeClock()
                link = FakeStm32Link(
                    clock,
                    lines,
                    interrupt_when_empty=interrupt_when_empty,
                )
                result = run_test(
                    argparse.Namespace(dry_run=False),
                    input_fn=lambda prompt: "START",
                    stm32_factory=lambda: link,  # type: ignore[arg-type]
                    output=lambda line: None,
                    clock=clock,
                )

                self.assertEqual(result, expected_result)
                self.assertIn(("stop", False), link.calls)
                self.assertEqual(link.calls[-1], "close")


if __name__ == "__main__":
    unittest.main()
