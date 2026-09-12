"""Hardware-free tests for NEXT_TURN and the manual junction entry point."""

from __future__ import annotations

import argparse
import sys
import unittest
from collections import deque
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    LineNavigationEventType,
    NextTurn,
    Stm32Link,
    Stm32ProtocolError,
    parse_line_navigation_event,
)
from tests.test_junction_turn import run_test  # noqa: E402


class FakeSerial:
    def __init__(self, responses: list[bytes]) -> None:
        self.responses = deque(responses)
        self.writes: list[bytes] = []
        self.is_open = True

    def reset_input_buffer(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        return self.responses.popleft() if self.responses else b""

    def close(self) -> None:
        self.is_open = False


class NextTurnProtocolTests(unittest.TestCase):
    def make_link(self, response: bytes) -> tuple[Stm32Link, FakeSerial]:
        serial = FakeSerial([response])
        link = Stm32Link(serial_factory=lambda *args, **kwargs: serial)
        link.connect()
        return link, serial

    def test_supported_turns_send_and_ack(self) -> None:
        for turn in NextTurn:
            with self.subTest(turn=turn):
                expected = f"ACK,NEXT_TURN,{turn.value}"
                link, serial = self.make_link((expected + "\r\n").encode())
                self.assertEqual(link.set_next_turn(turn), expected)
                self.assertEqual(
                    serial.writes,
                    [f"NEXT_TURN,{turn.value}\n".encode("ascii")],
                )

    def test_invalid_turn_is_rejected_without_a_write(self) -> None:
        link, serial = self.make_link(b"")
        with self.assertRaisesRegex(ValueError, "turn must be"):
            link.set_next_turn("UP")
        self.assertEqual(serial.writes, [])

    def test_stm32_error_is_raised(self) -> None:
        link, _ = self.make_link(b"ERR,NEXT_TURN\r\n")
        with self.assertRaisesRegex(Stm32ProtocolError, "ERR,NEXT_TURN"):
            link.set_next_turn(NextTurn.LEFT)


class LineNavigationEventTests(unittest.TestCase):
    def test_junction_event(self) -> None:
        event = parse_line_navigation_event("JUNCTION_DETECTED\r\n")
        self.assertIsNotNone(event)
        assert event is not None
        self.assertIs(event.event_type, LineNavigationEventType.JUNCTION_DETECTED)
        self.assertIsNone(event.turn)

    def test_turn_started_and_done_events(self) -> None:
        started = parse_line_navigation_event("TURN_STARTED,RIGHT")
        done = parse_line_navigation_event("TURN_DONE,RIGHT")
        self.assertEqual(started.turn, NextTurn.RIGHT)  # type: ignore[union-attr]
        self.assertEqual(done.turn, NextTurn.RIGHT)  # type: ignore[union-attr]
        self.assertIs(  # type: ignore[union-attr]
            done.event_type,
            LineNavigationEventType.TURN_DONE,
        )

    def test_unrelated_line_is_ignored(self) -> None:
        self.assertIsNone(parse_line_navigation_event("ACK,LINE_START"))

    def test_malformed_turn_event_is_rejected(self) -> None:
        with self.assertRaises(Stm32ProtocolError):
            parse_line_navigation_event("TURN_DONE,NONE")


class ManualEntrySafetyTests(unittest.TestCase):
    def test_without_exact_start_factory_is_never_called(self) -> None:
        calls = 0

        def forbidden_factory() -> Stm32Link:
            nonlocal calls
            calls += 1
            raise AssertionError("hardware factory must not be called")

        output: list[str] = []
        result = run_test(
            argparse.Namespace(turn="LEFT"),
            input_fn=lambda prompt: "start",
            stm32_factory=forbidden_factory,
            output=output.append,
        )

        self.assertEqual(result, 2)
        self.assertEqual(calls, 0)
        self.assertTrue(any("Cancelled" in line for line in output))

    def test_start_sets_turn_before_line_start_and_cleans_up(self) -> None:
        actions: list[object] = []

        class FakeLiveLink:
            is_connected = False

            def connect(self) -> None:
                self.is_connected = True
                actions.append("connect")

            def set_next_turn(self, turn: NextTurn) -> str:
                actions.append(("next_turn", turn))
                return f"ACK,NEXT_TURN,{turn.value}"

            def send_command(self, command: str) -> str:
                actions.append(("send", command))
                return command

            def wait_ack(self, command: str) -> str:
                actions.append(("wait_ack", command))
                return f"ACK,{command}"

            def readline(self) -> bytes:
                raise KeyboardInterrupt

            def stop(self, **kwargs: object) -> None:
                actions.append("stop")

            def close(self) -> None:
                self.is_connected = False
                actions.append("close")

        result = run_test(
            argparse.Namespace(turn="LEFT"),
            input_fn=lambda prompt: "START",
            stm32_factory=FakeLiveLink,  # type: ignore[arg-type]
            output=lambda line: None,
        )

        self.assertEqual(result, 130)
        self.assertEqual(
            actions,
            [
                "connect",
                ("next_turn", NextTurn.LEFT),
                ("send", "LINE_START"),
                ("wait_ack", "LINE_START"),
                "stop",
                "close",
            ],
        )


if __name__ == "__main__":
    unittest.main()
