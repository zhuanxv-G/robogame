"""Hardware-free tests for the old-candidate arm task packaging."""

from __future__ import annotations

import unittest
from collections import deque

from competition.grab_task import (
    GRAB_SEQUENCE,
    execute_grab_steps,
    grab_orange,
)
from hardware.stm32_link import (
    Stm32NotConnectedError,
    Stm32ProtocolError,
)
from tests.test_old_grab_sequence import ActionKind, OLD_SEQUENCE


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
        lines: list[bytes] | None = None,
        *,
        interrupt_when_read: bool = False,
    ) -> None:
        self.clock = clock
        self.lines = deque(lines or [])
        self.is_connected = True
        self.calls: list[object] = []
        self.interrupt_when_read = interrupt_when_read

    def readline(self) -> bytes:
        self.clock.advance(0.05)
        if self.interrupt_when_read:
            raise KeyboardInterrupt
        if self.lines:
            return self.lines.popleft()
        return b""

    def send_command(self, command: str) -> str:
        self.calls.append(("send", command))
        return command

    def wait_ack(self, command: str, timeout: float = 2.0) -> str:
        self.calls.append(("wait_ack", command, timeout))
        return f"ACK,{command}"

    def wait_done(self, expected: str, timeout: float = 10.0) -> str:
        self.calls.append(("wait_done", expected, timeout))
        return expected

    def servo_set(self, index: int, angle: int, timeout: float = 2.0) -> str:
        command = f"SERVO_SET,{index},{angle}"
        self.calls.append(("servo_set", index, angle, timeout))
        return f"ACK,{command}"

    def stop(self, **kwargs: object) -> str:
        require_ack = kwargs.get("require_ack")
        self.calls.append(("stop", require_ack))
        return "ACK,STOP"

    def close(self) -> None:
        self.is_connected = False
        self.calls.append("close")


class GrabTaskTests(unittest.TestCase):
    def test_sequence_matches_every_non_chassis_old_action_in_order(self) -> None:
        expected = [
            action.description
            for action in OLD_SEQUENCE
            if action.kind is not ActionKind.CHASSIS_DISTANCE
        ]
        actual = [step.source for step in GRAB_SEQUENCE]

        self.assertEqual(len(expected), 24)
        self.assertEqual(actual, expected)

    def test_complete_task_logs_ack_done_and_uses_two_safety_stops(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock)
        output: list[str] = []

        grab_orange(
            link,  # type: ignore[arg-type]
            output=output.append,
            clock=clock,
        )

        self.assertEqual(
            [call for call in link.calls if call == ("stop", True)],
            [("stop", True), ("stop", True)],
        )
        stepper_commands = [
            call[1]
            for call in link.calls
            if isinstance(call, tuple)
            and call[0] == "send"
            and str(call[1]).startswith("STEPPER_MOVE,")
        ]
        self.assertEqual(
            stepper_commands,
            [
                "STEPPER_MOVE,3000,800",
                "STEPPER_MOVE,-3000,800",
                "STEPPER_MOVE,5000,800",
                "STEPPER_MOVE,-5000,800",
            ],
        )
        self.assertTrue(any("RX ACK:" in line for line in output))
        self.assertTrue(any("RX DONE: STEPPER_MOVE_DONE" in line for line in output))
        self.assertTrue(any("Settle after backoff" in line for line in output))
        self.assertTrue(any("Settle after retreat" in line for line in output))

    def test_async_err_requests_emergency_stop(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock, [b"ERR,ARM_FAULT\r\n"])

        with self.assertRaises(Stm32ProtocolError):
            grab_orange(
                link,  # type: ignore[arg-type]
                output=lambda line: None,
                clock=clock,
            )

        self.assertIn(("stop", False), link.calls)

    def test_segment_requires_connected_link(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock)
        link.is_connected = False

        with self.assertRaises(Stm32NotConnectedError):
            execute_grab_steps(
                link,  # type: ignore[arg-type]
                GRAB_SEQUENCE,
                output=lambda line: None,
                clock=clock,
            )

    def test_ctrl_c_requests_emergency_stop(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock, interrupt_when_read=True)

        with self.assertRaises(KeyboardInterrupt):
            grab_orange(
                link,  # type: ignore[arg-type]
                output=lambda line: None,
                clock=clock,
            )

        self.assertIn(("stop", False), link.calls)


if __name__ == "__main__":
    unittest.main()
