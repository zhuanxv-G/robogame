"""Hardware-free coverage for the line-to-grab mission controller."""

from __future__ import annotations

import unittest
from collections import deque
from unittest import mock

from hardware.stm32_link import Stm32ProtocolError
from competition.grab_mission import (
    BACKOFF_DISTANCE,
    BACKOFF_TIME,
    GRAB_AREA_APPROACH_TIME,
    REAL_SPEED,
    TARGET_DISTANCE,
    WALL_APPROACH_TIME,
    MissionPhase,
    main,
    run_grab_mission,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeStm32Link:
    """Minimal fake for the Stm32Link surface used by the mission."""

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


JUNCTION_LINES = [
    b"JUNCTION_DETECTED\r\n",
    b"TURN_STARTED,RIGHT\r\n",
    b"SEARCH_LINE\r\n",
    b"LINE_REACQUIRED\r\n",
    b"TURN_DONE,RIGHT\r\n",
]


def _sent_commands(link: FakeStm32Link) -> list[str]:
    return [
        call[1]
        for call in link.calls
        if isinstance(call, tuple) and call[0] == "send"
    ]


class GrabMissionTests(unittest.TestCase):
    def test_duration_constants_derive_from_distance_and_real_speed(self) -> None:
        self.assertEqual(WALL_APPROACH_TIME, TARGET_DISTANCE / REAL_SPEED)
        self.assertAlmostEqual(WALL_APPROACH_TIME, 3.220338983, places=6)
        self.assertEqual(GRAB_AREA_APPROACH_TIME, TARGET_DISTANCE / REAL_SPEED)
        self.assertEqual(BACKOFF_TIME, BACKOFF_DISTANCE / REAL_SPEED)

    def test_full_mission_success_shares_link_with_grab(self) -> None:
        clock = FakeClock()
        output: list[str] = []
        link = FakeStm32Link(clock, list(JUNCTION_LINES))
        link.is_connected = True

        with mock.patch("competition.grab_mission.grab_orange") as grab_spy:
            result = run_grab_mission(
                link,
                output=output.append,
                clock=clock,
            )

        self.assertIs(result, MissionPhase.DONE)
        sent = _sent_commands(link)
        # Startup order: RIGHT preset comes after the safety STOP ACK.
        self.assertEqual(sent[:3], [
            "NEXT_TURN,RIGHT",
            "LINE_START",
            "LINE_STOP",
        ])
        self.assertIn("VEL,0.00,0.80,0.00", sent)
        self.assertIn("VEL,0.00,-0.80,0.00", sent)
        self.assertGreater(sent.count("VEL,0.00,0.80,0.00"), 1)
        # Three confirmed STOPS: startup, wall approach end, backoff end.
        self.assertEqual(
            [call for call in link.calls if call == ("stop", True)],
            [("stop", True)] * 3,
        )
        # The caller-supplied link is left open for reuse.
        self.assertTrue(link.is_connected)
        self.assertNotIn("close", link.calls)
        # The verified arm task receives the shared connection.
        grab_spy.assert_called_once()
        self.assertIs(grab_spy.call_args.kwargs["stm32"], link)
        for expected in (
            "=== GRAB MISSION START ===",
            "START LINE FOLLOW",
            "Waiting junction...",
            "JUNCTION DETECTED",
            "TURN COMPLETE",
            "LINE REACQUIRED",
            "Continue line following",
            "Wall approach:",
            "distance=0.38m",
            "time=3.22s",
            "Wall reached",
            "Backoff 5cm",
            "Start arm grab",
            "Grab complete",
            "=== MISSION SUCCESS ===",
        ):
            self.assertTrue(
                any(expected in line for line in output),
                f"missing output: {expected}",
            )

    def test_wrong_turn_direction_enters_error_and_stops(self) -> None:
        clock = FakeClock()
        output: list[str] = []
        link = FakeStm32Link(
            clock,
            [b"JUNCTION_DETECTED\r\n", b"TURN_STARTED,LEFT\r\n"],
        )
        link.is_connected = True

        with mock.patch("competition.grab_mission.grab_orange") as grab_spy:
            with self.assertRaises(Stm32ProtocolError):
                run_grab_mission(link, output=output.append, clock=clock)

        grab_spy.assert_not_called()
        self.assertIn(("stop", False), link.calls)
        self.assertTrue(
            any("Expected TURN_STARTED,RIGHT" in line for line in output)
        )
        self.assertTrue(
            any("=== MISSION ABORTED:" in line for line in output)
        )

    def test_out_of_order_turn_event_before_junction_aborts(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            [b"TURN_STARTED,RIGHT\r\n"],
        )
        link.is_connected = True
        output: list[str] = []

        with self.assertRaises(Stm32ProtocolError):
            run_grab_mission(link, output=output.append, clock=clock)

        self.assertIn(("stop", False), link.calls)
        self.assertTrue(
            any("Out-of-order event before JUNCTION_DETECTED" in line
                for line in output)
        )

    def test_err_and_ctrl_c_both_request_cleanup_stop(self) -> None:
        scenarios = (
            (
                [b"ERR,LINE_SENSOR_IO\r\n"],
                False,
                Stm32ProtocolError,
            ),
            ([], True, KeyboardInterrupt),
        )
        for lines, interrupt_when_empty, expected_error in scenarios:
            with self.subTest(expected_error=expected_error):
                clock = FakeClock()
                link = FakeStm32Link(
                    clock,
                    lines,
                    interrupt_when_empty=interrupt_when_empty,
                )
                link.is_connected = True
                output: list[str] = []

                with self.assertRaises(expected_error):
                    run_grab_mission(
                        link,
                        output=output.append,
                        clock=clock,
                    )

                self.assertIn(("stop", False), link.calls)
                self.assertTrue(
                    any("=== MISSION ABORTED" in line for line in output)
                )

    def test_dry_run_never_constructs_hardware(self) -> None:
        calls = 0

        def forbidden_factory() -> FakeStm32Link:
            nonlocal calls
            calls += 1
            raise AssertionError("hardware must not be constructed")

        result = main(
            ["--dry-run"],
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

        result = main(
            [],
            stm32_factory=forbidden_factory,  # type: ignore[arg-type]
            input_fn=lambda prompt: "start",
            output=lambda line: None,
        )

        self.assertEqual(result, 2)
        self.assertEqual(calls, 0)

    def test_main_connects_the_link_it_created_before_running(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock, list(JUNCTION_LINES))

        with mock.patch("competition.grab_mission.grab_orange") as grab_spy:
            result = main(
                [],
                stm32_factory=lambda: link,  # type: ignore[arg-type]
                input_fn=lambda prompt: "START",
                output=lambda line: None,
                clock=clock,
            )

        self.assertEqual(result, 0)
        self.assertIn("connect", link.calls)
        grab_spy.assert_called_once()
        # main closes the link it created, even on the success path.
        self.assertEqual(link.calls[-1], "close")
        self.assertFalse(link.is_connected)


if __name__ == "__main__":
    unittest.main()
