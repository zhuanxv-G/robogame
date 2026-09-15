"""Hardware-free coverage for the line-to-grab mission controller."""

from __future__ import annotations

import unittest
from collections import deque
from unittest import mock

from hardware.stm32_link import Stm32ProtocolError
from competition.grab_mission import (
    BACKOFF_DISTANCE,
    BACKOFF_TIME,
    LINE_DURATION,
    POST_PICK_RETREAT_DISTANCE,
    POST_PICK_RETREAT_TIME,
    REAL_SPEED,
    TARGET_DISTANCE,
    MissionPhase,
    main,
    run_grab_mission,
)
from competition.grab_task import (
    PICK_ARM_STEPS,
    PLACE_ARM_STEPS,
    PREPARE_ARM_STEPS,
    PRE_PICK_SETTLE_STEPS,
    PRE_PLACE_SETTLE_STEPS,
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
        self.timed_calls: list[tuple[float, object]] = []

    def connect(self) -> None:
        self.is_connected = True
        self.calls.append("connect")

    def close(self) -> None:
        self.is_connected = False
        self.calls.append("close")

    def send_command(self, command: str) -> str:
        call = ("send", command)
        self.calls.append(call)
        self.timed_calls.append((self.clock(), call))
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
        self.assertEqual(LINE_DURATION, TARGET_DISTANCE / REAL_SPEED)
        self.assertAlmostEqual(LINE_DURATION, 3.220338983, places=6)
        self.assertEqual(BACKOFF_TIME, BACKOFF_DISTANCE / REAL_SPEED)
        self.assertEqual(
            POST_PICK_RETREAT_TIME,
            POST_PICK_RETREAT_DISTANCE / REAL_SPEED,
        )

    def test_full_mission_success_shares_link_with_grab(self) -> None:
        clock = FakeClock()
        output: list[str] = []
        link = FakeStm32Link(clock, list(JUNCTION_LINES))
        link.is_connected = True

        arm_steps_seen: list[object] = []

        def record_arm_steps(
            stm32: object,
            steps: object,
            **kwargs: object,
        ) -> None:
            self.assertIs(stm32, link)
            arm_steps_seen.append(steps)
            link.calls.append(("arm", steps))

        with mock.patch(
            "competition.grab_mission.execute_grab_steps",
            side_effect=record_arm_steps,
        ):
            result = run_grab_mission(
                link,
                output=output.append,
                clock=clock,
            )

        self.assertIs(result, MissionPhase.DONE)
        sent = _sent_commands(link)
        # Startup order: RIGHT preset comes after the safety STOP ACK.
        self.assertEqual(sent[:5], [
            "NEXT_TURN,RIGHT",
            "LINE_START",
            "LINE_STOP",
            "LINE_START",
            "LINE_STOP",
        ])
        self.assertIn("VEL,0.00,-0.80,0.00", sent)
        self.assertNotIn("VEL,0.00,0.80,0.00", sent)
        # Startup, both backward segments, and final mission STOP.
        self.assertEqual(
            [call for call in link.calls if call == ("stop", True)],
            [("stop", True)] * 4,
        )
        # The caller-supplied link is left open for reuse.
        self.assertTrue(link.is_connected)
        self.assertNotIn("close", link.calls)
        self.assertEqual(
            arm_steps_seen,
            [
                PREPARE_ARM_STEPS,
                PRE_PICK_SETTLE_STEPS,
                PICK_ARM_STEPS,
                PRE_PLACE_SETTLE_STEPS,
                PLACE_ARM_STEPS,
            ],
        )
        line_stop_indices = [
            index
            for index, call in enumerate(link.calls)
            if call == ("send", "LINE_STOP")
        ]
        line_start_indices = [
            index
            for index, call in enumerate(link.calls)
            if call == ("send", "LINE_START")
        ]
        self.assertEqual(len(line_stop_indices), 2)
        self.assertEqual(len(line_start_indices), 2)
        turn_pause_index, wall_stop_index = line_stop_indices
        _, wall_approach_start_index = line_start_indices
        prepare_index = link.calls.index(("arm", PREPARE_ARM_STEPS))
        pre_pick_settle_index = link.calls.index(
            ("arm", PRE_PICK_SETTLE_STEPS)
        )
        pick_index = link.calls.index(("arm", PICK_ARM_STEPS))
        retreat_settle_index = link.calls.index(
            ("arm", PRE_PLACE_SETTLE_STEPS)
        )
        place_index = link.calls.index(("arm", PLACE_ARM_STEPS))
        backward_vel_indices = [
            index
            for index, call in enumerate(link.calls)
            if call == ("send", "VEL,0.00,-0.80,0.00")
        ]
        self.assertLess(turn_pause_index, prepare_index)
        self.assertLess(prepare_index, wall_approach_start_index)
        self.assertLess(wall_approach_start_index, wall_stop_index)
        self.assertTrue(
            any(
                prepare_index < index < pre_pick_settle_index
                for index in backward_vel_indices
            )
        )
        self.assertLess(pre_pick_settle_index, pick_index)
        self.assertTrue(
            any(
                pick_index < index < retreat_settle_index
                for index in backward_vel_indices
            )
        )
        self.assertLess(pick_index, retreat_settle_index)
        self.assertLess(retreat_settle_index, place_index)
        for expected in (
            "=== GRAB MISSION START ===",
            "START LINE FOLLOW",
            "Waiting junction...",
            "JUNCTION DETECTED",
            "TURN COMPLETE",
            "LINE REACQUIRED",
            "Pause line following for arm preparation",
            "Prepare arm before wall approach",
            "Resume line following to wall",
            "Continue line following",
            "Wall reached by line following",
            "Prepare arm",
            "Backoff 5cm",
            "Pick object",
            "Post-pick retreat 30cm",
            "Place object",
            "Grab and placement complete",
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

        with mock.patch(
            "competition.grab_mission.execute_grab_steps"
        ) as arm_spy:
            with self.assertRaises(Stm32ProtocolError):
                run_grab_mission(link, output=output.append, clock=clock)

        arm_spy.assert_not_called()
        self.assertIn(("stop", False), link.calls)
        self.assertTrue(
            any("Expected TURN_STARTED,RIGHT" in line for line in output)
        )
        self.assertTrue(
            any("=== MISSION ABORTED:" in line for line in output)
        )

    def test_line_duration_starts_when_line_follow_resumes_after_prepare(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            [
                b"JUNCTION_DETECTED\r\n",
                b"TURN_STARTED,RIGHT\r\n",
                b"LINE_REACQUIRED\r\n",
                b"",
                b"",
                b"",
                b"",
                b"TURN_DONE,RIGHT\r\n",
            ],
        )
        link.is_connected = True

        with mock.patch("competition.grab_mission.execute_grab_steps"):
            run_grab_mission(link, output=lambda line: None, clock=clock)

        line_start_times = [
            timestamp
            for timestamp, call in link.timed_calls
            if call == ("send", "LINE_START")
        ]
        line_stop_times = [
            timestamp
            for timestamp, call in link.timed_calls
            if call == ("send", "LINE_STOP")
        ]
        wall_approach_started_at = line_start_times[-1]
        wall_stop_at = line_stop_times[-1]
        self.assertGreaterEqual(
            wall_stop_at,
            wall_approach_started_at + LINE_DURATION,
        )
        self.assertLess(
            wall_stop_at,
            wall_approach_started_at + LINE_DURATION + 0.25,
        )

    def test_arm_prepare_failure_stops_without_resuming_line_follow(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock, list(JUNCTION_LINES))
        link.is_connected = True

        with mock.patch(
            "competition.grab_mission.execute_grab_steps",
            side_effect=Stm32ProtocolError("STM32 error: ERR,SERVO_SET"),
        ):
            with self.assertRaises(Stm32ProtocolError):
                run_grab_mission(link, output=lambda line: None, clock=clock)

        sent = _sent_commands(link)
        self.assertEqual(sent.count("LINE_START"), 1)
        self.assertEqual(sent.count("LINE_STOP"), 1)
        self.assertIn(("stop", False), link.calls)

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

        with mock.patch("competition.grab_mission.execute_grab_steps") as arm_spy:
            result = main(
                [],
                stm32_factory=lambda: link,  # type: ignore[arg-type]
                input_fn=lambda prompt: "START",
                output=lambda line: None,
                clock=clock,
            )

        self.assertEqual(result, 0)
        self.assertIn("connect", link.calls)
        self.assertEqual(arm_spy.call_count, 5)
        # main closes the link it created, even on the success path.
        self.assertEqual(link.calls[-1], "close")
        self.assertFalse(link.is_connected)


if __name__ == "__main__":
    unittest.main()
