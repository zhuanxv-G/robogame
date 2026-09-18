"""Hardware-free tests for the complete fixed-route competition mission."""

from __future__ import annotations

import unittest
from collections import deque
from unittest import mock

from competition.full_mission import (
    FIRST_JUNCTION_ADVANCE_SPEED,
    FIRST_JUNCTION_ADVANCE_TIME,
    FIRST_TURN_OMEGA,
    FIRST_TURN_TIME,
    SECOND_JUNCTION_DISTANCE,
    SECOND_JUNCTION_DRIVE_TIME,
    TURN_BACK_TO_LINE_OMEGA,
    TURN_BACK_TO_LINE_TIME,
    TURN_TO_GRAB_OMEGA,
    TURN_TO_GRAB_TIME,
    FullMissionState,
    main,
    run_full_mission,
)
from competition.grab_task import (
    PICK_ARM_STEPS,
    PLACE_ARM_STEPS,
    PREPARE_ARM_STEPS,
    PRE_PICK_SETTLE_STEPS,
    PRE_PLACE_SETTLE_STEPS,
    REST_ARM_STEPS,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _turn_lines(direction: str) -> list[bytes]:
    return [
        b"JUNCTION_DETECTED\r\n",
        f"TURN_STARTED,{direction}\r\n".encode(),
        b"SEARCH_LINE\r\n",
        b"LINE_REACQUIRED\r\n",
        f"TURN_DONE,{direction}\r\n".encode(),
    ]


class FakeStm32Link:
    """Fake that releases route events only after the matching LINE_START."""

    def __init__(
        self,
        clock: FakeClock,
        *,
        first_line_batch: list[bytes] | None = None,
        second_line_batch: list[bytes] | None = None,
        final_line_batch: list[bytes] | None = None,
        interrupt_when_empty: bool = False,
    ) -> None:
        self.clock = clock
        self.is_connected = False
        self.calls: list[object] = []
        self.timed_calls: list[tuple[float, object]] = []
        self.lines: deque[bytes] = deque()
        self.line_start_count = 0
        self.first_line_batch = (
            [b"JUNCTION_DETECTED\r\n"]
            if first_line_batch is None
            else first_line_batch
        )
        self.second_line_batch = (
            [] if second_line_batch is None else second_line_batch
        )
        self.final_line_batch = (
            [b"JUNCTION_DETECTED\r\n"]
            if final_line_batch is None
            else final_line_batch
        )
        self.interrupt_when_empty = interrupt_when_empty

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
        if command == "LINE_START":
            self.line_start_count += 1
            if self.line_start_count == 1:
                self.lines.extend(self.first_line_batch)
            elif self.line_start_count == 2:
                self.lines.extend(self.second_line_batch)
            elif self.line_start_count == 4:
                self.lines.extend(self.final_line_batch)
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


def _sent_commands(link: FakeStm32Link) -> list[str]:
    return [
        str(call[1])
        for call in link.calls
        if isinstance(call, tuple) and call[0] == "send"
    ]


class FullMissionTests(unittest.TestCase):
    def test_complete_route_reuses_grab_segments_and_finishes_stopped(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock)
        link.is_connected = True
        output: list[str] = []
        arm_steps_seen: list[object] = []

        def record_arm_steps(
            stm32: object,
            steps: object,
            **kwargs: object,
        ) -> None:
            self.assertIs(stm32, link)
            arm_steps_seen.append(steps)

        with mock.patch(
            "competition.full_mission.execute_grab_steps",
            side_effect=record_arm_steps,
        ):
            result = run_full_mission(
                link,  # type: ignore[arg-type]
                output=output.append,
                clock=clock,
            )

        self.assertIs(result, FullMissionState.FINISHED)
        commands = _sent_commands(link)
        self.assertEqual(
            [command for command in commands if command.startswith("NEXT_TURN")],
            ["NEXT_TURN,RIGHT", "NEXT_TURN,LEFT", "NEXT_TURN,RIGHT"],
        )
        self.assertEqual(commands.count("LINE_START"), 4)
        self.assertEqual(commands.count("LINE_STOP"), 4)
        self.assertIn(
            f"VEL,0.00,{FIRST_JUNCTION_ADVANCE_SPEED:.2f},0.00",
            commands,
        )
        self.assertIn(
            f"VEL,0.00,0.00,{FIRST_TURN_OMEGA:.2f}",
            commands,
        )
        initial_line_start = commands.index("LINE_START")
        first_line_stop = commands.index("LINE_STOP")
        first_center_advance = commands.index(
            f"VEL,0.00,{FIRST_JUNCTION_ADVANCE_SPEED:.2f},0.00"
        )
        first_fixed_right = commands.index(
            f"VEL,0.00,0.00,{FIRST_TURN_OMEGA:.2f}"
        )
        left_preset = commands.index("NEXT_TURN,LEFT")
        self.assertLess(initial_line_start, first_line_stop)
        self.assertLess(first_line_stop, first_center_advance)
        self.assertLess(first_center_advance, first_fixed_right)
        self.assertLess(first_fixed_right, left_preset)
        self.assertIn("VEL,0.00,-0.80,0.00", commands)
        self.assertIn(
            f"VEL,0.00,0.00,{TURN_TO_GRAB_OMEGA:.2f}",
            commands,
        )
        self.assertIn(
            f"VEL,0.00,0.00,{TURN_BACK_TO_LINE_OMEGA:.2f}",
            commands,
        )
        self.assertEqual(
            arm_steps_seen,
            [
                PREPARE_ARM_STEPS,
                PRE_PICK_SETTLE_STEPS,
                PICK_ARM_STEPS,
                PRE_PLACE_SETTLE_STEPS,
                PLACE_ARM_STEPS,
                REST_ARM_STEPS,
            ],
        )
        self.assertEqual(link.calls[-1], ("stop", True))
        self.assertIn("BUILD_ZONE_REACHED", output)
        self.assertIn("[STATE] FINISHED", output)
        self.assertIn("[MISSION]", output)
        self.assertNotIn("close", link.calls)

        left_preset_at = next(
            timestamp
            for timestamp, call in link.timed_calls
            if call == ("send", "NEXT_TURN,LEFT")
        )
        first_line_stop_at = next(
            timestamp
            for timestamp, call in link.timed_calls
            if call == ("send", "LINE_STOP") and timestamp >= left_preset_at
        )
        self.assertGreaterEqual(
            first_line_stop_at - left_preset_at,
            SECOND_JUNCTION_DRIVE_TIME,
        )
        self.assertLess(
            first_line_stop_at - left_preset_at,
            SECOND_JUNCTION_DRIVE_TIME + 0.25,
        )

    def test_stale_first_left_event_cannot_override_forced_right(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            first_line_batch=[
                b"JUNCTION_DETECTED\r\n",
                b"ADVANCE_START\r\n",
                b"ADVANCE_DONE\r\n",
                b"TURN_PHASE_START\r\n",
                b"TURN_STARTED,LEFT\r\n",
            ],
        )
        link.is_connected = True

        with mock.patch("competition.full_mission.execute_grab_steps"):
            result = run_full_mission(
                link,  # type: ignore[arg-type]
                output=lambda line: None,
                clock=clock,
            )

        self.assertIs(result, FullMissionState.FINISHED)
        commands = _sent_commands(link)
        self.assertIn(
            f"VEL,0.00,0.00,{FIRST_TURN_OMEGA:.2f}",
            commands,
        )
        self.assertNotIn(("stop", False), link.calls)

    def test_stale_final_left_event_cannot_override_forced_right(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            final_line_batch=[
                b"JUNCTION_DETECTED\r\n",
                b"ADVANCE_START\r\n",
                b"ADVANCE_DONE\r\n",
                b"TURN_PHASE_START\r\n",
                b"TURN_STARTED,LEFT\r\n",
            ],
        )
        link.is_connected = True
        output: list[str] = []

        with mock.patch("competition.full_mission.execute_grab_steps"):
            result = run_full_mission(
                link,  # type: ignore[arg-type]
                output=output.append,
                clock=clock,
            )

        self.assertIs(result, FullMissionState.FINISHED)
        commands = _sent_commands(link)
        self.assertEqual(commands.count("LINE_STOP"), 4)
        self.assertIn(
            f"VEL,0.00,0.00,{FIRST_TURN_OMEGA:.2f}",
            commands,
        )
        self.assertIn("Force right turn at final junction", output)
        self.assertNotIn(("stop", False), link.calls)

    def test_fixed_route_parameters_use_current_speed_baseline(self) -> None:
        self.assertEqual(FIRST_JUNCTION_ADVANCE_TIME, 2.2)
        self.assertEqual(FIRST_JUNCTION_ADVANCE_SPEED, 0.80)
        self.assertEqual(FIRST_TURN_TIME, 11.5)
        self.assertEqual(FIRST_TURN_OMEGA, -0.80)
        self.assertEqual(SECOND_JUNCTION_DISTANCE, 0.60)
        self.assertAlmostEqual(
            SECOND_JUNCTION_DRIVE_TIME,
            SECOND_JUNCTION_DISTANCE / 0.118,
        )
        self.assertEqual(TURN_TO_GRAB_TIME, 11.5)
        self.assertEqual(TURN_TO_GRAB_OMEGA, 0.80)
        self.assertEqual(TURN_BACK_TO_LINE_TIME, 11.5)
        self.assertEqual(TURN_BACK_TO_LINE_OMEGA, -0.80)

    def test_detected_second_junction_uses_existing_left_turn(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            second_line_batch=_turn_lines("LEFT"),
        )
        link.is_connected = True

        with mock.patch("competition.full_mission.execute_grab_steps"):
            result = run_full_mission(
                link,  # type: ignore[arg-type]
                output=lambda line: None,
                clock=clock,
            )

        self.assertIs(result, FullMissionState.FINISHED)
        commands = _sent_commands(link)
        self.assertNotIn(
            f"VEL,0.00,0.00,{TURN_TO_GRAB_OMEGA:.2f}",
            commands,
        )
        self.assertEqual(commands.count("LINE_STOP"), 4)

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

    def test_main_connects_and_closes_its_single_link(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(clock)

        with mock.patch("competition.full_mission.execute_grab_steps"):
            result = main(
                [],
                stm32_factory=lambda: link,  # type: ignore[arg-type]
                input_fn=lambda prompt: "START",
                output=lambda line: None,
                clock=clock,
            )

        self.assertEqual(result, 0)
        self.assertEqual(link.calls[0], "connect")
        self.assertEqual(link.calls[-1], "close")
        self.assertFalse(link.is_connected)

    def test_ctrl_c_requests_stop_and_releases_resource(self) -> None:
        clock = FakeClock()
        link = FakeStm32Link(
            clock,
            first_line_batch=[],
            interrupt_when_empty=True,
        )

        result = main(
            [],
            stm32_factory=lambda: link,  # type: ignore[arg-type]
            input_fn=lambda prompt: "START",
            output=lambda line: None,
            clock=clock,
        )

        self.assertEqual(result, 130)
        self.assertIn(("stop", False), link.calls)
        self.assertEqual(link.calls[-1], "close")
        self.assertFalse(link.is_connected)


if __name__ == "__main__":
    unittest.main()
