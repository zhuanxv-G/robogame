"""Hardware-free tests for ordered multi-junction route execution."""

from __future__ import annotations

import sys
import unittest
from collections import deque
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    LineNavigationEvent,
    LineNavigationEventType,
    NextTurn,
    Stm32ProtocolError,
)
from navigation.route_action_resolver import RouteAction  # noqa: E402
from navigation.route_executor import (  # noqa: E402
    RouteExecutor,
    RouteExecutorConfig,
    RouteExecutorEventError,
    RouteExecutorNavigationError,
    RouteExecutorPlanningError,
    RouteExecutorState,
    RouteExecutorTimeoutError,
)
from navigation.topological_navigation import TopologicalLocation  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class FakeTopologicalNavigator:
    def __init__(
        self,
        route: list[TopologicalLocation] | None,
    ) -> None:
        self.route = route
        self.plan_calls: list[tuple[object, object]] = []
        self.update_calls: list[int] = []

    def plan_route(
        self,
        start: TopologicalLocation | str,
        target: TopologicalLocation | str,
    ) -> list[TopologicalLocation]:
        self.plan_calls.append((start, target))
        if self.route is None:
            raise RuntimeError("no route")
        return list(self.route)

    def update_from_tag(self, tag_id: int) -> TopologicalLocation | None:
        self.update_calls.append(tag_id)
        if tag_id == 4:
            return TopologicalLocation.RIGHT_ORANGE_MATERIAL
        return None


class FakeRouteActionResolver:
    def __init__(self, actions: list[RouteAction]) -> None:
        self.actions = actions
        self.calls: list[tuple[tuple[object, ...], object]] = []

    def resolve_route_actions(
        self,
        route: list[TopologicalLocation] | tuple[TopologicalLocation, ...],
        *,
        current_heading: object = None,
    ) -> list[RouteAction]:
        self.calls.append((tuple(route), current_heading))
        return list(self.actions)


class FakeNavigationController:
    def __init__(self, update_states: list[object] | None = None) -> None:
        self._state = "IDLE"
        self._update_states = deque(update_states or [])
        self.start_calls: list[tuple[int, float, int]] = []
        self.continue_calls: list[tuple[int, float, int]] = []
        self.update_calls = 0

    @property
    def current_state(self) -> str:
        return self._state

    def start_line_to_tag(
        self,
        tag_id: int,
        arrival_area: float,
        confirm_frames: int,
    ) -> str:
        self.start_calls.append((tag_id, arrival_area, confirm_frames))
        self._state = "LINE_FOLLOWING"
        return self._state

    def continue_line_to_tag(
        self,
        tag_id: int,
        arrival_area: float,
        confirm_frames: int,
    ) -> str:
        self.continue_calls.append((tag_id, arrival_area, confirm_frames))
        self._state = "LINE_FOLLOWING"
        return self._state

    def update(self) -> str:
        self.update_calls += 1
        if self._update_states:
            next_state = self._update_states.popleft()
            if isinstance(next_state, Exception):
                raise next_state
            self._state = str(next_state)
        return self._state


class FakeStm32Link:
    def __init__(self, lines: list[bytes] | None = None) -> None:
        self.lines = deque(lines or [])
        self.calls: list[tuple[object, ...]] = []
        self.stop_calls = 0

    def set_next_turn(
        self,
        turn: NextTurn | str,
        timeout: float = 2.0,
    ) -> str:
        normalized = turn if isinstance(turn, NextTurn) else NextTurn(turn)
        self.calls.append(("next_turn", normalized))
        return f"ACK,NEXT_TURN,{normalized.value}"

    def send_command(self, command: str) -> str:
        self.calls.append(("send", command))
        return command

    def wait_ack(self, command: str, timeout: float = 2.0) -> str:
        self.calls.append(("wait_ack", command))
        return f"ACK,{command}"

    def readline(self) -> bytes:
        return self.lines.popleft() if self.lines else b""

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
        debug_label: str = "stop",
    ) -> str | None:
        self.stop_calls += 1
        self.calls.append(("stop", require_ack, debug_label))
        return "ACK,STOP" if require_ack else None


def event(
    event_type: LineNavigationEventType,
    turn: NextTurn | None = None,
) -> LineNavigationEvent:
    return LineNavigationEvent(event_type, turn)


class RouteExecutorTests(unittest.TestCase):
    def make_executor(
        self,
        actions: list[RouteAction],
        *,
        route: list[TopologicalLocation] | None = None,
        update_states: list[object] | None = None,
        lines: list[bytes] | None = None,
        clock: FakeClock | None = None,
    ) -> tuple[
        RouteExecutor,
        FakeTopologicalNavigator,
        FakeRouteActionResolver,
        FakeNavigationController,
        FakeStm32Link,
        FakeClock,
    ]:
        route_nodes = route
        if route_nodes is None:
            route_nodes = [
                TopologicalLocation.START_ZONE,
                TopologicalLocation.RIGHT_ORANGE_MATERIAL,
            ]
        navigator = FakeTopologicalNavigator(route_nodes)
        resolver = FakeRouteActionResolver(actions)
        navigation = FakeNavigationController(update_states)
        stm32 = FakeStm32Link(lines)
        fake_clock = clock or FakeClock()
        executor = RouteExecutor(
            navigator,
            resolver,
            navigation,
            stm32,
            config=RouteExecutorConfig(
                junction_wait_timeout_s=10.0,
                turn_done_timeout_s=5.0,
                final_tag_timeout_s=20.0,
                fault_stop_timeout_s=0.1,
            ),
            clock=fake_clock,
        )
        executor.prepare_route(
            route_nodes[0],
            route_nodes[-1],
            target_tag=4,
            arrival_area=10000,
            confirm_frames=3,
        )
        return (
            executor,
            navigator,
            resolver,
            navigation,
            stm32,
            fake_clock,
        )

    @staticmethod
    def complete_action(executor: RouteExecutor, turn: NextTurn) -> None:
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.JUNCTION_DETECTED)
        )
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.TURN_STARTED, turn)
        )
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.TURN_DONE, turn)
        )

    def test_right_then_left_enters_final_tag_stage(self) -> None:
        lines = [
            b"JUNCTION_DETECTED\r\n",
            b"TURN_STARTED,RIGHT\r\n",
            b"TURN_DONE,RIGHT\r\n",
            b"JUNCTION_DETECTED\r\n",
            b"TURN_STARTED,LEFT\r\n",
            b"TURN_DONE,LEFT\r\n",
        ]
        executor, _, _, navigation, stm32, _ = self.make_executor(
            [RouteAction.RIGHT, RouteAction.LEFT],
            lines=lines,
        )

        self.assertIs(executor.start(), RouteExecutorState.WAITING_JUNCTION)
        for _ in lines:
            executor.poll()

        self.assertIs(executor.state, RouteExecutorState.ROUTE_TURNS_DONE)
        self.assertEqual(executor.current_action_index, 2)
        self.assertEqual(
            [call for call in stm32.calls if call[0] == "next_turn"],
            [
                ("next_turn", NextTurn.RIGHT),
                ("next_turn", NextTurn.LEFT),
            ],
        )
        self.assertIs(executor.poll(), RouteExecutorState.FINAL_TAG_APPROACH)
        self.assertEqual(navigation.continue_calls, [(4, 10000.0, 3)])
        self.assertEqual(navigation.start_calls, [])

    def test_next_turn_is_set_only_after_matching_turn_done(self) -> None:
        executor, _, _, _, stm32, _ = self.make_executor(
            [RouteAction.RIGHT, RouteAction.STRAIGHT, RouteAction.LEFT]
        )
        executor.start()

        self.assertEqual(self._turn_calls(stm32), [NextTurn.RIGHT])
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.JUNCTION_DETECTED)
        )
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.TURN_STARTED, NextTurn.RIGHT)
        )
        self.assertEqual(self._turn_calls(stm32), [NextTurn.RIGHT])

        executor.handle_line_navigation_event(
            event(LineNavigationEventType.TURN_DONE, NextTurn.RIGHT)
        )
        self.assertEqual(
            self._turn_calls(stm32),
            [NextTurn.RIGHT, NextTurn.STRAIGHT],
        )
        self.complete_action(executor, NextTurn.STRAIGHT)
        self.assertEqual(
            self._turn_calls(stm32),
            [NextTurn.RIGHT, NextTurn.STRAIGHT, NextTurn.LEFT],
        )
        self.complete_action(executor, NextTurn.LEFT)
        self.assertIs(executor.state, RouteExecutorState.ROUTE_TURNS_DONE)

    def test_mismatched_turn_done_faults_and_stops(self) -> None:
        executor, _, _, _, stm32, _ = self.make_executor([RouteAction.RIGHT])
        executor.start()
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.JUNCTION_DETECTED)
        )
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.TURN_STARTED, NextTurn.RIGHT)
        )

        with self.assertRaisesRegex(RouteExecutorEventError, "expected RIGHT"):
            executor.handle_line_navigation_event(
                event(LineNavigationEventType.TURN_DONE, NextTurn.LEFT)
            )

        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(executor.current_action_index, 0)
        self.assertEqual(stm32.stop_calls, 1)

    def test_duplicate_turn_done_cannot_advance_again(self) -> None:
        executor, _, _, _, stm32, _ = self.make_executor(
            [RouteAction.RIGHT, RouteAction.LEFT]
        )
        executor.start()
        self.complete_action(executor, NextTurn.RIGHT)

        with self.assertRaises(RouteExecutorEventError):
            executor.handle_line_navigation_event(
                event(LineNavigationEventType.TURN_DONE, NextTurn.RIGHT)
            )

        self.assertEqual(executor.current_action_index, 1)
        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_turn_done_timeout_faults_and_stops(self) -> None:
        clock = FakeClock()
        executor, _, _, _, stm32, _ = self.make_executor(
            [RouteAction.RIGHT],
            clock=clock,
        )
        executor.start()
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.JUNCTION_DETECTED)
        )
        executor.handle_line_navigation_event(
            event(LineNavigationEventType.TURN_STARTED, NextTurn.RIGHT)
        )
        clock.now = 5.1

        with self.assertRaises(RouteExecutorTimeoutError):
            executor.check_timeout()

        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_turns_done_without_tag_arrival_is_not_finished(self) -> None:
        executor, navigator, _, navigation, _, _ = self.make_executor(
            [RouteAction.RIGHT],
            update_states=["LINE_FOLLOWING"],
        )
        executor.start()
        self.complete_action(executor, NextTurn.RIGHT)
        executor.poll()

        self.assertIs(executor.poll(), RouteExecutorState.FINAL_TAG_APPROACH)
        self.assertEqual(navigation.update_calls, 1)
        self.assertEqual(navigator.update_calls, [])

    def test_final_tag_arrival_updates_topology_then_finishes(self) -> None:
        executor, navigator, _, _, _, _ = self.make_executor(
            [RouteAction.RIGHT],
            update_states=["ARRIVED"],
        )
        executor.start()
        self.complete_action(executor, NextTurn.RIGHT)
        executor.poll()

        self.assertIs(executor.poll(), RouteExecutorState.ARRIVED)
        self.assertEqual(navigator.update_calls, [4])
        self.assertIs(executor.poll(), RouteExecutorState.FINISHED)

    def test_navigation_fault_propagates_and_stops(self) -> None:
        executor, _, _, _, stm32, _ = self.make_executor(
            [],
            update_states=["FAULT"],
            route=[TopologicalLocation.RIGHT_ORANGE_MATERIAL],
        )
        executor.start()

        with self.assertRaises(RouteExecutorNavigationError):
            executor.poll()

        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_apriltag_exception_faults_and_stops(self) -> None:
        executor, _, _, _, stm32, _ = self.make_executor(
            [],
            update_states=[RuntimeError("camera failed")],
            route=[TopologicalLocation.RIGHT_ORANGE_MATERIAL],
        )
        executor.start()

        with self.assertRaisesRegex(RuntimeError, "camera failed"):
            executor.poll()

        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_stm32_protocol_error_faults_and_stops(self) -> None:
        executor, _, _, _, stm32, _ = self.make_executor([RouteAction.RIGHT])
        executor.start()

        with self.assertRaisesRegex(Stm32ProtocolError, "TURN_TIMEOUT"):
            executor.process_stm32_line("ERR,TURN_TIMEOUT")

        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_empty_navigator_route_fails_explicitly(self) -> None:
        navigator = FakeTopologicalNavigator([])
        resolver = FakeRouteActionResolver([])
        navigation = FakeNavigationController()
        stm32 = FakeStm32Link()
        executor = RouteExecutor(navigator, resolver, navigation, stm32)

        with self.assertRaisesRegex(
            RouteExecutorPlanningError,
            "empty route",
        ):
            executor.prepare_route(
                TopologicalLocation.START_ZONE,
                TopologicalLocation.RIGHT_ORANGE_MATERIAL,
                target_tag=4,
                arrival_area=10000,
                confirm_frames=3,
            )

        self.assertIs(executor.state, RouteExecutorState.FAULT)
        self.assertEqual(stm32.stop_calls, 1)

    def test_idle_status_does_not_report_route_turns_done(self) -> None:
        executor = RouteExecutor(
            FakeTopologicalNavigator(None),
            FakeRouteActionResolver([]),
            FakeNavigationController(),
            FakeStm32Link(),
        )

        status = executor.get_status()

        self.assertIs(status["state"], RouteExecutorState.IDLE)
        self.assertFalse(status["route_turns_done"])

    def test_no_junction_route_starts_final_tag_approach_directly(self) -> None:
        executor, _, _, navigation, stm32, _ = self.make_executor(
            [],
            route=[TopologicalLocation.RIGHT_ORANGE_MATERIAL],
        )

        self.assertIs(executor.start(), RouteExecutorState.FINAL_TAG_APPROACH)
        self.assertEqual(navigation.start_calls, [(4, 10000.0, 3)])
        self.assertEqual(navigation.continue_calls, [])
        self.assertEqual(self._turn_calls(stm32), [])
        self.assertFalse(
            any(call[:2] == ("send", "LINE_START") for call in stm32.calls)
        )

    @staticmethod
    def _turn_calls(stm32: FakeStm32Link) -> list[NextTurn]:
        return [
            call[1]
            for call in stm32.calls
            if call[0] == "next_turn"
        ]


if __name__ == "__main__":
    unittest.main()
