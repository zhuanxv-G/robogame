"""Single-reader execution of topology turns followed by Tag arrival."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Protocol, TypedDict

from hardware.stm32_link import (
    LineNavigationEvent,
    LineNavigationEventType,
    NextTurn,
    Stm32ProtocolError,
    parse_line_navigation_event,
)

from .field_model import TAG_TOPOLOGY_LOCATIONS
from .route_action_resolver import Heading, RouteAction
from .topological_navigation import TopologicalLocation


DEFAULT_JUNCTION_WAIT_TIMEOUT_S = 30.0
DEFAULT_TURN_DONE_TIMEOUT_S = 5.0
DEFAULT_FINAL_TAG_TIMEOUT_S = 30.0
DEFAULT_FAULT_STOP_TIMEOUT_S = 0.5


class RouteExecutorError(RuntimeError):
    """Base error for route execution failures."""


class RouteExecutorPlanningError(RouteExecutorError):
    """Raised when a requested route plan is incomplete or inconsistent."""


class RouteExecutorLifecycleError(RouteExecutorError):
    """Raised when an operation is invalid in the current executor state."""


class RouteExecutorEventError(RouteExecutorError):
    """Raised for an unknown, mismatched, repeated, or out-of-order event."""


class RouteExecutorTimeoutError(RouteExecutorError, TimeoutError):
    """Raised when a Pi-side route watchdog expires."""


class RouteExecutorNavigationError(RouteExecutorError):
    """Raised when final AprilTag navigation does not follow its contract."""


class RouteExecutorState(str, Enum):
    """Observable phases of one prepared route."""

    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    WAITING_JUNCTION = "WAITING_JUNCTION"
    TURNING = "TURNING"
    WAITING_TURN_DONE = "WAITING_TURN_DONE"
    ROUTE_TURNS_DONE = "ROUTE_TURNS_DONE"
    FINAL_TAG_APPROACH = "FINAL_TAG_APPROACH"
    ARRIVED = "ARRIVED"
    FINISHED = "FINISHED"
    FAULT = "FAULT"


@dataclass(frozen=True, slots=True)
class RouteExecutorConfig:
    """Pi-side watchdogs; these do not change STM32 LineNav timeouts."""

    junction_wait_timeout_s: float = DEFAULT_JUNCTION_WAIT_TIMEOUT_S
    turn_done_timeout_s: float = DEFAULT_TURN_DONE_TIMEOUT_S
    final_tag_timeout_s: float = DEFAULT_FINAL_TAG_TIMEOUT_S
    fault_stop_timeout_s: float = DEFAULT_FAULT_STOP_TIMEOUT_S

    def __post_init__(self) -> None:
        for name in (
            "junction_wait_timeout_s",
            "turn_done_timeout_s",
            "final_tag_timeout_s",
            "fault_stop_timeout_s",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and greater than 0")


@dataclass(frozen=True, slots=True)
class RoutePlan:
    """Hardware-free plan consumed by :class:`RouteExecutor`."""

    route_nodes: tuple[TopologicalLocation, ...]
    turn_actions: tuple[RouteAction, ...]
    target_location: TopologicalLocation
    target_tag: int
    arrival_area: float
    confirm_frames: int
    current_heading: Heading | None


class RouteExecutorStatus(TypedDict):
    """Observable snapshot of route progress."""

    state: RouteExecutorState
    route_nodes: tuple[TopologicalLocation, ...]
    turn_actions: tuple[RouteAction, ...]
    current_action_index: int
    current_action: RouteAction | None
    target_location: TopologicalLocation | None
    target_tag: int | None
    route_turns_done: bool
    last_event: LineNavigationEvent | None
    error: str | None


class TopologicalNavigatorPort(Protocol):
    """TopologicalNavigator operations used by the executor."""

    def plan_route(
        self,
        start: TopologicalLocation | str,
        target: TopologicalLocation | str,
    ) -> list[TopologicalLocation]: ...

    def update_from_tag(self, tag_id: int) -> TopologicalLocation | None: ...


class RouteActionResolverPort(Protocol):
    """RouteActionResolver operation used by the executor."""

    def resolve_route_actions(
        self,
        route: Sequence[TopologicalLocation | str],
        *,
        current_heading: Heading | str | None = None,
    ) -> list[RouteAction]: ...


class NavigationControllerPort(Protocol):
    """NavigationController operations used during final Tag arrival."""

    @property
    def current_state(self) -> object: ...

    def start_line_to_tag(
        self,
        tag_id: int,
        arrival_area: float,
        confirm_frames: int,
    ) -> object: ...

    def continue_line_to_tag(
        self,
        tag_id: int,
        arrival_area: float,
        confirm_frames: int,
    ) -> object: ...

    def update(self) -> object: ...


class RouteStm32Link(Protocol):
    """Single Stm32Link operations used by the executor."""

    def set_next_turn(
        self,
        turn: NextTurn | str,
        timeout: float = 2.0,
    ) -> str: ...

    def send_command(self, command: str) -> str: ...

    def wait_ack(self, command: str, timeout: float = 2.0) -> str: ...

    def readline(self) -> bytes: ...

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
        debug_label: str = "stop",
    ) -> str | None: ...


def _normalize_location(
    location: TopologicalLocation | str,
) -> TopologicalLocation:
    if isinstance(location, TopologicalLocation):
        return location
    try:
        return TopologicalLocation(str(location))
    except ValueError as exc:
        raise RouteExecutorPlanningError(
            f"unknown topological location: {location}"
        ) from exc


def _normalize_heading(heading: Heading | str | None) -> Heading | None:
    if heading is None or isinstance(heading, Heading):
        return heading
    try:
        return Heading(str(heading).upper())
    except ValueError as exc:
        raise RouteExecutorPlanningError(f"unknown heading: {heading}") from exc


def _normalize_arrival_area(arrival_area: float) -> float:
    if isinstance(arrival_area, bool):
        raise TypeError("arrival_area must be a number")
    try:
        normalized = float(arrival_area)
    except (TypeError, ValueError) as exc:
        raise TypeError("arrival_area must be a number") from exc
    if not math.isfinite(normalized) or normalized <= 0:
        raise ValueError("arrival_area must be finite and greater than 0")
    return normalized


def build_route_plan(
    navigator: TopologicalNavigatorPort,
    resolver: RouteActionResolverPort,
    start_location: TopologicalLocation | str,
    target_location: TopologicalLocation | str,
    target_tag: int,
    arrival_area: float,
    confirm_frames: int,
    *,
    current_heading: Heading | str | None = None,
) -> RoutePlan:
    """Plan and resolve a route without opening or commanding hardware."""
    start = _normalize_location(start_location)
    target = _normalize_location(target_location)
    heading = _normalize_heading(current_heading)

    if isinstance(target_tag, bool) or not isinstance(target_tag, int):
        raise TypeError("target_tag must be an integer")
    if target_tag <= 0:
        raise ValueError("target_tag must be positive")
    mapped_location = TAG_TOPOLOGY_LOCATIONS.get(target_tag)
    if mapped_location != target.value:
        raise RouteExecutorPlanningError(
            f"target Tag {target_tag} maps to {mapped_location!r}, not "
            f"{target.value!r}"
        )
    normalized_area = _normalize_arrival_area(arrival_area)
    if isinstance(confirm_frames, bool) or not isinstance(confirm_frames, int):
        raise TypeError("confirm_frames must be an integer")
    if confirm_frames <= 0:
        raise ValueError("confirm_frames must be positive")

    raw_route = navigator.plan_route(start, target)
    if not raw_route:
        raise RouteExecutorPlanningError("navigator returned an empty route")
    route = tuple(_normalize_location(node) for node in raw_route)
    if route[0] is not start or route[-1] is not target:
        raise RouteExecutorPlanningError(
            "navigator route endpoints do not match the requested route"
        )

    raw_actions = resolver.resolve_route_actions(
        route,
        current_heading=heading,
    )
    try:
        actions = tuple(
            action if isinstance(action, RouteAction) else RouteAction(action)
            for action in raw_actions
        )
    except (TypeError, ValueError) as exc:
        raise RouteExecutorPlanningError(
            "resolver returned an unknown route action"
        ) from exc

    return RoutePlan(
        route_nodes=route,
        turn_actions=actions,
        target_location=target,
        target_tag=target_tag,
        arrival_area=normalized_area,
        confirm_frames=confirm_frames,
        current_heading=heading,
    )


class RouteExecutor:
    """Execute one prepared route through a single injected STM32 reader."""

    _STM32_EVENT_STATES = {
        RouteExecutorState.WAITING_JUNCTION,
        RouteExecutorState.TURNING,
        RouteExecutorState.WAITING_TURN_DONE,
    }

    def __init__(
        self,
        navigator: TopologicalNavigatorPort,
        resolver: RouteActionResolverPort,
        navigation_controller: NavigationControllerPort,
        stm32_link: RouteStm32Link,
        *,
        config: RouteExecutorConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._navigator = navigator
        self._resolver = resolver
        self._navigation = navigation_controller
        self._stm32 = stm32_link
        self._config = config or RouteExecutorConfig()
        self._clock = clock
        self._state = RouteExecutorState.IDLE
        self._plan: RoutePlan | None = None
        self._current_action_index = 0
        self._deadline: float | None = None
        self._last_event: LineNavigationEvent | None = None
        self._error: str | None = None
        self._fault_stop_attempted = False

    @property
    def state(self) -> RouteExecutorState:
        return self._state

    @property
    def plan(self) -> RoutePlan | None:
        return self._plan

    @property
    def current_action_index(self) -> int:
        return self._current_action_index

    @property
    def current_action(self) -> RouteAction | None:
        if (
            self._plan is None
            or self._current_action_index >= len(self._plan.turn_actions)
        ):
            return None
        return self._plan.turn_actions[self._current_action_index]

    @property
    def last_event(self) -> LineNavigationEvent | None:
        return self._last_event

    def prepare_route(
        self,
        start_location: TopologicalLocation | str,
        target_location: TopologicalLocation | str,
        target_tag: int,
        arrival_area: float,
        confirm_frames: int,
        *,
        current_heading: Heading | str | None = None,
    ) -> RoutePlan:
        """Prepare a route and actions without starting LineNav."""
        if self._state not in {
            RouteExecutorState.IDLE,
            RouteExecutorState.FINISHED,
        }:
            exc = RouteExecutorLifecycleError(
                f"cannot prepare route while state is {self._state.value}"
            )
            self._enter_fault(exc)
            raise exc

        self._state = RouteExecutorState.PLANNING
        self._plan = None
        self._current_action_index = 0
        self._deadline = None
        self._last_event = None
        self._error = None
        self._fault_stop_attempted = False

        try:
            plan = build_route_plan(
                self._navigator,
                self._resolver,
                start_location,
                target_location,
                target_tag,
                arrival_area,
                confirm_frames,
                current_heading=current_heading,
            )
        except Exception as exc:
            self._enter_fault(exc)
            raise

        self._plan = plan
        self._state = RouteExecutorState.READY
        return plan

    def start(self) -> RouteExecutorState:
        """Explicitly start a prepared route and no other lifecycle state."""
        try:
            if self._state is not RouteExecutorState.READY or self._plan is None:
                raise RouteExecutorLifecycleError(
                    f"cannot start route while state is {self._state.value}"
                )

            if not self._plan.turn_actions:
                self._begin_final_tag_approach(line_already_running=False)
                return self._state

            self._preset_current_action()
            command = self._stm32.send_command("LINE_START")
            self._stm32.wait_ack(command)
            self._state = RouteExecutorState.WAITING_JUNCTION
            self._set_deadline(self._config.junction_wait_timeout_s)
            return self._state
        except Exception as exc:
            self._enter_fault(exc)
            raise

    def poll(self) -> RouteExecutorState:
        """Advance one STM32-event or AprilTag frame without another reader."""
        self._last_event = None
        try:
            if self._state in self._STM32_EVENT_STATES:
                self._check_timeout_raw()
                raw_line = self._stm32.readline()
                if raw_line:
                    self._process_stm32_line_raw(raw_line)
                self._check_timeout_raw()
            elif self._state is RouteExecutorState.ROUTE_TURNS_DONE:
                self._begin_final_tag_approach(line_already_running=True)
            elif self._state is RouteExecutorState.FINAL_TAG_APPROACH:
                self._check_timeout_raw()
                self._poll_final_tag_raw()
                self._check_timeout_raw()
            elif self._state is RouteExecutorState.ARRIVED:
                self._state = RouteExecutorState.FINISHED
            return self._state
        except Exception as exc:
            self._enter_fault(exc)
            raise

    def process_stm32_line(self, raw_line: bytes | str) -> RouteExecutorState:
        """Process one already-read STM32 line for deterministic integrations."""
        try:
            self._process_stm32_line_raw(raw_line)
            return self._state
        except Exception as exc:
            self._enter_fault(exc)
            raise

    def handle_line_navigation_event(
        self,
        event: LineNavigationEvent,
    ) -> RouteExecutorState:
        """Apply one parsed event with strict ordering and turn matching."""
        try:
            self._handle_line_navigation_event_raw(event)
            return self._state
        except Exception as exc:
            self._enter_fault(exc)
            raise

    def check_timeout(self) -> RouteExecutorState:
        """Apply the current Pi watchdog without performing device I/O."""
        try:
            self._check_timeout_raw()
            return self._state
        except Exception as exc:
            self._enter_fault(exc)
            raise

    def get_status(self) -> RouteExecutorStatus:
        """Return immutable route data plus current execution progress."""
        plan = self._plan
        actions = () if plan is None else plan.turn_actions
        return {
            "state": self._state,
            "route_nodes": () if plan is None else plan.route_nodes,
            "turn_actions": actions,
            "current_action_index": self._current_action_index,
            "current_action": self.current_action,
            "target_location": None if plan is None else plan.target_location,
            "target_tag": None if plan is None else plan.target_tag,
            "route_turns_done": (
                plan is not None
                and self._current_action_index >= len(actions)
            ),
            "last_event": self._last_event,
            "error": self._error,
        }

    def _preset_current_action(self) -> None:
        action = self.current_action
        if action is None:
            raise RouteExecutorLifecycleError("no route action is pending")
        self._stm32.set_next_turn(NextTurn(action.value))

    def _process_stm32_line_raw(self, raw_line: bytes | str) -> None:
        if self._state not in self._STM32_EVENT_STATES:
            raise RouteExecutorEventError(
                f"STM32 event is invalid while state is {self._state.value}"
            )
        line = (
            raw_line.decode(errors="replace").strip()
            if isinstance(raw_line, bytes)
            else raw_line.strip()
        )
        if not line:
            return
        if line.startswith("ERR,"):
            raise Stm32ProtocolError(f"STM32 error: {line}")
        event = parse_line_navigation_event(line)
        if event is None:
            raise RouteExecutorEventError(
                f"unexpected STM32 line while executing route: {line}"
            )
        self._handle_line_navigation_event_raw(event)

    def _handle_line_navigation_event_raw(
        self,
        event: LineNavigationEvent,
    ) -> None:
        if not isinstance(event, LineNavigationEvent):
            raise RouteExecutorEventError(
                f"unknown line navigation event: {event!r}"
            )
        self._last_event = event

        if event.event_type is LineNavigationEventType.JUNCTION_DETECTED:
            if self._state is not RouteExecutorState.WAITING_JUNCTION:
                self._raise_out_of_order(event)
            self._state = RouteExecutorState.TURNING
            self._set_deadline(self._config.turn_done_timeout_s)
            return

        expected_turn = self._expected_turn()
        if event.turn is not expected_turn:
            raise RouteExecutorEventError(
                f"expected {expected_turn.value}, received "
                f"{event.event_type.value},{getattr(event.turn, 'value', None)}"
            )

        if event.event_type is LineNavigationEventType.TURN_STARTED:
            if self._state is not RouteExecutorState.TURNING:
                self._raise_out_of_order(event)
            self._state = RouteExecutorState.WAITING_TURN_DONE
            self._set_deadline(self._config.turn_done_timeout_s)
            return

        if event.event_type is not LineNavigationEventType.TURN_DONE:
            raise RouteExecutorEventError(
                f"unknown line navigation event: {event.event_type!r}"
            )
        if self._state is not RouteExecutorState.WAITING_TURN_DONE:
            self._raise_out_of_order(event)

        self._current_action_index += 1
        if self.current_action is None:
            self._state = RouteExecutorState.ROUTE_TURNS_DONE
            self._deadline = None
            return

        self._preset_current_action()
        self._state = RouteExecutorState.WAITING_JUNCTION
        self._set_deadline(self._config.junction_wait_timeout_s)

    def _expected_turn(self) -> NextTurn:
        action = self.current_action
        if action is None:
            raise RouteExecutorEventError("no turn action is currently expected")
        return NextTurn(action.value)

    def _raise_out_of_order(self, event: LineNavigationEvent) -> None:
        raise RouteExecutorEventError(
            f"out-of-order {event.event_type.value} while state is "
            f"{self._state.value}"
        )

    def _begin_final_tag_approach(self, *, line_already_running: bool) -> None:
        if self._plan is None:
            raise RouteExecutorLifecycleError("route has not been prepared")
        if line_already_running:
            navigation_state = self._navigation.continue_line_to_tag(
                self._plan.target_tag,
                self._plan.arrival_area,
                self._plan.confirm_frames,
            )
        else:
            navigation_state = self._navigation.start_line_to_tag(
                self._plan.target_tag,
                self._plan.arrival_area,
                self._plan.confirm_frames,
            )
        if self._state_value(navigation_state) != "LINE_FOLLOWING":
            raise RouteExecutorNavigationError(
                "NavigationController did not enter LINE_FOLLOWING"
            )
        self._state = RouteExecutorState.FINAL_TAG_APPROACH
        self._set_deadline(self._config.final_tag_timeout_s)

    def _poll_final_tag_raw(self) -> None:
        navigation_state = self._navigation.update()
        state_value = self._state_value(navigation_state)
        if state_value == "FAULT":
            raise RouteExecutorNavigationError(
                "NavigationController entered FAULT"
            )
        if state_value == "LINE_FOLLOWING":
            return
        if state_value != "ARRIVED":
            raise RouteExecutorNavigationError(
                f"unexpected NavigationController state: {state_value}"
            )
        if self._plan is None:
            raise RouteExecutorLifecycleError("route has not been prepared")

        confirmed_location = self._navigator.update_from_tag(
            self._plan.target_tag
        )
        if confirmed_location is not self._plan.target_location:
            raise RouteExecutorNavigationError(
                f"target Tag {self._plan.target_tag} updated location to "
                f"{confirmed_location!r}, expected "
                f"{self._plan.target_location.value}"
            )
        self._deadline = None
        self._state = RouteExecutorState.ARRIVED

    def _check_timeout_raw(self) -> None:
        if self._deadline is None or self._clock() < self._deadline:
            return
        raise RouteExecutorTimeoutError(
            f"timeout while state is {self._state.value}"
        )

    def _set_deadline(self, timeout_s: float) -> None:
        self._deadline = self._clock() + timeout_s

    @staticmethod
    def _state_value(state: object) -> str:
        return str(getattr(state, "value", state))

    def _enter_fault(self, exc: Exception) -> None:
        if self._state is RouteExecutorState.FAULT:
            return
        self._state = RouteExecutorState.FAULT
        self._deadline = None
        self._error = f"{type(exc).__name__}: {exc}"
        if self._fault_stop_attempted:
            return
        self._fault_stop_attempted = True
        try:
            self._stm32.stop(
                timeout=self._config.fault_stop_timeout_s,
                debug_label="route_executor_fault",
            )
        except Exception as stop_exc:
            self._error += (
                f"; STOP failed: {type(stop_exc).__name__}: {stop_exc}"
            )
