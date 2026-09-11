"""Event-driven orchestration for the first RoboGame competition workflow."""

from __future__ import annotations

from enum import Enum
from typing import Protocol, TypedDict

from .mission_state import MissionState
from .navigation_controller import MaterialZone
from .visual_approach_controller import VisualApproachState


class CompetitionPhase(str, Enum):
    """Fine-grained runner phases beneath the top-level mission state."""

    START = "START"
    FOLLOW_LINE = "FOLLOW_LINE"
    GET_ORANGE = "GET_ORANGE"
    VISUAL_APPROACH = "VISUAL_APPROACH"
    GRAB_ORANGE = "GRAB_ORANGE"
    BUILD = "BUILD"
    FINISH = "FINISH"
    FAULT = "FAULT"


class CompetitionEvent(str, Enum):
    """Events accepted from future hardware adapters or a simulator."""

    LINE_DONE = "LINE_DONE"
    TARGET_READY = "TARGET_READY"
    ARM_DONE = "ARM_DONE"
    BUILD_DONE = "BUILD_DONE"
    FAULT = "FAULT"


class CompetitionRunnerError(RuntimeError):
    """Raised for an event that cannot be applied to the current phase."""


class MissionStateManager(Protocol):
    """MissionController operations used by the runner."""

    @property
    def current_state(self) -> MissionState: ...

    def transition(self, next_state: MissionState) -> MissionState: ...

    def reset(self) -> MissionState: ...


class NavigationActions(Protocol):
    """NavigationController operations used by the runner."""

    def navigate_to_material_zone(self, zone: MaterialZone | str) -> object: ...

    def start_line_follow(self) -> object: ...

    def stop_navigation(self) -> object: ...


class VisualApproachActions(Protocol):
    """VisualApproachController operations used by the runner."""

    def update(self) -> object: ...

    def is_ready(self) -> bool: ...


class TaskActions(Protocol):
    """TaskController operations used by the runner."""

    def grab_orange(self) -> object: ...

    def build(self) -> None: ...


class CompetitionRunnerStatus(TypedDict):
    """Observable status for a UI, simulator, or future main loop."""

    phase: CompetitionPhase
    mission_state: MissionState
    simulation_mode: bool
    waiting_for_event: CompetitionEvent | None
    last_event: CompetitionEvent | None
    last_action: str | None
    blocked_reason: str | None
    error: str | None


class CompetitionRunner:
    """Coordinate the first event-driven competition workflow.

    The default simulation mode changes state only. It does not call any
    controller method that could read hardware or send a command. A future
    live entry must explicitly opt out of simulation mode after connecting
    and validating every dependency.
    """

    def __init__(
        self,
        mission_controller: MissionStateManager,
        navigation_controller: NavigationActions,
        visual_controller: VisualApproachActions,
        task_controller: TaskActions,
        *,
        simulation_mode: bool = True,
    ) -> None:
        if mission_controller.current_state is not MissionState.START:
            raise ValueError("CompetitionRunner requires MissionState.START")

        self._mission = mission_controller
        self._navigation = navigation_controller
        self._visual = visual_controller
        self._tasks = task_controller
        self.simulation_mode = simulation_mode
        self._phase = CompetitionPhase.START
        self._phase_history = [self._phase]
        self._waiting_for_event: CompetitionEvent | None = None
        self._last_event: CompetitionEvent | None = None
        self._last_action: str | None = None
        self._blocked_reason: str | None = None
        self._error: str | None = None

    @property
    def current_phase(self) -> CompetitionPhase:
        """Return the runner's fine-grained phase."""
        return self._phase

    @property
    def phase_history(self) -> tuple[CompetitionPhase, ...]:
        """Return an immutable history including transient workflow phases."""
        return tuple(self._phase_history)

    def reset(self) -> CompetitionRunnerStatus:
        """Reset runner and mission state without touching controller hardware."""
        self._mission.reset()
        self._phase = CompetitionPhase.START
        self._phase_history = [self._phase]
        self._waiting_for_event = None
        self._last_event = None
        self._last_action = None
        self._blocked_reason = None
        self._error = None
        return self.get_status()

    def start(self) -> CompetitionRunnerStatus:
        """Enter FOLLOW_LINE and start navigation outside simulation mode."""
        self._require_phase(CompetitionPhase.START)
        self._transition(
            CompetitionPhase.FOLLOW_LINE,
            MissionState.FOLLOW_LINE,
        )
        self._waiting_for_event = CompetitionEvent.LINE_DONE
        self._last_action = "start_line_follow"

        if self.simulation_mode:
            return self.get_status()

        try:
            self._navigation.navigate_to_material_zone(MaterialZone.ORANGE)
            self._navigation.start_line_follow()
        except Exception as exc:
            self._enter_fault(exc)
        return self.get_status()

    def run_once(self) -> CompetitionRunnerStatus:
        """Run one non-blocking orchestration step without sleeping.

        Only the visual approach phase currently has polling behavior. In
        simulation mode even that poll is skipped and TARGET_READY must be
        injected with handle_event().
        """
        if (
            self._phase is not CompetitionPhase.VISUAL_APPROACH
            or self.simulation_mode
        ):
            return self.get_status()

        self._last_action = "visual_approach_update"
        try:
            visual_state = self._visual.update()
            if visual_state is VisualApproachState.FAULT:
                self._enter_fault(
                    RuntimeError("VisualApproachController reported FAULT")
                )
                return self.get_status()
            if self._visual.is_ready():
                return self.handle_event(CompetitionEvent.TARGET_READY)
        except Exception as exc:
            self._enter_fault(exc)
        return self.get_status()

    def handle_event(
        self,
        event: CompetitionEvent | str,
    ) -> CompetitionRunnerStatus:
        """Apply one externally supplied workflow event."""
        competition_event = CompetitionEvent(event)
        self._last_event = competition_event

        if competition_event is CompetitionEvent.FAULT:
            self._enter_fault(RuntimeError("FAULT event received"))
            return self.get_status()

        if competition_event is CompetitionEvent.LINE_DONE:
            self._handle_line_done()
        elif competition_event is CompetitionEvent.TARGET_READY:
            self._handle_target_ready()
        elif competition_event is CompetitionEvent.ARM_DONE:
            self._handle_arm_done()
        elif competition_event is CompetitionEvent.BUILD_DONE:
            self._handle_build_done()
        return self.get_status()

    def _handle_line_done(self) -> None:
        self._require_phase(CompetitionPhase.FOLLOW_LINE)
        self._last_action = "stop_navigation"

        if not self.simulation_mode:
            try:
                self._navigation.stop_navigation()
            except Exception as exc:
                self._enter_fault(exc)
                return

        self._transition(
            CompetitionPhase.GET_ORANGE,
            MissionState.GET_ORANGE,
        )
        self._transition(CompetitionPhase.VISUAL_APPROACH)
        self._waiting_for_event = CompetitionEvent.TARGET_READY
        self._last_action = "wait_for_visual_ready"

    def _handle_target_ready(self) -> None:
        self._require_phase(CompetitionPhase.VISUAL_APPROACH)
        self._transition(CompetitionPhase.GRAB_ORANGE)
        self._last_action = "grab_orange"

        if self.simulation_mode:
            self._waiting_for_event = CompetitionEvent.ARM_DONE
            return

        try:
            result = self._tasks.grab_orange()
        except Exception as exc:
            self._enter_fault(exc)
            return

        if result is NotImplemented:
            self._blocked_reason = (
                "TaskController.grab_orange() is not implemented"
            )
            self._waiting_for_event = None
            return

        self._waiting_for_event = CompetitionEvent.ARM_DONE

    def _handle_arm_done(self) -> None:
        self._require_phase(CompetitionPhase.GRAB_ORANGE)
        if self._blocked_reason is not None:
            raise CompetitionRunnerError(self._blocked_reason)

        self._transition(CompetitionPhase.BUILD, MissionState.BUILD)
        self._last_action = "build"

        if not self.simulation_mode:
            try:
                self._tasks.build()
            except Exception as exc:
                self._enter_fault(exc)
                return
        self._waiting_for_event = CompetitionEvent.BUILD_DONE

    def _handle_build_done(self) -> None:
        self._require_phase(CompetitionPhase.BUILD)
        self._transition(CompetitionPhase.FINISH, MissionState.FINISH)
        self._waiting_for_event = None
        self._last_action = "finish"

    def _transition(
        self,
        phase: CompetitionPhase,
        mission_state: MissionState | None = None,
    ) -> None:
        self._phase = phase
        self._phase_history.append(phase)
        self._blocked_reason = None
        if mission_state is not None:
            self._mission.transition(mission_state)

    def _require_phase(self, expected: CompetitionPhase) -> None:
        if self._phase is not expected:
            raise CompetitionRunnerError(
                f"Event requires {expected.value}, current phase is "
                f"{self._phase.value}"
            )

    def _enter_fault(self, exc: Exception) -> None:
        self._error = f"{type(exc).__name__}: {exc}"
        self._waiting_for_event = None
        self._transition(CompetitionPhase.FAULT, MissionState.FAULT)
        self._last_action = "stop_navigation_after_fault"

        if self.simulation_mode:
            return
        try:
            self._navigation.stop_navigation()
        except Exception as stop_exc:
            self._error += (
                f"; stop failed: {type(stop_exc).__name__}: {stop_exc}"
            )

    def get_status(self) -> CompetitionRunnerStatus:
        """Return the current orchestration status."""
        return {
            "phase": self._phase,
            "mission_state": self._mission.current_state,
            "simulation_mode": self.simulation_mode,
            "waiting_for_event": self._waiting_for_event,
            "last_event": self._last_event,
            "last_action": self._last_action,
            "blocked_reason": self._blocked_reason,
            "error": self._error,
        }
