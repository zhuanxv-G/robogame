"""State-only competition mission controller."""

from __future__ import annotations

from .mission_state import MissionState


class MissionController:
    """Store mission state without performing robot actions."""

    def __init__(self, initial_state: MissionState = MissionState.START) -> None:
        self._initial_state = MissionState(initial_state)
        self._current_state = self._initial_state

    @property
    def current_state(self) -> MissionState:
        """Return the current top-level mission state."""
        return self._current_state

    def transition(self, next_state: MissionState) -> MissionState:
        """Change state directly; transition policy will be added later."""
        self._current_state = MissionState(next_state)
        return self._current_state

    def reset(self) -> MissionState:
        """Restore the controller's initial state."""
        self._current_state = self._initial_state
        return self._current_state
