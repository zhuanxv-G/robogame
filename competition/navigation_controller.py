"""Pi-side navigation abstraction for the competition framework."""

from __future__ import annotations

from enum import Enum
from typing import Any, Protocol, TypedDict


class NavigationState(str, Enum):
    """Navigation details kept separate from top-level MissionState."""

    IDLE = "IDLE"
    LINE_FOLLOWING = "LINE_FOLLOWING"
    TAG_SEARCH = "TAG_SEARCH"
    TAG_ALIGN = "TAG_ALIGN"
    ARRIVED = "ARRIVED"
    FAULT = "FAULT"


class MaterialZone(str, Enum):
    """Material destinations currently named by the competition plan."""

    ORANGE = "ORANGE"
    PURPLE = "PURPLE"


class Stm32NavigationLink(Protocol):
    """Stm32Link operations used by the navigation layer."""

    def send_command(self, command: str) -> str: ...

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
    ) -> str | None: ...


class AprilTagInput(Protocol):
    """AprilTagService operation used by the navigation layer."""

    def detect(self) -> list[dict[str, Any]]: ...


class NavigationStatus(TypedDict):
    """Observable navigation state for a future mission orchestrator."""

    state: NavigationState
    target_zone: MaterialZone | None
    target_tag_id: int | None
    tag_visible: bool
    tag_detection: dict[str, Any] | None
    last_command: str | None
    error: str | None


class NavigationController:
    """Coordinate navigation through injected hardware service objects.

    Construction and reset only initialize software state. The controller
    never opens a serial port or camera and does not own a control loop.
    """

    def __init__(
        self,
        stm32_link: Stm32NavigationLink,
        apriltag_service: AprilTagInput,
    ) -> None:
        self._stm32 = stm32_link
        self._apriltag = apriltag_service
        self.reset()

    @property
    def current_state(self) -> NavigationState:
        """Return the current navigation-detail state."""
        return self._state

    def reset(self) -> NavigationState:
        """Reset software state without sending commands or reading a camera."""
        self._state = NavigationState.IDLE
        self._target_zone: MaterialZone | None = None
        self._target_tag_id: int | None = None
        self._tag_visible = False
        self._tag_detection: dict[str, Any] | None = None
        self._last_command: str | None = None
        self._error: str | None = None
        return self._state

    def start_line_follow(self) -> NavigationState:
        """Start STM32 line following with the existing LINE_START command."""
        try:
            self._last_command = self._stm32.send_command("LINE_START")
        except Exception as exc:
            self._set_fault(exc)
            raise

        self._state = NavigationState.LINE_FOLLOWING
        self._target_tag_id = None
        self._tag_visible = False
        self._tag_detection = None
        self._error = None
        return self._state

    def stop_navigation(self) -> NavigationState:
        """Stop navigation through Stm32Link using the existing STOP command."""
        try:
            self._stm32.stop()
        except Exception as exc:
            self._set_fault(exc)
            raise

        self._state = NavigationState.IDLE
        self._tag_visible = False
        self._tag_detection = None
        self._last_command = "STOP"
        self._error = None
        return self._state

    def navigate_to_material_zone(
        self,
        zone: MaterialZone | str,
    ) -> NavigationState:
        """Record an ORANGE or PURPLE destination without inventing a route."""
        if isinstance(zone, MaterialZone):
            target_zone = zone
        else:
            target_zone = MaterialZone(str(zone).upper())

        self._target_zone = target_zone
        return self._state

    def align_with_tag(self, tag_id: int) -> NavigationState:
        """Observe one AprilTagService frame for a requested tag.

        Seeing the requested tag records TAG_ALIGN, but does not generate any
        chassis correction because no verified Tag alignment policy exists.
        """
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise TypeError("tag_id must be an integer")
        if tag_id <= 0:
            raise ValueError("tag_id must be positive")

        self._target_tag_id = tag_id
        self._tag_visible = False
        self._tag_detection = None
        self._state = NavigationState.TAG_SEARCH

        try:
            detections = self._apriltag.detect()
        except Exception as exc:
            self._set_fault(exc)
            raise

        for detection in detections:
            if detection.get("id") == tag_id:
                self._tag_visible = True
                self._tag_detection = detection.copy()
                self._state = NavigationState.TAG_ALIGN
                break

        self._error = None
        return self._state

    def get_navigation_status(self) -> NavigationStatus:
        """Return a copy of the current navigation status."""
        tag_detection = (
            None
            if self._tag_detection is None
            else self._tag_detection.copy()
        )
        return {
            "state": self._state,
            "target_zone": self._target_zone,
            "target_tag_id": self._target_tag_id,
            "tag_visible": self._tag_visible,
            "tag_detection": tag_detection,
            "last_command": self._last_command,
            "error": self._error,
        }

    def handle_event(self, event: object) -> object:
        """Reserve the future STM32 navigation-event entry point.

        The current protocol has no verified completion or error events, so
        this method intentionally performs no transition.
        """
        del event
        return NotImplemented

    def _set_fault(self, exc: Exception) -> None:
        self._state = NavigationState.FAULT
        self._error = f"{type(exc).__name__}: {exc}"
