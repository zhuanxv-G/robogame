"""Pi-side navigation abstraction for the competition framework."""

from __future__ import annotations

import math
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
    error_x: float | None
    area: float | None
    arrival_area: float | None
    confirm_frames: int | None
    confirm_count: int
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
        self._error_x: float | None = None
        self._area: float | None = None
        self._arrival_area: float | None = None
        self._confirm_frames: int | None = None
        self._confirm_count = 0
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
        self._error_x = None
        self._area = None
        self._arrival_area = None
        self._confirm_frames = None
        self._confirm_count = 0
        self._error = None
        return self._state

    def start_line_to_tag(
        self,
        tag_id: int,
        arrival_area: float,
        confirm_frames: int,
    ) -> NavigationState:
        """Start STM32 line following and watch one Tag as the stop marker.

        ``arrival_area`` is deliberately supplied by the caller because the
        camera mounting and field stop distance still need calibration.
        Calling this method performs one LINE_START write; subsequent work is
        advanced one camera frame at a time by :meth:`update`.
        """
        self._validate_tag_id(tag_id)
        if isinstance(arrival_area, bool):
            raise TypeError("arrival_area must be a number")
        try:
            normalized_area = float(arrival_area)
        except (TypeError, ValueError) as exc:
            raise TypeError("arrival_area must be a number") from exc
        if not math.isfinite(normalized_area) or normalized_area <= 0:
            raise ValueError("arrival_area must be finite and greater than 0")
        if isinstance(confirm_frames, bool) or not isinstance(
            confirm_frames,
            int,
        ):
            raise TypeError("confirm_frames must be an integer")
        if confirm_frames <= 0:
            raise ValueError("confirm_frames must be positive")

        self._target_tag_id = tag_id
        self._tag_visible = False
        self._tag_detection = None
        self._error_x = None
        self._area = None
        self._arrival_area = normalized_area
        self._confirm_frames = confirm_frames
        self._confirm_count = 0
        self._error = None

        try:
            self._last_command = self._stm32.send_command("LINE_START")
        except Exception as exc:
            self._enter_fault(exc)
            raise

        self._state = NavigationState.LINE_FOLLOWING
        return self._state

    def update(self) -> NavigationState:
        """Process one AprilTag frame for an active line-to-Tag run."""
        if (
            self._state is not NavigationState.LINE_FOLLOWING
            or self._target_tag_id is None
            or self._arrival_area is None
            or self._confirm_frames is None
        ):
            return self._state

        try:
            detections = self._apriltag.detect()
            target = next(
                (
                    detection
                    for detection in detections
                    if detection.get("id") == self._target_tag_id
                ),
                None,
            )
            if target is None:
                self._clear_current_tag_observation()
                return self._state

            error_x = float(target["error_x"])
            area = float(target["area"])
            if not math.isfinite(error_x) or not math.isfinite(area) or area < 0:
                raise ValueError("invalid AprilTag error_x or area")

            self._tag_visible = True
            self._tag_detection = target.copy()
            self._error_x = error_x
            self._area = area

            if area >= self._arrival_area:
                self._confirm_count += 1
            else:
                self._confirm_count = 0

            if self._confirm_count >= self._confirm_frames:
                self._confirm_arrival()
            return self._state
        except Exception as exc:
            if self._state is not NavigationState.FAULT:
                self._enter_fault(exc)
            raise

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
        self._error_x = None
        self._area = None
        self._confirm_count = 0
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
        self._validate_tag_id(tag_id)

        self._target_tag_id = tag_id
        self._tag_visible = False
        self._tag_detection = None
        self._error_x = None
        self._area = None
        self._arrival_area = None
        self._confirm_frames = None
        self._confirm_count = 0
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
                error_x = detection.get("error_x")
                area = detection.get("area")
                self._error_x = self._optional_finite_float(error_x)
                self._area = self._optional_finite_float(area)
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
            "error_x": self._error_x,
            "area": self._area,
            "arrival_area": self._arrival_area,
            "confirm_frames": self._confirm_frames,
            "confirm_count": self._confirm_count,
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

    @staticmethod
    def _validate_tag_id(tag_id: int) -> None:
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise TypeError("tag_id must be an integer")
        if tag_id <= 0:
            raise ValueError("tag_id must be positive")

    @staticmethod
    def _optional_finite_float(value: object) -> float | None:
        if value is None:
            return None
        try:
            normalized = float(value)
        except (TypeError, ValueError):
            return None
        return normalized if math.isfinite(normalized) else None

    def _clear_current_tag_observation(self) -> None:
        self._tag_visible = False
        self._tag_detection = None
        self._error_x = None
        self._area = None
        self._confirm_count = 0

    def _confirm_arrival(self) -> None:
        try:
            self._stm32.stop(require_ack=True)
        except Exception as exc:
            # STOP was already attempted. Keep FAULT observable and let the
            # caller's outer safety cleanup decide whether to retry it.
            self._set_fault(exc)
            raise

        self._last_command = "STOP"
        self._state = NavigationState.ARRIVED
        self._error = None

    def _enter_fault(self, exc: Exception) -> None:
        self._clear_current_tag_observation()
        self._set_fault(exc)
        try:
            self._stm32.stop(require_ack=True)
            self._last_command = "STOP"
        except Exception as stop_exc:
            self._error = (self._error or "navigation fault") + (
                "; STOP failed: "
                f"{type(stop_exc).__name__}: {stop_exc}"
            )
