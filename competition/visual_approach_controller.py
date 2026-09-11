"""K230-guided align and approach controller for the competition layer."""

from __future__ import annotations

import time
from collections.abc import Callable
from enum import Enum
from typing import Protocol, TypedDict

from vision.k230_link import K230Result, TargetResult


# Values are copied unchanged from the verified main_controller_align.py flow.
ALIGN_EXIT_ERROR = 30
ALIGN_ENTER_ERROR = 70
ALIGN_OMEGA = 0.45
APPROACH_SPEED = 0.20
GRAB_AREA_MIN = 20000
GRAB_CONFIRM_COUNT = 3
AREA_ALPHA = 0.20
TARGET_LOST_TIMEOUT = 0.30
K230_TIMEOUT = 0.60


class VisualApproachState(str, Enum):
    """States owned by the visual approach layer."""

    STOP = "STOP"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    READY = "READY"
    FAULT = "FAULT"


class K230Input(Protocol):
    """K230Link operation required by this controller."""

    def read_latest(self) -> K230Result | None: ...


class ChassisOutput(Protocol):
    """Stm32Link operations required by this controller."""

    def set_velocity(self, x: float, y: float, omega: float) -> str: ...

    def stop(
        self,
        *,
        require_ack: bool = False,
        timeout: float = 2.0,
    ) -> str | None: ...


class VisualApproachStatus(TypedDict):
    """Observable state returned to a future mission loop."""

    state: VisualApproachState
    motion: str
    error_x: int | None
    pixels: int | None
    area_ema: float | None
    grab_confirm_count: int
    command: str | None
    ready: bool


class VisualApproachController:
    """Preserve the verified K230 align/approach behavior behind injected links.

    Construction only stores dependencies and state. This class never opens
    serial ports, creates cameras, or starts its own control loop.
    """

    def __init__(
        self,
        k230_link: K230Input,
        stm32_link: ChassisOutput,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._k230 = k230_link
        self._stm32 = stm32_link
        self._clock = clock
        self.reset()

    @property
    def current_state(self) -> VisualApproachState:
        """Return the current visual approach state."""
        return self._state

    def reset(self) -> VisualApproachState:
        """Reset software state without sending a hardware command."""
        now = self._clock()
        self._state = VisualApproachState.STOP
        self._area_ema: float | None = None
        self._grab_confirm_count = 0
        self._last_k230_time = now
        self._last_target_time = now
        self._stopped = True
        self._ready_stop_acknowledged = False
        self._motion = "STOP"
        self._last_error_x: int | None = None
        self._last_pixels: int | None = None
        self._last_command: str | None = None
        return self._state

    def update(self) -> VisualApproachState:
        """Process the newest queued K230 result and apply safety timeouts."""
        try:
            now = self._clock()
            result = self._k230.read_latest()

            if (
                result is not None
                and self._state is not VisualApproachState.FAULT
            ):
                self._last_k230_time = now

                if result["target"]:
                    if self._state is VisualApproachState.APPROACH:
                        self._approach_target(result, now)
                    else:
                        self._align_target(result, now)

            self._apply_timeouts(now)
            return self._state
        except Exception:
            self._enter_fault()
            raise

    def align_target(
        self,
        target: TargetResult | None = None,
    ) -> VisualApproachState:
        """Execute one ALIGN update, reading K230 when no target is given."""
        if target is None:
            return self._update_stage(self._align_target)
        if self._state is VisualApproachState.FAULT:
            return self._state
        try:
            return self._align_target(target, self._clock())
        except Exception:
            self._enter_fault()
            raise

    def approach_target(
        self,
        target: TargetResult | None = None,
    ) -> VisualApproachState:
        """Execute one APPROACH update, reading K230 when none is given."""
        if target is None:
            return self._update_stage(self._approach_target)
        if self._state is VisualApproachState.FAULT:
            return self._state
        try:
            return self._approach_target(target, self._clock())
        except Exception:
            self._enter_fault()
            raise

    def _update_stage(
        self,
        stage: Callable[[TargetResult, float], VisualApproachState],
    ) -> VisualApproachState:
        if self._state is VisualApproachState.FAULT:
            return self._state

        try:
            now = self._clock()
            result = self._k230.read_latest()
            if result is not None:
                self._last_k230_time = now
                if result["target"]:
                    stage(result, now)
            self._apply_timeouts(now)
            return self._state
        except Exception:
            self._enter_fault()
            raise

    def _align_target(
        self,
        target: TargetResult,
        now: float,
    ) -> VisualApproachState:
        error_x = self._observe_target(target, now)

        if self._state is VisualApproachState.READY:
            return self._state

        if abs(error_x) <= ALIGN_EXIT_ERROR:
            self._state = VisualApproachState.APPROACH
        else:
            self._state = VisualApproachState.ALIGN

        self._send_motion(error_x)
        return self._state

    def _approach_target(
        self,
        target: TargetResult,
        now: float,
    ) -> VisualApproachState:
        error_x = self._observe_target(target, now)

        if self._state is VisualApproachState.READY:
            return self._state

        if abs(error_x) >= ALIGN_ENTER_ERROR:
            self._state = VisualApproachState.ALIGN
        else:
            self._state = VisualApproachState.APPROACH

        self._send_motion(error_x)
        return self._state

    def _observe_target(self, target: TargetResult, now: float) -> int:
        error_x = int(target["error_x"])
        pixels = int(target["pixels"])

        self._last_k230_time = now
        self._last_target_time = now
        self._last_error_x = error_x
        self._last_pixels = pixels

        if self._state in (
            VisualApproachState.STOP,
            VisualApproachState.ALIGN,
            VisualApproachState.APPROACH,
        ):
            self._stopped = False

        if self._area_ema is None:
            self._area_ema = float(pixels)
        else:
            self._area_ema = (
                AREA_ALPHA * pixels
                + (1.0 - AREA_ALPHA) * self._area_ema
            )

        grab_candidate = (
            self._area_ema >= GRAB_AREA_MIN
            and abs(error_x) <= ALIGN_EXIT_ERROR
        )
        if grab_candidate:
            self._grab_confirm_count += 1
        else:
            self._grab_confirm_count = 0

        if self._grab_confirm_count >= GRAB_CONFIRM_COUNT:
            self._confirm_ready_stop()

        return error_x

    def _send_motion(self, error_x: int) -> None:
        if self._state is VisualApproachState.ALIGN:
            omega = -ALIGN_OMEGA if error_x > 0 else ALIGN_OMEGA
            self._last_command = self._stm32.set_velocity(0.0, 0.0, omega)
            self._motion = "ROTATE_RIGHT" if error_x > 0 else "ROTATE_LEFT"
        elif self._state is VisualApproachState.APPROACH:
            self._last_command = self._stm32.set_velocity(
                0.0,
                APPROACH_SPEED,
                0.0,
            )
            self._motion = "FORWARD"

    def _confirm_ready_stop(self) -> None:
        self._state = VisualApproachState.READY
        if self._ready_stop_acknowledged:
            return

        self._stm32.stop(require_ack=True)
        self._stopped = True
        self._ready_stop_acknowledged = True
        self._last_command = "STOP"
        self._motion = "READY"

    def is_ready(self) -> bool:
        """Return whether READY was reached and STOP was acknowledged."""
        return (
            self._state is VisualApproachState.READY
            and self._ready_stop_acknowledged
        )

    def stop(self) -> VisualApproachState:
        """Stop through Stm32Link and reset approach tracking."""
        try:
            self._stm32.stop()
        except Exception:
            self._enter_fault()
            raise

        self._set_stopped()
        return self._state

    def _set_stopped(self) -> None:
        self._state = VisualApproachState.STOP
        self._area_ema = None
        self._grab_confirm_count = 0
        self._stopped = True
        self._ready_stop_acknowledged = False
        self._motion = "STOP"
        self._last_command = "STOP"

    def _apply_timeouts(self, now: float) -> None:
        if self._stopped:
            return

        if now - self._last_target_time > TARGET_LOST_TIMEOUT:
            self.stop()
        elif now - self._last_k230_time > K230_TIMEOUT:
            self.stop()

    def _enter_fault(self) -> None:
        if self._state is VisualApproachState.FAULT:
            return

        self._state = VisualApproachState.FAULT
        self._motion = "FAULT"
        try:
            self._stm32.stop()
            self._last_command = "STOP"
        except Exception:
            pass
        self._stopped = True

    def get_status(self) -> VisualApproachStatus:
        """Return a snapshot suitable for future mission orchestration."""
        return {
            "state": self._state,
            "motion": self._motion,
            "error_x": self._last_error_x,
            "pixels": self._last_pixels,
            "area_ema": self._area_ema,
            "grab_confirm_count": self._grab_confirm_count,
            "command": self._last_command,
            "ready": self.is_ready(),
        }
