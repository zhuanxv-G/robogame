"""First complete autonomous mission: line follow -> wall -> backoff -> grab.

``LINE_TO_GRAB_MISSION`` composes the already-verified building blocks into
one fixed-route task without changing any of them:

* The STM32 LineNav start, junction events and RIGHT turn reacquisition
  follow the operator-confirmed flow of ``tests/test_line_to_wall.py``.
* The post-reacquire timed line follow, the VEL wall approach and the
  backoff reuse the same calibration constants and the verified VEL refresh
  pattern of ``tests/test_old_grab_sequence.py`` (REMOTE watchdog = 1000 ms,
  so motion commands must be refreshed every 100 ms).
* The arm action is delegated unchanged to ``competition/grab_task.py``
  ``grab_orange()``, sharing the mission's single ``Stm32Link``.

Mission flow (one explicit state per phase)::

    IDLE            STOP / ACK, NEXT_TURN,RIGHT / ACK
    LINE_FOLLOW     LINE_START / ACK -> JUNCTION_DETECTED
    WAIT_JUNCTION   TURN_STARTED,RIGHT
    TURNING         TURN_DONE,RIGHT + LINE_REACQUIRED (either order)
    LINE_REACQUIRED timed line follow to the grab area -> LINE_STOP / ACK
    APPROACH_WALL   VEL forward 0.38 m (timed) -> STOP / ACK (wall reached)
    BACKOFF         VEL backward 0.05 m (timed) -> STOP / ACK
    GRAB            grab_orange()
    DONE            mission success
    ERROR           any exception: best-effort STOP, then re-raise

Safety: a global STOP is requested with ACK before any motion, after every
timed VEL segment and by the arm task itself.  Any serial failure, STM32
``ERR`` reply, protocol anomaly or Ctrl+C enters the ERROR state, requests a
best-effort STOP and re-raises the original exception.  This module never
modifies the STM32 firmware, ``grab_task.py`` or the existing test scripts.

Importing this module has no hardware side effects.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from enum import Enum
from pathlib import Path


if __package__ in (None, ""):
    project_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(project_root))

from hardware.stm32_link import (  # noqa: E402
    NextTurn,
    Stm32Link,
    Stm32NotConnectedError,
    Stm32ProtocolError,
)
from competition.grab_task import grab_orange  # noqa: E402


# Field calibration: keep the command value and the measured physical speed
# separate, as in tests/test_line_to_wall.py and test_old_grab_sequence.py.
TARGET_DISTANCE = 0.38          # car front to wall distance at the grab area
REAL_SPEED = 0.118              # measured chassis speed for a 0.80 command
WALL_APPROACH_TIME = TARGET_DISTANCE / REAL_SPEED
GRAB_AREA_APPROACH_TIME = TARGET_DISTANCE / REAL_SPEED  # post-reacquire
# line follow to the grab area; same calibration as the wall approach and
# as LINE_DURATION in tests/test_line_to_wall.py.
BACKOFF_DISTANCE = 0.05
BACKOFF_TIME = BACKOFF_DISTANCE / REAL_SPEED  # backward speed not yet
# measured separately; keeps the forward baseline for now.

VEL_COMMAND_SPEED = 0.80        # VEL.Y command value for both VEL segments
VEL_ACK_TIMEOUT_S = 0.5         # VEL answers ACK,VEL only (no DONE)
VEL_REFRESH_S = 0.10            # REMOTE watchdog is 1000 ms

JUNCTION_TIMEOUT_S = 30.0
TURN_TIMEOUT_S = 25.0           # JUNCTION_DETECTED -> both turn events
ACK_TIMEOUT_S = 2.0
SERIAL_POLL_TIMEOUT_S = 0.05
EXPECTED_TURN = NextTurn.RIGHT


class MissionPhase(str, Enum):
    """Explicit states of the LINE_TO_GRAB_MISSION state machine."""

    IDLE = "IDLE"
    LINE_FOLLOW = "LINE_FOLLOW"
    WAIT_JUNCTION = "WAIT_JUNCTION"
    TURNING = "TURNING"
    LINE_REACQUIRED = "LINE_REACQUIRED"
    APPROACH_WALL = "APPROACH_WALL"
    BACKOFF = "BACKOFF"
    GRAB = "GRAB"
    DONE = "DONE"
    ERROR = "ERROR"


class GrabMissionController:
    """State machine connecting line follow and the verified arm grab."""

    def __init__(
        self,
        stm32: Stm32Link,
        *,
        output: Callable[[str], object] = print,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._stm32 = stm32
        self._output = output
        self._clock = clock
        self._phase = MissionPhase.IDLE
        self._turn_deadline: float | None = None
        self._handlers: dict[MissionPhase, Callable[[], None]] = {
            MissionPhase.IDLE: self._phase_startup,
            MissionPhase.LINE_FOLLOW: self._phase_line_follow,
            MissionPhase.WAIT_JUNCTION: self._phase_wait_junction,
            MissionPhase.TURNING: self._phase_turning,
            MissionPhase.LINE_REACQUIRED: self._phase_line_reacquired,
            MissionPhase.APPROACH_WALL: self._phase_approach_wall,
            MissionPhase.BACKOFF: self._phase_backoff,
            MissionPhase.GRAB: self._phase_grab,
        }

    @property
    def phase(self) -> MissionPhase:
        """Return the current mission state."""
        return self._phase

    def run(self) -> MissionPhase:
        """Execute one full mission; returns DONE or re-raises after abort."""
        try:
            self._set_phase(MissionPhase.IDLE)
            while self._phase is not MissionPhase.DONE:
                handler = self._handlers[self._phase]
                handler()
            return self._phase
        except KeyboardInterrupt:
            self.abort("Ctrl+C")
            raise
        except Exception as exc:
            self.abort(f"{type(exc).__name__}: {exc}")
            raise

    def abort(self, reason: str) -> None:
        """Enter ERROR, request a best-effort global STOP, log the abort."""
        if self._phase is MissionPhase.ERROR:
            return
        self._set_phase(MissionPhase.ERROR)
        self._emergency_stop(reason)
        if reason == "Ctrl+C":
            self._output("=== MISSION ABORTED (Ctrl+C) ===")
        else:
            self._output(f"=== MISSION ABORTED: {reason} ===")

    def _set_phase(self, phase: MissionPhase) -> None:
        self._phase = phase

    # --- startup ---

    def _phase_startup(self) -> None:
        """IDLE: safety STOP, preset the RIGHT junction action."""
        self._output("")
        self._output("=== GRAB MISSION START ===")
        self._output("")
        self._output("SAFETY STOP")
        self._output("TX: STOP")
        stop_ack = self._stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label="grab_mission_startup",
        )
        self._output(f"RX: {stop_ack}")
        self._send_with_ack(f"NEXT_TURN,{EXPECTED_TURN.value}")
        self._set_phase(MissionPhase.LINE_FOLLOW)

    # --- line follow and junction handling ---

    def _phase_line_follow(self) -> None:
        """LINE_START, then wait for the first confirmed junction."""
        self._output("")
        self._output("START LINE FOLLOW")
        self._send_with_ack("LINE_START")
        self._output("Waiting junction...")
        deadline = self._clock() + JUNCTION_TIMEOUT_S
        while self._clock() < deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            if (
                line.startswith(("TURN_STARTED,", "TURN_DONE,"))
                or line == "LINE_REACQUIRED"
            ):
                raise Stm32ProtocolError(
                    f"Out-of-order event before JUNCTION_DETECTED: {line}"
                )
            if line == "JUNCTION_DETECTED":
                self._output("JUNCTION DETECTED")
                self._set_phase(MissionPhase.WAIT_JUNCTION)
                return
        raise TimeoutError("Timeout waiting for: JUNCTION_DETECTED")

    def _phase_wait_junction(self) -> None:
        """Wait for TURN_STARTED within the shared turn deadline."""
        self._output("Waiting turn and line reacquisition...")
        self._turn_deadline = self._clock() + TURN_TIMEOUT_S
        while self._clock() < self._turn_deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            self._validate_turn_direction(line)
            if line.startswith("TURN_STARTED,"):
                self._set_phase(MissionPhase.TURNING)
                return
            if line.startswith("TURN_DONE,") or line == "LINE_REACQUIRED":
                raise Stm32ProtocolError(
                    f"Out-of-order event before TURN_STARTED: {line}"
                )
        raise TimeoutError("Timeout waiting for: TURN_STARTED")

    def _phase_turning(self) -> None:
        """Wait for TURN_DONE and LINE_REACQUIRED (either order, both RIGHT)."""
        assert self._turn_deadline is not None
        turn_done_seen = False
        reacquired_seen = False
        while self._clock() < self._turn_deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            self._validate_turn_direction(line)
            if line.startswith("TURN_DONE,"):
                if turn_done_seen:
                    raise Stm32ProtocolError(f"Duplicate TURN_DONE: {line}")
                self._output("TURN COMPLETE")
                turn_done_seen = True
            elif line == "LINE_REACQUIRED":
                if reacquired_seen:
                    raise Stm32ProtocolError("Duplicate LINE_REACQUIRED")
                self._output("LINE REACQUIRED")
                reacquired_seen = True
            elif line.startswith("TURN_STARTED,"):
                raise Stm32ProtocolError(f"Duplicate TURN_STARTED: {line}")
            if turn_done_seen and reacquired_seen:
                self._set_phase(MissionPhase.LINE_REACQUIRED)
                return
        raise TimeoutError(
            "Timeout waiting for: TURN_DONE and LINE_REACQUIRED"
        )

    def _phase_line_reacquired(self) -> None:
        """Timed line follow to the grab area, keeping the heading on the line."""
        self._output("Continue line following")
        self._output(f"Duration: {GRAB_AREA_APPROACH_TIME:.2f}s")
        deadline = self._clock() + GRAB_AREA_APPROACH_TIME
        while self._clock() < deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            self._validate_turn_direction(line)
            if line == "JUNCTION_DETECTED":
                raise Stm32ProtocolError(
                    "Unexpected second JUNCTION_DETECTED during wall approach"
                )
        self._output("LINE_STOP")
        self._send_with_ack("LINE_STOP")
        self._set_phase(MissionPhase.APPROACH_WALL)

    # --- wall approach and backoff ---

    def _phase_approach_wall(self) -> None:
        """VEL forward for WALL_APPROACH_TIME until the car presses the wall."""
        self._output("")
        self._output("Wall approach:")
        self._output(f"distance={TARGET_DISTANCE:.2f}m")
        self._output(f"time={WALL_APPROACH_TIME:.2f}s")
        self._drive_timed_distance(
            +VEL_COMMAND_SPEED,
            WALL_APPROACH_TIME,
            debug_label="wall_approach",
        )
        self._output("Wall reached")
        self._set_phase(MissionPhase.BACKOFF)

    def _phase_backoff(self) -> None:
        """VEL backward for BACKOFF_TIME to leave grab clearance."""
        self._output("")
        self._output("Backoff 5cm")
        self._output(
            f"distance={BACKOFF_DISTANCE:.2f}m  time={BACKOFF_TIME:.2f}s"
        )
        self._drive_timed_distance(
            -VEL_COMMAND_SPEED,
            BACKOFF_TIME,
            debug_label="backoff",
        )
        self._set_phase(MissionPhase.GRAB)

    # --- grab and completion ---

    def _phase_grab(self) -> None:
        """Delegate the verified arm-only orange grab to grab_task."""
        self._output("")
        self._output("Start arm grab")
        grab_orange(stm32=self._stm32, output=self._output)
        self._output("Grab complete")
        self._output("")
        self._output("=== MISSION SUCCESS ===")
        self._set_phase(MissionPhase.DONE)

    # --- shared helpers ---

    def _send_with_ack(self, command: str) -> str:
        """Send one protocol command and log its echoed ACK."""
        self._output(f"TX: {command}")
        sent_command = self._stm32.send_command(command)
        ack = self._stm32.wait_ack(sent_command, timeout=ACK_TIMEOUT_S)
        self._output(f"RX: {ack}")
        return ack

    def _drive_timed_distance(
        self,
        velocity_y: float,
        duration_s: float,
        *,
        debug_label: str,
    ) -> None:
        """Drive at one VEL.Y for a timed distance, refreshing for the watchdog.

        Follows the verified pattern of tests/test_old_grab_sequence.py:
        send VEL, wait for ACK,VEL, refresh every 100 ms, then STOP with ACK.
        """
        deadline = self._clock() + duration_s
        while self._clock() < deadline:
            refresh_due = min(deadline, self._clock() + VEL_REFRESH_S)
            command = f"VEL,0.00,{velocity_y:.2f},0.00"
            self._output(f"TX: {command}")
            self._stm32.send_command(command)
            ack = self._stm32.wait_ack("VEL", VEL_ACK_TIMEOUT_S)
            self._output(f"RX: {ack}")
            remaining_s = refresh_due - self._clock()
            if remaining_s > 0.0:
                self._guarded_delay(min(VEL_REFRESH_S, remaining_s))
        self._output("TX: STOP")
        stop_ack = self._stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label=f"{debug_label}_end",
        )
        self._output(f"RX: {stop_ack}")

    def _guarded_delay(self, seconds: float) -> None:
        """Wait while watching the serial stream for asynchronous ERR lines."""
        deadline = self._clock() + seconds
        while self._clock() < deadline:
            self._read_protocol_line()

    def _read_protocol_line(self) -> str | None:
        """Read one serial line, raising on STM32 ERR replies."""
        raw_line = self._stm32.readline()
        if not raw_line:
            return None
        line = raw_line.decode(errors="replace").strip()
        if not line:
            return None
        self._output(f"RX: {line}")
        if line.startswith("ERR,"):
            raise Stm32ProtocolError(f"STM32 error: {line}")
        return line

    @staticmethod
    def _validate_turn_direction(line: str) -> None:
        """Reject any turn event whose direction is not the expected RIGHT."""
        for prefix in ("TURN_STARTED,", "TURN_DONE,"):
            if not line.startswith(prefix):
                continue
            fields = line.split(",")
            if len(fields) != 2 or fields[1] != EXPECTED_TURN.value:
                raise Stm32ProtocolError(
                    f"Expected {prefix}{EXPECTED_TURN.value}, received {line}"
                )

    def _emergency_stop(self, reason: str) -> None:
        """Best-effort global STOP that never masks the original failure."""
        if not self._stm32.is_connected:
            return
        self._output(f"    Emergency STOP ({reason})")
        try:
            self._stm32.stop(require_ack=False, timeout=ACK_TIMEOUT_S)
        except Exception as exc:
            self._output(
                f"    Emergency STOP send failed: {type(exc).__name__}: {exc}"
            )


def run_grab_mission(
    stm32: Stm32Link | None = None,
    *,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> MissionPhase:
    """Execute one complete LINE_TO_GRAB_MISSION.

    Without ``stm32`` the mission opens its own serial link and closes it in
    ``finally``.  A caller-supplied link must already be connected and is
    left open afterwards, so the mission can share a navigation connection.
    Any serial failure, STM32 ``ERR`` reply or Ctrl+C enters the ERROR state,
    requests a best-effort STOP and re-raises the original exception.
    """
    owns_link = stm32 is None
    if owns_link:
        stm32 = Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)

    controller = GrabMissionController(stm32, output=output, clock=clock)
    try:
        if owns_link:
            stm32.connect()
        elif not stm32.is_connected:
            raise Stm32NotConnectedError(
                "supplied Stm32Link is not connected; call connect() first"
            )
        return controller.run()
    except KeyboardInterrupt:
        controller.abort("Ctrl+C")
        raise
    except Exception as exc:
        controller.abort(f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if owns_link:
            try:
                stm32.close()
            except Exception as exc:
                output(f"STM32 close failed: {type(exc).__name__}: {exc}")


def _default_stm32_factory() -> Stm32Link:
    return Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)


def _print_plan(output: Callable[[str], object]) -> None:
    output("RoboGame LINE_TO_GRAB_MISSION")
    output(f"target_distance={TARGET_DISTANCE:.3f} m")
    output(f"real_speed={REAL_SPEED:.3f} m/s")
    output(f"wall_approach_time={WALL_APPROACH_TIME:.2f} s")
    output(f"grab_area_approach_time={GRAB_AREA_APPROACH_TIME:.2f} s")
    output(f"backoff_distance={BACKOFF_DISTANCE:.2f} m")
    output(f"backoff_time={BACKOFF_TIME:.2f} s")
    output(f"junction_action={EXPECTED_TURN.value}")
    output(
        "Flow: STOP -> NEXT_TURN,RIGHT -> LINE_START -> "
        "JUNCTION_DETECTED -> TURN -> LINE_REACQUIRED -> timed line follow "
        "-> LINE_STOP -> VEL wall approach -> STOP -> backoff -> grab_orange()"
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    stm32_factory: Callable[[], Stm32Link] = _default_stm32_factory,
    input_fn: Callable[[str], str] = input,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Field entry: connect, execute one run_grab_mission(), release."""
    parser = argparse.ArgumentParser(
        description="Run the LINE_TO_GRAB_MISSION once."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the mission plan without opening UART",
    )
    args = parser.parse_args(argv)

    _print_plan(output)

    if args.dry_run:
        output("DRY RUN: no serial connection and no motion command sent.")
        return 0

    output("WARNING: this mission moves the chassis and the arm.")
    output("Clear the route first and keep a physical emergency stop ready.")
    output("Do not run another UART reader at the same time.")
    if input_fn("Type START to connect and begin: ") != "START":
        output("Cancelled: exact START confirmation not received.")
        return 2

    stm32: Stm32Link | None = None
    try:
        output("Connecting STM32...")
        stm32 = stm32_factory()
        run_grab_mission(stm32, output=output, clock=clock)
        return 0
    except KeyboardInterrupt:
        output("Ctrl+C: immediate STOP requested.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None:
            try:
                stm32.close()
            except Exception as exc:
                output(
                    f"FAULT: STM32 close failed: {type(exc).__name__}: {exc}"
                )


if __name__ == "__main__":
    raise SystemExit(main())
