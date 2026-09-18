"""First complete fixed-route RoboGame competition mission.

This entry intentionally uses no K230D, AprilTag, visual localization, or
random block strategy.  It composes the existing STM32 LineNav junction
events with the protected grab-task action segments:

    start -> RIGHT junction -> LEFT junction -> grab and retreat
          -> fixed RIGHT rotation -> reacquire LineNav
          -> RIGHT junction -> build zone -> stop

The module owns at most one :class:`~hardware.stm32_link.Stm32Link`.  Importing
it has no hardware side effects; the live CLI requires the exact ``START``
gate.  ``--dry-run`` only prints the plan.
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

from competition.grab_mission import (  # noqa: E402
    ACK_TIMEOUT_S,
    BACKOFF_DISTANCE,
    BACKOFF_TIME,
    JUNCTION_TIMEOUT_S,
    LINE_DURATION,
    POST_PICK_RETREAT_DISTANCE,
    POST_PICK_RETREAT_TIME,
    SERIAL_POLL_TIMEOUT_S,
    TARGET_DISTANCE,
    TURN_TIMEOUT_S,
    VEL_ACK_TIMEOUT_S,
    VEL_COMMAND_SPEED,
    VEL_REFRESH_S,
)
from competition.grab_task import (  # noqa: E402
    PICK_ARM_STEPS,
    PLACE_ARM_STEPS,
    PREPARE_ARM_STEPS,
    PRE_PICK_SETTLE_STEPS,
    PRE_PLACE_SETTLE_STEPS,
    REST_ARM_STEPS,
    execute_grab_steps,
)
from hardware.stm32_link import (  # noqa: E402
    NextTurn,
    Stm32Link,
    Stm32NotConnectedError,
    Stm32ProtocolError,
)


# Fixed return-to-line calibration.  These values deliberately reuse the
# current STM32 RIGHT-turn baseline and remain a single adjustment point for
# later field calibration.  They are not claimed as full-mission validation.
TURN_BACK_TO_LINE_TIME = 11.5
TURN_BACK_TO_LINE_OMEGA = -0.80


class FullMissionState(str, Enum):
    """Observable phases of the fixed-route mission."""

    START = "START"
    LINE_TO_GRAB = "LINE_TO_GRAB"
    TURN_TO_GRAB = "TURN_TO_GRAB"
    GRAB = "GRAB"
    RETURN_TO_LINE = "RETURN_TO_LINE"
    LINE_TO_BUILD = "LINE_TO_BUILD"
    TURN_TO_BUILD = "TURN_TO_BUILD"
    FINISHED = "FINISHED"
    FAULT = "FAULT"


class FullMission:
    """Run the no-vision fixed route using one shared STM32 connection."""

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
        self._state = FullMissionState.START
        self._handlers: dict[FullMissionState, Callable[[], None]] = {
            FullMissionState.START: self._run_start,
            FullMissionState.LINE_TO_GRAB: self._run_line_to_grab,
            FullMissionState.TURN_TO_GRAB: self._run_turn_to_grab,
            FullMissionState.GRAB: self._run_grab,
            FullMissionState.RETURN_TO_LINE: self._run_return_to_line,
            FullMissionState.LINE_TO_BUILD: self._run_line_to_build,
            FullMissionState.TURN_TO_BUILD: self._run_turn_to_build,
        }

    @property
    def state(self) -> FullMissionState:
        """Return the current top-level mission state."""
        return self._state

    def run(self) -> FullMissionState:
        """Run until FINISHED, or STOP and re-raise on any failure."""
        if not self._stm32.is_connected:
            raise Stm32NotConnectedError(
                "supplied Stm32Link is not connected; call connect() first"
            )

        self._output("[MISSION]")
        self._output("Starting")
        self._log_state()
        try:
            while self._state is not FullMissionState.FINISHED:
                self._handlers[self._state]()
            self._output("[MISSION]")
            self._output("Finished")
            return self._state
        except KeyboardInterrupt:
            self._abort("Ctrl+C")
            raise
        except Exception as exc:
            self._abort(f"{type(exc).__name__}: {exc}")
            raise

    def _set_state(self, state: FullMissionState) -> None:
        self._state = state
        self._log_state()

    def _log_state(self) -> None:
        self._output(f"[STATE] {self._state.value}")

    # --- fixed route states ---

    def _run_start(self) -> None:
        self._output("[SAFETY] Initial STOP")
        self._stop_confirmed("full_mission_start")
        self._send_with_ack(f"NEXT_TURN,{NextTurn.RIGHT.value}")
        self._send_with_ack("LINE_START")
        self._set_state(FullMissionState.LINE_TO_GRAB)

    def _run_line_to_grab(self) -> None:
        self._output("[LINE]")
        self._output("Waiting first junction")
        self._wait_for_junction()
        self._output("[TURN]")
        self._output("Right turn")
        self._wait_for_turn(NextTurn.RIGHT)

        # LineNav resumes automatically after TURN_DONE.  Preset only after
        # the first turn has completed so the pending action cannot be
        # overwritten early.
        self._send_with_ack(f"NEXT_TURN,{NextTurn.LEFT.value}")
        self._output("[LINE]")
        self._output("Waiting second junction")
        self._wait_for_junction()
        self._set_state(FullMissionState.TURN_TO_GRAB)

    def _run_turn_to_grab(self) -> None:
        self._output("[TURN]")
        self._output("Left turn")
        self._wait_for_turn(NextTurn.LEFT)
        self._set_state(FullMissionState.GRAB)

    def _run_grab(self) -> None:
        self._output("[GRAB]")
        self._output("Starting grab sequence")

        # This is the existing grab_mission choreography.  All Servo,
        # Stepper, waits, chassis distances and their ordering come from the
        # protected grab modules imported above.
        self._send_with_ack("LINE_STOP")
        execute_grab_steps(
            self._stm32,
            PREPARE_ARM_STEPS,
            output=self._output,
            clock=self._clock,
        )

        self._output(
            f"[GRAB] Line approach {TARGET_DISTANCE:.2f} m "
            f"({LINE_DURATION:.2f} s)"
        )
        line_started_at = self._clock()
        self._send_with_ack("LINE_START")
        self._wait_wall_approach(line_started_at + LINE_DURATION)
        self._send_with_ack("LINE_STOP")

        self._output(f"[GRAB] Backoff {BACKOFF_DISTANCE:.2f} m")
        self._drive_timed_velocity(
            y=-VEL_COMMAND_SPEED,
            omega=0.0,
            duration_s=BACKOFF_TIME,
            stop_label="full_mission_grab_backoff",
        )
        execute_grab_steps(
            self._stm32,
            PRE_PICK_SETTLE_STEPS,
            output=self._output,
            clock=self._clock,
        )
        execute_grab_steps(
            self._stm32,
            PICK_ARM_STEPS,
            output=self._output,
            clock=self._clock,
        )

        self._output(
            f"[GRAB] Post-pick retreat {POST_PICK_RETREAT_DISTANCE:.2f} m"
        )
        self._drive_timed_velocity(
            y=-VEL_COMMAND_SPEED,
            omega=0.0,
            duration_s=POST_PICK_RETREAT_TIME,
            stop_label="full_mission_grab_retreat",
        )
        execute_grab_steps(
            self._stm32,
            PRE_PLACE_SETTLE_STEPS,
            output=self._output,
            clock=self._clock,
        )
        execute_grab_steps(
            self._stm32,
            PLACE_ARM_STEPS,
            output=self._output,
            clock=self._clock,
        )
        execute_grab_steps(
            self._stm32,
            REST_ARM_STEPS,
            output=self._output,
            clock=self._clock,
        )
        self._stop_confirmed("full_mission_grab_complete")
        self._output("[GRAB] Complete; chassis retreated about 0.35 m")
        self._set_state(FullMissionState.RETURN_TO_LINE)

    def _run_return_to_line(self) -> None:
        self._output("[TURN]")
        self._output(
            "Right turn back to line "
            f"(fixed {TURN_BACK_TO_LINE_TIME:.2f} s)"
        )
        self._drive_timed_velocity(
            y=0.0,
            omega=TURN_BACK_TO_LINE_OMEGA,
            duration_s=TURN_BACK_TO_LINE_TIME,
            stop_label="full_mission_return_turn",
        )

        self._output("[LINE]")
        self._output("Re-entering line-follow mode")
        self._send_with_ack(f"NEXT_TURN,{NextTurn.RIGHT.value}")
        self._send_with_ack("LINE_START")
        self._set_state(FullMissionState.LINE_TO_BUILD)

    def _run_line_to_build(self) -> None:
        self._output("[LINE]")
        self._output("Waiting third junction")
        self._wait_for_junction()
        self._set_state(FullMissionState.TURN_TO_BUILD)

    def _run_turn_to_build(self) -> None:
        self._output("[TURN]")
        self._output("Right turn")
        self._wait_for_turn(NextTurn.RIGHT)
        self._stop_confirmed("full_mission_build_zone")
        self._output("BUILD_ZONE_REACHED")
        self._set_state(FullMissionState.FINISHED)

    # --- LineNav protocol helpers ---

    def _wait_for_junction(self) -> None:
        deadline = self._clock() + JUNCTION_TIMEOUT_S
        while self._clock() < deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            if line == "JUNCTION_DETECTED":
                self._output("[LINE] JUNCTION_DETECTED")
                return
            if line.startswith(("TURN_STARTED,", "TURN_DONE,")) or (
                line == "LINE_REACQUIRED"
            ):
                raise Stm32ProtocolError(
                    f"Out-of-order event before JUNCTION_DETECTED: {line}"
                )
        raise TimeoutError("Timeout waiting for: JUNCTION_DETECTED")

    def _wait_for_turn(self, expected: NextTurn) -> None:
        deadline = self._clock() + TURN_TIMEOUT_S
        turn_started = False
        turn_done = False
        line_reacquired = False

        while self._clock() < deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            self._validate_turn_direction(line, expected)

            if line.startswith("TURN_STARTED,"):
                if turn_started:
                    raise Stm32ProtocolError(f"Duplicate TURN_STARTED: {line}")
                turn_started = True
                self._output(f"[TURN] TURN_STARTED,{expected.value}")
                continue

            if line.startswith("TURN_DONE,"):
                if not turn_started:
                    raise Stm32ProtocolError(
                        f"Out-of-order event before TURN_STARTED: {line}"
                    )
                if turn_done:
                    raise Stm32ProtocolError(f"Duplicate TURN_DONE: {line}")
                turn_done = True
                self._output(f"[TURN] TURN_DONE,{expected.value}")
            elif line == "LINE_REACQUIRED":
                if not turn_started:
                    raise Stm32ProtocolError(
                        "Out-of-order LINE_REACQUIRED before TURN_STARTED"
                    )
                if line_reacquired:
                    raise Stm32ProtocolError("Duplicate LINE_REACQUIRED")
                line_reacquired = True
                self._output("[LINE] LINE_REACQUIRED")
            elif line == "JUNCTION_DETECTED":
                raise Stm32ProtocolError(
                    "Duplicate JUNCTION_DETECTED during turn"
                )

            if turn_done and line_reacquired:
                return

        raise TimeoutError(
            f"Timeout waiting for TURN_DONE,{expected.value} "
            "and LINE_REACQUIRED"
        )

    @staticmethod
    def _validate_turn_direction(line: str, expected: NextTurn) -> None:
        for prefix in ("TURN_STARTED,", "TURN_DONE,"):
            if not line.startswith(prefix):
                continue
            fields = line.split(",")
            if len(fields) != 2 or fields[1] != expected.value:
                raise Stm32ProtocolError(
                    f"Expected {prefix}{expected.value}, received {line}"
                )

    # --- shared motion and safety helpers ---

    def _wait_wall_approach(self, deadline: float) -> None:
        while self._clock() < deadline:
            line = self._read_protocol_line()
            if line is None:
                continue
            if (
                line == "JUNCTION_DETECTED"
                or line == "LINE_REACQUIRED"
                or line.startswith(("TURN_STARTED,", "TURN_DONE,"))
            ):
                raise Stm32ProtocolError(
                    f"Unexpected LineNav event during wall approach: {line}"
                )

    def _drive_timed_velocity(
        self,
        *,
        y: float,
        omega: float,
        duration_s: float,
        stop_label: str,
    ) -> None:
        deadline = self._clock() + duration_s
        while self._clock() < deadline:
            refresh_due = min(deadline, self._clock() + VEL_REFRESH_S)
            command = f"VEL,0.00,{y:.2f},{omega:.2f}"
            self._output(f"TX: {command}")
            self._stm32.send_command(command)
            ack = self._stm32.wait_ack("VEL", VEL_ACK_TIMEOUT_S)
            self._output(f"RX: {ack}")
            remaining_s = refresh_due - self._clock()
            if remaining_s > 0.0:
                self._guarded_delay(min(VEL_REFRESH_S, remaining_s))
        self._stop_confirmed(stop_label)

    def _guarded_delay(self, seconds: float) -> None:
        deadline = self._clock() + seconds
        while self._clock() < deadline:
            self._read_protocol_line()

    def _send_with_ack(self, command: str) -> str:
        self._output(f"TX: {command}")
        sent = self._stm32.send_command(command)
        ack = self._stm32.wait_ack(sent, ACK_TIMEOUT_S)
        self._output(f"RX: {ack}")
        return ack

    def _stop_confirmed(self, label: str) -> str:
        self._output("TX: STOP")
        ack = self._stm32.stop(
            require_ack=True,
            timeout=ACK_TIMEOUT_S,
            debug_label=label,
        )
        if ack is None:  # Stm32Link promises a string when ACK is required.
            raise Stm32ProtocolError("STOP completed without ACK,STOP")
        self._output(f"RX: {ack}")
        return ack

    def _read_protocol_line(self) -> str | None:
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

    def _abort(self, reason: str) -> None:
        if self._state is FullMissionState.FAULT:
            return
        self._state = FullMissionState.FAULT
        self._log_state()
        self._output(f"[SAFETY] Emergency STOP ({reason})")
        if self._stm32.is_connected:
            try:
                self._stm32.stop(
                    require_ack=False,
                    timeout=ACK_TIMEOUT_S,
                    debug_label="full_mission_fault",
                )
            except Exception as stop_exc:
                self._output(
                    "[SAFETY] STOP send failed: "
                    f"{type(stop_exc).__name__}: {stop_exc}"
                )
        self._output(f"[MISSION] Aborted: {reason}")


def run_full_mission(
    stm32: Stm32Link | None = None,
    *,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> FullMissionState:
    """Run the fixed route, optionally owning the single STM32 link."""
    owns_link = stm32 is None
    if owns_link:
        stm32 = Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)

    mission = FullMission(stm32, output=output, clock=clock)
    try:
        if owns_link:
            stm32.connect()
        return mission.run()
    finally:
        if owns_link:
            try:
                stm32.close()
            except Exception as exc:
                output(f"STM32 close failed: {type(exc).__name__}: {exc}")


def _default_stm32_factory() -> Stm32Link:
    return Stm32Link(timeout=SERIAL_POLL_TIMEOUT_S)


def _print_plan(output: Callable[[str], object]) -> None:
    output("RoboGame FULL FIXED ROUTE MISSION")
    output("route=RIGHT -> LEFT -> GRAB -> fixed RIGHT -> RIGHT -> BUILD")
    output(f"grab_line_approach={TARGET_DISTANCE:.2f} m")
    output(
        "grab_retreat="
        f"{BACKOFF_DISTANCE + POST_PICK_RETREAT_DISTANCE:.2f} m"
    )
    output(f"turn_back_to_line_time={TURN_BACK_TO_LINE_TIME:.2f} s")
    output(f"turn_back_to_line_omega={TURN_BACK_TO_LINE_OMEGA:.2f} rad/s")
    output("build_action=disabled (stop after BUILD_ZONE_REACHED)")


def main(
    argv: Sequence[str] | None = None,
    *,
    stm32_factory: Callable[[], Stm32Link] = _default_stm32_factory,
    input_fn: Callable[[str], str] = input,
    output: Callable[[str], object] = print,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Live field entry with dry-run and exact START safety gates."""
    parser = argparse.ArgumentParser(
        description="Run the first no-vision fixed-route competition mission."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the route and calibration without opening STM32",
    )
    args = parser.parse_args(argv)

    _print_plan(output)
    if args.dry_run:
        output("DRY RUN complete: no serial port opened, no commands sent.")
        return 0

    output("WARNING: this entry drives the chassis, Servo and Stepper.")
    output("WARNING: clear the field and prepare the physical emergency stop.")
    if input_fn("输入 START 才会连接并执行完整比赛流程: ").strip() != "START":
        output("已取消：未打开串口，未发送任何命令。")
        return 2

    stm32: Stm32Link | None = None
    try:
        stm32 = stm32_factory()
        stm32.connect()
        result = run_full_mission(stm32, output=output, clock=clock)
        return 0 if result is FullMissionState.FINISHED else 1
    except KeyboardInterrupt:
        output("Ctrl+C: mission stopped safely.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None:
            try:
                stm32.close()
            except Exception as exc:
                output(f"STM32 close failed: {type(exc).__name__}: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
