"""Manual pre-match Stepper raise for the RoboGame robot.

This is a fixed relative upward move, not homing.  The lift axis has no limit
switch or encoder, so running this script repeatedly keeps moving it upward.
Importing this module does not open the serial port or move hardware.
"""

from __future__ import annotations

from collections.abc import Callable

from hardware.stm32_link import Stm32Link


# Keep these aligned with ArmSequence.startup_raise().
STARTUP_RAISE_STEPS = -1500
STARTUP_RAISE_PPS = 800

HELLO_VERSION = 1
ACK_TIMEOUT_S = 2.0


def _emergency_stop(
    stm32: Stm32Link,
    *,
    reason: str,
    output: Callable[[str], object],
) -> None:
    """Request a best-effort STOP without hiding the original failure."""
    if not stm32.is_connected:
        return

    output(f"Safety STOP ({reason})")
    output("TX: STOP")
    try:
        reply = stm32.stop(
            require_ack=False,
            timeout=ACK_TIMEOUT_S,
            debug_label="robot_startup_error",
        )
        if reply is not None:
            output(f"RX: {reply}")
    except Exception as stop_error:
        output(
            "STOP failed: "
            f"{type(stop_error).__name__}: {stop_error}"
        )


def run_startup_raise(
    stm32: Stm32Link,
    *,
    output: Callable[[str], object] = print,
) -> None:
    """Handshake, perform one fixed relative raise, then stop with ACK."""
    hello_command = f"HELLO_STM32,{HELLO_VERSION}"
    output(f"TX: {hello_command}")
    hello_reply = stm32.hello(
        version=HELLO_VERSION,
        timeout=ACK_TIMEOUT_S,
    )
    output(f"RX: {hello_reply}")

    stepper_command = (
        f"STEPPER_MOVE,{STARTUP_RAISE_STEPS},{STARTUP_RAISE_PPS}"
    )
    output(f"TX: {stepper_command}")
    stm32.send_command(stepper_command)

    stepper_ack = stm32.wait_ack(stepper_command, timeout=ACK_TIMEOUT_S)
    output(f"RX ACK: {stepper_ack}")

    nominal_seconds = abs(STARTUP_RAISE_STEPS) / float(STARTUP_RAISE_PPS)
    done_timeout_s = nominal_seconds * 2.0 + 8.0
    stepper_done = stm32.wait_done(
        f"STEPPER_MOVE_DONE,{STARTUP_RAISE_STEPS}",
        timeout=done_timeout_s,
    )
    output(f"RX DONE: {stepper_done}")

    output("TX: STOP")
    stop_reply = stm32.stop(
        require_ack=True,
        timeout=ACK_TIMEOUT_S,
        debug_label="robot_startup_complete",
    )
    output(f"RX: {stop_reply}")


def main(
    *,
    stm32_factory: Callable[[], Stm32Link] = Stm32Link,
    output: Callable[[str], object] = print,
) -> int:
    """Connect once, execute the manual startup raise, and close the link."""
    output("Starting robot startup raise")
    output(
        "WARNING: fixed relative raise only; this is not homing and has "
        "no limit-switch protection."
    )

    stm32: Stm32Link | None = None
    exit_code = 0
    try:
        stm32 = stm32_factory()
        output("Connecting STM32...")
        stm32.connect()
        output("STM32 connected")

        run_startup_raise(stm32, output=output)
        output("Robot startup raise completed")
    except KeyboardInterrupt:
        if stm32 is not None:
            _emergency_stop(stm32, reason="Ctrl+C", output=output)
        output("Robot startup raise aborted by Ctrl+C")
        exit_code = 130
    except Exception as exc:
        if stm32 is not None:
            _emergency_stop(
                stm32,
                reason=type(exc).__name__,
                output=output,
            )
        output(f"Robot startup raise failed: {type(exc).__name__}: {exc}")
        exit_code = 1
    finally:
        if stm32 is not None:
            try:
                stm32.close()
                output("STM32 serial closed")
            except Exception as close_error:
                output(
                    "STM32 close failed: "
                    f"{type(close_error).__name__}: {close_error}"
                )
                if exit_code == 0:
                    exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
