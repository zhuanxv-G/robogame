"""Manual line-to-Tag arrival and topology-update integration test.

Importing this module has no hardware side effects. Run it explicitly and
type START before it opens the camera/serial link or sends LINE_START.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from competition.navigation_controller import (  # noqa: E402
    NavigationController,
    NavigationState,
)
from hardware.stm32_link import Stm32Link  # noqa: E402
from navigation.topological_navigation import (  # noqa: E402
    TopologicalLocation,
    TopologicalNavigator,
)
from vision.apriltag_service import AprilTagService  # noqa: E402


STATUS_INTERVAL_S = 1.0


class ObservedAprilTagInput:
    """Expose one AprilTagService while retaining its latest returned IDs.

    This observer owns no camera and performs no extra detection. The exact
    same detection list is passed to NavigationController, whose target and
    arrival logic remains authoritative.
    """

    def __init__(self, service: AprilTagService) -> None:
        self._service = service
        self.last_detections: list[dict[str, Any]] = []

    def detect(self) -> list[dict[str, Any]]:
        detections = self._service.detect()
        self.last_detections = [detection.copy() for detection in detections]
        return detections

    def detected_tag_ids(self) -> tuple[int, ...]:
        tag_ids: list[int] = []
        for detection in self.last_detections:
            tag_id = detection.get("id")
            if isinstance(tag_id, int) and not isinstance(tag_id, bool):
                tag_ids.append(tag_id)
        return tuple(tag_ids)


class StatusReporter:
    """Print navigation changes immediately and otherwise at a low rate."""

    def __init__(
        self,
        *,
        output: Callable[[str], object],
        clock: Callable[[], float],
        interval: float = STATUS_INTERVAL_S,
    ) -> None:
        self._output = output
        self._clock = clock
        self._interval = interval
        self._last_fingerprint: tuple[object, ...] | None = None
        self._last_print_time: float | None = None

    @staticmethod
    def _format_measurement(value: float | None) -> str:
        return "-" if value is None else f"{value:.1f}"

    def report(
        self,
        controller: NavigationController,
        observed_apriltag: ObservedAprilTagInput,
        *,
        force: bool = False,
    ) -> None:
        status = controller.get_navigation_status()
        detected_tag_ids = observed_apriltag.detected_tag_ids()
        fingerprint = (
            status["state"],
            status["target_tag_id"],
            status["tag_visible"],
            detected_tag_ids,
            status["confirm_count"],
        )
        now = self._clock()
        interval_elapsed = (
            self._last_print_time is None
            or now - self._last_print_time >= self._interval
        )
        if not force and fingerprint == self._last_fingerprint:
            if not interval_elapsed:
                return

        detected_tag = (
            "|".join(str(tag_id) for tag_id in detected_tag_ids)
            if detected_tag_ids
            else "-"
        )
        confirm_frames = status["confirm_frames"]
        self._output(
            f"navigation_state={status['state'].value} "
            f"target_tag={status['target_tag_id']} "
            f"tag_visible={int(status['tag_visible'])} "
            f"detected_tag={detected_tag} "
            f"error_x={self._format_measurement(status['error_x'])} "
            f"area={self._format_measurement(status['area'])} "
            f"confirm_count={status['confirm_count']}/"
            f"{confirm_frames if confirm_frames is not None else '-'}"
        )
        self._last_fingerprint = fingerprint
        self._last_print_time = now


def positive_int(value: str) -> int:
    """Parse a positive integer command-line value."""
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if result <= 0:
        raise argparse.ArgumentTypeError("must be greater than 0")
    return result


def positive_area(value: str) -> float:
    """Parse a finite positive AprilTag pixel-area threshold."""
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError(
            "must be a finite number greater than 0"
        )
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Manually verify STM32 line following, target-Tag arrival, and "
            "the resulting topological location update."
        )
    )
    parser.add_argument("--tag-id", type=positive_int, required=True)
    parser.add_argument(
        "--arrival-area",
        type=positive_area,
        required=True,
        help="field-calibrated target Tag pixel area (no built-in default)",
    )
    parser.add_argument(
        "--confirm-frames",
        type=positive_int,
        default=3,
        help="consecutive qualifying frames required (default: %(default)s)",
    )
    return parser.parse_args(argv)


def update_topology_after_arrival(
    controller: NavigationController,
    navigator: TopologicalNavigator,
    requested_tag_id: int,
) -> tuple[int, TopologicalLocation]:
    """Apply only the target Tag confirmed by NavigationController ARRIVED."""
    status = controller.get_navigation_status()
    if controller.current_state is not NavigationState.ARRIVED:
        raise RuntimeError("topology cannot update before ARRIVED")

    confirmed_tag = status["target_tag_id"]
    if confirmed_tag != requested_tag_id:
        raise RuntimeError(
            "ARRIVED target does not match the requested topology Tag"
        )
    if confirmed_tag is None:
        raise RuntimeError("ARRIVED has no confirmed target Tag")

    location = navigator.update_from_tag(confirmed_tag)
    if location is None:
        raise ValueError(
            f"Tag {confirmed_tag} has no known topological location"
        )
    return confirmed_tag, location


def run_live_test(
    args: argparse.Namespace,
    *,
    input_func: Callable[[str], str] = input,
    output: Callable[[str], object] = print,
    stm32_factory: Callable[[], Stm32Link] = Stm32Link,
    apriltag_factory: Callable[[], AprilTagService] = AprilTagService,
    controller_factory: Callable[
        [Stm32Link, ObservedAprilTagInput], NavigationController
    ] = NavigationController,
    navigator_factory: Callable[[], TopologicalNavigator] = (
        TopologicalNavigator
    ),
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Run the explicitly confirmed live integration using injected owners."""
    output("RoboGame navigation/topology manual live test")
    output(f"target_tag={args.tag_id}")
    output(f"arrival_area={args.arrival_area}")
    output(f"confirm_frames={args.confirm_frames}")
    output("WARNING: START will send LINE_START and move the chassis.")

    confirmation = input_func("Type START to continue: ").strip()
    if confirmation != "START":
        output("Cancelled; no hardware was opened and no command was sent.")
        return 0

    stm32 = stm32_factory()
    apriltag = apriltag_factory()
    observed_apriltag = ObservedAprilTagInput(apriltag)
    controller = controller_factory(stm32, observed_apriltag)
    navigator = navigator_factory()
    reporter = StatusReporter(output=output, clock=clock)

    try:
        # Camera first: a camera initialization failure cannot leave a moving
        # chassis. Constructors above do not open either hardware device.
        apriltag.connect()
        stm32.connect()
        controller.start_line_to_tag(
            args.tag_id,
            args.arrival_area,
            args.confirm_frames,
        )
        output("LINE_START sent; press Ctrl+C at any time to STOP.")
        reporter.report(controller, observed_apriltag, force=True)

        while True:
            state = controller.update()
            reporter.report(controller, observed_apriltag)

            if state is NavigationState.ARRIVED:
                confirmed_tag, location = update_topology_after_arrival(
                    controller,
                    navigator,
                    args.tag_id,
                )
                output(f"confirmed_tag={confirmed_tag}")
                output(f"current_location={location.name}")
                return 0

            if state is NavigationState.FAULT:
                status = controller.get_navigation_status()
                output(f"FAULT: {status['error'] or 'navigation fault'}")
                return 1
    except KeyboardInterrupt:
        output("Ctrl+C: stopping navigation.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        arrived = controller.current_state is NavigationState.ARRIVED
        if stm32.is_connected and not arrived:
            try:
                stm32.stop(require_ack=True)
                output("STOP acknowledged by STM32.")
            except Exception as exc:
                output(f"STOP failed: {type(exc).__name__}: {exc}")
        try:
            apriltag.close()
        except Exception as exc:
            output(f"Camera close failed: {type(exc).__name__}: {exc}")
        try:
            stm32.close()
        except Exception as exc:
            output(f"STM32 close failed: {type(exc).__name__}: {exc}")


def main(argv: list[str] | None = None) -> int:
    return run_live_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
