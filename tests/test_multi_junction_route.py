"""Operator-confirmed live test for one complete multi-junction route.

Importing this module is hardware-safe.  STM32 and AprilTag services are
constructed and connected only after the operator types the exact uppercase
word ``START``.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from competition.navigation_controller import NavigationController  # noqa: E402
from hardware.stm32_link import LineNavigationEvent, Stm32Link  # noqa: E402
from navigation.route_action_resolver import (  # noqa: E402
    Heading,
    RouteActionResolver,
)
from navigation.route_executor import (  # noqa: E402
    DEFAULT_FINAL_TAG_TIMEOUT_S,
    DEFAULT_JUNCTION_WAIT_TIMEOUT_S,
    DEFAULT_TURN_DONE_TIMEOUT_S,
    RouteExecutor,
    RouteExecutorConfig,
    RouteExecutorState,
    RoutePlan,
    build_route_plan,
)
from navigation.topological_navigation import (  # noqa: E402
    TopologicalLocation,
    TopologicalNavigator,
)
from vision.apriltag_service import AprilTagService  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manually run one topology route through multiple junctions",
    )
    locations = tuple(location.value for location in TopologicalLocation)
    parser.add_argument("--start", required=True, choices=locations)
    parser.add_argument("--target", required=True, choices=locations)
    parser.add_argument("--target-tag", required=True, type=int)
    parser.add_argument(
        "--arrival-area",
        required=True,
        type=float,
        help="explicit, field-verified pixel-area threshold for target Tag",
    )
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument(
        "--heading",
        choices=tuple(heading.value for heading in Heading),
        help="required when --start is itself a junction",
    )
    parser.add_argument(
        "--junction-timeout",
        type=float,
        default=DEFAULT_JUNCTION_WAIT_TIMEOUT_S,
    )
    parser.add_argument(
        "--turn-timeout",
        type=float,
        default=DEFAULT_TURN_DONE_TIMEOUT_S,
    )
    parser.add_argument(
        "--tag-timeout",
        type=float,
        default=DEFAULT_FINAL_TAG_TIMEOUT_S,
    )
    return parser.parse_args(argv)


def prepare_preview(
    args: argparse.Namespace,
) -> tuple[TopologicalNavigator, RouteActionResolver, RoutePlan]:
    """Prepare the printable plan without constructing hardware services."""
    navigator = TopologicalNavigator()
    resolver = RouteActionResolver()
    plan = build_route_plan(
        navigator,
        resolver,
        args.start,
        args.target,
        args.target_tag,
        args.arrival_area,
        args.confirm_frames,
        current_heading=args.heading,
    )
    return navigator, resolver, plan


def print_plan(plan: RoutePlan, output: Callable[[str], object]) -> None:
    output("start_location:")
    output(plan.route_nodes[0].value)
    output("target_location:")
    output(plan.target_location.value)
    output("route:")
    output(" -> ".join(node.value for node in plan.route_nodes))
    output("actions:")
    output(
        " -> ".join(action.value for action in plan.turn_actions)
        if plan.turn_actions
        else "(none)"
    )
    output("target_tag:")
    output(str(plan.target_tag))
    output("arrival_area:")
    output(str(plan.arrival_area))
    output("confirm_frames:")
    output(str(plan.confirm_frames))


def format_event(event: LineNavigationEvent) -> str:
    if event.turn is None:
        return event.event_type.value
    return f"{event.event_type.value},{event.turn.value}"


def run_test(
    args: argparse.Namespace,
    *,
    input_fn: Callable[[str], str] = input,
    stm32_factory: Callable[[], Stm32Link] = Stm32Link,
    apriltag_factory: Callable[[], AprilTagService] = AprilTagService,
    navigation_factory: Callable[
        [Stm32Link, AprilTagService], NavigationController
    ] = NavigationController,
    output: Callable[[str], object] = print,
) -> int:
    """Run one route only after exact operator confirmation."""
    try:
        navigator, resolver, preview = prepare_preview(args)
        config = RouteExecutorConfig(
            junction_wait_timeout_s=args.junction_timeout,
            turn_done_timeout_s=args.turn_timeout,
            final_tag_timeout_s=args.tag_timeout,
        )
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1

    print_plan(preview, output)
    output("The robot may traverse multiple junctions after confirmation.")
    output("Keep the emergency stop ready and run no other STM32 reader.")
    if input_fn("Type START to connect and begin: ") != "START":
        output("Cancelled: exact START confirmation not received.")
        return 2

    stm32: Stm32Link | None = None
    apriltag: AprilTagService | None = None
    executor: RouteExecutor | None = None
    finished = False
    try:
        stm32 = stm32_factory()
        apriltag = apriltag_factory()
        stm32.connect()
        apriltag.connect()

        navigation = navigation_factory(stm32, apriltag)
        executor = RouteExecutor(
            navigator,
            resolver,
            navigation,
            stm32,
            config=config,
        )
        plan = executor.prepare_route(
            args.start,
            args.target,
            args.target_tag,
            args.arrival_area,
            args.confirm_frames,
            current_heading=args.heading,
        )

        output("ROUTE START")
        state = executor.start()
        if plan.turn_actions:
            _print_action(executor, output)
        else:
            output("ALL JUNCTION ACTIONS DONE")
            output(f"SEARCHING TARGET TAG {plan.target_tag}")

        while state not in {
            RouteExecutorState.FINISHED,
            RouteExecutorState.FAULT,
        }:
            previous_state = state
            previous_index = executor.current_action_index
            state = executor.poll()

            if executor.last_event is not None:
                output(format_event(executor.last_event))
            if executor.current_action_index > previous_index:
                if executor.current_action is None:
                    output("ALL JUNCTION ACTIONS DONE")
                else:
                    _print_action(executor, output)
            if (
                state is RouteExecutorState.FINAL_TAG_APPROACH
                and previous_state is not RouteExecutorState.FINAL_TAG_APPROACH
            ):
                output(f"SEARCHING TARGET TAG {plan.target_tag}")
            if state is RouteExecutorState.ARRIVED:
                output(f"TAG {plan.target_tag} ARRIVED")
                output(
                    "current_location="
                    f"{navigator.current_location.value}"
                )

        if state is RouteExecutorState.FAULT:
            raise RuntimeError(executor.get_status()["error"] or "route fault")

        output("ROUTE FINISHED")
        finished = True
        return 0
    except KeyboardInterrupt:
        output("Ctrl+C: STOP requested.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None and stm32.is_connected and not finished:
            try:
                stm32.stop(
                    timeout=0.5,
                    debug_label="multi_junction_route_cleanup",
                )
            except Exception as exc:
                output(f"FAULT: STOP failed: {type(exc).__name__}: {exc}")
        if apriltag is not None:
            try:
                apriltag.close()
            except Exception as exc:
                output(f"FAULT: AprilTag close failed: {type(exc).__name__}: {exc}")
        if stm32 is not None:
            try:
                stm32.close()
            except Exception as exc:
                output(f"FAULT: STM32 close failed: {type(exc).__name__}: {exc}")


def _print_action(
    executor: RouteExecutor,
    output: Callable[[str], object],
) -> None:
    plan = executor.plan
    action = executor.current_action
    if plan is None or action is None:
        return
    output(
        f"action {executor.current_action_index + 1}/"
        f"{len(plan.turn_actions)}:"
    )
    output(action.value)


def main(argv: Sequence[str] | None = None) -> int:
    return run_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
