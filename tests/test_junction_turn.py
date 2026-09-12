"""Operator-confirmed live test for one STM32 junction action.

This module is safe to import. It connects to hardware and sends ``LINE_START``
only after the operator types the exact uppercase word ``START``.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hardware.stm32_link import (  # noqa: E402
    LineNavigationEventType,
    NextTurn,
    Stm32Link,
    Stm32ProtocolError,
    parse_line_navigation_event,
)
from navigation.route_action_resolver import (  # noqa: E402
    Heading,
    RouteAction,
    RouteActionResolver,
    RouteActionUnknownError,
)
from navigation.topological_navigation import (  # noqa: E402
    TopologicalLocation,
    TopologicalNavigator,
)


@dataclass(frozen=True, slots=True)
class JunctionTestPlan:
    """Hardware-free result prepared before operator confirmation."""

    route: tuple[TopologicalLocation, ...]
    actions: tuple[RouteAction, ...]
    next_action: RouteAction
    target: TopologicalLocation | None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manually test one confirmed-junction action",
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--turn",
        choices=("LEFT", "RIGHT", "STRAIGHT"),
        help="pending action to execute at the next real junction",
    )
    mode.add_argument(
        "--start",
        choices=tuple(location.value for location in TopologicalLocation),
        help="current topological node for route mode",
    )
    parser.add_argument(
        "--target",
        choices=tuple(location.value for location in TopologicalLocation),
        help="target topological node for route mode",
    )
    parser.add_argument(
        "--heading",
        choices=tuple(heading.value for heading in Heading),
        help="required when route mode starts at a junction",
    )
    args = parser.parse_args(argv)

    if args.start is not None and args.target is None:
        parser.error("--target is required with --start")
    if args.start is None and args.target is not None:
        parser.error("--target is valid only with --start")
    if args.start is None and args.heading is not None:
        parser.error("--heading is valid only with --start/--target")
    return args


def prepare_test_plan(args: argparse.Namespace) -> JunctionTestPlan:
    """Build the manual or topology-derived test plan without hardware."""
    requested_turn = getattr(args, "turn", None)
    if requested_turn is not None:
        action = RouteAction(requested_turn)
        return JunctionTestPlan((), (action,), action, None)

    start = getattr(args, "start", None)
    target = getattr(args, "target", None)
    if start is None or target is None:
        raise ValueError("choose --turn or provide both --start and --target")

    navigator = TopologicalNavigator()
    route = navigator.plan_route(start, target)
    heading_value = getattr(args, "heading", None)
    actions = RouteActionResolver().resolve_route_actions(
        route,
        current_heading=heading_value,
    )
    if not actions:
        raise RouteActionUnknownError(
            "route contains no junction action to test"
        )
    return JunctionTestPlan(
        tuple(route),
        tuple(actions),
        actions[0],
        TopologicalLocation(target),
    )


def _print_test_plan(
    plan: JunctionTestPlan,
    output: Callable[[str], object],
) -> None:
    output("route:")
    if plan.route:
        output(" -> ".join(node.value for node in plan.route))
    else:
        output("(manual next junction)")
    output("actions:")
    output(" -> ".join(action.value for action in plan.actions))
    output("next_action:")
    output(plan.next_action.value)
    output("target:")
    output(
        plan.target.value
        if plan.target is not None
        else "(manual next junction)"
    )


def run_test(
    args: argparse.Namespace,
    *,
    input_fn: Callable[[str], str] = input,
    stm32_factory: Callable[[], Stm32Link] = Stm32Link,
    output: Callable[[str], object] = print,
) -> int:
    """Run the live test after an exact operator confirmation."""
    try:
        plan = prepare_test_plan(args)
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1

    output("RoboGame junction turn live test")
    _print_test_plan(plan, output)
    if plan.next_action is RouteAction.STRAIGHT:
        output("STRAIGHT is permitted only where a real straight exit exists.")
    output("Expected: normal line -> junction -> turn -> reacquire -> line follow")
    output("The robot may move after confirmation. Keep emergency stop ready.")

    if input_fn("Type START to connect and begin: ") != "START":
        output("Cancelled: exact START confirmation not received.")
        return 2

    stm32: Stm32Link | None = None
    motion_stopped = False
    try:
        stm32 = stm32_factory()
        stm32.connect()

        turn_ack = stm32.set_next_turn(NextTurn(plan.next_action.value))
        output(turn_ack)

        line_command = stm32.send_command("LINE_START")
        line_ack = stm32.wait_ack(line_command)
        output(line_ack)
        output("LineNav active; press Ctrl+C to STOP and exit.")

        while True:
            raw_line = stm32.readline()
            if not raw_line:
                continue
            line = raw_line.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("ERR,"):
                raise Stm32ProtocolError(f"STM32 error: {line}")

            event = parse_line_navigation_event(line)
            if event is not None:
                output(line)
                if event.event_type is LineNavigationEventType.TURN_DONE:
                    stop_ack = stm32.stop(
                        require_ack=True,
                        debug_label="junction_test_turn_done",
                    )
                    motion_stopped = True
                    if stop_ack is not None:
                        output(stop_ack)
                    return 0
            elif line.startswith(("LINE_", "JUNCTION_", "TURN_")):
                output(line)
    except KeyboardInterrupt:
        output("Ctrl+C: STOP requested.")
        return 130
    except Exception as exc:
        output(f"FAULT: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if stm32 is not None:
            if stm32.is_connected and not motion_stopped:
                try:
                    stm32.stop(timeout=0.5, debug_label="junction_test_cleanup")
                except Exception as exc:
                    output(f"FAULT: STOP failed: {type(exc).__name__}: {exc}")
            try:
                stm32.close()
            except Exception as exc:
                output(f"FAULT: STM32 close failed: {type(exc).__name__}: {exc}")


def main(argv: Sequence[str] | None = None) -> int:
    return run_test(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
