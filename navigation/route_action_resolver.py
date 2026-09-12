"""Convert confirmed topology routes into junction actions without hardware.

The resolver reads only discrete field-model metadata.  It neither opens a
device nor sends protocol commands, and it never substitutes STRAIGHT when a
direction is missing.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from enum import Enum

from .field_model import (
    TOPOLOGY_EDGE_DIRECTIONS,
    TOPOLOGY_EDGES,
    TOPOLOGY_NODES,
)
from .topological_navigation import TopologicalLocation


class Heading(str, Enum):
    """Discrete headings in the project coordinate convention."""

    NORTH = "NORTH"
    EAST = "EAST"
    SOUTH = "SOUTH"
    WEST = "WEST"


class RouteAction(str, Enum):
    """One action that STM32 can execute at a confirmed junction."""

    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STRAIGHT = "STRAIGHT"


class RouteActionUnknownError(RuntimeError):
    """Raised when confirmed metadata cannot determine a route action."""


_OPPOSITE_HEADING = {
    Heading.NORTH: Heading.SOUTH,
    Heading.EAST: Heading.WEST,
    Heading.SOUTH: Heading.NORTH,
    Heading.WEST: Heading.EAST,
}
_CLOCKWISE_HEADINGS = (
    Heading.NORTH,
    Heading.EAST,
    Heading.SOUTH,
    Heading.WEST,
)


class RouteActionResolver:
    """Resolve topology edges and routes into LEFT/RIGHT/STRAIGHT actions.

    Route actions are emitted only for nodes marked ``kind=junction``.  A
    corridor node therefore does not consume a pending turn.  If the route
    starts at a junction, ``current_heading`` is required so that the action
    at that first node is not guessed.  If it starts elsewhere, the first
    confirmed edge supplies the arrival heading for the first junction.
    """

    def __init__(
        self,
        *,
        edge_directions: Mapping[tuple[str, str], str | Heading] | None = None,
        topology_edges: Sequence[tuple[str, str]] | None = None,
        topology_nodes: Mapping[str, Mapping[str, object]] | None = None,
    ) -> None:
        direction_source = (
            TOPOLOGY_EDGE_DIRECTIONS
            if edge_directions is None
            else edge_directions
        )
        edge_source = TOPOLOGY_EDGES if topology_edges is None else topology_edges
        node_source = TOPOLOGY_NODES if topology_nodes is None else topology_nodes

        self._edge_directions = {
            (str(first), str(second)): self._normalize_heading(direction)
            for (first, second), direction in direction_source.items()
        }
        self._topology_edges = {
            frozenset((str(first), str(second)))
            for first, second in edge_source
        }
        self._topology_nodes = {
            str(node): dict(metadata) for node, metadata in node_source.items()
        }

    @staticmethod
    def _normalize_node(node: TopologicalLocation | str) -> str:
        if isinstance(node, TopologicalLocation):
            return node.value
        return str(node)

    @staticmethod
    def _normalize_heading(heading: Heading | str) -> Heading:
        if isinstance(heading, Heading):
            return heading
        try:
            return Heading(str(heading).upper())
        except ValueError as exc:
            raise RouteActionUnknownError(
                f"unknown heading metadata: {heading!r}"
            ) from exc

    def edge_direction(
        self,
        current_node: TopologicalLocation | str,
        next_node: TopologicalLocation | str,
    ) -> Heading:
        """Return the confirmed travel heading for one adjacent topology edge."""
        current = self._normalize_node(current_node)
        following = self._normalize_node(next_node)

        if current not in self._topology_nodes:
            raise RouteActionUnknownError(f"unknown topology node: {current}")
        if following not in self._topology_nodes:
            raise RouteActionUnknownError(f"unknown topology node: {following}")
        if current == following or frozenset((current, following)) not in (
            self._topology_edges
        ):
            raise RouteActionUnknownError(
                f"no confirmed topology edge from {current} to {following}"
            )

        forward = self._edge_directions.get((current, following))
        if forward is not None:
            return forward
        reverse = self._edge_directions.get((following, current))
        if reverse is not None:
            return _OPPOSITE_HEADING[reverse]
        raise RouteActionUnknownError(
            f"no direction metadata for topology edge {current} -> {following}"
        )

    def resolve_action(
        self,
        current_node: TopologicalLocation | str,
        next_node: TopologicalLocation | str,
        current_heading: Heading | str,
    ) -> RouteAction:
        """Resolve the turn from ``current_heading`` onto one topology edge."""
        incoming = self._normalize_heading(current_heading)
        outgoing = self.edge_direction(current_node, next_node)
        return self._resolve_heading_transition(incoming, outgoing)

    def resolve_route_actions(
        self,
        route: Sequence[TopologicalLocation | str],
        *,
        current_heading: Heading | str | None = None,
    ) -> list[RouteAction]:
        """Return actions for every junction encountered along ``route``.

        The returned list is ordered by physical encounter.  Non-junction
        nodes are used to carry heading but do not produce actions.  Missing
        edge direction, a turn at a non-junction, or a U-turn fails explicitly.
        """
        nodes = [self._normalize_node(node) for node in route]
        if not nodes:
            raise ValueError("route must contain at least one topology node")
        for node in nodes:
            if node not in self._topology_nodes:
                raise RouteActionUnknownError(f"unknown topology node: {node}")
        if len(nodes) == 1:
            return []

        incoming = (
            None
            if current_heading is None
            else self._normalize_heading(current_heading)
        )
        actions: list[RouteAction] = []

        for current, following in zip(nodes, nodes[1:]):
            outgoing = self.edge_direction(current, following)
            kind = self._topology_nodes[current].get("kind")

            if kind == "junction":
                if incoming is None:
                    raise RouteActionUnknownError(
                        "current_heading is required when a route starts at "
                        f"junction {current}"
                    )
                actions.append(
                    self._resolve_heading_transition(incoming, outgoing)
                )
            elif incoming is not None and incoming is not outgoing:
                raise RouteActionUnknownError(
                    f"route changes heading at non-junction node {current}: "
                    f"{incoming.value} -> {outgoing.value}"
                )

            incoming = outgoing

        return actions

    @staticmethod
    def _resolve_heading_transition(
        incoming: Heading,
        outgoing: Heading,
    ) -> RouteAction:
        incoming_index = _CLOCKWISE_HEADINGS.index(incoming)
        outgoing_index = _CLOCKWISE_HEADINGS.index(outgoing)
        quarter_turns = (outgoing_index - incoming_index) % 4

        if quarter_turns == 0:
            return RouteAction.STRAIGHT
        if quarter_turns == 1:
            return RouteAction.RIGHT
        if quarter_turns == 3:
            return RouteAction.LEFT
        raise RouteActionUnknownError(
            f"U-turn is not a supported junction action: "
            f"{incoming.value} -> {outgoing.value}"
        )


def resolve_route_actions(
    route: Sequence[TopologicalLocation | str],
    *,
    current_heading: Heading | str | None = None,
) -> list[RouteAction]:
    """Resolve a route using the confirmed default field model."""
    return RouteActionResolver().resolve_route_actions(
        route,
        current_heading=current_heading,
    )
