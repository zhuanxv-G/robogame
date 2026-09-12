"""Hardware-free topological localization and route planning."""

from __future__ import annotations

from collections import deque
from enum import Enum
from typing import TypedDict

from .field_model import (
    TAG_TOPOLOGY_LOCATIONS,
    TOPOLOGY_EDGES,
    TOPOLOGY_NODES,
)


class TopologicalLocation(str, Enum):
    """Logical places supported by the current measured-free field model."""

    START_ZONE = "start_zone"
    START_T_JUNCTION = "start_t_junction"
    GRAY_CORRIDOR = "gray_corridor"
    LEFT_MATERIAL_T_JUNCTION = "left_material_t_junction"
    LEFT_ORANGE_MATERIAL = "left_orange_material"
    RIGHT_MATERIAL_T_JUNCTION = "right_material_t_junction"
    ROOF_MATERIAL_T_JUNCTION = "roof_material_t_junction"
    RIGHT_ORANGE_MATERIAL = "right_orange_material"
    PURPLE_MATERIAL = "purple_material"
    BUILD_ZONE = "build_zone"
    RIGHT_CORRIDOR_REFERENCE = "right_corridor_reference"
    BOTTOM_DIRECTION_REFERENCE = "bottom_direction_reference"


class RouteNotFoundError(RuntimeError):
    """Raised when the confirmed topology contains no requested route."""


class TopologicalNavigationStatus(TypedDict):
    """Observable logical localization and planning state."""

    current_location: TopologicalLocation | None
    current_tag: int | None
    target_location: TopologicalLocation | None


class TopologicalNavigator:
    """Track landmark locations and plan unweighted logical routes.

    The navigator owns no camera, serial link, or motion controller. Planning
    never changes ``current_location`` because a route is not proof of motion.
    An upper layer may call :meth:`update_from_tag` after a confirmed Tag
    arrival to update the logical location.
    """

    def __init__(self) -> None:
        self.current_location: TopologicalLocation | None = None
        self.current_tag: int | None = None
        self.target_location: TopologicalLocation | None = None
        self._graph = self._build_graph()
        self._tag_locations = {
            tag_id: self._normalize_location(location)
            for tag_id, location in TAG_TOPOLOGY_LOCATIONS.items()
        }

    @staticmethod
    def _normalize_location(
        location: TopologicalLocation | str,
    ) -> TopologicalLocation:
        if isinstance(location, TopologicalLocation):
            return location
        try:
            return TopologicalLocation(str(location))
        except ValueError as exc:
            raise ValueError(f"unknown topological location: {location}") from exc

    @classmethod
    def _build_graph(
        cls,
    ) -> dict[TopologicalLocation, tuple[TopologicalLocation, ...]]:
        adjacency: dict[TopologicalLocation, list[TopologicalLocation]] = {
            cls._normalize_location(node): [] for node in TOPOLOGY_NODES
        }

        for first_name, second_name in TOPOLOGY_EDGES:
            first = cls._normalize_location(first_name)
            second = cls._normalize_location(second_name)
            if first not in adjacency or second not in adjacency:
                raise ValueError(
                    f"topology edge references an unknown node: {first_name}, "
                    f"{second_name}"
                )
            adjacency[first].append(second)
            adjacency[second].append(first)

        return {
            node: tuple(neighbors) for node, neighbors in adjacency.items()
        }

    def update_from_tag(self, tag_id: int) -> TopologicalLocation | None:
        """Update logical position from a known landmark.

        An unknown Tag returns ``None`` and leaves the last confirmed Tag and
        location unchanged.
        """
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise TypeError("tag_id must be an integer")

        location = self._tag_locations.get(tag_id)
        if location is None:
            return None

        self.current_tag = tag_id
        self.current_location = location
        return location

    def plan_route(
        self,
        start: TopologicalLocation | str,
        target: TopologicalLocation | str,
    ) -> list[TopologicalLocation]:
        """Return the shortest logical-node path using breadth-first search."""
        start_location = self._normalize_location(start)
        target_location = self._normalize_location(target)
        self.target_location = target_location

        if start_location == target_location:
            return [start_location]

        queue = deque([[start_location]])
        visited = {start_location}

        while queue:
            route = queue.popleft()
            for neighbor in self._graph[route[-1]]:
                if neighbor in visited:
                    continue
                next_route = [*route, neighbor]
                if neighbor == target_location:
                    return next_route
                visited.add(neighbor)
                queue.append(next_route)

        raise RouteNotFoundError(
            f"no confirmed route from {start_location.value} "
            f"to {target_location.value}"
        )

    def get_status(self) -> TopologicalNavigationStatus:
        """Return the current hardware-free localization/planning state."""
        return {
            "current_location": self.current_location,
            "current_tag": self.current_tag,
            "target_location": self.target_location,
        }
