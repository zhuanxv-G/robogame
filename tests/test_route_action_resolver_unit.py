"""Pure-software tests for topology-to-junction action resolution."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from navigation.field_model import TOPOLOGY_EDGES  # noqa: E402
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


class RouteActionResolverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.resolver = RouteActionResolver()

    def test_known_topology_route_resolves_in_encounter_order(self) -> None:
        route = TopologicalNavigator().plan_route(
            TopologicalLocation.START_ZONE,
            TopologicalLocation.PURPLE_MATERIAL,
        )

        self.assertEqual(
            self.resolver.resolve_route_actions(route),
            [
                RouteAction.RIGHT,
                RouteAction.STRAIGHT,
                RouteAction.LEFT,
                RouteAction.LEFT,
            ],
        )

    def test_every_confirmed_edge_has_explicit_direction_metadata(self) -> None:
        for first, second in TOPOLOGY_EDGES:
            with self.subTest(edge=(first, second)):
                self.assertIsInstance(
                    self.resolver.edge_direction(first, second),
                    Heading,
                )

    def test_four_node_route_can_produce_three_junction_actions(self) -> None:
        route = [
            TopologicalLocation.LEFT_MATERIAL_T_JUNCTION,
            TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
            TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
            TopologicalLocation.RIGHT_ORANGE_MATERIAL,
        ]

        self.assertEqual(
            self.resolver.resolve_route_actions(
                route,
                current_heading=Heading.SOUTH,
            ),
            [
                RouteAction.LEFT,
                RouteAction.LEFT,
                RouteAction.STRAIGHT,
            ],
        )

    def test_left(self) -> None:
        self.assertIs(
            self.resolver.resolve_action(
                TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
                TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
                Heading.EAST,
            ),
            RouteAction.LEFT,
        )

    def test_right(self) -> None:
        self.assertIs(
            self.resolver.resolve_action(
                TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
                TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
                Heading.WEST,
            ),
            RouteAction.RIGHT,
        )

    def test_straight(self) -> None:
        self.assertIs(
            self.resolver.resolve_action(
                TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
                TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
                Heading.NORTH,
            ),
            RouteAction.STRAIGHT,
        )

    def test_unknown_edge_fails_explicitly(self) -> None:
        with self.assertRaisesRegex(
            RouteActionUnknownError,
            "no confirmed topology edge",
        ):
            self.resolver.resolve_action(
                TopologicalLocation.BUILD_ZONE,
                TopologicalLocation.START_ZONE,
                Heading.NORTH,
            )

    def test_missing_direction_does_not_default_to_straight(self) -> None:
        resolver = RouteActionResolver(edge_directions={})

        with self.assertRaisesRegex(
            RouteActionUnknownError,
            "no direction metadata",
        ):
            resolver.resolve_action(
                TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
                TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
                Heading.NORTH,
            )

    def test_route_starting_at_junction_requires_current_heading(self) -> None:
        route = [
            TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
            TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
            TopologicalLocation.PURPLE_MATERIAL,
        ]

        with self.assertRaisesRegex(
            RouteActionUnknownError,
            "current_heading is required",
        ):
            self.resolver.resolve_route_actions(route)

        self.assertEqual(
            self.resolver.resolve_route_actions(
                route,
                current_heading=Heading.EAST,
            ),
            [RouteAction.LEFT, RouteAction.LEFT],
        )


if __name__ == "__main__":
    unittest.main()
