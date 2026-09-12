"""Pure-software tests for the landmark/topological field model."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from navigation.field_map import get_zone_by_tag  # noqa: E402
from navigation.field_model import (  # noqa: E402
    ORANGE_PICKUP,
    PURPLE_PICKUP,
    TOPOLOGY_NODES,
    ZONE_MAP,
)
from navigation.topological_navigation import (  # noqa: E402
    RouteNotFoundError,
    TopologicalLocation,
    TopologicalNavigator,
)
from strategy.mission import (  # noqa: E402
    ORANGE_SOURCE_PRIORITY,
    PREFERRED_ORANGE_SOURCE,
    PREFERRED_ORANGE_TAG_ID,
)


class TopologicalNavigationTests(unittest.TestCase):
    def test_tag_2_updates_left_orange_location(self) -> None:
        navigator = TopologicalNavigator()

        location = navigator.update_from_tag(2)

        self.assertIs(location, TopologicalLocation.LEFT_ORANGE_MATERIAL)
        self.assertIs(
            navigator.current_location,
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
        )
        self.assertEqual(navigator.current_tag, 2)

    def test_tag_4_updates_right_orange_location(self) -> None:
        navigator = TopologicalNavigator()

        location = navigator.update_from_tag(4)

        self.assertIs(location, TopologicalLocation.RIGHT_ORANGE_MATERIAL)
        self.assertIs(
            navigator.current_location,
            TopologicalLocation.RIGHT_ORANGE_MATERIAL,
        )
        self.assertEqual(navigator.current_tag, 4)

    def test_both_wall_material_regions_support_orange_pickup(self) -> None:
        left = ZONE_MAP[TopologicalLocation.LEFT_ORANGE_MATERIAL.value]
        right = ZONE_MAP[TopologicalLocation.RIGHT_ORANGE_MATERIAL.value]

        self.assertIn(ORANGE_PICKUP, left["capabilities"])
        self.assertIn(ORANGE_PICKUP, right["capabilities"])
        self.assertIn(ORANGE_PICKUP, get_zone_by_tag(2)["capabilities"])
        self.assertIn(ORANGE_PICKUP, get_zone_by_tag(4)["capabilities"])

    def test_first_strategy_still_prefers_tag_2_side(self) -> None:
        self.assertIs(
            PREFERRED_ORANGE_SOURCE,
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
        )
        self.assertEqual(PREFERRED_ORANGE_TAG_ID, 2)
        self.assertEqual(
            ORANGE_SOURCE_PRIORITY,
            (
                TopologicalLocation.LEFT_ORANGE_MATERIAL,
                TopologicalLocation.RIGHT_ORANGE_MATERIAL,
            ),
        )

    def test_tag_3_updates_purple_location(self) -> None:
        navigator = TopologicalNavigator()

        location = navigator.update_from_tag(3)

        self.assertIs(location, TopologicalLocation.PURPLE_MATERIAL)
        self.assertIn(
            PURPLE_PICKUP,
            ZONE_MAP[TopologicalLocation.PURPLE_MATERIAL.value]["capabilities"],
        )

    def test_unknown_tag_does_not_fabricate_or_replace_location(self) -> None:
        navigator = TopologicalNavigator()
        navigator.update_from_tag(2)

        location = navigator.update_from_tag(99)

        self.assertIsNone(location)
        self.assertIs(
            navigator.current_location,
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
        )
        self.assertEqual(navigator.current_tag, 2)

    def test_known_start_and_target_generate_logical_route(self) -> None:
        navigator = TopologicalNavigator()

        route = navigator.plan_route(
            TopologicalLocation.START_ZONE,
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
        )

        self.assertEqual(
            route,
            [
                TopologicalLocation.START_ZONE,
                TopologicalLocation.START_T_JUNCTION,
                TopologicalLocation.GRAY_CORRIDOR,
                TopologicalLocation.LEFT_MATERIAL_T_JUNCTION,
                TopologicalLocation.LEFT_ORANGE_MATERIAL,
            ],
        )
        self.assertIsNone(navigator.current_location)
        self.assertIs(
            navigator.target_location,
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
        )

    def test_left_orange_to_purple_route_uses_confirmed_graph(self) -> None:
        navigator = TopologicalNavigator()

        route = navigator.plan_route(
            TopologicalLocation.LEFT_ORANGE_MATERIAL,
            TopologicalLocation.PURPLE_MATERIAL,
        )

        self.assertEqual(
            route,
            [
                TopologicalLocation.LEFT_ORANGE_MATERIAL,
                TopologicalLocation.LEFT_MATERIAL_T_JUNCTION,
                TopologicalLocation.GRAY_CORRIDOR,
                TopologicalLocation.RIGHT_MATERIAL_T_JUNCTION,
                TopologicalLocation.ROOF_MATERIAL_T_JUNCTION,
                TopologicalLocation.PURPLE_MATERIAL,
            ],
        )

    def test_missing_route_fails_instead_of_inventing_direct_path(self) -> None:
        navigator = TopologicalNavigator()

        with self.assertRaisesRegex(RouteNotFoundError, "no confirmed route"):
            navigator.plan_route(
                TopologicalLocation.START_ZONE,
                TopologicalLocation.BUILD_ZONE,
            )

        self.assertIsNone(navigator.current_location)
        self.assertIs(
            navigator.target_location,
            TopologicalLocation.BUILD_ZONE,
        )
        self.assertNotIn("x", TOPOLOGY_NODES["build_zone"])
        self.assertNotIn("y", TOPOLOGY_NODES["build_zone"])


if __name__ == "__main__":
    unittest.main()
