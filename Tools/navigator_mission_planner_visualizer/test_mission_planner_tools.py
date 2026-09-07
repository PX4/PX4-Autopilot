"""Regression checks for current planner fixtures and the visualizer's output."""

import json
from pathlib import Path
import unittest

import mission_planner_tools as viewer


class MissionPlannerVisualizerTest(unittest.TestCase):
    def test_current_planner_fixture_uses_shared_reference_coordinates(self):
        test_source = (
            Path(__file__).resolve().parents[2]
            / "src/modules/navigator/test/test_mission_route_planner.cpp"
        ).read_text()
        start = test_source.index(
            "TEST_F(MissionRoutePlannerTest, PlansNominalMissionResumeJoinToNextItem)"
        )
        end = test_source.index("TEST_F(", start + 10)
        parsed = viewer.parse_cpp_code(test_source[start:end])
        mission = parsed[0]["mission"]
        self.assertEqual(len(mission), 3)
        self.assertAlmostEqual(mission[0].lat, 47.397742)
        self.assertAlmostEqual(mission[0].lon, 8.545594)
        self.assertEqual(mission[0].alt, 500.0)
        self.assertGreater(mission[2].lat, mission[1].lat)
        self.assertEqual(len(parsed[4]), 1)
        self.assertGreater(parsed[4][0].lon, mission[0].lon)

        html = viewer._build_base_map(parsed, viewer.get_viewer_center(parsed), 12).get_root().render()
        self.assertIn("L.polyline", html)
        deck = json.loads(viewer.create_3d_map(parsed).to_json())
        self.assertEqual(len(deck["layers"]), 2)

    def test_local_constants_override_helper_defaults(self):
        parsed = viewer.parse_cpp_code("""
            constexpr double kBaseLat = 48.0;
            constexpr double kBaseLon = 9.0;
            constexpr float kAlt = 600.f;
            std::vector<mission_item_s> mission{
                makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt + 25.f),
            };
        """)
        waypoint = parsed[0]["mission"][0]
        self.assertAlmostEqual(waypoint.lat, 48.0)
        self.assertAlmostEqual(waypoint.lon, 9.0)
        self.assertEqual(waypoint.alt, 625.0)

    def test_public_positions_and_point_field_references(self):
        parsed = viewer.parse_cpp_code("""
            const mission_route::Position vehicle{47.0, 8.0, 500.f};
            request.vehicle_position = mission_route::Position{47.01, 8.01, 510.f};
            const auto destination = makePositionAbsolute(vehicle.lat, vehicle.lon, vehicle.alt);
        """)
        points = {(p.lat, p.lon, p.alt) for p in parsed[4]}
        self.assertEqual(points, {(47.0, 8.0, 500.0), (47.01, 8.01, 510.0)})

    def test_creator_snippets_round_trip_with_legacy_layers(self):
        creator = {mode: [] for mode in viewer.CREATOR_MODES}
        creator["Mission"] = [
            (47.0, 8.0, 500.0, 0.0),
            (47.001, 8.001, 550.0, 0.0),
            (47.002, 8.002, 500.0, 0.0),
        ]
        creator["Vehicle Location"] = [(47.0005, 8.0005, 550.0, 5.0, 10.0)]
        creator["Rally Points"] = [(47.003, 8.003, 500.0, 0.0)]
        creator["Fence: Circle (Exclusion)"] = [(47.004, 8.004, 500.0, 50.0)]
        creator["Fence: Polygon (Inclusion)"] = creator["Mission"]
        creator["Path Check"] = creator["Mission"][:2]
        parsed = viewer.parse_cpp_code(viewer.generate_full_cpp_snippet(creator, "example"))
        mission = next(iter(parsed[0].values()))
        self.assertEqual([wp.cmd for wp in mission], ["NAV_CMD_TAKEOFF", "NAV_CMD_WAYPOINT", "NAV_CMD_LAND"])
        self.assertEqual(len(parsed[1]), 1)
        self.assertEqual(len(parsed[2]), 1)
        self.assertEqual(len(parsed[3]), 1)
        self.assertEqual((parsed[3][0].vel_n, parsed[3][0].vel_e), (5.0, 10.0))
        self.assertEqual(len(parsed[5]), 1)
        self.assertEqual(len(parsed[6]), 1)
        self.assertEqual(parsed[2][0].radius, 50.0)


if __name__ == "__main__":
    unittest.main()
