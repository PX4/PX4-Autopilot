"""Parser checks against the syntax the mission route tests actually use."""

from pathlib import Path
import unittest

from planner_visualizer.cpp_parser import header_constants, parse_cpp, resolve_number
from planner_visualizer.geometry import latlon_to_offset

REPO = Path(__file__).resolve().parents[2]
PLANNER_TESTS = REPO / "src/modules/navigator/test/test_mission_route_planner.cpp"


def planner_test(name):
    source = PLANNER_TESTS.read_text()
    start = source.index(f"TEST_F(MissionRoutePlannerTest, {name})")
    return source[start : source.index("TEST_F(", start + 10)]


def offsets(scene, point):
    base = scene.base()
    north, east = latlon_to_offset(base.lat, base.lon, point.lat, point.lon)
    return round(north, 1), round(east, 1)


class RepoSourcesTest(unittest.TestCase):
    def test_helper_header_defines_the_reference(self):
        constants = header_constants()
        self.assertEqual(
            (constants["kBaseLat"], constants["kBaseLon"], constants["kAlt"]),
            (47.397742, 8.545594, 500.0),
        )

    def test_planner_test_with_safe_point_positions_and_velocity(self):
        scene = parse_cpp(
            planner_test("SelectedBranchLegShortcutRequiresCrosstrackAndAltitudeProximity")
        )
        self.assertEqual(
            [offsets(scene, wp) for wp in scene.missions["mission"]], [(0.0, 0.0), (600.0, 0.0)]
        )
        self.assertEqual([offsets(scene, wp) for wp in scene.safe_points], [(300.0, 50.0)])
        self.assertEqual(
            {position.name: offsets(scene, position) for position in scene.positions},
            {"vehicle_position": (300.0, 10.0), "off_leg_position": (350.0, 10.0)},
        )
        self.assertTrue(scene.positions[0].is_vehicle)
        self.assertFalse(scene.positions[1].is_vehicle)
        self.assertEqual(scene.velocity, (5.0, 0.0))

    def test_shared_data_header_functions(self):
        scene = parse_cpp((REPO / "src/modules/navigator/test/test_mission_route_data.h").read_text())
        # Every dataset namespace defines mission(); duplicates get a suffix.
        self.assertEqual(list(scene.missions), ["mission", "mission_2", "mission_3"])
        # DO_JUMP and VTOL transition items are not positional and are skipped.
        self.assertEqual([len(items) for items in scene.missions.values()], [14, 13, 5])
        self.assertEqual(scene.missions["mission"][0].cmd, "NAV_CMD_TAKEOFF")
        self.assertEqual(scene.missions["mission"][-1].cmd, "NAV_CMD_LAND")
        self.assertEqual(len(scene.safe_points), 17)


class ContainerTest(unittest.TestCase):
    def test_container_forms(self):
        scene = parse_cpp(
            """
            std::vector<mission_item_s> mission{
                makeTakeoffItem(kBaseLat, kBaseLon, kAlt),
                makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
                makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f, NAV_CMD_LOITER_TO_ALT),
                makeDoJump(0, 2),
            };
            mission.push_back(makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt));
            const auto extra = std::vector<mission_item_s>{
                makePositionItem(kBaseLat, kBaseLon + 0.01, kAlt),
            };
            std::array<mission_item_s, 2> approach{{
                makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt),
                makeLandItem(kBaseLat, kBaseLon + 0.01, kAlt),
            }};
            std::vector<mission_item_s> appended;
            appended.push_back(makePositionItem(kBaseLat, kBaseLon, kAlt));
            static inline std::vector<mission_item_s> buildMission()
            {
                return {makePositionItem(kBaseLat + 0.01, kBaseLon, kAlt)};
            }
            std::vector<mission_item_s> empty;
        """
        )
        self.assertEqual(set(scene.missions), {"mission", "extra", "approach", "appended", "buildMission"})
        mission = scene.missions["mission"]
        self.assertEqual(
            [wp.cmd for wp in mission], ["NAV_CMD_TAKEOFF", "NAV_CMD_LOITER_TO_ALT", "NAV_CMD_LAND"]
        )
        self.assertEqual([offsets(scene, wp) for wp in mission], [(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)])
        self.assertEqual(mission[1].alt, 520.0)
        self.assertEqual(scene.missions["approach"][1].cmd, "NAV_CMD_LAND")
        self.assertAlmostEqual(scene.missions["buildMission"][0].lat, 47.407742)

    def test_same_name_declared_twice_keeps_both_and_their_push_backs(self):
        scene = parse_cpp(
            """
            std::vector<mission_item_s> mission{makePositionItem(47.0, 8.0, 500.f)};
            mission.push_back(makePositionItem(47.1, 8.0, 500.f));
            std::vector<mission_item_s> mission{makePositionItem(48.0, 9.0, 500.f)};
            mission.push_back(makePositionItem(48.1, 9.0, 500.f));
        """
        )
        self.assertEqual([wp.lat for wp in scene.missions["mission"]], [47.0, 47.1])
        self.assertEqual([wp.lat for wp in scene.missions["mission_2"]], [48.0, 48.1])

    def test_items_outside_a_container_form_one_mission(self):
        scene = parse_cpp(
            """
            executor.loadTestMission({
                makePositionItem(kBaseLat, kBaseLon, kAlt),
                makePositionItem(kBaseLat + 0.001, kBaseLon, kAlt),
            });
        """
        )
        self.assertEqual(list(scene.missions), ["mission"])
        self.assertEqual([wp.lat for wp in scene.missions["mission"]], [47.397742, 47.398742])


class ValuesTest(unittest.TestCase):
    def test_local_constants_override_the_header_and_resolve_in_any_order(self):
        scene = parse_cpp(
            """
            constexpr float kAlt = kBase + 100.f;
            static constexpr float kBase = 5e2F;
            constexpr double kBaseLat = 48.0;
            std::vector<mission_item_s> mission{
                makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
            };
        """
        )
        waypoint = scene.missions["mission"][0]
        self.assertAlmostEqual(waypoint.lat, 48.0)
        self.assertEqual((waypoint.lon, waypoint.alt), (8.545594, 600.0))
        self.assertEqual(scene.base().alt, 600.0)

    def test_number_expressions(self):
        constants = {"kVel": 15.0}
        self.assertEqual(resolve_number("corner_dataset::kVel", constants), 15.0)
        self.assertEqual(resolve_number("-(kVel - 5.f) * 2", constants), -20.0)
        self.assertEqual(resolve_number("2.5e1f", constants), 25.0)
        for unsupported in ("unknown", "1.f / 0.f", "2 ** 3", "abs(-1)", "true", "\x00", ""):
            with self.subTest(expression=unsupported):
                self.assertIsNone(resolve_number(unsupported, constants))

    def test_positions_keep_their_variable_names(self):
        scene = parse_cpp(
            """
            const mission_route::Position vehicle_position =
                makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 20.f, kAlt);
            mission_route::Position goal{47.0, 8.0, 510.f};
            request.vehicle_position = mission_route::Position{47.01, 8.01, 520.f};
            makePositionAbsolute(47.02, 8.02, 530.f);
            mission_route::Position copy = vehicle_position;
        """
        )
        self.assertEqual(
            [(position.name, position.alt) for position in scene.positions],
            [("vehicle_position", 500.0), ("position_1", 530.0), ("goal", 510.0), ("vehicle_position", 520.0)],
        )
        self.assertEqual(offsets(scene, scene.positions[0]), (50.0, 20.0))

    def test_velocity_needs_both_components(self):
        scene = parse_cpp(
            "request.velocity_north_m_s = -15.f;\n"
            "request.velocity_east_m_s = kVel;\n"
            "constexpr float kVel = 15.f;"
        )
        self.assertEqual(scene.velocity, (-15.0, 15.0))
        self.assertIsNone(parse_cpp("request.velocity_north_m_s = 5.f;").velocity)

    def test_safe_point_helpers(self):
        scene = parse_cpp(
            """
            const mission_item_s best = makeSafePointFromOffset(kBaseLat, kBaseLon, 100.f, 5.f, kAlt);
            std::vector<mission_item_s> safe_points{
                makeSafePointAbsolute(47.0, 8.0, 466.1f), // 0
            };
        """
        )
        self.assertEqual(offsets(scene, scene.safe_points[0]), (100.0, 5.0))
        self.assertEqual((scene.safe_points[1].lat, scene.safe_points[1].alt), (47.0, 466.1))
        self.assertEqual(scene.safe_points[1].cmd, "NAV_CMD_RALLY_POINT")
        self.assertEqual(scene.missions, {})

    def test_unresolved_items_comments_and_strings_are_ignored(self):
        scene = parse_cpp(
            """
            // makePositionItem(1.0, 2.0, 3.f);
            /* makeSafePointAbsolute(1.0, 2.0, 3.f); */
            ASSERT_EQ(status, kNone) << "mission{ not a container, makePositionItem(1, 2, 3)";
            std::vector<mission_item_s> mission{
                makePositionItem(unknown, 8.0, 500.f),
                makePositionItem(47.0, 8.0, height),
                makePositionItem(47.0, 8.0, 500.f),
            };
        """
        )
        self.assertEqual(len(scene.missions["mission"]), 1)
        self.assertEqual(scene.safe_points, [])
        self.assertTrue(parse_cpp("// nothing here").is_empty())
        self.assertTrue(parse_cpp("").is_empty())
        self.assertEqual(parse_cpp("").summary(), "nothing")


if __name__ == "__main__":
    unittest.main()
