"""Parser checks against the syntax the mission route tests actually use."""

from dataclasses import fields
from pathlib import Path
import unittest
from unittest.mock import patch

from planner_visualizer.cpp_parser import (
    geofence_reference,
    header_constants,
    parse_cpp,
    request_defaults,
    resolve_number,
    route_position_altitude,
)
from planner_visualizer.geometry import latlon_to_offset
from planner_visualizer.planner_cli import PlanRequest

REPO = Path(__file__).resolve().parents[2]
NAVIGATOR_TESTS = REPO / "src/modules/navigator/test"
PLANNER_TESTS = NAVIGATOR_TESTS / "test_mission_route_planner.cpp"
# The C++ test sources change often, so tests that read them check structure, not exact counts.
needs_repo_sources = unittest.skipUnless(PLANNER_TESTS.is_file(), "needs the PX4-Autopilot navigator tests")


def repo_source(path):
    """A repository source file, or a skip when it is missing."""
    if not path.is_file():
        raise unittest.SkipTest(f"needs {path.name} from a PX4-Autopilot checkout")
    return path.read_text()


def repo_test(path, fixture, name):
    """The body of one TEST_F, or a skip when the test was renamed or removed."""
    source = repo_source(path)
    start = source.find(f"TEST_F({fixture}, {name})")
    if start < 0:
        raise unittest.SkipTest(f"{fixture}.{name} is not in {path.name}")
    end = source.find("TEST_F(", start + 10)
    return source[start : end if end >= 0 else len(source)]


def offsets(scene, point):
    base = scene.base()
    north, east = latlon_to_offset(base.lat, base.lon, point.lat, point.lon)
    return round(north, 1), round(east, 1)


@needs_repo_sources
class RepoSourcesTest(unittest.TestCase):
    def test_helper_header_defines_the_reference(self):
        constants = header_constants()
        self.assertEqual(
            (constants["kBaseLat"], constants["kBaseLon"], constants["kAlt"]),
            (47.397742, 8.545594, 500.0),
        )

    def test_planner_test_with_safe_point_positions_and_velocity(self):
        scene = parse_cpp(repo_test(
            PLANNER_TESTS, "MissionRoutePlannerTest", "SelectedBranchLegShortcutRequiresCrosstrackAndAltitudeProximity"
        ))
        self.assertGreaterEqual(len(scene.missions.get("mission", [])), 2)
        self.assertTrue(scene.safe_points)
        self.assertTrue(any(position.is_vehicle for position in scene.positions))
        self.assertIsNotNone(scene.velocity)

    def test_geofence_fixture_values_are_read_from_the_cpp(self):
        self.assertIsNotNone(geofence_reference())
        self.assertIsNotNone(route_position_altitude())
        square = parse_cpp("loadFence(exclusionSquare());").fences
        self.assertEqual([len(fence.vertices) for fence in square], [4])

    def test_request_defaults_cover_every_float_field_of_the_request(self):
        # Otherwise the generated tests would repeat values the helpers already set.
        float_fields = {item.name for item in fields(PlanRequest) if item.type is float}
        for helper in ("makeRtlRouteRequest", "makeMissionResumeRequest"):
            with self.subTest(helper=helper):
                self.assertLessEqual(float_fields, set(request_defaults(helper)))

    def test_shared_data_header_functions(self):
        scene = parse_cpp(repo_source(NAVIGATOR_TESTS / "test_mission_route_data.h"))
        # Every dataset namespace defines mission(); duplicates get a suffix.
        self.assertGreaterEqual(len(scene.missions), 2)
        self.assertEqual(list(scene.missions)[:2], ["mission", "mission_2"])
        self.assertTrue(all(scene.missions.values()))
        self.assertTrue(scene.safe_points)


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

    def test_first_mission_is_the_first_declared(self):
        scene = parse_cpp(
            "std::vector<mission_item_s> mission;\n"
            "mission.push_back(makePositionItem(47.0, 8.0, 500.f));\n"
            "std::vector<mission_item_s> other{makePositionItem(48.0, 9.0, 500.f)};\n"
        )
        self.assertEqual(list(scene.missions), ["mission", "other"])

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


class FenceTest(unittest.TestCase):
    def test_generated_records_keep_polygon_groups_and_circle_radius(self):
        scene = parse_cpp(
            """
            const auto makeFencePoint = [](uint16_t command, double lat, double lon,
                                          uint16_t vertex_count, float radius) { return mission_fence_point_s{}; };
            std::vector<mission_fence_point_s> fence_points{
                makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 47.0, 8.0, 3, 0.f),
                makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 47.1, 8.0, 3, 0.f),
                makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 47.0, 8.1, 3, 0.f),
                makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 48.0, 9.0, 3, 0.f),
                makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 48.1, 9.0, 3, 0.f),
                makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION, 48.0, 9.1, 3, 0.f),
                makeFencePoint(NAV_CMD_FENCE_CIRCLE_EXCLUSION, 47.01, 8.01, 0, 25.f),
            };
            """
        )
        self.assertEqual(len(scene.fences), 3)
        self.assertEqual(scene.fences[0].vertices, [(47.0, 8.0), (47.1, 8.0), (47.0, 8.1)])
        self.assertEqual(scene.fences[1].vertices[0], (48.0, 9.0))
        self.assertTrue(scene.fences[0].inclusion)
        self.assertFalse(scene.fences[2].inclusion)
        self.assertEqual(scene.fences[2].radius_m, 25.0)
        self.assertFalse(scene.is_empty())

    def test_invalid_geometry_is_not_drawn_as_a_partial_fence(self):
        for latitude, count in (("unknown", "3"), ("91", "3"), ("1e999", "3"), ("47", "4"), ("47", "3.5")):
            with self.subTest(latitude=latitude, count=count):
                text = ",\n".join(
                    f"makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION, {lat}, 8.0, {count}, 0.f)"
                    for lat in ("47", latitude, "47.1")
                )
                self.assertEqual(parse_cpp(text).fences, [])
        for radius in ("0", "-1", "unknown", "1e999"):
            with self.subTest(radius=radius):
                self.assertEqual(parse_cpp(
                    f"makeFencePoint(NAV_CMD_FENCE_CIRCLE_INCLUSION, 47, 8, 0, {radius});"
                ).fences, [])

    def test_polygon_records_do_not_cross_container_boundaries(self):
        point = "makeFencePoint(NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION, 47, 8, 3, 0.f)"
        scene = parse_cpp(f"std::vector<mission_fence_point_s> a{{{point}, {point}}};\n"
                          f"std::vector<mission_fence_point_s> b{{{point}}};")
        self.assertEqual(scene.fences, [])

    @needs_repo_sources
    def test_geofence_fixture_helpers_use_their_own_reference(self):
        scene = parse_cpp(
            """
            auto inclusion = polygon(true, {{0.f, 0.f}, {100.f, 0.f}, {0.f, 100.f}});
            loadFence(circle(false, {50.f, 20.f}, 15.f));
            loadFence(exclusionSquare());
            startFenceUpdate(mission, {exclusionCircle(100.f, 50.f, 20.f)}, DM_KEY_FENCE_POINTS_1);
            """
        )
        self.assertEqual(len(scene.fences), 4)
        self.assertTrue(scene.fences[0].inclusion)
        self.assertEqual(scene.fences[0].vertices[0], geofence_reference())
        north, east = latlon_to_offset(*geofence_reference(), *scene.fences[1].vertices[0])
        self.assertAlmostEqual(north, 50.0, places=3)
        self.assertAlmostEqual(east, 20.0, places=3)
        self.assertEqual(scene.fences[1].radius_m, 15.0)
        self.assertEqual(len(scene.fences[2].vertices), 4)
        base = scene.base()
        north, east = latlon_to_offset(base.lat, base.lon, *scene.fences[3].vertices[0])
        self.assertAlmostEqual(north, 100.0, places=3)
        self.assertAlmostEqual(east, 50.0, places=3)
        self.assertEqual(scene.fences[3].radius_m, 20.0)

    def test_invalid_fixture_helpers_and_comments_are_ignored(self):
        scene = parse_cpp(
            """
            polygon(true, {{0, 0}, {100, 0}});
            polygon(false, {{0, 0}, {100, 0}, {unknown, 100}});
            circle(false, {0, 0}, unknown);
            circle(true, {0, 0}, -1);
            exclusionCircle(0, 0, 0);
            // loadFence(exclusionSquare());
            """
        )
        self.assertEqual(scene.fences, [])

    @needs_repo_sources
    def test_route_positions_use_the_geofence_reference(self):
        scene = parse_cpp(
            "const Position vehicle = routePosition(0.f, 100.f);\n"
            "const Position high = routePosition(10.f, 0.f, 520.f);"
        )
        north, east = latlon_to_offset(*geofence_reference(), scene.positions[0].lat, scene.positions[0].lon)
        self.assertAlmostEqual(north, 0.0, places=3)
        self.assertAlmostEqual(east, 100.0, places=3)
        self.assertEqual([position.alt for position in scene.positions], [route_position_altitude(), 520.0])
        self.assertTrue(scene.positions[0].is_vehicle)

    def test_route_position_altitude_is_its_default_argument(self):
        source = (
            "static constexpr float kAltitude = 500.f;\n"
            "Position routePosition(float north, float east, float altitude = kAltitude + 10.f) const\n"
        )
        with patch("planner_visualizer.cpp_parser.repo_file", return_value=source):
            # The undecorated function, so the cache keeps the value of the real source.
            self.assertEqual(route_position_altitude.__wrapped__(), 510.0)

    def test_fixture_helpers_are_skipped_without_their_cpp_sources(self):
        with patch("planner_visualizer.cpp_parser.geofence_reference", return_value=None), patch(
            "planner_visualizer.cpp_parser.fence_shapes", return_value={}
        ):
            scene = parse_cpp(
                "loadFence(polygon(true, {{0.f, 0.f}, {100.f, 0.f}, {0.f, 100.f}}));\n"
                "loadFence(exclusionSquare());\n"
                "const Position vehicle = routePosition(0.f, 100.f);"
            )
        self.assertEqual((scene.fences, scene.positions), ([], []))

    @needs_repo_sources
    def test_repository_geofence_case_includes_vehicle_and_candidate_positions(self):
        scene = parse_cpp(repo_test(
            NAVIGATOR_TESTS / "test_mission_route_geofence.cpp",
            "MissionRouteGeofenceTest",
            "PrunesCrossingSegmentsAndPreservesCandidateOrder",
        ))
        self.assertTrue(scene.fences)
        self.assertTrue(scene.positions)
        self.assertTrue(scene.positions[0].is_vehicle)


if __name__ == "__main__":
    unittest.main()
