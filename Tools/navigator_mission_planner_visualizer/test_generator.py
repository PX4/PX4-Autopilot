"""Generated C++ must look like the tests and parse back to the same fixtures."""

import shutil
import subprocess
import unittest

from planner_visualizer.cpp_parser import header_constants, parse_cpp
from planner_visualizer.generator import cpp_float, generate_cpp
from planner_visualizer.geometry import offset_to_latlon
from planner_visualizer.models import Base, Fence, Position, Scene, Waypoint, base_from
from planner_visualizer import planner_cli
from planner_visualizer.planner_cli import PlanRequest, parse_output
from planner_visualizer.unit_test import generate_unit_test

BASE = Base(47.397742, 8.545594, 500.0)


def at(north, east, alt=500.0, cmd="NAV_CMD_WAYPOINT"):
    lat, lon = offset_to_latlon(BASE.lat, BASE.lon, north, east)
    return Waypoint(lat, lon, alt, cmd)


def make_scene():
    vehicle = at(10.0, 0.0)
    return Scene(
        missions={
            "mission": [
                at(0.0, 0.0, cmd="NAV_CMD_TAKEOFF"),
                at(200.0, 0.0, 520.0),
                at(300.0, 12.5, 500.0, "NAV_CMD_LOITER_TO_ALT"),
                at(400.0, 0.0, 490.0, "NAV_CMD_LAND"),
            ]
        },
        safe_points=[at(60.0, 5.0, cmd="NAV_CMD_RALLY_POINT")],
        positions=[Position("vehicle_position", vehicle.lat, vehicle.lon, 500.0)],
        velocity=(5.0, -0.04),
    )


class GeneratorTest(unittest.TestCase):
    def test_offset_output_matches_the_test_style(self):
        self.assertEqual(
            generate_cpp(make_scene(), BASE),
            "std::vector<mission_item_s> mission{\n"
            "\tmakeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), // 0\n"
            "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f), // 1\n"
            "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 12.5f, kAlt, NAV_CMD_LOITER_TO_ALT), // 2\n"
            "\tmakeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f), // 3\n"
            "};\n"
            "\n"
            "std::vector<mission_item_s> safe_points{\n"
            "\tmakeSafePointFromOffset(kBaseLat, kBaseLon, 60.f, 5.f, kAlt), // 0\n"
            "};\n"
            "\n"
            "const mission_route::Position vehicle_position = "
            "makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);\n"
            "\n"
            "request.velocity_north_m_s = 5.f;\n"
            "request.velocity_east_m_s = -0.04f;",
        )

    def test_absolute_output_uses_the_absolute_helpers(self):
        code = generate_cpp(make_scene(), None)
        self.assertIn("makeTakeoffItem(47.3977420, 8.5455940, 500.f), // 0", code)
        self.assertIn("makePositionItem(47.3995406, 8.5455940, 520.f), // 1", code)
        self.assertIn("makeSafePointAbsolute(", code)
        self.assertIn("vehicle_position = makePositionAbsolute(", code)
        self.assertNotIn("kBaseLat", code)
        self.assertNotIn("kAlt", code)

    def test_positions_without_kalt_fall_back_to_literal_altitudes(self):
        scene = Scene(positions=[Position("position_0", 47.0, 8.0, 512.0)])
        code = generate_cpp(scene, Base(47.0, 8.0))
        self.assertEqual(
            code,
            "const mission_route::Position position_0 = "
            "makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, 512.f);",
        )
        self.assertEqual(generate_cpp(Scene(), BASE), "")

    def test_round_trip_through_the_parser(self):
        scene = make_scene()
        expected_items = scene.missions["mission"] + scene.safe_points
        for base in (BASE, None):
            with self.subTest(base=base):
                parsed = parse_cpp(generate_cpp(scene, base))
                self.assertEqual(parsed.velocity, (5.0, -0.04))
                parsed_items = parsed.missions["mission"] + parsed.safe_points
                self.assertEqual([wp.cmd for wp in parsed_items], [wp.cmd for wp in expected_items])
                for expected, actual in zip(expected_items, parsed_items):
                    self.assertAlmostEqual(expected.lat, actual.lat, places=6)
                    self.assertAlmostEqual(expected.lon, actual.lon, places=6)
                    self.assertAlmostEqual(expected.alt, actual.alt, places=3)
                self.assertEqual([position.name for position in parsed.positions], ["vehicle_position"])

    def test_float_literals(self):
        values = (200.0, 12.5, -10.0, 0.0, -0.0004, 0.06, 500.04)
        expected = ["200.f", "12.5f", "-10.f", "0.f", "0.f", "0.06f", "500.04f"]
        self.assertEqual([cpp_float(v) for v in values], expected)

    def test_inputs_keep_the_cli_altitude_precision(self):
        # Rounded to 500 m, these inputs would miss an expected 500.040 m by more than the 0.01 m tolerance.
        scene = Scene(missions={"mission": [at(0.0, 0.0, 500.04), at(100.0, 0.0, 500.04)]})
        self.assertIn("makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 0.04f), // 1",
                      generate_cpp(scene, BASE))
        self.assertIn(", 500.04f), // 1", generate_cpp(scene, None))

    def test_fences_round_trip_without_using_fixture_map_origin(self):
        scene = make_scene()
        scene.fences = [
            Fence(True, [(47.396, 8.544), (47.400, 8.544), (47.400, 8.550), (47.396, 8.550)]),
            Fence(False, [(47.398, 8.546)], 23.456),
        ]
        for base in (BASE, None):
            with self.subTest(base=base):
                code = generate_cpp(scene, base)
                parsed = parse_cpp(code)
                self.assertEqual(parsed.fences, scene.fences)
                self.assertEqual(len(parsed.positions), 1)
                self.assertIn("std::vector<mission_fence_point_s> fence_points", code)

    def test_fence_unit_test_uses_real_checker_and_sets_request_defaults(self):
        scene = make_scene()
        scene.fences = [Fence(False, [(47.398, 8.546)], 20.0)]
        request = PlanRequest(projection_search_distance_m=60.0)
        result = parse_output("ACTION RTL\nSTATUS FAIL NoValidCandidateFound\nEND\n")
        code = generate_unit_test(scene, BASE, request, result, "Reject fence crossing")
        self.assertIn("TEST_F(MissionRouteGeofenceTest, RejectFenceCrossing)", code)
        self.assertIn("ASSERT_TRUE(loadFence(fence_points));", code)
        self.assertIn("GeofencePathChecker checker{_fence, vehicle_position};", code)
        self.assertIn("MissionRoutePlanner planner{provider, &checker};", code)
        self.assertIn("request.projection_search_distance_m = 60.000f;", code)
        self.assertIn("request.home_altitude_amsl = 500.000f;", code)
        self.assertIn("get_home_position()->alt = 500.000f;", code)
        self.assertNotIn("kBaseLat", code)
        self.assertNotIn("kAlt", code)
        self.assertNotIn("TestPlanner", code)
        self.assertNotIn("makeRtlRouteRequest", code)
        self.assertEqual(parse_cpp(code).fences, scene.fences)

    def test_fence_export_preserves_points_near_the_boundary(self):
        from test_planner_cli import make_request, make_scene

        scene = make_scene()
        vehicle = offset_to_latlon(BASE.lat, BASE.lon, 700, 40.04)
        center = offset_to_latlon(BASE.lat, BASE.lon, 700, 20)
        scene.positions = [Position("vehicle_position", *vehicle, 500.049)]
        scene.fences = [Fence(False, [center], 20.02)]
        code = generate_cpp(scene, BASE)
        self.assertNotIn("FromOffset", code)
        self.assertNotIn("kBaseLat", code)
        self.assertIn(f"makePositionAbsolute({vehicle[0]:.9f}, {vehicle[1]:.9f}, 500.049f)", code)
        parsed = parse_cpp(code)
        request = make_request()
        # Exactly the same CLI input proves that geometry and velocity survive export.
        self.assertEqual(planner_cli.encode(parsed, request), planner_cli.encode(scene, request))

    def test_fence_export_runs_the_same_boundary_case_in_the_real_planner(self):
        from test_planner_cli import built_cli_binary, make_request, make_scene

        binary = built_cli_binary()
        scene = make_scene()
        scene.positions = [Position("vehicle_position", *offset_to_latlon(BASE.lat, BASE.lon, 700, 40.04), 500.0)]
        scene.fences = [Fence(False, [offset_to_latlon(BASE.lat, BASE.lon, 700, 20)], 20.02)]
        request = make_request()
        original = planner_cli.run(binary, planner_cli.encode(scene, request))
        exported = parse_cpp(generate_cpp(scene, BASE))
        restored = planner_cli.run(binary, planner_cli.encode(exported, request))
        self.assertEqual(original.failure, "NoValidCandidateFound")
        self.assertEqual(restored.status, original.status)

    def test_header_base_is_what_the_generator_defaults_to(self):
        self.assertEqual(base_from(header_constants()), BASE)

    def test_generated_code_compiles(self):
        compiler = shutil.which("c++")
        if compiler is None:
            self.skipTest("no C++ compiler available")
        # Stand-ins with the helper signatures, so only the generated syntax is checked.
        stubs = """
            #include <vector>
            struct mission_item_s {};
            struct mission_fence_point_s {
                double lat, lon;
                union { unsigned short vertex_count; float circle_radius; };
                unsigned short nav_cmd;
                unsigned char frame;
            };
            using uint16_t = unsigned short;
            enum { NAV_FRAME_GLOBAL, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION,
                   NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION, NAV_CMD_FENCE_CIRCLE_INCLUSION,
                   NAV_CMD_FENCE_CIRCLE_EXCLUSION };
            namespace mission_route { struct Position { double lat, lon; float alt; }; }
            struct { float velocity_north_m_s, velocity_east_m_s; } request;
            enum { NAV_CMD_WAYPOINT, NAV_CMD_LOITER_TO_ALT };
            constexpr double kBaseLat = 47.397742, kBaseLon = 8.545594;
            constexpr float kAlt = 500.f;
            mission_item_s makePositionItem(double, double, float, int = 0) { return {}; }
            mission_item_s makePositionItemFromOffset(double, double, float, float, float, int = 0) { return {}; }
            mission_item_s makeTakeoffItem(double, double, float) { return {}; }
            mission_item_s makeTakeoffItemFromOffset(double, double, float, float, float) { return {}; }
            mission_item_s makeLandItem(double, double, float) { return {}; }
            mission_item_s makeLandItemFromOffset(double, double, float, float, float) { return {}; }
            mission_item_s makeSafePointAbsolute(double, double, float) { return {}; }
            mission_item_s makeSafePointFromOffset(double, double, float, float, float) { return {}; }
            mission_route::Position makePositionAbsolute(double, double, float) { return {}; }
            mission_route::Position makePositionFromOffset(double, double, float, float, float) { return {}; }
        """
        scene = make_scene()
        scene.fences = [
            Fence(True, [(47.396, 8.544), (47.400, 8.544), (47.400, 8.550)]),
            Fence(False, [(47.398, 8.546)], 20.0),
        ]
        body = "\n".join(
            f"void fixture_{index}() {{\n{generate_cpp(scene, base)}\n}}"
            for index, base in enumerate((BASE, None))
        )
        result = subprocess.run(
            [compiler, "-std=c++14", "-Werror", "-fsyntax-only", "-x", "c++", "-"],
            input=stubs + body,
            text=True,
            capture_output=True,
            timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
