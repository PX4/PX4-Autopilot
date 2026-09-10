"""Generated C++ must look like the tests and parse back to the same fixtures."""

import shutil
import subprocess
import unittest

from planner_visualizer.cpp_parser import header_constants, parse_cpp
from planner_visualizer.generator import cpp_float, generate_cpp
from planner_visualizer.geometry import offset_to_latlon
from planner_visualizer.models import Base, Position, Scene, Waypoint, base_from

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
            "request.velocity_east_m_s = 0.f;",
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
                self.assertEqual(parsed.velocity, (5.0, 0.0))
                parsed_items = parsed.missions["mission"] + parsed.safe_points
                self.assertEqual([wp.cmd for wp in parsed_items], [wp.cmd for wp in expected_items])
                for expected, actual in zip(expected_items, parsed_items):
                    self.assertAlmostEqual(expected.lat, actual.lat, places=6)
                    self.assertAlmostEqual(expected.lon, actual.lon, places=6)
                    self.assertAlmostEqual(expected.alt, actual.alt, places=1)
                self.assertEqual([position.name for position in parsed.positions], ["vehicle_position"])

    def test_float_literals(self):
        values = (200.0, 12.5, -10.0, 0.0, -0.01, 0.06)
        self.assertEqual([cpp_float(v) for v in values], ["200.f", "12.5f", "-10.f", "0.f", "0.f", "0.1f"])

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
        body = "\n".join(
            f"void fixture_{index}() {{\n{generate_cpp(make_scene(), base)}\n}}"
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
