"""The bridge to the planner CLI: scenario text, output parsing, staleness and the unit test it writes."""

import os
from pathlib import Path
import stat
import tempfile
import time
import unittest

from planner_visualizer import planner_cli
from planner_visualizer.geometry import offset_to_latlon
from planner_visualizer.models import Base, Position, Scene, Waypoint
from planner_visualizer.planner_cli import PlanRequest
from planner_visualizer.unit_test import generate_unit_test, test_name

BASE = (47.397742, 8.545594)

# What the CLI prints for make_scene() with PlanRequest(mission_index=2, margins 60 m), trace shortened.
CLI_OUTPUT = """ACTION RTL
DEBUG 557881941984 Route current segment bounds: [500.000, 1000.000]
DEBUG 557881942050 Route plan to safe_point target=2 rev=0 direct=0 skip_alt=0 branch_off=2->3
STATUS OK
PLAN valid 1
PLAN join_lat 47.404037252
PLAN join_lon 8.545594000
PLAN join_alt 500.000
PLAN first_mission_item_index 2
PLAN direction_reversed 0
PLAN use_current_altitude 0
PLAN active_jump_anchor -1
PLAN vtol_transition_action none
PLAN goal_type safe_point
PLAN goal_lat 47.407634538
PLAN goal_lon 8.545859716
PLAN goal_alt 500.000
PLAN safe_point_index 1
PLAN branch_off_lat 47.407634538
PLAN branch_off_lon 8.545594000
PLAN branch_off_alt 500.000
PLAN branch_off_mission_item_index 3
PLAN fly_direct_to_goal 0
END
"""


def point(north, east, alt=500.0, cmd="NAV_CMD_WAYPOINT"):
    lat, lon = offset_to_latlon(*BASE, north, east)
    return Waypoint(lat, lon, alt, cmd)


def make_scene():
    vehicle_lat, vehicle_lon = offset_to_latlon(*BASE, 700.0, 40.0)
    return Scene(
        missions={
            "mission": [
                point(0, 0, cmd="NAV_CMD_TAKEOFF"),
                point(500, 0),
                point(1000, 0),
                point(1500, 0),
                point(2000, 0, cmd="NAV_CMD_LAND"),
            ]
        },
        safe_points=[point(300, 20, cmd="NAV_CMD_RALLY_POINT"), point(1100, 20, cmd="NAV_CMD_RALLY_POINT")],
        positions=[Position("vehicle_position", vehicle_lat, vehicle_lon, 500.0)],
        velocity=(15.0, 0.0),
    )


def make_request():
    return PlanRequest(
        action="RTL",
        mission_index=2,
        projection_search_distance_m=60.0,
        safe_point_projection_search_distance_m=60.0,
    )


class EncodeTest(unittest.TestCase):
    def test_scenario_lists_items_rallies_vehicle_settings_and_action(self):
        lines = planner_cli.encode(make_scene(), make_request()).splitlines()
        self.assertEqual(lines[0], "ITEM TAKEOFF 47.397742000 8.545594000 500.000")
        self.assertEqual(lines[4][:9], "ITEM LAND")
        self.assertEqual(len([line for line in lines if line.startswith("RALLY ")]), 2)
        self.assertEqual(len([line for line in lines if line.startswith("VEHICLE ")]), 1)
        self.assertIn("SET mission_index 2", lines)
        self.assertIn("SET projection_search_distance_m 60.000", lines)
        self.assertIn("SET is_fixed_wing 0", lines)
        self.assertIn("SET vtol_state_on_mission_upload UNDEFINED", lines)
        self.assertIn("SET velocity_north_m_s 15.000", lines)
        self.assertEqual(lines[-1], "ACTION RTL")

    def test_scenario_needs_items_and_a_vehicle(self):
        with self.assertRaises(ValueError):
            planner_cli.encode(Scene(), make_request())
        scene = make_scene()
        scene.positions = []
        with self.assertRaises(ValueError):
            planner_cli.encode(scene, make_request())

    def test_land_index_finds_the_landing_item(self):
        self.assertEqual(planner_cli.land_index(make_scene().missions["mission"]), 4)
        self.assertEqual(planner_cli.land_index([point(0, 0)]), -1)


class ParseTest(unittest.TestCase):
    def test_output_splits_plan_and_trace(self):
        result = planner_cli.parse_output(CLI_OUTPUT)
        self.assertEqual(result.action, "RTL")
        self.assertTrue(result.ok)
        self.assertEqual(result.failure, "")
        self.assertEqual(result.int_field("first_mission_item_index"), 2)
        self.assertEqual(result.int_field("safe_point_index"), 1)
        self.assertFalse(result.bool_field("direction_reversed"))
        self.assertEqual(result.plan["goal_type"], "safe_point")
        self.assertAlmostEqual(result.position("join")[0], 47.404037252)
        self.assertIsNone(result.position("missing"))
        self.assertEqual(len(result.trace), 2)
        self.assertIn("Route plan to safe_point", result.trace[1])

    def test_rows_group_positions_and_drop_the_valid_flag(self):
        rows = {row["field"]: row["value"] for row in planner_cli.parse_output(CLI_OUTPUT).rows()}
        self.assertNotIn("valid", rows)
        self.assertNotIn("join_lon", rows)
        self.assertEqual(rows["join"], "47.4040373, 8.5455940, 500.0 m")
        self.assertEqual(rows["goal_type"], "safe_point")
        self.assertEqual(rows["branch_off_mission_item_index"], "3")
        missing = planner_cli.parse_output(
            "PLAN branch_off_lat nan\nPLAN branch_off_lon nan\nPLAN branch_off_alt nan\n"
        )
        self.assertEqual(missing.rows(), [{"field": "branch_off", "value": "none"}])

    def test_failure_status_is_reported(self):
        result = planner_cli.parse_output("ACTION RTL\nSTATUS FAIL no valid path\nEND\n")
        self.assertFalse(result.ok)
        self.assertEqual(result.failure, "no valid path")


RESUME_OUTPUT = (
    "ACTION RESUME\nSTATUS OK\nPLAN join_lat 47.404037252\nPLAN join_lon 8.545594000\n"
    "PLAN join_alt 500.000\nPLAN first_mission_item_index 2\nPLAN direction_reversed 0\nEND\n"
)


class VehiclesTest(unittest.TestCase):
    def test_every_vehicle_position_is_planned_or_else_the_first_position(self):
        scene = make_scene()
        scene.positions.append(Position("vehicle_position_2", 47.41, 8.55, 500.0))
        scene.positions.append(Position("goal", 47.42, 8.55, 500.0))
        names = [position.name for position in planner_cli.vehicle_positions(scene)]
        self.assertEqual(names, ["vehicle_position", "vehicle_position_2"])
        self.assertEqual(planner_cli.vehicle_position(scene).name, "vehicle_position")
        only_goal = Scene(positions=[Position("goal", 47.42, 8.55, 500.0)])
        self.assertEqual([position.name for position in planner_cli.vehicle_positions(only_goal)], ["goal"])
        self.assertEqual(planner_cli.vehicle_positions(Scene()), [])

    def test_encode_takes_the_vehicle_to_plan_for(self):
        scene = make_scene()
        second = Position("vehicle_position_2", 47.41, 8.55, 510.0)
        scene.positions.append(second)
        lines = planner_cli.encode(scene, make_request(), second).splitlines()
        self.assertIn("VEHICLE 47.410000000 8.550000000 510.000", lines)
        self.assertEqual(len([line for line in lines if line.startswith("VEHICLE ")]), 1)


class PlanPathTest(unittest.TestCase):
    def test_path_runs_vehicle_join_route_branch_off_goal(self):
        scene = make_scene()
        result = planner_cli.parse_output(CLI_OUTPUT)
        path = planner_cli.plan_path(scene.missions["mission"], scene.positions[0], result)
        self.assertEqual(len(path), 5)
        self.assertEqual(path[0], (scene.positions[0].lat, scene.positions[0].lon))
        self.assertAlmostEqual(path[1][0], 47.404037252)
        self.assertEqual(path[2], (scene.missions["mission"][2].lat, scene.missions["mission"][2].lon))
        self.assertAlmostEqual(path[3][0], 47.407634538)  # branch-off replaces item 3
        self.assertAlmostEqual(path[4][1], 8.545859716)  # goal

    def test_direct_to_goal_is_a_straight_line(self):
        output = CLI_OUTPUT.replace("PLAN fly_direct_to_goal 0", "PLAN fly_direct_to_goal 1")
        scene = make_scene()
        path = planner_cli.plan_path(
            scene.missions["mission"], scene.positions[0], planner_cli.parse_output(output)
        )
        self.assertEqual(len(path), 2)

    def test_path_stops_at_the_first_land_item(self):
        scene = make_scene()
        items = scene.missions["mission"] + [point(2500, 0), point(3000, 0)]
        path = planner_cli.plan_path(items, scene.positions[0], planner_cli.parse_output(RESUME_OUTPUT))
        # vehicle, join, items 2 and 3, then the LAND at 4 and nothing beyond it
        self.assertEqual(len(path), 5)
        self.assertEqual(path[-1], (items[4].lat, items[4].lon))

    def test_landing_goal_is_not_drawn_twice(self):
        scene = make_scene()
        items = scene.missions["mission"]
        land = items[4]
        output = RESUME_OUTPUT.replace("ACTION RESUME", "ACTION RTL").replace(
            "END\n",
            f"PLAN goal_type mission_land\nPLAN goal_lat {land.lat:.9f}\nPLAN goal_lon {land.lon:.9f}\n"
            "PLAN goal_alt 500.000\nPLAN fly_direct_to_goal 0\nEND\n",
        )
        path = planner_cli.plan_path(items, scene.positions[0], planner_cli.parse_output(output))
        self.assertEqual(len(path), 5)

    def test_failed_plan_has_no_path(self):
        result = planner_cli.parse_output("ACTION RTL\nSTATUS FAIL no valid path\nEND\n")
        self.assertEqual(planner_cli.plan_path([], make_scene().positions[0], result), [])


class BinaryTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        (self.root / "Makefile").write_text("")
        (self.root / "src/modules/navigator").mkdir(parents=True)
        (self.root / planner_cli.BUILD_DIR).mkdir(parents=True)
        self.binary = planner_cli.binary_path(self.root)

    def write_binary(self, output):
        self.binary.write_text(
            "#!/bin/sh\ncat > /dev/null\ncat <<'END_OF_OUTPUT'\n" + output + "END_OF_OUTPUT\n"
        )
        self.binary.chmod(self.binary.stat().st_mode | stat.S_IXUSR)

    def test_repo_root_walks_up_to_the_checkout(self):
        self.assertEqual(planner_cli.repo_root(self.root / "src/modules/navigator/x.cpp"), self.root)
        self.assertIsNone(planner_cli.repo_root(Path(tempfile.mkdtemp()) / "x"))

    def test_build_command_points_at_the_cli_directory(self):
        command = planner_cli.build_command(self.root)
        self.assertTrue(command.startswith("make px4_sitl_test EXTERNAL_MODULES_LOCATION="))
        self.assertIn(str(self.root / planner_cli.CLI_LOCATION), command)
        self.assertTrue(command.endswith(planner_cli.BINARY_NAME))

    def test_sources_newer_than_the_binary_are_stale(self):
        self.write_binary("END\n")
        old = time.time() - 100
        os.utime(self.binary, (old, old))
        source = self.root / "src/modules/navigator/mission_route_planner.cpp"
        source.write_text("")
        self.assertEqual(
            planner_cli.stale_sources(self.root, self.binary),
            ["src/modules/navigator/mission_route_planner.cpp"],
        )
        os.utime(source, (old - 10, old - 10))
        self.assertEqual(planner_cli.stale_sources(self.root, self.binary), [])
        self.assertEqual(planner_cli.stale_sources(self.root, self.root / "missing"), [])

    def test_run_feeds_the_scenario_and_parses_the_answer(self):
        self.write_binary(CLI_OUTPUT)
        result = planner_cli.run(self.binary, planner_cli.encode(make_scene(), make_request()))
        self.assertTrue(result.ok)
        self.assertEqual(result.int_field("safe_point_index"), 1)

    def test_run_reports_a_failing_binary(self):
        self.binary.write_text("#!/bin/sh\necho 'invalid scenario: no VEHICLE line' >&2\nexit 2\n")
        self.binary.chmod(self.binary.stat().st_mode | stat.S_IXUSR)
        with self.assertRaises(RuntimeError) as raised:
            planner_cli.run(self.binary, "ACTION RTL\n")
        self.assertIn("no VEHICLE line", str(raised.exception))


class UnitTestGenerationTest(unittest.TestCase):
    def setUp(self):
        self.base = Base(*BASE, 500.0)
        self.scene = make_scene()
        self.result = planner_cli.parse_output(CLI_OUTPUT)

    def test_return_test_has_fixtures_request_and_expectations(self):
        code = generate_unit_test(
            self.scene, self.base, make_request(), self.result, "rally beyond the vehicle wins"
        )
        self.assertIn("TEST_F(MissionRoutePlannerTest, RallyBeyondTheVehicleWins)", code)
        self.assertIn("\tstd::vector<mission_item_s> mission{", code)
        self.assertIn("makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), // 0", code)
        self.assertIn("std::vector<mission_item_s> safe_points{", code)
        self.assertIn("TestPlanner planner(mission, safe_points);", code)
        self.assertIn("makePositionFromOffset(kBaseLat, kBaseLon, 700.f, 40.f, kAlt);", code)
        self.assertIn(
            "mission_route::RtlRouteRequest request = makeRtlRouteRequest(vehicle_position, 2);", code
        )
        self.assertIn("request.velocity_north_m_s = 15.f;", code)
        # 60 m margins and 10 m radii are the helper defaults, so they are not repeated.
        self.assertNotIn("projection_search_distance_m", code)
        self.assertNotIn("acceptance_radius_m", code)
        self.assertNotIn("fw_u_turn_penalty_m", code)
        self.assertIn("planner.planRtlRoute(request, plan);", code)
        self.assertIn("EXPECT_EQ(plan.first_mission_item_index, 2);", code)
        self.assertIn("EXPECT_FALSE(plan.direction_reversed);", code)
        self.assertIn("EXPECT_EQ(plan.goal_type, mission_route::GoalType::kSafePoint);", code)
        self.assertIn("EXPECT_EQ(plan.safe_point_index, 1);", code)
        self.assertIn("EXPECT_EQ(plan.branch_off_mission_item_index, 3);", code)
        self.assertIn("EXPECT_NEAR(plan.join_position.lat, 47.4040373, 1e-6);", code)
        self.assertIn("EXPECT_FALSE(plan.fly_direct_to_goal);", code)
        self.assertTrue(code.rstrip().endswith("}"))

    def test_non_default_request_fields_are_written(self):
        request = make_request()
        request.projection_search_distance_m = 150.0
        request.is_fixed_wing = True
        request.active_jump_anchor = 8
        request.mission_land_index = 4
        code = generate_unit_test(self.scene, self.base, request, self.result, "x")
        self.assertIn("request.projection_search_distance_m = 150.f;", code)
        self.assertIn("request.is_fixed_wing = true;", code)
        self.assertIn("request.active_jump_anchor = {8};", code)
        self.assertIn("request.mission_land_index = 4;", code)

    def test_resume_test_uses_the_resume_helper(self):
        request = make_request()
        request.action = "RESUME"
        result = planner_cli.parse_output(CLI_OUTPUT.replace("ACTION RTL", "ACTION RESUME"))
        code = generate_unit_test(self.scene, self.base, request, result, "resume")
        self.assertIn("makeMissionResumeRequest(vehicle_position, 2)", code)
        self.assertIn("planner.planMissionResumeJoin(request, plan);", code)
        self.assertNotIn("goal_type", code)
        self.assertNotIn("safe_point_projection_search_distance_m", code)

    def test_failed_plan_asserts_the_failure(self):
        result = planner_cli.parse_output("ACTION RTL\nSTATUS FAIL no valid path\nEND\n")
        code = generate_unit_test(self.scene, self.base, make_request(), result, "fails")
        self.assertIn("// The planner reported: no valid path", code)
        self.assertIn("EXPECT_NE(status, mission_route::FailureReason::kNone);", code)

    def test_only_the_planned_vehicle_is_emitted(self):
        scene = make_scene()
        second = Position("vehicle_position_2", *offset_to_latlon(*BASE, 900.0, 40.0), 500.0)
        scene.positions.append(second)
        code = generate_unit_test(scene, self.base, make_request(), self.result, "second", second)
        self.assertIn("makeRtlRouteRequest(vehicle_position_2, 2)", code)
        self.assertIn("const mission_route::Position vehicle_position_2 =", code)
        self.assertNotIn("Position vehicle_position =", code)

    def test_names_become_identifiers(self):
        self.assertEqual(test_name("rally 2 wins over 1!"), "Rally2WinsOver1")
        self.assertEqual(test_name("GeneratedScenario"), "GeneratedScenario")
        self.assertEqual(test_name("keeps LAND as first target"), "KeepsLANDAsFirstTarget")
        self.assertEqual(test_name("42"), "GeneratedScenario")
        self.assertEqual(test_name(""), "GeneratedScenario")


if __name__ == "__main__":
    unittest.main()
