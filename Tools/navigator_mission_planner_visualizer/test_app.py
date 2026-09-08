"""Streamlit flows, with map events injected at the component boundary."""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from streamlit.testing.v1 import AppTest

from planner_visualizer.planner_cli import parse_output
from test_planner_cli import CLI_OUTPUT

APP = Path(__file__).with_name("mission_planner_tools.py")
SOURCE = (
    "std::vector<mission_item_s> mission{\n"
    "\tmakeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),\n"
    "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),\n"
    "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),\n"
    "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt),\n"
    "\tmakeLandItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt),\n"
    "};\n"
    "std::vector<mission_item_s> safe_points{\n"
    "\tmakeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 20.f, kAlt),\n"
    "\tmakeSafePointFromOffset(kBaseLat, kBaseLon, 1100.f, 20.f, kAlt),\n"
    "};\n"
    "const mission_route::Position vehicle_position =\n"
    "\tmakePositionFromOffset(kBaseLat, kBaseLon, 700.f, 40.f, kAlt);\n"
)


class AppTest_(unittest.TestCase):
    def setUp(self):
        patcher = patch("planner_visualizer.app.st_folium", return_value={})
        self.map_component = patcher.start()
        self.addCleanup(patcher.stop)
        self.app = AppTest.from_file(str(APP), default_timeout=10).run()
        self.assertFalse(self.app.exception)

    def click(self, lat, lon):
        self.map_component.return_value = {"last_clicked": {"lat": lat, "lng": lon}}
        self.app.run()
        self.assertFalse(self.app.exception)

    def marker_click(self, tooltip, lat, lon):
        """A click on a marker: st_folium keeps the last map click and adds the object fields."""
        event = dict(self.map_component.return_value)
        event["last_object_clicked"] = {"lat": lat, "lng": lon}
        event["last_object_clicked_tooltip"] = tooltip
        self.map_component.return_value = event
        self.app.run()
        self.assertFalse(self.app.exception)

    def button(self, label):
        return next(button for button in self.app.button if button.label == label)

    def code(self):
        return self.app.code[0].value if self.app.code else ""

    def test_code_follows_the_edits_without_a_button(self):
        self.assertEqual(self.code(), "")
        self.click(47.3995, 8.5458)
        self.assertIn("makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt), // 0", self.code())

        self.app.radio(key="mission_kind").set_value("Land").run()
        self.app.number_input(key="alt").set_value(490.0).run()
        self.click(47.4013, 8.5456)
        self.assertIn("makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f), // 1", self.code())

        self.app.selectbox(key="item_type").set_value("Vehicle position").run()
        self.app.number_input(key="vel_north").set_value(5.0).run()
        self.click(47.3986, 8.5457)
        self.assertIn(
            "vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 10.f, kAlt - 10.f);",
            self.code(),
        )
        self.assertIn("request.velocity_north_m_s = 5.f;", self.code())

        self.app.radio(key="coordinates").set_value("Absolute lat/lon").run()
        self.assertIn("makePositionItem(47.3995406, 8.5458597, 500.f), // 0", self.code())
        self.assertNotIn("kBaseLat", self.code())

        self.button("Undo").click().run()
        self.assertNotIn("vehicle_position", self.code())
        self.button("Clear all new items").click().run()
        self.assertEqual(self.code(), "")
        self.assertTrue(self.button("Clear all new items").disabled)

    def test_marker_clicks_and_control_changes_add_nothing(self):
        self.click(47.3995, 8.5458)
        self.map_component.return_value = {
            "last_clicked": {"lat": 47.3995, "lng": 8.5458},
            "last_object_clicked": {"lat": 47.3, "lng": 8.5},
        }
        self.app.selectbox(key="item_type").set_value("Safe points").run()
        self.app.number_input(key="alt").set_value(600.0).run()
        self.assertEqual(self.app.session_state["editor"].count(), 1)
        self.assertNotIn("safe_points", self.code())

    def test_parse_zooms_to_the_fixtures_and_keeps_new_items(self):
        self.click(47.3995, 8.5458)
        self.app.text_area(key="source").set_value(
            "std::vector<mission_item_s> mission{\n"
            "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),\n"
            "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),\n"
            "};\n"
            "const mission_route::Position vehicle_position =\n"
            "\tmakePositionFromOffset(kBaseLat, kBaseLon, 50.f, 20.f, kAlt);\n"
        )
        self.button("Parse").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(self.app.session_state["parsed"].summary(), "2 mission items, 1 positions")
        (center_lat, _), zoom = self.app.session_state["view"]
        self.assertAlmostEqual(center_lat, 47.397742 + 0.0044965, places=5)
        self.assertLess(zoom, 16)
        self.assertEqual(self.app.session_state["editor"].count(), 1)
        self.assertTrue(any("Showing 2 mission items" in caption.value for caption in self.app.caption))

        self.button("Load parsed items into the editor").click().run()
        self.assertEqual(self.app.session_state["editor"].count(), 3)
        self.assertIn("1000.f, 0.f, kAlt), // 1", self.code())

        self.button("Clear parsed").click().run()
        self.assertIsNone(self.app.session_state["parsed"])
        self.assertEqual(self.app.session_state["editor"].count(), 3)
        self.assertTrue(self.button("Load parsed items into the editor").disabled)

    def test_recenter_returns_to_the_base_view(self):
        self.app.text_area(key="source").set_value(
            "std::vector<mission_item_s> mission{\n"
            "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),\n"
            "\tmakePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),\n"
            "};\n"
        )
        self.button("Parse").click().run()
        (center_lat, _), zoom = self.app.session_state["view"]
        self.assertGreater(center_lat, 47.3990)
        self.assertLess(zoom, 16)

        self.button("Recenter map on base").click().run()
        self.assertFalse(self.app.exception)
        self.assertEqual(self.app.session_state["view"], ((47.397742, 8.545594), 16))
        # A new widget key makes st_folium move even when the view tuple did not change.
        self.assertEqual(self.app.session_state["map_key"], 1)
        self.button("Recenter map on base").click().run()
        self.assertEqual(self.app.session_state["map_key"], 2)

    def test_empty_source_warns(self):
        self.button("Parse").click().run()
        self.assertTrue(any("No mission items" in warning.value for warning in self.app.warning))
        self.assertIsNone(self.app.session_state["parsed"])

    def test_without_the_binary_the_panel_shows_the_build_command(self):
        root = Path(tempfile.mkdtemp())
        with patch("planner_visualizer.planner_cli.repo_root", return_value=root):
            self.app.run()
        self.assertTrue(any("EXTERNAL_MODULES_LOCATION" in code.value for code in self.app.code))
        self.assertFalse(any(button.label == "Plan Return" for button in self.app.button))

    def test_planner_flow_runs_the_binary_and_writes_a_test(self):
        root = Path(tempfile.mkdtemp())
        (root / "Makefile").write_text("")
        (root / "src/modules/navigator").mkdir(parents=True)
        (root / "build/px4_sitl_test").mkdir(parents=True)
        (root / "build/px4_sitl_test/mission_route_planner_cli").write_text("")
        with patch("planner_visualizer.planner_cli.repo_root", return_value=root), patch(
            "planner_visualizer.planner_cli.run", return_value=parse_output(CLI_OUTPUT)
        ) as run:
            self.app.run()
            self.assertTrue(self.button("Plan Return").disabled)
            self.app.text_area(key="source").set_value(SOURCE)
            self.button("Parse").click().run()
            self.app.number_input(key="req_mission_index").set_value(2).run()
            self.button("Plan Return").click().run()
            self.assertFalse(self.app.exception)
            self.assertEqual(run.call_count, 1)
            scenario = run.call_args[0][1]
            self.assertIn("SET mission_index 2", scenario)
            self.assertIn("ACTION RTL", scenario)
            success = [message.value for message in self.app.success]
            self.assertIn("RTL: first item 2, nominal direction, goal safe_point", success)
            self.assertIn("join", list(self.app.table[0].value.index))
            code = "\n".join(block.value for block in self.app.code)
            self.assertIn("TEST_F(MissionRoutePlannerTest, GeneratedScenario)", code)
            self.assertIn("Route plan to safe_point", code)

            self.app.number_input(key="req_rally_margin").set_value(0.0).run()
            self.assertTrue(any("changed since this plan" in caption.value for caption in self.app.caption))

    def test_selected_item_is_edited_from_the_sidebar(self):
        self.click(47.3995, 8.5458)  # mission[0], 200 m north and 20 m east of the base
        self.click(47.4013, 8.5456)  # mission[1]
        self.assertTrue(any("Click one of the new items" in caption.value for caption in self.app.caption))
        self.marker_click("New mission[0]NAV_CMD_WAYPOINTalt 500.0 m", 47.3995, 8.5458)
        self.assertEqual(self.app.session_state["selected"], ("Mission", 0))
        self.assertEqual(self.app.number_input(key="edit_north").value, 200.0)
        self.assertEqual(self.app.number_input(key="edit_east").value, 20.0)
        self.app.number_input(key="edit_alt").set_value(550.0).run()
        self.app.selectbox(key="edit_cmd").set_value("Takeoff").run()
        self.assertIn(
            "makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 200.f, 20.f, kAlt + 50.f), // 0", self.code()
        )
        self.app.number_input(key="edit_east").set_value(60.0).run()
        self.assertIn("200.f, 60.f, kAlt + 50.f), // 0", self.code())
        # A marker click is replayed on every rerun and must not reselect after a deselect.
        self.button("Deselect").click().run()
        self.assertIsNone(self.app.session_state["selected"])
        self.marker_click("New mission[0]NAV_CMD_TAKEOFFalt 550.0 m", 47.3995, 8.5458)
        self.assertEqual(self.app.session_state["selected"], ("Mission", 0))
        # Move to the next map click, which adds nothing itself.
        self.button("Move to next map click").click().run()
        self.assertTrue(self.app.session_state["moving"])
        self.click(47.3986, 8.5457)  # 100 m north, 10 m east
        self.assertFalse(self.app.session_state["moving"])
        self.assertIn("100.f, 10.f, kAlt + 50.f), // 0", self.code())
        self.assertEqual(self.app.session_state["editor"].count(), 2)
        self.assertEqual(self.app.number_input(key="edit_north").value, 100.0)
        self.button("Delete").click().run()
        self.assertIsNone(self.app.session_state["selected"])
        self.assertEqual(self.app.session_state["editor"].count(), 1)
        self.assertNotIn("Takeoff", self.code())

    def test_undo_refills_the_edit_form(self):
        self.click(47.3995, 8.5458)
        self.marker_click("New mission[0]NAV_CMD_WAYPOINTalt 500.0 m", 47.3995, 8.5458)
        self.app.number_input(key="edit_alt").set_value(550.0).run()
        self.assertIn("kAlt + 50.f", self.code())
        self.button("Undo").click().run()
        self.assertFalse(self.app.exception)
        self.assertNotIn("kAlt + 50.f", self.code())
        self.assertEqual(self.app.number_input(key="edit_alt").value, 500.0)
        self.assertEqual(self.app.session_state["selected"], ("Mission", 0))

    def test_parsed_markers_and_plan_pins_select_nothing(self):
        self.click(47.3995, 8.5458)
        self.marker_click("Parsed mission[0]NAV_CMD_WAYPOINTalt 500.0 m", 47.3995, 8.5458)
        self.assertIsNone(self.app.session_state["selected"])
        self.marker_click("Join pointalt 500.0 m", 47.3995, 8.5458)
        self.assertIsNone(self.app.session_state["selected"])

    def test_every_vehicle_position_gets_a_plan(self):
        root = Path(tempfile.mkdtemp())
        (root / "Makefile").write_text("")
        (root / "src/modules/navigator").mkdir(parents=True)
        (root / "build/px4_sitl_test").mkdir(parents=True)
        (root / "build/px4_sitl_test/mission_route_planner_cli").write_text("")
        source = SOURCE + (
            "const mission_route::Position vehicle_position_2 =\n"
            "\tmakePositionFromOffset(kBaseLat, kBaseLon, 1300.f, 40.f, kAlt);\n"
        )
        with patch("planner_visualizer.planner_cli.repo_root", return_value=root), patch(
            "planner_visualizer.planner_cli.run", return_value=parse_output(CLI_OUTPUT)
        ) as run:
            self.app.text_area(key="source").set_value(source)
            self.button("Parse").click().run()
            self.button("Plan Return").click().run()
            self.assertFalse(self.app.exception)
            self.assertEqual(run.call_count, 2)
            first, second = (call[0][1] for call in run.call_args_list)
            self.assertNotEqual(first, second)
            self.assertEqual(list(self.app.table[0].value.index), ["vehicle_position", "vehicle_position_2"])
            self.app.selectbox(key="plan_vehicle").set_value("vehicle_position_2").run()
            self.assertFalse(self.app.exception)
            code = "\n".join(block.value for block in self.app.code)
            self.assertIn("makeRtlRouteRequest(vehicle_position_2, ", code)
            self.assertNotIn("Position vehicle_position =", code)

    def test_3d_view_is_optional(self):
        self.click(47.3995, 8.5458)
        self.app.checkbox(key="show_3d").set_value(True).run()
        self.assertFalse(self.app.exception)


if __name__ == "__main__":
    unittest.main()
