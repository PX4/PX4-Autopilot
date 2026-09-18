"""Editor checks: map clicks, selection and edits, undo and the scene the items turn into."""

import unittest

from planner_visualizer.editor import ClickSettings, Editor
from planner_visualizer.geometry import latlon_to_offset
from planner_visualizer.models import MISSION, POSITION, SAFE_POINTS, VEHICLE, Base, Position, Scene, Waypoint

BASE = Base(47.397742, 8.545594, 500.0)


def click(lat=47.4, lon=8.55):
    return {"last_clicked": {"lat": lat, "lng": lon}}


class ClickTest(unittest.TestCase):
    def test_marker_clicks_and_replayed_clicks_add_nothing(self):
        editor = Editor()
        self.assertTrue(editor.add_click(click(), MISSION, ClickSettings(), None))
        # st_folium keeps returning the last click on every rerun.
        self.assertFalse(editor.add_click(click(), SAFE_POINTS, ClickSettings(alt=600), None))
        # Clicking a marker only fills last_object_clicked.
        marker = {"last_object_clicked": {"lat": 47.5, "lng": 8.6}}
        self.assertFalse(editor.add_click(marker, MISSION, ClickSettings(), None))
        self.assertEqual(editor.count(), 1)
        self.assertTrue(editor.add_click(click(47.41), SAFE_POINTS, ClickSettings(alt=600), None))
        self.assertEqual(editor.points[SAFE_POINTS], [Waypoint(47.41, 8.55, 600.0, "NAV_CMD_RALLY_POINT")])

    def test_settings_go_into_the_point(self):
        editor = Editor()
        editor.add_click(click(), MISSION, ClickSettings(alt=510.0, cmd="NAV_CMD_TAKEOFF"), None)
        editor.add_click(click(47.41), VEHICLE, ClickSettings(alt=520.0), None)
        self.assertEqual(editor.points[MISSION], [Waypoint(47.4, 8.55, 510.0, "NAV_CMD_TAKEOFF")])
        self.assertEqual(editor.points[VEHICLE], [Waypoint(47.41, 8.55, 520.0, "")])

    def test_clicks_snap_to_the_grid_around_the_base(self):
        editor = Editor()
        editor.add_click(click(47.3995, 8.5458), MISSION, ClickSettings(grid_m=10.0), BASE)
        point = editor.points[MISSION][0]
        north, east = latlon_to_offset(BASE.lat, BASE.lon, point.lat, point.lon)
        self.assertEqual((round(north, 6), round(east, 6)), (200.0, 20.0))
        editor.add_click(click(47.3995, 8.5459), MISSION, ClickSettings(grid_m=0.0), BASE)
        self.assertEqual((editor.points[MISSION][1].lat, editor.points[MISSION][1].lon), (47.3995, 8.5459))

    def test_stacked_items_are_allowed_but_exact_duplicates_are_not(self):
        editor = Editor()
        self.assertTrue(editor.add(MISSION, Waypoint(47.0, 8.0, 500.0)))
        self.assertTrue(editor.add(MISSION, Waypoint(47.0, 8.0, 300.0, "NAV_CMD_LAND")))
        self.assertFalse(editor.add(MISSION, Waypoint(47.0, 8.0, 500.0)))
        self.assertEqual(editor.count(MISSION), 2)

    def test_malformed_clicks_change_nothing(self):
        editor = Editor()
        events = (
            None,
            {},
            {"last_clicked": None},
            {"last_clicked": {}},
            click(float("nan")),
            click(91),
            click(47, "x"),
        )
        for event in events:
            with self.subTest(event=event):
                self.assertFalse(editor.add_click(event, MISSION, ClickSettings(), None))
        self.assertEqual(editor.count(), 0)
        self.assertIsNone(editor.last_click)


class HistoryTest(unittest.TestCase):
    def test_undo_and_clear_work_across_types(self):
        editor = Editor()
        editor.add(MISSION, Waypoint(47.0, 8.0, 500.0))
        editor.add(VEHICLE, Waypoint(47.1, 8.0, 500.0, ""))
        self.assertTrue(editor.clear(MISSION))
        self.assertEqual((editor.count(MISSION), editor.count()), (0, 1))
        self.assertTrue(editor.undo())
        self.assertEqual(editor.count(), 2)
        self.assertTrue(editor.clear())
        self.assertFalse(editor.clear())
        self.assertTrue(editor.undo())
        self.assertEqual(editor.count(), 2)
        self.assertTrue(editor.undo())
        self.assertTrue(editor.undo())
        self.assertFalse(editor.undo())
        self.assertEqual(editor.count(), 0)


class SceneTest(unittest.TestCase):
    def test_scene_names_items_like_the_tests_do(self):
        editor = Editor()
        editor.add(MISSION, Waypoint(47.0, 8.0, 500.0))
        editor.add(SAFE_POINTS, Waypoint(47.1, 8.0, 500.0, "NAV_CMD_RALLY_POINT"))
        editor.add(VEHICLE, Waypoint(47.2, 8.0, 500.0, ""))
        editor.add(VEHICLE, Waypoint(47.3, 8.0, 500.0, ""))
        editor.add(POSITION, Waypoint(47.4, 8.0, 500.0, ""))
        editor.velocity = (5.0, 0.0)
        scene = editor.scene()
        self.assertEqual(list(scene.missions), ["mission"])
        self.assertEqual(len(scene.safe_points), 1)
        self.assertEqual(
            [position.name for position in scene.positions],
            ["vehicle_position", "vehicle_position_2", "position_0"],
        )
        self.assertEqual(scene.velocity, (5.0, 0.0))
        editor.clear(VEHICLE)
        self.assertIsNone(editor.scene().velocity)
        self.assertTrue(Editor().scene().is_empty())

    def test_load_puts_a_parsed_scene_into_the_editor(self):
        editor = Editor()
        editor.add(MISSION, Waypoint(46.0, 7.0, 400.0))
        scene = Scene(
            missions={"a": [Waypoint(47.0, 8.0, 500.0)], "b": [Waypoint(47.1, 8.0, 500.0)]},
            safe_points=[Waypoint(47.2, 8.0, 500.0, "NAV_CMD_RALLY_POINT")],
            positions=[Position("vehicle_position", 47.3, 8.0, 500.0), Position("goal", 47.4, 8.0, 500.0)],
            velocity=(1.0, 2.0),
        )
        editor.load(scene)
        self.assertEqual([wp.lat for wp in editor.points[MISSION]], [47.0, 47.1])
        self.assertEqual(len(editor.points[SAFE_POINTS]), 1)
        self.assertEqual([wp.lat for wp in editor.points[VEHICLE]], [47.3])
        self.assertEqual([wp.lat for wp in editor.points[POSITION]], [47.4])
        self.assertEqual(editor.velocity, (1.0, 2.0))
        self.assertTrue(editor.undo())
        self.assertEqual([wp.lat for wp in editor.points[MISSION]], [46.0])


class EditTest(unittest.TestCase):
    def setUp(self):
        self.editor = Editor()
        self.editor.add(MISSION, Waypoint(47.0, 8.0, 500.0))
        self.editor.add(MISSION, Waypoint(47.01, 8.0, 500.0, "NAV_CMD_LAND"))
        self.editor.add(SAFE_POINTS, Waypoint(47.02, 8.0, 480.0, "NAV_CMD_RALLY_POINT"))
        self.editor.add(VEHICLE, Waypoint(47.03, 8.0, 500.0, ""))
        self.editor.add(VEHICLE, Waypoint(47.04, 8.0, 500.0, ""))
        self.editor.add(POSITION, Waypoint(47.05, 8.0, 500.0, ""))

    def test_locate_reads_the_tooltips_of_new_items(self):
        cases = {
            "New mission[1]NAV_CMD_LANDalt 500.0 m": (MISSION, 1),
            "New mission[1]\nNAV_CMD_LAND\nalt 500.0 m": (MISSION, 1),
            "New safe_points[0]alt 480.0 m": (SAFE_POINTS, 0),
            "New vehicle_positionalt 500.0 m": (VEHICLE, 0),
            "New vehicle_position_2alt 500.0 m": (VEHICLE, 1),
            "New position_0alt 500.0 m": (POSITION, 0),
        }
        for tooltip, expected in cases.items():
            with self.subTest(tooltip=tooltip):
                self.assertEqual(self.editor.locate(tooltip), expected)
        for tooltip in ("Parsed mission[0]", "New mission[7]", "Join point", "Selected item", None, ""):
            with self.subTest(tooltip=tooltip):
                self.assertIsNone(self.editor.locate(tooltip))

    def test_names_match_the_generated_code(self):
        self.assertEqual(self.editor.name((MISSION, 1)), "mission[1]")
        self.assertEqual(self.editor.name((SAFE_POINTS, 0)), "safe_points[0]")
        self.assertEqual(self.editor.name((VEHICLE, 1)), "vehicle_position_2")
        names = [position.name for position in self.editor.scene().positions]
        self.assertEqual(names, ["vehicle_position", "vehicle_position_2", "position_0"])

    def test_update_ignores_float_noise_and_records_real_edits(self):
        point = self.editor.get((MISSION, 0))
        self.assertFalse(self.editor.update((MISSION, 0), point.lat + 1e-9, point.lon, point.alt, point.cmd))
        history = len(self.editor.history)
        self.assertTrue(self.editor.update((MISSION, 0), point.lat, point.lon, 550.0, "NAV_CMD_TAKEOFF"))
        self.assertEqual(self.editor.get((MISSION, 0)), Waypoint(47.0, 8.0, 550.0, "NAV_CMD_TAKEOFF"))
        self.assertEqual(len(self.editor.history), history + 1)
        self.assertTrue(self.editor.undo())
        self.assertEqual(self.editor.get((MISSION, 0)), point)
        self.assertFalse(self.editor.update((MISSION, 9), 47.0, 8.0, 500.0, ""))

    def test_move_to_click_consumes_the_click_and_snaps(self):
        event = click(47.3995, 8.5458)
        self.assertTrue(self.editor.move_to_click(event, (SAFE_POINTS, 0), BASE, 10.0))
        moved = self.editor.get((SAFE_POINTS, 0))
        north, east = latlon_to_offset(BASE.lat, BASE.lon, moved.lat, moved.lon)
        self.assertEqual((round(north, 6), round(east, 6)), (200.0, 20.0))
        self.assertEqual((moved.alt, moved.cmd), (480.0, "NAV_CMD_RALLY_POINT"))
        # The same click must not turn into a new point afterwards.
        self.assertFalse(self.editor.add_click(event, MISSION, ClickSettings(), BASE))
        # A stale selection leaves the click for the next taker.
        self.assertFalse(self.editor.move_to_click(click(47.5), (MISSION, 9), None, 0.0))
        self.assertTrue(self.editor.add_click(click(47.5), MISSION, ClickSettings(), None))

    def test_remove_shifts_the_following_items(self):
        self.assertTrue(self.editor.remove((VEHICLE, 0)))
        self.assertEqual(self.editor.count(VEHICLE), 1)
        self.assertEqual(self.editor.get((VEHICLE, 0)), Waypoint(47.04, 8.0, 500.0, ""))
        self.assertFalse(self.editor.remove((VEHICLE, 5)))
        self.assertTrue(self.editor.undo())
        self.assertEqual(self.editor.count(VEHICLE), 2)


if __name__ == "__main__":
    unittest.main()
