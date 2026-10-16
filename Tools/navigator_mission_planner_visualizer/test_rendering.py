"""Map rendering checks without a Streamlit server."""

import json
import unittest

import folium

from planner_visualizer.editor import Editor
from planner_visualizer.map2d import (
    NEW_COLORS, PLAN_COLORS, add_plan_layer, build_base_map, build_new_items_layer, legend_html,
)
from planner_visualizer.planner_cli import parse_output
from planner_visualizer.map3d import NEW_RGB, PARSED_RGB, build_deck
from planner_visualizer.models import MISSION, SAFE_POINTS, VEHICLE, Fence, Position, Scene, Waypoint


def make_scene():
    return Scene(
        missions={"mission": [Waypoint(47.0, 8.0, 500.0), Waypoint(47.01, 8.0, 520.0, "NAV_CMD_LAND")]},
        safe_points=[Waypoint(47.02, 8.02, 480.0, "NAV_CMD_RALLY_POINT")],
        positions=[Position("vehicle_position", 47.005, 8.001, 500.0), Position("goal", 47.03, 8.03, 500.0)],
        velocity=(5.0, 0.0),
    )


def render(element):
    return element.get_root().render()


PLAN_OUTPUT = (
    "ACTION RTL\nSTATUS OK\nPLAN join_lat 47.005\nPLAN join_lon 8.0\nPLAN join_alt 505.0\n"
    "PLAN first_mission_item_index 1\nPLAN direction_reversed 0\nPLAN goal_type safe_point\n"
    "PLAN goal_lat 47.02\nPLAN goal_lon 8.02\nPLAN goal_alt 480.0\n"
    "PLAN branch_off_lat 47.01\nPLAN branch_off_lon 8.0\nPLAN branch_off_alt 520.0\n"
    "PLAN branch_off_mission_item_index 1\nPLAN fly_direct_to_goal 0\nEND\n"
)


class Map2dTest(unittest.TestCase):
    def test_fences_and_unfinished_polygon_are_visible(self):
        scene = Scene(fences=[Fence(True, [(47, 8), (47.01, 8), (47.01, 8.01)]),
                              Fence(False, [(47, 8)], 50)])
        draft = Fence(False, [(47.02, 8.02), (47.02, 8.03)])
        fmap = build_base_map((47, 8), 15, scene)
        build_new_items_layer(Scene(), False, draft_fence=draft).add_to(fmap)
        html = render(fmap)
        self.assertIn("Inclusion polygon", html)
        self.assertIn("Exclusion circle, radius 50 m", html)
        self.assertIn('"radius": 50', html)
        self.assertIn('"dashArray": "5 5"', html)
        self.assertEqual(len(scene.all_points()), 67)

    def test_parsed_fences_are_dashed_and_new_ones_solid(self):
        fence = Fence(False, [(47, 8)], 50)
        self.assertIn('"dashArray": "6 4"', render(build_base_map((47, 8), 15, Scene(fences=[fence]))))
        layer = build_new_items_layer(Scene(fences=[fence]), False)
        html = render(folium.Map(location=(47, 8)).add_child(layer))
        self.assertIn("New geofence[0]", html)
        self.assertNotIn('"dashArray": "6 4"', html)

    def test_base_map_has_a_static_legend(self):
        html = render(build_base_map((47.0, 8.0), 15, None))
        labels = ("Legend", "New safe point, vehicle, position", "Parsed test, read-only", "Join, branch-off, goal")
        for label in labels:
            with self.subTest(label=label):
                self.assertIn(label, html)
        # The base map must not change between clicks, so neither may the legend.
        self.assertEqual(legend_html(), legend_html())
        self.assertIn("<details", legend_html())
        self.assertNotIn("<details open", legend_html())

    def test_plan_colours_differ_from_new_item_pins(self):
        self.assertFalse(set(PLAN_COLORS) & set(NEW_COLORS.values()))

    def test_plan_layer_shows_path_join_branch_off_and_goal(self):
        layer = build_new_items_layer(Scene(), highlight_mission=False)
        path = [(47.005, 8.001), (47.005, 8.0), (47.01, 8.0), (47.02, 8.02)]
        add_plan_layer(layer, parse_output(PLAN_OUTPUT), path)
        html = render(folium.Map(location=(47.0, 8.0), zoom_start=12).add_child(layer))
        for label in (
            "Planned path, RTL",
            "Join point",
            "Branch-off",
            "Goal: safe_point",
        ):
            with self.subTest(label=label):
                self.assertIn(label, html)

    def test_selected_item_gets_a_ring(self):
        scene = make_scene()
        layer = build_new_items_layer(scene, highlight_mission=False, selected=scene.safe_points[0])
        self.assertIn(
            "Selected item", render(folium.Map(location=(47.0, 8.0), zoom_start=12).add_child(layer))
        )

    def test_plan_layer_names_the_vehicle_and_can_skip_the_goal_pins(self):
        layer = build_new_items_layer(Scene(), highlight_mission=False)
        path = [(47.005, 8.001), (47.005, 8.0), (47.01, 8.0), (47.02, 8.02)]
        add_plan_layer(
            layer, parse_output(PLAN_OUTPUT), path, "vehicle_position_2", "purple", goal_pins=False
        )
        html = render(folium.Map(location=(47.0, 8.0), zoom_start=12).add_child(layer))
        self.assertIn("Planned path, RTL, vehicle_position_2", html)
        self.assertIn("Join point, vehicle_position_2", html)
        self.assertNotIn("Goal: safe_point", html)
        self.assertNotIn("Branch-off", html)

    def test_parsed_items_show_their_names_on_hover(self):
        html = render(build_base_map((47.0, 8.0), 15, make_scene()))
        for label in (
            "Parsed mission[0]",
            "Parsed mission: 0 to 1, 1112 m",
            "NAV_CMD_LAND",
            "Parsed safe_points[0]",
            "Parsed vehicle_position",
            "velocity N/E 5.0/0.0 m/s",
            "Parsed goal",
            "cursor",
        ):
            with self.subTest(label=label):
                self.assertIn(label, html)
        self.assertNotIn("bindPopup", html)
        self.assertIn('"bubblingMouseEvents": false', html)

    def test_new_items_are_a_separate_layer(self):
        base_map = build_base_map((47.0, 8.0), 15, make_scene())
        children_before = tuple(base_map._children)
        editor = Editor()
        editor.add(MISSION, Waypoint(48.0, 9.0, 500.0))
        editor.add(MISSION, Waypoint(48.01, 9.0, 500.0))
        editor.add(VEHICLE, Waypoint(48.0, 9.01, 500.0, ""))
        editor.velocity = (0.0, 10.0)

        layer = build_new_items_layer(editor.scene(), highlight_mission=True)

        self.assertEqual(tuple(base_map._children), children_before)
        self.assertIsNone(layer._parent)
        layer.add_to(folium.Map(location=(48.0, 9.0)))
        html = render(layer)
        self.assertIn("New mission[1]", html)
        self.assertIn("New vehicle_position", html)
        self.assertIn("velocity N/E 0.0/10.0 m/s", html)
        self.assertIn('"radius": 7', html)


class Map3dTest(unittest.TestCase):
    def test_fences_use_ground_outlines_in_3d(self):
        scene = Scene(fences=[Fence(True, [(47, 8), (47.01, 8), (47.01, 8.01)]),
                              Fence(False, [(47, 8)], 50)])
        deck = json.loads(build_deck(None, scene, (47, 8), 16).to_json())
        layer = deck["layers"][0]
        self.assertEqual(layer["id"], "geofences")
        self.assertEqual(layer["data"][0]["polygon"], [[8, 47], [8, 47.01], [8.01, 47.01]])
        self.assertEqual(len(layer["data"][1]["polygon"]), 64)

    def test_deck_has_paths_points_and_velocity_at_altitude(self):
        editor = Editor()
        editor.add(MISSION, Waypoint(47.05, 8.05, 600.0))
        deck = json.loads(build_deck(make_scene(), editor.scene(), (47.0, 8.0), 14).to_json())
        layers = {layer["id"]: layer for layer in deck["layers"]}
        self.assertEqual(set(layers), {"missions", "points", "velocity"})
        self.assertEqual(layers["missions"]["data"][0]["path"], [[8.0, 47.0, 500.0], [8.0, 47.01, 520.0]])
        labels = [point["label"] for point in layers["points"]["data"]]
        self.assertIn("Parsed vehicle_position, 500.0 m", labels)
        self.assertIn("New mission[0], 600.0 m", labels)
        self.assertEqual(len(layers["velocity"]["data"]), 3)
        self.assertEqual(layers["velocity"]["data"][0]["source"][2], 500.0)
        self.assertEqual(deck["initialViewState"]["zoom"], 14)
        self.assertEqual(deck["mapProvider"], "carto")

    def test_new_items_use_the_2d_colours_and_parsed_ones_gray(self):
        editor = Editor()
        editor.add(MISSION, Waypoint(47.05, 8.05, 600.0))
        editor.add(SAFE_POINTS, Waypoint(47.06, 8.05, 600.0, "NAV_CMD_RALLY_POINT"))
        editor.add(VEHICLE, Waypoint(47.07, 8.05, 600.0, ""))
        deck = json.loads(build_deck(make_scene(), editor.scene(), (47.0, 8.0), 14).to_json())
        points = {point["label"]: point["color"] for layer in deck["layers"] if layer["id"] == "points"
                  for point in layer["data"]}
        self.assertEqual(points["New mission[0], 600.0 m"], NEW_RGB[MISSION])
        self.assertEqual(points["New safe_points[0], 600.0 m"], NEW_RGB[SAFE_POINTS])
        self.assertEqual(points["New vehicle_position, 600.0 m"], NEW_RGB[VEHICLE])
        self.assertEqual(points["Parsed safe_points[0], 480.0 m"], PARSED_RGB)

    def test_empty_scenes_give_an_empty_deck(self):
        deck = json.loads(build_deck(None, Scene(), (47.0, 8.0), 16).to_json())
        self.assertEqual(deck["layers"], [])


if __name__ == "__main__":
    unittest.main()
