"""Folium layers for the 2D map. Parsed fixtures are drawn in gray, new items in colour."""

from typing import Dict, Optional, Sequence, Tuple

import folium
from folium.plugins import MousePosition

from .geometry import distance_m, velocity_arrow
from .models import ITEM_TYPES, MISSION, POSITION, SAFE_POINTS, VEHICLE, Scene, Waypoint

NEW_COLORS = {MISSION: "blue", SAFE_POINTS: "purple", VEHICLE: "green", POSITION: "orange"}
PARSED_COLORS = {item_type: "gray" for item_type in ITEM_TYPES}
PARSED_MISSION_COLORS = ("darkblue", "darkred", "cadetblue", "black")
PIN_ICONS = {SAFE_POINTS: "flag", VEHICLE: "plane", POSITION: "crosshairs"}
SELECTED_COLOR = "orange"


def build_base_map(center: Tuple[float, float], zoom: int, parsed: Optional[Scene]) -> folium.Map:
    """Tiles plus the parsed fixtures.

    New items are not drawn here on purpose: st_folium remounts the map, and resets
    the view, whenever this HTML changes, so it has to stay identical between clicks.
    """
    fmap = folium.Map(location=center, zoom_start=zoom, tiles="OpenStreetMap", control_scale=True)
    MousePosition(position="bottomleft", num_digits=7, prefix="cursor").add_to(fmap)
    if parsed is not None:
        _draw_scene(fmap, parsed, "Parsed", PARSED_MISSION_COLORS, PARSED_COLORS, False)
    return fmap


def build_new_items_layer(
    scene: Scene, highlight_mission: bool, selected: Optional[Waypoint] = None
) -> folium.FeatureGroup:
    """The editor's items, with a ring around the selected one. Handed to st_folium separately."""
    layer = folium.FeatureGroup(name="new items")
    _draw_scene(layer, scene, "New", (NEW_COLORS[MISSION],), NEW_COLORS, highlight_mission)
    if selected is not None:
        folium.CircleMarker(
            (selected.lat, selected.lon),
            radius=14,
            color=SELECTED_COLOR,
            weight=3,
            fill=False,
            tooltip="Selected item",
            bubbling_mouse_events=False,
        ).add_to(layer)
    return layer


def _draw_scene(
    layer,
    scene: Scene,
    prefix: str,
    mission_colors: Sequence[str],
    pin_colors: Dict[str, str],
    highlight_last: bool,
) -> None:
    for index, (name, items) in enumerate(scene.missions.items()):
        color = mission_colors[index % len(mission_colors)]
        _draw_mission(layer, name, items, color, prefix, highlight_last)
    for index, point in enumerate(scene.safe_points):
        label = f"{prefix} safe_points[{index}]<br>alt {point.alt:.1f} m"
        _pin(layer, point.lat, point.lon, pin_colors[SAFE_POINTS], PIN_ICONS[SAFE_POINTS], label)
    for position in scene.positions:
        item_type = VEHICLE if position.is_vehicle else POSITION
        label = f"{prefix} {position.name}<br>alt {position.alt:.1f} m"
        _pin(layer, position.lat, position.lon, pin_colors[item_type], PIN_ICONS[item_type], label)
        if position.is_vehicle and scene.velocity is not None:
            north, east = scene.velocity
            label += f"<br>velocity N/E {north:.1f}/{east:.1f} m/s"
            for start, end in velocity_arrow(position.lat, position.lon, north, east):
                _line(layer, start, end, pin_colors[item_type], label)


def _draw_mission(
    layer, name: str, items: Sequence[Waypoint], color: str, prefix: str, highlight_last: bool
) -> None:
    for index in range(len(items) - 1):
        start, end = items[index], items[index + 1]
        leg = distance_m(start.lat, start.lon, end.lat, end.lon)
        label = f"{prefix} {name}: {index} to {index + 1}, {leg:.0f} m"
        _line(layer, (start.lat, start.lon), (end.lat, end.lon), color, label)
    for index, item in enumerate(items):
        is_last = highlight_last and index == len(items) - 1
        label = f"{prefix} {name}[{index}]<br>{item.cmd}<br>alt {item.alt:.1f} m"
        # Bubbling off: a click on a marker must not also count as a map click.
        folium.CircleMarker(
            (item.lat, item.lon),
            radius=7 if is_last else 5,
            color=color,
            fill=True,
            fill_opacity=0.9,
            tooltip=label,
            bubbling_mouse_events=False,
        ).add_to(layer)


def _line(layer, start, end, color: str, label: str) -> None:
    folium.PolyLine(
        [start, end], color=color, weight=3, opacity=0.8, tooltip=label, bubbling_mouse_events=False
    ).add_to(layer)


def _pin(layer, lat: float, lon: float, color: str, icon: str, label: str) -> None:
    # Marker clicks do not reach the map, so they never add a point either.
    folium.Marker(
        (lat, lon), tooltip=label, icon=folium.Icon(color=color, icon=icon, prefix="fa")
    ).add_to(layer)


# ---- The planner's answer ----

PLAN_COLOR = "red"
GOAL_COLOR = "green"
# One per vehicle position when several are planned; all valid folium icon colours.
PLAN_COLORS = ("red", "purple", "orange", "cadetblue")


def add_plan_layer(
    layer,
    result,
    path: Sequence[Tuple[float, float]],
    name: str = "",
    color: str = PLAN_COLOR,
    goal_pins: bool = True,
) -> None:
    """Draw one vehicle's planned path and join point, plus the branch-off and goal when asked."""
    suffix = f", {name}" if name else ""
    if len(path) >= 2:
        folium.PolyLine(
            list(path),
            color=color,
            weight=4,
            opacity=0.9,
            dash_array="8 6",
            tooltip=f"Planned path, {result.action}{suffix}",
            bubbling_mouse_events=False,
        ).add_to(layer)
    join = result.position("join")
    if join is not None:
        _pin(layer, join[0], join[1], color, "sign-in", f"Join point{suffix}<br>alt {join[2]:.1f} m")
    if not goal_pins:
        return
    branch_off = result.position("branch_off")
    if branch_off is not None and result.plan.get("goal_type") == "safe_point":
        label = f"Branch-off{suffix}<br>alt {branch_off[2]:.1f} m"
        _pin(layer, branch_off[0], branch_off[1], "darkred", "sign-out", label)
    goal = result.position("goal")
    if goal is not None:
        label = f"Goal: {result.plan.get('goal_type', 'goal')}{suffix}<br>alt {goal[2]:.1f} m"
        _pin(layer, goal[0], goal[1], GOAL_COLOR, "flag-checkered", label)
