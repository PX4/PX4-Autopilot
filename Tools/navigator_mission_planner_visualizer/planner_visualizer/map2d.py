"""Folium layers for the 2D map. Parsed fixtures are drawn in gray, new items in colour."""

from typing import Dict, Optional, Sequence, Tuple

from branca.element import MacroElement
import folium
from folium.plugins import MousePosition
from jinja2 import Template

from .geometry import distance_m, velocity_arrow
from .models import ITEM_TYPES, MISSION, POSITION, SAFE_POINTS, VEHICLE, Fence, Scene, Waypoint

NEW_COLORS = {MISSION: "blue", SAFE_POINTS: "purple", VEHICLE: "green", POSITION: "orange"}
PARSED_COLORS = {item_type: "gray" for item_type in ITEM_TYPES}
# The first parsed mission is gray like the other parsed fixtures; a blue one looked new.
PARSED_MISSION_COLORS = ("dimgray", "darkred", "cadetblue", "black")
PIN_ICONS = {SAFE_POINTS: "flag", VEHICLE: "plane", POSITION: "crosshairs"}
SELECTED_COLOR = "orange"
FENCE_COLORS = {True: "green", False: "red"}
# Parsed fences get a dashed outline, so a deleted new fence does not seem to stay.
PARSED_FENCE_DASH = "6 4"


def build_base_map(center: Tuple[float, float], zoom: int, parsed: Optional[Scene]) -> folium.Map:
    """Tiles plus the parsed fixtures.

    New items are not drawn here on purpose: st_folium remounts the map, and resets
    the view, whenever this HTML changes, so it has to stay identical between clicks.
    """
    fmap = folium.Map(location=center, zoom_start=zoom, tiles="OpenStreetMap", control_scale=True)
    # The plugin's default placeholder, "Unavailable", read like an error.
    MousePosition(
        position="bottomleft", num_digits=7, prefix="cursor", empty_string="cursor lat : lon"
    ).add_to(fmap)
    Legend().add_to(fmap)
    if parsed is not None:
        _draw_scene(
            fmap, parsed, "Parsed", PARSED_MISSION_COLORS, PARSED_COLORS, False, PARSED_FENCE_DASH
        )
    return fmap


def build_new_items_layer(
    scene: Scene, highlight_mission: bool, selected: Optional[Waypoint] = None,
    draft_fence: Optional[Fence] = None,
) -> folium.FeatureGroup:
    """The editor's items, with a ring around the selected one. Handed to st_folium separately."""
    layer = folium.FeatureGroup(name="new items")
    _draw_scene(layer, scene, "New", (NEW_COLORS[MISSION],), NEW_COLORS, highlight_mission)
    if draft_fence is not None:
        color = FENCE_COLORS[draft_fence.inclusion]
        if len(draft_fence.vertices) > 1:
            folium.PolyLine(
                draft_fence.vertices, color=color, dash_array="5 5", interactive=False
            ).add_to(layer)
        for point in draft_fence.vertices:
            folium.CircleMarker(point, radius=4, color=color, fill=True, interactive=False).add_to(layer)
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
    fence_dash: Optional[str] = None,
) -> None:
    # Draw areas first so they do not cover mission markers or consume map clicks.
    for index, fence in enumerate(scene.fences):
        options = dict(
            color=FENCE_COLORS[fence.inclusion], weight=2, fill=True, fill_opacity=0.1,
            dash_array=fence_dash,
            tooltip=f"{prefix} geofence[{index}]: {fence.label()}", bubbling_mouse_events=True,
        )
        if fence.radius_m is None:
            folium.Polygon(fence.vertices, **options).add_to(layer)
        else:
            folium.Circle(fence.vertices[0], radius=fence.radius_m, **options).add_to(layer)
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
BRANCH_OFF_COLOR = "darkred"
# One per vehicle position when several are planned; all valid folium icon colours.
# No new item pin uses them, so a join pin never looks like a safe point.
PLAN_COLORS = ("red", "black", "cadetblue", "darkblue")


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
        _pin(layer, branch_off[0], branch_off[1], BRANCH_OFF_COLOR, "sign-out", label)
    goal = result.position("goal")
    if goal is not None:
        label = f"Goal: {result.plan.get('goal_type', 'goal')}{suffix}<br>alt {goal[2]:.1f} m"
        _pin(layer, goal[0], goal[1], GOAL_COLOR, "flag-checkered", label)


# ---- The legend ----

# Fill colours of the Leaflet.awesome-markers pins, so legend icons match them.
_PIN_FILLS = {
    "purple": "#d252b9", "green": "#72b026", "orange": "#f69730",
    "gray": "#575757", "red": "#d63e2a", "darkred": "#a23336",
}


def _swatch_line(color: str, style: str = "solid") -> str:
    return f'<span style="display:inline-block;width:18px;border-top:3px {style} {color}"></span>'


def _swatch_area(color: str, fill: str) -> str:
    return (
        f'<span style="display:inline-block;width:12px;height:9px;border:2px solid {color};'
        f'background:{fill}"></span>'
    )


def _swatch_icon(icon: str, color: str) -> str:
    return f'<i class="fa fa-{icon}" style="color:{_PIN_FILLS[color]};width:15px;text-align:center"></i>'


def legend_html() -> str:
    """A collapsible key to the colours and pins, built from the constants above."""
    pins = "".join(
        _swatch_icon(PIN_ICONS[kind], NEW_COLORS[kind]) for kind in (SAFE_POINTS, VEHICLE, POSITION)
    )
    rows = (
        (_swatch_line(NEW_COLORS[MISSION]), "New mission"),
        (pins, "New safe point, vehicle, position"),
        (_swatch_line(PARSED_MISSION_COLORS[0]) + _swatch_icon("map-marker", "gray"),
         "Parsed test, read-only"),
        (_swatch_area(FENCE_COLORS[True], "rgba(0,128,0,.15)") + " "
         + _swatch_area(FENCE_COLORS[False], "rgba(255,0,0,.15)"), "Inclusion, exclusion geofence"),
        (_swatch_line("gray", "dashed"), "Parsed geofence outline"),
        (_swatch_line(PLAN_COLOR, "dashed"), "Planned path"),
        (_swatch_icon("sign-in", PLAN_COLOR) + _swatch_icon("sign-out", BRANCH_OFF_COLOR)
         + _swatch_icon("flag-checkered", GOAL_COLOR), "Join, branch-off, goal"),
    )
    body = "".join(
        f'<div style="display:flex;align-items:center;gap:6px;margin-top:3px">'
        f'<span style="flex:0 0 52px;display:flex;align-items:center;gap:2px">{symbols}</span>{text}</div>'
        for symbols, text in rows
    )
    return (
        '<details style="background:#fff;padding:4px 8px;border-radius:4px;font:12px/1.3 sans-serif;'
        'box-shadow:0 1px 5px rgba(0,0,0,.4);max-width:270px">'
        f'<summary style="cursor:pointer;font-weight:600">Legend</summary>{body}</details>'
    )


class Legend(MacroElement):
    """The legend as a Leaflet control. It never changes, so the base map HTML stays stable."""

    _template = Template(
        """
        {% macro script(this, kwargs) %}
        var {{ this.get_name() }} = L.control({position: "topright"});
        {{ this.get_name() }}.onAdd = function () {
            var div = L.DomUtil.create("div");
            div.innerHTML = {{ this.html|tojson }};
            // Clicks on the legend must not add items to the map.
            L.DomEvent.disableClickPropagation(div);
            L.DomEvent.disableScrollPropagation(div);
            return div;
        };
        {{ this.get_name() }}.addTo({{ this._parent.get_name() }});
        {% endmacro %}
        """
    )

    def __init__(self):
        super().__init__()
        self._name = "Legend"
        self.html = legend_html()
