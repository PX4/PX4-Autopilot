"""Deck.gl view of the parsed and new items at their altitude."""

from typing import Dict, List, Optional, Tuple

import pydeck as pdk

from .geometry import velocity_arrow
from .models import ITEM_TYPES, MISSION, POSITION, SAFE_POINTS, VEHICLE, Scene

PARSED_RGB = [110, 110, 110]
# The colours of the 2D map's new items, so both views read the same.
NEW_RGB = {
    MISSION: [31, 119, 180], SAFE_POINTS: [210, 82, 185], VEHICLE: [114, 176, 38], POSITION: [246, 151, 48],
}


def build_deck(parsed: Optional[Scene], new: Scene, center: Tuple[float, float], zoom: int) -> pdk.Deck:
    paths: List[dict] = []
    points: List[dict] = []
    lines: List[dict] = []
    fences: List[dict] = []
    parsed_colors = {item_type: PARSED_RGB for item_type in ITEM_TYPES}
    for scene, colors, prefix in ((parsed, parsed_colors, "Parsed"), (new, NEW_RGB, "New")):
        if scene is not None:
            _collect(scene, colors, prefix, paths, points, lines)
            for index, fence in enumerate(scene.fences):
                fences.append({"polygon": [[lon, lat] for lat, lon in fence.outline()],
                               "color": [0, 128, 0, 45] if fence.inclusion else [220, 0, 0, 45],
                               "label": f"{prefix} geofence[{index}]: {fence.label()}"})

    layers = []
    if fences:
        layers.append(
            pdk.Layer("PolygonLayer", fences, id="geofences", get_polygon="polygon",
                      get_fill_color="color", get_line_color="color", stroked=True,
                      line_width_min_pixels=2, pickable=True)
        )
    if paths:
        layers.append(
            pdk.Layer("PathLayer", paths, id="missions", get_path="path", get_color="color",
                      width_min_pixels=3, pickable=True)
        )
    if points:
        layers.append(
            pdk.Layer("ScatterplotLayer", points, id="points", get_position="position",
                      get_fill_color="color", get_radius=6, radius_min_pixels=4, pickable=True)
        )
    if lines:
        layers.append(
            pdk.Layer("LineLayer", lines, id="velocity", get_source_position="source",
                      get_target_position="target", get_color="color", width_min_pixels=2)
        )
    view = pdk.ViewState(latitude=center[0], longitude=center[1], zoom=zoom, pitch=45)
    return pdk.Deck(
        layers=layers,
        initial_view_state=view,
        tooltip={"text": "{label}"},
        map_provider="carto",
        map_style="light",
    )


def _collect(scene: Scene, colors: Dict[str, List[int]], prefix: str, paths, points, lines) -> None:
    color = colors[MISSION]
    for name, items in scene.missions.items():
        if len(items) >= 2:
            paths.append({"path": [[wp.lon, wp.lat, wp.alt] for wp in items], "color": color,
                          "label": f"{prefix} {name}"})
        for index, wp in enumerate(items):
            points.append({"position": [wp.lon, wp.lat, wp.alt], "color": color,
                           "label": f"{prefix} {name}[{index}], {wp.alt:.1f} m"})
    for index, point in enumerate(scene.safe_points):
        points.append({"position": [point.lon, point.lat, point.alt], "color": colors[SAFE_POINTS],
                       "label": f"{prefix} safe_points[{index}], {point.alt:.1f} m"})
    for position in scene.positions:
        color = colors[VEHICLE if position.is_vehicle else POSITION]
        points.append({"position": [position.lon, position.lat, position.alt], "color": color,
                       "label": f"{prefix} {position.name}, {position.alt:.1f} m"})
        if position.is_vehicle and scene.velocity is not None:
            for start, end in velocity_arrow(position.lat, position.lon, *scene.velocity):
                lines.append({"source": [start[1], start[0], position.alt],
                              "target": [end[1], end[0], position.alt], "color": color})
