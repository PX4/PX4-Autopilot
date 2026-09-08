"""The items added on the map, with undo. Independent of Streamlit so it can be tested."""

from copy import deepcopy
from dataclasses import dataclass, field
import math
import re
from typing import Dict, List, Mapping, Optional, Tuple

from .geometry import distance_m, snap_to_grid
from .models import ITEM_TYPES, MISSION, POSITION, SAFE_POINTS, VEHICLE, Base, Position, Scene, Waypoint

_MAX_UNDO = 100
# Moves below this are float noise from converting offsets back to lat/lon, not edits.
_MIN_MOVE_M = 0.005
# Start of a new item's map tooltip, as map2d writes it: "New mission[3]", "New vehicle_position_2".
_TOOLTIP = re.compile(
    r"^New (?:(mission|safe_points)\[(\d+)\]|(vehicle_position)(?:_(\d+))?|(position)_(\d+))(?!\d)"
)

Points = Dict[str, List[Waypoint]]
# One item of the editor: its type and its index in that type.
Selection = Tuple[str, int]


def _empty_points() -> Points:
    return {item_type: [] for item_type in ITEM_TYPES}


@dataclass(frozen=True)
class ClickSettings:
    """Sidebar values applied to the next map click."""

    alt: float = 500.0
    cmd: str = "NAV_CMD_WAYPOINT"
    grid_m: float = 0.0


@dataclass
class Editor:
    points: Points = field(default_factory=_empty_points)
    # Request velocity (north, east) in m/s, shown on the vehicle positions.
    velocity: Tuple[float, float] = (0.0, 0.0)
    history: List[Points] = field(default_factory=list, repr=False)
    # st_folium reports the same click on every rerun. Remembering the one already
    # turned into a point keeps mode changes, undo and clear from adding it again.
    last_click: Optional[Tuple[float, float]] = None

    def take_click(
        self, event: Optional[Mapping], base: Optional[Base], grid_m: float
    ) -> Optional[Tuple[float, float]]:
        """A fresh map click snapped to the grid, or None for a replayed, missing or malformed one."""
        click = (event or {}).get("last_clicked")
        try:
            lat, lon = float(click["lat"]), float(click["lng"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(lat) and math.isfinite(lon)) or abs(lat) > 90 or abs(lon) > 180:
            return None
        if (lat, lon) == self.last_click:
            return None
        self.last_click = (lat, lon)
        if base is not None:
            lat, lon = snap_to_grid(base.lat, base.lon, lat, lon, grid_m)
        return lat, lon

    def add_click(
        self, event: Optional[Mapping], item_type: str, settings: ClickSettings, base: Optional[Base]
    ) -> bool:
        """Add a point for a fresh map click. Marker clicks arrive in another field and are ignored."""
        click = self.take_click(event, base, settings.grid_m)
        if click is None:
            return False
        cmd = {MISSION: settings.cmd, SAFE_POINTS: "NAV_CMD_RALLY_POINT"}.get(item_type, "")
        return self.add(item_type, Waypoint(click[0], click[1], settings.alt, cmd))

    def move_to_click(
        self, event: Optional[Mapping], selection: Selection, base: Optional[Base], grid_m: float
    ) -> bool:
        """Put the selected item where a fresh map click landed."""
        point = self.get(selection)
        if point is None:
            return False
        click = self.take_click(event, base, grid_m)
        if click is None:
            return False
        return self.update(selection, click[0], click[1], point.alt, point.cmd)

    def add(self, item_type: str, point: Waypoint) -> bool:
        if point in self.points[item_type]:
            return False
        self._checkpoint()
        self.points[item_type].append(point)
        return True

    def get(self, selection: Optional[Selection]) -> Optional[Waypoint]:
        if selection is None:
            return None
        item_type, index = selection
        points = self.points.get(item_type, [])
        return points[index] if 0 <= index < len(points) else None

    def update(self, selection: Selection, lat: float, lon: float, alt: float, cmd: str) -> bool:
        """Replace the selected item; a move shorter than float noise with the same fields is no edit."""
        point = self.get(selection)
        if point is None:
            return False
        moved = distance_m(point.lat, point.lon, lat, lon) > _MIN_MOVE_M
        if not moved and math.isclose(point.alt, alt) and point.cmd == cmd:
            return False
        lat, lon = (lat, lon) if moved else (point.lat, point.lon)
        self._checkpoint()
        self.points[selection[0]][selection[1]] = Waypoint(lat, lon, alt, cmd)
        return True

    def remove(self, selection: Selection) -> bool:
        if self.get(selection) is None:
            return False
        self._checkpoint()
        del self.points[selection[0]][selection[1]]
        return True

    def name(self, selection: Selection) -> str:
        """The item as the generated C++ names it: mission[3], safe_points[0], vehicle_position_2."""
        item_type, index = selection
        if item_type == MISSION:
            return f"mission[{index}]"
        if item_type == SAFE_POINTS:
            return f"safe_points[{index}]"
        if item_type == VEHICLE:
            return "vehicle_position" if index == 0 else f"vehicle_position_{index + 1}"
        return f"position_{index}"

    def locate(self, tooltip: Optional[str]) -> Optional[Selection]:
        """The item behind a clicked marker, from the tooltip map2d gave it. None for anything else."""
        match = _TOOLTIP.match(tooltip or "")
        if match is None:
            return None
        mission_or_safe, bracket_index, vehicle, vehicle_number, position, position_index = match.groups()
        if mission_or_safe:
            selection = (MISSION if mission_or_safe == "mission" else SAFE_POINTS, int(bracket_index))
        elif vehicle:
            selection = (VEHICLE, int(vehicle_number) - 1 if vehicle_number else 0)
        else:
            selection = (POSITION, int(position_index))
        return selection if self.get(selection) is not None else None

    def undo(self) -> bool:
        if not self.history:
            return False
        self.points = self.history.pop()
        return True

    def clear(self, item_type: Optional[str] = None) -> bool:
        item_types = (item_type,) if item_type else ITEM_TYPES
        if not any(self.points[key] for key in item_types):
            return False
        self._checkpoint()
        for key in item_types:
            self.points[key] = []
        return True

    def count(self, item_type: Optional[str] = None) -> int:
        if item_type:
            return len(self.points[item_type])
        return sum(len(points) for points in self.points.values())

    def load(self, scene: Scene) -> None:
        """Replace the items with a parsed scene, so an existing test can be extended."""
        self._checkpoint()
        self.points = _empty_points()
        self.points[MISSION] = [item for items in scene.missions.values() for item in items]
        self.points[SAFE_POINTS] = list(scene.safe_points)
        for position in scene.positions:
            item_type = VEHICLE if position.is_vehicle else POSITION
            self.points[item_type].append(Waypoint(position.lat, position.lon, position.alt, ""))
        if scene.velocity is not None:
            self.velocity = scene.velocity

    def scene(self) -> Scene:
        """The items as a Scene, named the way the tests name them."""
        scene = Scene()
        if self.points[MISSION]:
            scene.missions["mission"] = list(self.points[MISSION])
        scene.safe_points = list(self.points[SAFE_POINTS])
        for item_type in (VEHICLE, POSITION):
            for index, point in enumerate(self.points[item_type]):
                name = self.name((item_type, index))
                scene.positions.append(Position(name, point.lat, point.lon, point.alt))
        if self.points[VEHICLE] and any(self.velocity):
            scene.velocity = self.velocity
        return scene

    def _checkpoint(self) -> None:
        self.history.append(deepcopy(self.points))
        del self.history[:-_MAX_UNDO]
