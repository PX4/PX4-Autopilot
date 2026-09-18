"""Data shared by the parser, the editor, the generator and the map views."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Test helper header defining the reference coordinates (kBaseLat, kBaseLon, kAlt).
HELPER_HEADER = "src/modules/navigator/test/support/mission_route_test_helpers.h"

# Map view used until something is parsed, when the helper header cannot be found.
DEFAULT_CENTER = (47.397742, 8.545594)
DEFAULT_ZOOM = 16

# Item types the editor can create.
MISSION = "Mission"
SAFE_POINTS = "Safe points"
VEHICLE = "Vehicle position"
POSITION = "Position"
ITEM_TYPES = (MISSION, SAFE_POINTS, VEHICLE, POSITION)

# Mission item kinds offered in the sidebar and the nav command each one generates.
MISSION_COMMANDS = {
    "Waypoint": "NAV_CMD_WAYPOINT",
    "Takeoff": "NAV_CMD_TAKEOFF",
    "Land": "NAV_CMD_LAND",
    "Loiter to altitude": "NAV_CMD_LOITER_TO_ALT",
}


@dataclass(frozen=True)
class Waypoint:
    """A positional item: mission item, safe point or a bare editor point."""

    lat: float
    lon: float
    alt: float
    cmd: str = "NAV_CMD_WAYPOINT"


@dataclass(frozen=True)
class Position:
    """A named mission_route::Position, e.g. the vehicle position of a request."""

    name: str
    lat: float
    lon: float
    alt: float

    @property
    def is_vehicle(self) -> bool:
        return "vehicle" in self.name.lower()


@dataclass(frozen=True)
class Base:
    """Reference the offset helpers are relative to: kBaseLat, kBaseLon and kAlt."""

    lat: float
    lon: float
    alt: Optional[float] = None


def base_from(constants: Dict[str, float]) -> Optional[Base]:
    if "kBaseLat" not in constants or "kBaseLon" not in constants:
        return None
    return Base(constants["kBaseLat"], constants["kBaseLon"], constants.get("kAlt"))


@dataclass
class Scene:
    """One set of fixtures, either parsed from C++ or built in the editor."""

    missions: Dict[str, List[Waypoint]] = field(default_factory=dict)
    safe_points: List[Waypoint] = field(default_factory=list)
    positions: List[Position] = field(default_factory=list)
    # Request velocity (north, east) in m/s, drawn as an arrow on vehicle positions.
    velocity: Optional[Tuple[float, float]] = None
    # Numeric constants seen while parsing, header defaults included.
    constants: Dict[str, float] = field(default_factory=dict)

    def is_empty(self) -> bool:
        return not (any(self.missions.values()) or self.safe_points or self.positions)

    def all_points(self) -> List[Tuple[float, float]]:
        points = [(wp.lat, wp.lon) for items in self.missions.values() for wp in items]
        points += [(wp.lat, wp.lon) for wp in self.safe_points]
        points += [(pos.lat, pos.lon) for pos in self.positions]
        return points

    def base(self) -> Optional[Base]:
        return base_from(self.constants)

    def summary(self) -> str:
        counts = {
            "mission items": sum(len(items) for items in self.missions.values()),
            "safe points": len(self.safe_points),
            "positions": len(self.positions),
        }
        parts = [f"{count} {label}" for label, count in counts.items() if count]
        if self.velocity is not None:
            parts.append("a velocity")
        return ", ".join(parts) if parts else "nothing"
