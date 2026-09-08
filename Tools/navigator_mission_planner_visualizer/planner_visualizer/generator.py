"""Turn a Scene into the C++ a mission route test would contain."""

from typing import List, Optional

from .geometry import latlon_to_offset
from .models import Base, Position, Scene, Waypoint

_ITEM_HELPER = {"NAV_CMD_TAKEOFF": "makeTakeoffItem", "NAV_CMD_LAND": "makeLandItem"}
_PLAIN_COMMANDS = ("NAV_CMD_WAYPOINT", "NAV_CMD_TAKEOFF", "NAV_CMD_LAND")


def generate_cpp(scene: Scene, base: Optional[Base]) -> str:
    """C++ for every populated part of the scene.

    With a base the output uses the FromOffset helpers relative to kBaseLat/kBaseLon,
    which is how the planner tests are written. Without one it uses absolute lat/lon.
    """
    sections = [_mission(name, items, base) for name, items in scene.missions.items() if items]
    if scene.safe_points:
        sections.append(_safe_points(scene.safe_points, base))
    if scene.positions:
        sections.append("\n".join(_position(position, base) for position in scene.positions))
    if scene.velocity is not None:
        sections.append(_velocity(scene.velocity))
    return "\n\n".join(sections)


def cpp_float(value: float) -> str:
    """Float literal in the style of the tests: 200.f, 12.5f, -10.f."""
    if abs(value) < 0.05:
        value = 0.0
    text = f"{value:.1f}"
    if text.endswith(".0"):
        text = text[:-1]
    return text + "f"


def _altitude(alt: float, base: Optional[Base]) -> str:
    """kAlt, kAlt + 20.f, kAlt - 10.f when a base altitude is known, else a literal."""
    if base is None or base.alt is None:
        return cpp_float(alt)
    delta = alt - base.alt
    if abs(delta) < 0.05:
        return "kAlt"
    return f"kAlt {'+' if delta > 0 else '-'} {cpp_float(abs(delta))}"


def _location_args(lat: float, lon: float, alt: float, base: Optional[Base]) -> str:
    if base is None:
        return f"{lat:.7f}, {lon:.7f}, {_altitude(alt, base)}"
    north, east = latlon_to_offset(base.lat, base.lon, lat, lon)
    return f"kBaseLat, kBaseLon, {cpp_float(north)}, {cpp_float(east)}, {_altitude(alt, base)}"


def _mission_item(item: Waypoint, base: Optional[Base]) -> str:
    helper = _ITEM_HELPER.get(item.cmd, "makePositionItem") + ("FromOffset" if base else "")
    args = _location_args(item.lat, item.lon, item.alt, base)
    if item.cmd not in _PLAIN_COMMANDS:
        args += f", {item.cmd}"
    return f"{helper}({args})"


def _mission(name: str, items: List[Waypoint], base: Optional[Base]) -> str:
    lines = [f"std::vector<mission_item_s> {name}{{"]
    lines += [f"\t{_mission_item(item, base)}, // {index}" for index, item in enumerate(items)]
    lines.append("};")
    return "\n".join(lines)


def _safe_points(points: List[Waypoint], base: Optional[Base]) -> str:
    helper = "makeSafePointFromOffset" if base else "makeSafePointAbsolute"
    lines = ["std::vector<mission_item_s> safe_points{"]
    lines += [
        f"\t{helper}({_location_args(point.lat, point.lon, point.alt, base)}), // {index}"
        for index, point in enumerate(points)
    ]
    lines.append("};")
    return "\n".join(lines)


def _position(position: Position, base: Optional[Base]) -> str:
    helper = "makePositionFromOffset" if base else "makePositionAbsolute"
    args = _location_args(position.lat, position.lon, position.alt, base)
    return f"const mission_route::Position {position.name} = {helper}({args});"


def _velocity(velocity) -> str:
    north, east = velocity
    return f"request.velocity_north_m_s = {cpp_float(north)};\nrequest.velocity_east_m_s = {cpp_float(east)};"
