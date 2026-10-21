"""Turn a Scene into the C++ a mission route test would contain."""

from typing import List, Optional

from .geometry import latlon_to_offset
from .models import Base, Fence, Position, Scene, Waypoint

_ITEM_HELPER = {"NAV_CMD_TAKEOFF": "makeTakeoffItem", "NAV_CMD_LAND": "makeLandItem"}
_PLAIN_COMMANDS = ("NAV_CMD_WAYPOINT", "NAV_CMD_TAKEOFF", "NAV_CMD_LAND")


def generate_cpp(scene: Scene, base: Optional[Base]) -> str:
    """C++ for every populated part of the scene.

    With a base the output uses the FromOffset helpers relative to kBaseLat/kBaseLon,
    which is how the planner tests are written. Without one it uses absolute lat/lon.
    Fence scenes always use absolute coordinates at the CLI's precision.
    """
    # Rounding an offset can move a point across a fence and even enable recovery.
    # Fence scenarios retain the CLI's absolute-coordinate and altitude precision.
    precise = bool(scene.fences)
    if precise:
        base = None
    sections = [_mission(name, items, base, precise) for name, items in scene.missions.items() if items]
    if scene.safe_points:
        sections.append(_safe_points(scene.safe_points, base, precise))
    if scene.fences:
        sections.append(_fences(scene.fences))
    if scene.positions:
        sections.append("\n".join(_position(position, base, precise) for position in scene.positions))
    if scene.velocity is not None:
        sections.append(_velocity(scene.velocity, precise))
    return "\n\n".join(sections)


def cpp_float(value: float) -> str:
    """Float literal in the style of the tests (200.f, 12.5f, -10.f) at the CLI's 1 mm precision."""
    text = f"{value:.3f}".rstrip("0")
    if text == "-0.":
        text = "0."
    return text + "f"


def _altitude(alt: float, base: Optional[Base]) -> str:
    """kAlt, kAlt + 20.f, kAlt - 10.f when a base altitude is known, else a literal."""
    if base is None or base.alt is None:
        return cpp_float(alt)
    delta = alt - base.alt
    step = cpp_float(abs(delta))
    if step == "0.f":
        return "kAlt"
    return f"kAlt {'+' if delta > 0 else '-'} {step}"


def _location_args(lat: float, lon: float, alt: float, base: Optional[Base], precise: bool = False) -> str:
    if precise:
        return f"{lat:.9f}, {lon:.9f}, {alt:.3f}f"
    if base is None:
        return f"{lat:.7f}, {lon:.7f}, {_altitude(alt, base)}"
    north, east = latlon_to_offset(base.lat, base.lon, lat, lon)
    return f"kBaseLat, kBaseLon, {cpp_float(north)}, {cpp_float(east)}, {_altitude(alt, base)}"


def _mission_item(item: Waypoint, base: Optional[Base], precise: bool = False) -> str:
    helper = _ITEM_HELPER.get(item.cmd, "makePositionItem") + ("FromOffset" if base else "")
    args = _location_args(item.lat, item.lon, item.alt, base, precise)
    if item.cmd not in _PLAIN_COMMANDS:
        args += f", {item.cmd}"
    return f"{helper}({args})"


def _mission(name: str, items: List[Waypoint], base: Optional[Base], precise: bool = False) -> str:
    lines = [f"std::vector<mission_item_s> {name}{{"]
    lines += [f"\t{_mission_item(item, base, precise)}, // {index}" for index, item in enumerate(items)]
    lines.append("};")
    return "\n".join(lines)


def _safe_points(points: List[Waypoint], base: Optional[Base], precise: bool = False) -> str:
    helper = "makeSafePointFromOffset" if base else "makeSafePointAbsolute"
    lines = ["std::vector<mission_item_s> safe_points{"]
    lines += [
        f"\t{helper}({_location_args(point.lat, point.lon, point.alt, base, precise)}), // {index}"
        for index, point in enumerate(points)
    ]
    lines.append("};")
    return "\n".join(lines)


def _position(position: Position, base: Optional[Base], precise: bool = False) -> str:
    helper = "makePositionFromOffset" if base else "makePositionAbsolute"
    args = _location_args(position.lat, position.lon, position.alt, base, precise)
    return f"const mission_route::Position {position.name} = {helper}({args});"


def _velocity(velocity, precise: bool = False) -> str:
    north, east = (f"{value:.3f}f" if precise else cpp_float(value) for value in velocity)
    return f"request.velocity_north_m_s = {north};\nrequest.velocity_east_m_s = {east};"


def _fences(fences: List[Fence]) -> str:
    """Self-contained fence records; absolute coordinates avoid fixture-specific map origins."""
    lines = [
        "const auto makeFencePoint = [](uint16_t command, double lat, double lon, uint16_t count, float radius) {",
        "\tmission_fence_point_s point{};",
        "\tpoint.nav_cmd = command;",
        "\tpoint.frame = NAV_FRAME_GLOBAL;",
        "\tpoint.lat = lat;",
        "\tpoint.lon = lon;",
        "",
        "\tif (radius > 0.f) {",
        "\t\tpoint.circle_radius = radius;",
        "",
        "\t} else {",
        "\t\tpoint.vertex_count = count;",
        "\t}",
        "",
        "\treturn point;",
        "};",
        "std::vector<mission_fence_point_s> fence_points{",
    ]
    for index, fence in enumerate(fences):
        mode = "INCLUSION" if fence.inclusion else "EXCLUSION"
        circle = fence.radius_m is not None
        command = f"NAV_CMD_FENCE_{'CIRCLE' if circle else 'POLYGON_VERTEX'}_{mode}"
        count = 0 if circle else len(fence.vertices)
        radius = f"{fence.radius_m:.3f}f" if circle else "0.f"
        lines.append(f"\t// Fence {index}: {mode.lower()} {'circle' if circle else 'polygon'}")
        for lat, lon in fence.vertices:
            lines.append(f"\tmakeFencePoint({command}, {lat:.9f}, {lon:.9f}, {count}, {radius}),")
    lines.append("};")
    return "\n".join(lines)
