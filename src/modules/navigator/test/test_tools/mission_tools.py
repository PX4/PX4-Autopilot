"""
PX4 Mission & Geofence Utility (Streamlit)

Features
- Unified Viewer & Creator: Paste C++ unit-test data to visualize it,
  then click to add new items (fences, missions, etc.) on the same map.
- Code Generator: Generates C++ snippets only for the newly added items.
- 3D Viewer: Optional pydeck visualization (View Only).
"""

from __future__ import annotations

import ast
from collections import defaultdict
from dataclasses import dataclass
import inspect
import math
from pathlib import Path
import re
from typing import Dict, List, Optional, Sequence, Tuple, Union

import folium
import pydeck as pdk
import streamlit as st

try:
    from streamlit_folium import st_folium
except Exception as e:
    st.error(
        "Missing dependency: streamlit-folium.\n\n"
        "Install it with:\n\n"
        "    pip install streamlit-folium\n\n"
        f"Details: {e}"
    )
    st.stop()

# ==========================================
# 1. CONSTANTS & CONFIGURATION
# ==========================================

DEFAULT_CENTER: Tuple[float, float] = (45.9766, 7.6585)  # Matterhorn
DEFAULT_ZOOM: int = 12
EARTH_RADIUS_M: float = 6371000.0
VELOCITY_ARROW_SCALE_M_PER_S: float = 2.0
VELOCITY_ARROW_MIN_M: float = 10.0
VELOCITY_ARROW_MAX_M: float = 200.0
VELOCITY_ARROW_HEAD_MIN_M: float = 5.0
VELOCITY_ARROW_HEAD_MAX_M: float = 25.0
VELOCITY_ARROW_HEAD_ANGLE_DEG: float = 25.0
VELOCITY_ARROW_MIN_SPEED_M_S: float = 0.1

# The available modes for the creator
CREATOR_MODES: Tuple[str, ...] = (
    "Mission",
    "Fence: Polygon (Inclusion)",
    "Fence: Polygon (Exclusion)",
    "Fence: Circle (Inclusion)",
    "Fence: Circle (Exclusion)",
    "Vehicle Location",
    "Path Check",
    "Rally Points",
)

# Regex to robustly find floats in C++ (handles 10.f, -5, 1e-3, etc.)
FLOAT_RE: str = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?(?:[fF])?"
_CPP_IDENTIFIER_RE = re.compile(r"[^a-zA-Z0-9_]")


def _strip_float_suffix(token: str) -> str:
    """Strip a trailing float suffix (f/F) from a numeric token."""
    t = token.strip()
    if t.endswith(("f", "F")):
        return t[:-1]
    return t


def _safe_float(token: str, default: float = 0.0) -> float:
    """Best-effort float parsing that tolerates 'f' suffixes and surrounding whitespace."""
    try:
        return float(_strip_float_suffix(token))
    except Exception:
        return default


def sanitize_cpp_identifier(raw: str, fallback: str = "kItem") -> str:
    """Convert a user string into a valid C++ identifier."""
    name = (raw or "").strip()
    if not name:
        return fallback
    name = _CPP_IDENTIFIER_RE.sub("_", name)
    if name[0].isdigit():
        name = f"_{name}"
    return name


def _strip_cpp_comments(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", "", src or "", flags=re.DOTALL)
    return re.sub(r"//.*?$", "", src, flags=re.MULTILINE)


def _remove_cpp_float_suffixes(expr: str) -> str:
    return re.sub(
        r"((?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)[fF]\b",
        r"\1",
        expr,
    )


def _find_matching_token(src: str, start_idx: int, open_char: str, close_char: str) -> int:
    depth = 0
    idx = start_idx
    in_string: Optional[str] = None
    while idx < len(src):
        ch = src[idx]

        if in_string is not None:
            if ch == "\\":
                idx += 2
                continue
            if ch == in_string:
                in_string = None
            idx += 1
            continue

        if src.startswith("//", idx):
            next_nl = src.find("\n", idx)
            if next_nl == -1:
                return -1
            idx = next_nl + 1
            continue

        if src.startswith("/*", idx):
            end_comment = src.find("*/", idx + 2)
            if end_comment == -1:
                return -1
            idx = end_comment + 2
            continue

        if ch in ('"', "'"):
            in_string = ch
            idx += 1
            continue

        if ch == open_char:
            depth += 1
        elif ch == close_char:
            depth -= 1
            if depth == 0:
                return idx

        idx += 1

    return -1


def _split_top_level(src: str, delimiter: str = ",") -> List[str]:
    parts: List[str] = []
    current: List[str] = []
    stack: List[str] = []
    in_string: Optional[str] = None
    idx = 0

    while idx < len(src):
        ch = src[idx]

        if in_string is not None:
            current.append(ch)
            if ch == "\\" and idx + 1 < len(src):
                idx += 1
                current.append(src[idx])
            elif ch == in_string:
                in_string = None
            idx += 1
            continue

        if ch in ('"', "'"):
            in_string = ch
            current.append(ch)
            idx += 1
            continue

        if ch in "([{":
            stack.append(ch)
            current.append(ch)
            idx += 1
            continue

        if ch in ")]}":
            if stack:
                stack.pop()
            current.append(ch)
            idx += 1
            continue

        if ch == delimiter and not stack:
            item = "".join(current).strip()
            if item:
                parts.append(item)
            current = []
            idx += 1
            continue

        current.append(ch)
        idx += 1

    item = "".join(current).strip()
    if item:
        parts.append(item)
    return parts


def _resolve_numeric_expr(expr: str, numeric_vars: Dict[str, float]) -> Optional[float]:
    expr = (expr or "").strip()
    if not expr:
        return None

    expr = _remove_cpp_float_suffixes(expr)

    try:
        node = ast.parse(expr, mode="eval")
    except SyntaxError:
        return None

    def _eval(ast_node: ast.AST) -> Optional[float]:
        if isinstance(ast_node, ast.Expression):
            return _eval(ast_node.body)

        if isinstance(ast_node, ast.Constant) and isinstance(ast_node.value, (int, float)):
            return float(ast_node.value)

        if isinstance(ast_node, ast.Name):
            return numeric_vars.get(ast_node.id)

        if isinstance(ast_node, ast.UnaryOp) and isinstance(ast_node.op, (ast.UAdd, ast.USub)):
            value = _eval(ast_node.operand)
            if value is None:
                return None
            return value if isinstance(ast_node.op, ast.UAdd) else -value

        if isinstance(ast_node, ast.BinOp) and isinstance(ast_node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            left = _eval(ast_node.left)
            right = _eval(ast_node.right)
            if left is None or right is None:
                return None
            if isinstance(ast_node.op, ast.Add):
                return left + right
            if isinstance(ast_node.op, ast.Sub):
                return left - right
            if isinstance(ast_node.op, ast.Mult):
                return left * right
            if right == 0.0:
                return None
            return left / right

        return None

    return _eval(node)


def _parse_call_expression(call_expr: str) -> Optional[Tuple[str, List[str]]]:
    match = re.match(r"([A-Za-z_]\w*)\s*\(", call_expr.strip())
    if not match:
        return None

    open_idx = call_expr.find("(", match.end() - 1)
    close_idx = _find_matching_token(call_expr, open_idx, "(", ")")
    if open_idx == -1 or close_idx == -1:
        return None

    fn_name = match.group(1)
    args_str = call_expr[open_idx + 1:close_idx]
    return fn_name, _split_top_level(args_str)


def st_folium_compat(
    m: folium.Map,
    *,
    key: str,
    height: int,
    returned_objects: Optional[Sequence[str]] = None,
) -> Dict:
    """Wrapper to handle st_folium arguments across versions."""
    kwargs = {}
    sig = inspect.signature(st_folium).parameters
    if "key" in sig:
        kwargs["key"] = key
    if "height" in sig:
        kwargs["height"] = height
    if "returned_objects" in sig and returned_objects is not None:
        kwargs["returned_objects"] = list(returned_objects)
    if "use_container_width" in sig:
        kwargs["use_container_width"] = True

    out = st_folium(m, **kwargs)
    return out or {}


def _sync_map_view_state(ret: Optional[Dict]) -> None:
    """Update the map view state from a st_folium return payload."""
    if not isinstance(ret, dict):
        return

    ret_center = ret.get("center")
    ret_zoom = ret.get("zoom")
    if ret_center is not None:
        new_c = None
        if isinstance(ret_center, dict):
            lat = ret_center.get("lat")
            lng = ret_center.get("lng", ret_center.get("lon"))
            new_c = (lat, lng)
        elif isinstance(ret_center, (list, tuple)) and len(ret_center) >= 2:
            new_c = (ret_center[0], ret_center[1])
        if new_c is not None and None not in new_c:
            if new_c != tuple(st.session_state["creator_map_center"]):
                st.session_state["creator_map_center"] = new_c

    if ret_zoom is not None:
        if ret_zoom != st.session_state["creator_map_zoom"]:
            st.session_state["creator_map_zoom"] = ret_zoom


# ==========================================
# 2. DATA STRUCTURES
# ==========================================

@dataclass(frozen=True)
class Waypoint:
    lat: float
    lon: float
    alt: float
    cmd: str = "NAV_CMD_WAYPOINT"


@dataclass(frozen=True)
class PolygonFence:
    name: str
    vertices: List[Tuple[float, float, float]]
    type: str = "Inclusion"


@dataclass(frozen=True)
class CircleFence:
    center: Tuple[float, float, float]
    radius: float
    type: str  # Inclusion | Exclusion


@dataclass(frozen=True)
class VehicleLocation:
    label: str
    lat: float
    lon: float
    alt: float
    vel_n: float
    vel_e: float
    v_xy_valid: bool = True


@dataclass(frozen=True)
class LatLonAltPoint:
    label: str
    lat: float
    lon: float
    alt: float


@dataclass(frozen=True)
class PathCheck:
    name: str
    lat_from: float
    lon_from: float
    alt_from: float
    lat_to: float
    lon_to: float


@dataclass(frozen=True)
class RallyPoint:
    lat: float
    lon: float
    alt: float
    id: int


@dataclass(frozen=True)
class Projection:
    lat: float
    lon: float
    alt: float
    label: str


ParseResult = Tuple[
    Dict[str, List[Waypoint]],
    List[PolygonFence],
    List[CircleFence],
    List[VehicleLocation],
    List[LatLonAltPoint],
    List[PathCheck],
    List[RallyPoint],
    List[Projection],
]


# ==========================================
# 3. MATH HELPERS (For Polygon Insertion)
# ==========================================

# Lat, Lon, Alt, Radius
CreatorPoint = Tuple[float, float, float, float]
VehicleCreatorPoint = Tuple[float, float, float, float, float]
CreatorItem = Union[CreatorPoint, VehicleCreatorPoint]
CreatorPointsMap = Dict[str, List[CreatorItem]]


def _normalize_vehicle_creator_points(
    points: Sequence[CreatorItem],
) -> Tuple[List[VehicleCreatorPoint], int, int]:
    normalized: List[VehicleCreatorPoint] = []
    dropped = 0
    legacy = 0
    for item in points:
        if isinstance(item, (list, tuple)):
            if len(item) == 4:
                legacy += 1
                lat, lon, alt, _ = item
                normalized.append((float(lat), float(lon), float(alt), 0.0, 0.0))
                continue
            if len(item) == 5:
                lat, lon, alt, vel_n, vel_e = item
                normalized.append((float(lat), float(lon), float(alt), float(vel_n), float(vel_e)))
                continue
        dropped += 1
    return normalized, dropped, legacy


def _to_local_xy_m(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    earth_radius_m = 6371000.0
    x = math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat)) * earth_radius_m
    y = math.radians(lat - ref_lat) * earth_radius_m
    return x, y


def _distance_point_to_segment_m(
    p_lat: float,
    p_lon: float,
    a_lat: float,
    a_lon: float,
    b_lat: float,
    b_lon: float,
) -> float:
    # Use P as origin for local tangent plane
    ax, ay = _to_local_xy_m(a_lat, a_lon, p_lat, p_lon)
    bx, by = _to_local_xy_m(b_lat, b_lon, p_lat, p_lon)

    # Vector AB
    abx = bx - ax
    aby = by - ay

    denom = abx * abx + aby * aby
    if denom == 0.0:
        return math.hypot(ax, ay)

    # Project point (0,0) onto line passing through A(ax,ay) and B(bx,by)
    # t is the parameter from A to B (0.0 to 1.0)
    # The vector from A to P(origin) is (-ax, -ay)
    # Dot product AP . AB
    t = -(ax * abx + ay * aby) / denom
    t = max(0.0, min(1.0, t))

    # Nearest point Q on segment
    qx = ax + t * abx
    qy = ay + t * aby

    return math.hypot(qx, qy)


def _add_vector_to_global_position(lat: float, lon: float, v_n: float, v_e: float) -> Tuple[float, float]:
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat_res = math.degrees(lat_rad + v_n / EARTH_RADIUS_M)
    lon_res = math.degrees(lon_rad + v_e / (EARTH_RADIUS_M * math.cos(lat_rad)))
    return lat_res, lon_res


def _rotate_ne_vector(n: float, e: float, angle_rad: float) -> Tuple[float, float]:
    """Rotate a vector in the local north/east plane by angle_rad."""
    e_r = e * math.cos(angle_rad) - n * math.sin(angle_rad)
    n_r = e * math.sin(angle_rad) + n * math.cos(angle_rad)
    return n_r, e_r


def _compute_velocity_arrow(
    lat: float,
    lon: float,
    vel_n: float,
    vel_e: float,
    v_xy_valid: bool = True,
) -> Optional[Dict[str, Tuple[float, float]]]:
    if not v_xy_valid:
        return None
    if not (math.isfinite(vel_n) and math.isfinite(vel_e)):
        return None
    speed = math.hypot(vel_n, vel_e)
    if speed <= VELOCITY_ARROW_MIN_SPEED_M_S:
        return None

    length_m = min(max(speed * VELOCITY_ARROW_SCALE_M_PER_S, VELOCITY_ARROW_MIN_M), VELOCITY_ARROW_MAX_M)
    dir_n = vel_n / speed
    dir_e = vel_e / speed
    tip_lat, tip_lon = _add_vector_to_global_position(lat, lon, dir_n * length_m, dir_e * length_m)

    head_len = min(max(length_m * 0.25, VELOCITY_ARROW_HEAD_MIN_M), VELOCITY_ARROW_HEAD_MAX_M)
    angle = math.radians(VELOCITY_ARROW_HEAD_ANGLE_DEG)
    back_n = -dir_n
    back_e = -dir_e
    left_n, left_e = _rotate_ne_vector(back_n, back_e, angle)
    right_n, right_e = _rotate_ne_vector(back_n, back_e, -angle)
    left_lat, left_lon = _add_vector_to_global_position(tip_lat, tip_lon, left_n * head_len, left_e * head_len)
    right_lat, right_lon = _add_vector_to_global_position(tip_lat, tip_lon, right_n * head_len, right_e * head_len)

    return {
        "tail": (lat, lon),
        "tip": (tip_lat, tip_lon),
        "left": (left_lat, left_lon),
        "right": (right_lat, right_lon),
    }


def _insert_point_on_polygon(points: Sequence[CreatorPoint], new_pt: CreatorPoint) -> List[CreatorPoint]:
    """
    Inserts a new point into a list of polygon vertices at the position
    that minimizes the distance to the edge it falls upon.
    """
    if len(points) < 3:
        return list(points) + [new_pt]

    best_idx = len(points)
    best_dist = None

    # 1. Check edges between points [0->1, 1->2, ... last->last]
    for idx in range(len(points) - 1):
        # Extract Lat/Lon from the 4-tuple (Lat, Lon, Alt, Rad)
        a_lat, a_lon = points[idx][0], points[idx][1]
        b_lat, b_lon = points[idx + 1][0], points[idx + 1][1]

        dist = _distance_point_to_segment_m(new_pt[0], new_pt[1], a_lat, a_lon, b_lat, b_lon)

        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_idx = idx + 1

    # 2. Check closing edge [last->0]
    a_lat, a_lon = points[-1][0], points[-1][1]
    b_lat, b_lon = points[0][0], points[0][1]

    closing_dist = _distance_point_to_segment_m(new_pt[0], new_pt[1], a_lat, a_lon, b_lat, b_lon)

    if best_dist is None or closing_dist < best_dist:
        best_idx = len(points)

    updated = list(points)
    updated.insert(best_idx, new_pt)
    return updated


# ==========================================
# 4. PARSING LOGIC (C++ -> Python)
# ==========================================

_COORD_ITEM_RE = re.compile(
    r"(?:Mission::LatLonAlt\s*)?\{([^{}]+)\}",
    re.MULTILINE,
)
_MISSION_HELPER_NAMES = (
    "makePositionItem",
    "makePositionItemFromOffset",
    "makeTakeoffItem",
    "makeTakeoffItemFromOffset",
    "makeLandItem",
    "makeLandItemFromOffset",
    "makeVtolTransitionItem",
    "makeDoJump",
)
_EMPTY_MISSION_HELPERS = {"makeVtolTransitionItem", "makeDoJump"}
_DEFAULT_NUMERIC_VARS: Optional[Dict[str, float]] = None


def _resolve_bool_token(token: Optional[str]) -> bool:
    if token is None:
        return True
    cleaned = token.strip().lower()
    if cleaned in ("true", "false"):
        return cleaned == "true"
    return True


def _resolve_point_field_reference(
    token: str,
    point_vars: Dict[str, Tuple[float, float, float]],
) -> Optional[float]:
    cleaned = token.strip()
    if "." not in cleaned:
        return None

    parts = cleaned.split(".")
    base_name = parts[0].split("::")[-1]
    field = parts[-1]
    point = point_vars.get(base_name)
    if point is None:
        return None

    lat, lon, alt = point
    if "lat" in field:
        return lat
    if "lon" in field:
        return lon
    if "alt" in field:
        return alt
    return None


def _resolve_numeric_value(
    token: str,
    numeric_vars: Dict[str, float],
    point_vars: Optional[Dict[str, Tuple[float, float, float]]] = None,
) -> Optional[float]:
    point_value = _resolve_point_field_reference(token, point_vars or {})
    if point_value is not None:
        return point_value
    return _resolve_numeric_expr(token, numeric_vars)


def _parse_latlon_token(
    token: str,
    point_vars: Dict[str, Tuple[float, float, float]],
    numeric_vars: Dict[str, float],
) -> Optional[Tuple[float, float, float]]:
    cleaned = token.strip()
    if not cleaned:
        return None

    if cleaned.startswith("Mission::LatLonAlt"):
        cleaned = cleaned[len("Mission::LatLonAlt"):].strip()

    if cleaned.startswith("{") and cleaned.endswith("}"):
        parts = _split_top_level(cleaned[1:-1])
        if len(parts) < 2:
            return None
        lat = _resolve_numeric_value(parts[0], numeric_vars, point_vars)
        lon = _resolve_numeric_value(parts[1], numeric_vars, point_vars)
        alt = _resolve_numeric_value(parts[2], numeric_vars, point_vars) if len(parts) >= 3 else 0.0
        if lat is None or lon is None or alt is None:
            return None
        return lat, lon, alt

    return point_vars.get(cleaned.split("::")[-1])


def _iter_function_calls(src: str, function_names: Sequence[str]) -> List[Tuple[str, List[str], int]]:
    pattern = re.compile(rf"\b({'|'.join(re.escape(name) for name in function_names)})\s*\(")
    calls: List[Tuple[str, List[str], int]] = []
    for match in pattern.finditer(src):
        open_idx = src.find("(", match.start())
        close_idx = _find_matching_token(src, open_idx, "(", ")")
        if open_idx == -1 or close_idx == -1:
            continue
        args = _split_top_level(src[open_idx + 1:close_idx])
        calls.append((match.group(1), args, match.start()))
    return calls


def _load_default_numeric_vars() -> Dict[str, float]:
    global _DEFAULT_NUMERIC_VARS
    if _DEFAULT_NUMERIC_VARS is not None:
        return dict(_DEFAULT_NUMERIC_VARS)

    helper_path = Path(__file__).resolve().parent.parent / "test_RTL_helpers.h"
    numeric_vars: Dict[str, float] = {}
    try:
        helper_src = _strip_cpp_comments(helper_path.read_text())
    except OSError:
        _DEFAULT_NUMERIC_VARS = {}
        return {}

    numeric_defs = [
        (match.group(1), match.group(2).strip())
        for match in re.finditer(
            r"(?:^|\s)(?:static\s+)?(?:inline\s+)?(?:constexpr\s+)?(?:const\s+)?"
            r"(?:float|double|int|int8_t|int16_t|int32_t|uint8_t|uint16_t|uint32_t|size_t)\s+"
            r"(\w+)\s*=\s*([^;]+)\s*;",
            helper_src,
            re.MULTILINE,
        )
    ]

    changed = True
    while changed:
        changed = False
        for name, expr in numeric_defs:
            if name in numeric_vars:
                continue
            value = _resolve_numeric_expr(expr, numeric_vars)
            if value is not None:
                numeric_vars[name] = value
                changed = True

    _DEFAULT_NUMERIC_VARS = dict(numeric_vars)
    return dict(numeric_vars)


def _extract_variable_definitions(
    src: str,
) -> Tuple[Dict[str, Tuple[float, float, float]], Dict[str, float], List[LatLonAltPoint]]:
    """Extract numeric constants and Mission::LatLonAlt variables from C++ source."""
    numeric_vars: Dict[str, float] = _load_default_numeric_vars()
    numeric_defs = [
        (match.group(1), match.group(2).strip())
        for match in re.finditer(
            r"(?:^|\s)(?:static\s+)?(?:inline\s+)?(?:constexpr\s+)?(?:const\s+)?"
            r"(?:float|double|int|int8_t|int16_t|int32_t|uint8_t|uint16_t|uint32_t|size_t)\s+"
            r"(\w+)\s*=\s*([^;]+)\s*;",
            src,
            re.MULTILINE,
        )
    ]

    changed = True
    while changed:
        changed = False
        for name, expr in numeric_defs:
            if name in numeric_vars:
                continue
            value = _resolve_numeric_expr(expr, numeric_vars)
            if value is not None:
                numeric_vars[name] = value
                changed = True

    point_vars: Dict[str, Tuple[float, float, float]] = {}
    latlon_points: List[LatLonAltPoint] = []
    latlon_var_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?Mission::LatLonAlt\s+(\w+)\s*(?:=\s*)?\{([^{}]+)\}\s*;",
        re.MULTILINE,
    )
    for match in latlon_var_pattern.finditer(src):
        parts = _split_top_level(match.group(2))
        if len(parts) < 2:
            continue
        lat = _resolve_numeric_expr(parts[0], numeric_vars)
        lon = _resolve_numeric_expr(parts[1], numeric_vars)
        alt = _resolve_numeric_expr(parts[2], numeric_vars) if len(parts) >= 3 else 0.0
        if lat is None or lon is None or alt is None:
            continue
        point_vars[match.group(1)] = (lat, lon, alt)

    for name, (lat, lon, alt) in point_vars.items():
        latlon_points.append(LatLonAltPoint(name, lat, lon, alt))

    return point_vars, numeric_vars, latlon_points


def _make_waypoint(
    lat_expr: str,
    lon_expr: str,
    alt_expr: str,
    cmd: str,
    numeric_vars: Dict[str, float],
) -> Optional[Waypoint]:
    lat = _resolve_numeric_expr(lat_expr, numeric_vars)
    lon = _resolve_numeric_expr(lon_expr, numeric_vars)
    alt = _resolve_numeric_expr(alt_expr, numeric_vars)
    if lat is None or lon is None or alt is None:
        return None
    return Waypoint(lat=lat, lon=lon, alt=alt, cmd=cmd)


def _make_waypoint_from_offset(
    base_lat_expr: str,
    base_lon_expr: str,
    north_expr: str,
    east_expr: str,
    alt_expr: str,
    cmd: str,
    numeric_vars: Dict[str, float],
) -> Optional[Waypoint]:
    base_lat = _resolve_numeric_expr(base_lat_expr, numeric_vars)
    base_lon = _resolve_numeric_expr(base_lon_expr, numeric_vars)
    north = _resolve_numeric_expr(north_expr, numeric_vars)
    east = _resolve_numeric_expr(east_expr, numeric_vars)
    alt = _resolve_numeric_expr(alt_expr, numeric_vars)
    if None in (base_lat, base_lon, north, east, alt):
        return None
    lat, lon = _add_vector_to_global_position(base_lat, base_lon, north, east)
    return Waypoint(lat=lat, lon=lon, alt=alt, cmd=cmd)


def _parse_mission_call(
    call_expr: str,
    numeric_vars: Dict[str, float],
) -> Optional[Waypoint]:
    parsed = _parse_call_expression(call_expr)
    if parsed is None:
        return None

    fn_name, args = parsed
    if fn_name in _EMPTY_MISSION_HELPERS:
        return None

    if fn_name == "makePositionItem" and len(args) >= 3:
        cmd = args[3].strip() if len(args) >= 4 else "NAV_CMD_WAYPOINT"
        return _make_waypoint(args[0], args[1], args[2], cmd, numeric_vars)

    if fn_name == "makePositionItemFromOffset" and len(args) >= 5:
        cmd = args[5].strip() if len(args) >= 6 else "NAV_CMD_WAYPOINT"
        return _make_waypoint_from_offset(args[0], args[1], args[2], args[3], args[4], cmd, numeric_vars)

    if fn_name == "makeTakeoffItem" and len(args) >= 3:
        return _make_waypoint(args[0], args[1], args[2], "NAV_CMD_TAKEOFF", numeric_vars)

    if fn_name == "makeTakeoffItemFromOffset" and len(args) >= 5:
        return _make_waypoint_from_offset(args[0], args[1], args[2], args[3], args[4], "NAV_CMD_TAKEOFF", numeric_vars)

    if fn_name == "makeLandItem" and len(args) >= 3:
        return _make_waypoint(args[0], args[1], args[2], "NAV_CMD_LAND", numeric_vars)

    if fn_name == "makeLandItemFromOffset" and len(args) >= 5:
        return _make_waypoint_from_offset(args[0], args[1], args[2], args[3], args[4], "NAV_CMD_LAND", numeric_vars)

    return None


def _parse_mission_item_list(items_src: str, numeric_vars: Dict[str, float]) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    for item in _split_top_level(items_src):
        waypoint = _parse_mission_call(item, numeric_vars)
        if waypoint is not None:
            waypoints.append(waypoint)
    return waypoints


def _unique_mission_name(base_name: str, existing: Dict[str, List[Waypoint]]) -> str:
    if base_name not in existing:
        return base_name

    suffix = 2
    while f"{base_name}_{suffix}" in existing:
        suffix += 1
    return f"{base_name}_{suffix}"


def _parse_named_mission_initializers(
    src: str,
    numeric_vars: Dict[str, float],
) -> Tuple[Dict[str, List[Waypoint]], List[Tuple[str, str, int]]]:
    missions: Dict[str, List[Waypoint]] = {}
    declared_names: List[Tuple[str, str, int]] = []

    vector_init_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::vector<\s*mission_item_s\s*>\s+(\w+)\s*(?:=\s*)?\{",
        re.MULTILINE,
    )
    for match in vector_init_pattern.finditer(src):
        raw_name = match.group(1)
        name = _unique_mission_name(raw_name, missions)
        declared_names.append((raw_name, name, match.start()))
        open_idx = src.find("{", match.start())
        close_idx = _find_matching_token(src, open_idx, "{", "}")
        if open_idx == -1 or close_idx == -1:
            continue
        missions[name] = _parse_mission_item_list(src[open_idx + 1:close_idx], numeric_vars)

    auto_vector_init_pattern = re.compile(
        r"(?:const\s+)?auto\s+(\w+)\s*=\s*std::vector<\s*mission_item_s\s*>\s*\{",
        re.MULTILINE,
    )
    for match in auto_vector_init_pattern.finditer(src):
        raw_name = match.group(1)
        name = _unique_mission_name(raw_name, missions)
        declared_names.append((raw_name, name, match.start()))
        open_idx = src.find("{", match.start())
        close_idx = _find_matching_token(src, open_idx, "{", "}")
        if open_idx == -1 or close_idx == -1:
            continue
        missions[name] = _parse_mission_item_list(src[open_idx + 1:close_idx], numeric_vars)

    array_init_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::array<\s*mission_item_s\s*,\s*\d+\s*>\s+(\w+)\s*(?:=\s*)?\{",
        re.MULTILINE,
    )
    for match in array_init_pattern.finditer(src):
        raw_name = match.group(1)
        name = _unique_mission_name(raw_name, missions)
        declared_names.append((raw_name, name, match.start()))
        open_idx = src.find("{", match.start())
        close_idx = _find_matching_token(src, open_idx, "{", "}")
        if open_idx == -1 or close_idx == -1:
            continue
        content = src[open_idx + 1:close_idx].strip()
        if content.startswith("{") and content.endswith("}"):
            content = content[1:-1]
        missions[name] = _parse_mission_item_list(content, numeric_vars)

    empty_decl_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::vector<\s*mission_item_s\s*>\s+(\w+)\s*;",
        re.MULTILINE,
    )
    for match in empty_decl_pattern.finditer(src):
        raw_name = match.group(1)
        name = _unique_mission_name(raw_name, missions)
        declared_names.append((raw_name, name, match.start()))
        missions.setdefault(name, [])

    function_pattern = re.compile(
        r"(?:static\s+)?(?:inline\s+)?(?:constexpr\s+)?(?:const\s+)?std::vector<\s*mission_item_s\s*>\s+(\w+)\s*\([^)]*\)\s*\{",
        re.MULTILINE,
    )
    for match in function_pattern.finditer(src):
        raw_name = match.group(1)
        name = _unique_mission_name(raw_name, missions)
        declared_names.append((raw_name, name, match.start()))
        body_open_idx = src.find("{", match.start())
        body_close_idx = _find_matching_token(src, body_open_idx, "{", "}")
        if body_open_idx == -1 or body_close_idx == -1:
            continue
        body = src[body_open_idx + 1:body_close_idx]
        return_match = re.search(r"return\s*\{", body)
        if not return_match:
            continue
        list_open_idx = body_open_idx + 1 + body.find("{", return_match.start())
        list_close_idx = _find_matching_token(src, list_open_idx, "{", "}")
        if list_close_idx == -1:
            continue
        missions[name] = _parse_mission_item_list(src[list_open_idx + 1:list_close_idx], numeric_vars)

    return missions, declared_names


def _parse_mission_push_backs(
    src: str,
    numeric_vars: Dict[str, float],
    declared_names: List[Tuple[str, str, int]],
    missions: Dict[str, List[Waypoint]],
) -> None:
    push_pattern = re.compile(r"(\w+)\.push_back\s*\(")
    for match in push_pattern.finditer(src):
        raw_name = match.group(1)
        candidates = [
            (unique_name, decl_pos) for candidate_raw_name, unique_name, decl_pos in declared_names
            if candidate_raw_name == raw_name and decl_pos <= match.start()
        ]
        if not candidates:
            continue
        name = max(candidates, key=lambda item: item[1])[0]
        open_idx = src.find("(", match.start())
        close_idx = _find_matching_token(src, open_idx, "(", ")")
        if open_idx == -1 or close_idx == -1:
            continue
        item_expr = src[open_idx + 1:close_idx].strip()
        waypoint = _parse_mission_call(item_expr, numeric_vars)
        if waypoint is not None:
            missions.setdefault(name, []).append(waypoint)


def _parse_legacy_mission_arrays(src: str) -> Dict[str, List[Waypoint]]:
    missions: Dict[str, List[Waypoint]] = {}
    mission_item_pattern = re.compile(
        rf"\{{\s*(NAV_CMD_\w+)\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})",
        re.MULTILINE,
    )

    mission_array_pattern = re.compile(
        rf"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::array<\s*MissionItemInput\s*,\s*\d+\s*>\s+(\w+)\s*\{{\s*\{{(.*?)\}}\s*\}}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in mission_array_pattern.finditer(src):
        missions[match.group(1)] = [
            Waypoint(cmd=item.group(1), lat=_safe_float(item.group(2)),
                     lon=_safe_float(item.group(3)), alt=_safe_float(item.group(4)))
            for item in mission_item_pattern.finditer(match.group(2))
        ]

    legacy_array_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?LegacyMissionItem\s+(\w+)\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in legacy_array_pattern.finditer(src):
        missions[match.group(1)] = [
            Waypoint(cmd=item.group(1), lat=_safe_float(item.group(2)),
                     lon=_safe_float(item.group(3)), alt=_safe_float(item.group(4)))
            for item in mission_item_pattern.finditer(match.group(2))
        ]

    return {name: waypoints for name, waypoints in missions.items() if waypoints}


def _parse_missions(src: str, numeric_vars: Dict[str, float]) -> Dict[str, List[Waypoint]]:
    missions = _parse_legacy_mission_arrays(src)
    named_missions, declared_names = _parse_named_mission_initializers(src, numeric_vars)
    missions.update(named_missions)
    _parse_mission_push_backs(src, numeric_vars, declared_names, missions)

    if missions:
        return {name: waypoints for name, waypoints in missions.items() if waypoints}

    inline_waypoints = []
    for fn_name, args, _ in _iter_function_calls(src, _MISSION_HELPER_NAMES):
        call_expr = f"{fn_name}({', '.join(args)})"
        waypoint = _parse_mission_call(call_expr, numeric_vars)
        if waypoint is not None:
            inline_waypoints.append(waypoint)

    if inline_waypoints:
        return {"mission": inline_waypoints}

    return {}


def _parse_coord_arrays(
    src: str,
    numeric_vars: Dict[str, float],
) -> Dict[str, List[Tuple[float, float, float]]]:
    """Parse std::array<Mission::LatLonAlt, N> coordinate arrays."""
    coord_arrays: Dict[str, List[Tuple[float, float, float]]] = {}
    coord_array_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::array<\s*Mission::LatLonAlt\s*,\s*\d+\s*>\s+(\w+)\s*\{\s*\{(.*?)\}\s*\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in coord_array_pattern.finditer(src):
        items: List[Tuple[float, float, float]] = []
        for coord_match in _COORD_ITEM_RE.finditer(match.group(2)):
            parts = _split_top_level(coord_match.group(1))
            if len(parts) < 2:
                continue
            lat = _resolve_numeric_expr(parts[0], numeric_vars)
            lon = _resolve_numeric_expr(parts[1], numeric_vars)
            alt = _resolve_numeric_expr(parts[2], numeric_vars) if len(parts) >= 3 else 0.0
            if lat is None or lon is None or alt is None:
                continue
            items.append((lat, lon, alt))
        if items:
            coord_arrays[match.group(1)] = items
    return coord_arrays


def _parse_rally_points(
    src: str,
    coord_arrays: Dict[str, List[Tuple[float, float, float]]],
    numeric_vars: Dict[str, float],
) -> List[RallyPoint]:
    """Parse rally points from coordinate arrays, legacy arrays, and helper calls."""
    rally_points: List[RallyPoint] = []

    for name, items in coord_arrays.items():
        if "rally" in name.lower():
            for i, (lat, lon, alt) in enumerate(items):
                rally_points.append(RallyPoint(lat, lon, alt, i))

    legacy_rally_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?LegacyRallyPoint\s+(\w+)\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in legacy_rally_pattern.finditer(src):
        idx_base = len(rally_points)
        for i, item in enumerate(_COORD_ITEM_RE.finditer(match.group(2))):
            parts = _split_top_level(item.group(1))
            if len(parts) < 2:
                continue
            lat = _resolve_numeric_expr(parts[0], numeric_vars)
            lon = _resolve_numeric_expr(parts[1], numeric_vars)
            alt = _resolve_numeric_expr(parts[2], numeric_vars) if len(parts) >= 3 else 0.0
            if lat is None or lon is None or alt is None:
                continue
            rally_points.append(RallyPoint(lat, lon, alt, idx_base + i))

    for fn_name, args, _ in _iter_function_calls(src, ("makeSafePointAbsolute", "makeSafePointFromOffset")):
        if fn_name == "makeSafePointAbsolute" and len(args) >= 3:
            lat = _resolve_numeric_expr(args[0], numeric_vars)
            lon = _resolve_numeric_expr(args[1], numeric_vars)
            alt = _resolve_numeric_expr(args[2], numeric_vars)
            if lat is None or lon is None or alt is None:
                continue
            rally_points.append(RallyPoint(lat, lon, alt, len(rally_points)))
            continue

        if fn_name == "makeSafePointFromOffset" and len(args) >= 5:
            base_lat = _resolve_numeric_expr(args[0], numeric_vars)
            base_lon = _resolve_numeric_expr(args[1], numeric_vars)
            north = _resolve_numeric_expr(args[2], numeric_vars)
            east = _resolve_numeric_expr(args[3], numeric_vars)
            alt = _resolve_numeric_expr(args[4], numeric_vars)
            if None in (base_lat, base_lon, north, east, alt):
                continue
            lat, lon = _add_vector_to_global_position(base_lat, base_lon, north, east)
            rally_points.append(RallyPoint(lat, lon, alt, len(rally_points)))

    return rally_points


def _parse_polygons(
    src: str,
    coord_arrays: Dict[str, List[Tuple[float, float, float]]],
) -> List[PolygonFence]:
    """Parse writePolygonFence() calls that reference coordinate arrays."""
    polygons: List[PolygonFence] = []
    for match in re.finditer(r"writePolygonFence\(\s*(\w+)\s*,\s*(NAV_CMD_\w+)\s*\)", src):
        var_name, cmd_type = match.group(1), match.group(2)
        if var_name in coord_arrays:
            polygons.append(
                PolygonFence(var_name, coord_arrays[var_name], "Exclusion" if "EXCLUSION" in cmd_type else "Inclusion")
            )
    return polygons


def _parse_circles(
    src: str,
    point_vars: Dict[str, Tuple[float, float, float]],
    numeric_vars: Dict[str, float],
) -> List[CircleFence]:
    """Parse writeCircleFence() calls."""
    circles: List[CircleFence] = []
    for match in re.finditer(r"writeCircleFence\(\s*(\w+)\s*,\s*([^,]+)\s*,\s*(NAV_CMD_\w+)\s*\)", src):
        center = point_vars.get(match.group(1))
        radius = _resolve_numeric_expr(match.group(2), numeric_vars)
        if center is None or radius is None:
            continue
        circles.append(
            CircleFence(center, radius, "Exclusion" if "EXCLUSION" in match.group(3) else "Inclusion")
        )
    return circles


def _parse_vehicles(src: str, numeric_vars: Dict[str, float]) -> List[VehicleLocation]:
    """Parse MakeLocation() calls and VehicleLocation struct arrays."""
    vehicles: List[VehicleLocation] = []

    make_loc_pattern = re.compile(
        r'MakeLocation\s*\(\s*"([^"]+)"\s*,\s*[^,]+\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,\)]+)(?:\s*,\s*([^\)]+))?\s*\)',
        re.DOTALL | re.MULTILINE,
    )
    for match in make_loc_pattern.finditer(src):
        lat = _resolve_numeric_expr(match.group(2), numeric_vars)
        lon = _resolve_numeric_expr(match.group(3), numeric_vars)
        alt = _resolve_numeric_expr(match.group(4), numeric_vars)
        if lat is None or lon is None or alt is None:
            continue
        vel_n = _resolve_numeric_expr(match.group(5), numeric_vars)
        vel_e = _resolve_numeric_expr(match.group(6), numeric_vars)
        vel_valid = vel_n is not None and vel_e is not None
        vehicles.append(VehicleLocation(
            match.group(1),
            lat,
            lon,
            alt,
            vel_n if vel_n is not None else 0.0,
            vel_e if vel_e is not None else 0.0,
            _resolve_bool_token(match.group(7)) and vel_valid,
        ))

    vehicle_loc_array_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?VehicleLocation\s+(\w+)\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in vehicle_loc_array_pattern.finditer(src):
        for item in re.finditer(r'\{\s*"([^"]+)"\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^}]+)\}', match.group(2)):
            lat = _resolve_numeric_expr(item.group(2), numeric_vars)
            lon = _resolve_numeric_expr(item.group(3), numeric_vars)
            alt = _resolve_numeric_expr(item.group(4), numeric_vars)
            vel_n = _resolve_numeric_expr(item.group(5), numeric_vars)
            vel_e = _resolve_numeric_expr(item.group(6), numeric_vars)
            if None in (lat, lon, alt):
                continue
            vehicles.append(VehicleLocation(
                item.group(1),
                lat,
                lon,
                alt,
                vel_n if vel_n is not None else 0.0,
                vel_e if vel_e is not None else 0.0,
                True,
            ))

    return vehicles


def _parse_latlon_points(
    src: str,
    point_vars: Dict[str, Tuple[float, float, float]],
    numeric_vars: Dict[str, float],
) -> List[LatLonAltPoint]:
    """Parse makePositionAbsolute, makePositionFromOffset, and MissionRoutePlanner::Position literals."""
    points: List[LatLonAltPoint] = []

    for fn_name, args, _ in _iter_function_calls(src, ("makePositionAbsolute", "makePositionFromOffset")):
        idx = len(points)
        if fn_name == "makePositionAbsolute" and len(args) >= 3:
            lat = _resolve_numeric_value(args[0], numeric_vars, point_vars)
            lon = _resolve_numeric_value(args[1], numeric_vars, point_vars)
            alt = _resolve_numeric_value(args[2], numeric_vars, point_vars)
            if None in (lat, lon, alt):
                continue
            points.append(LatLonAltPoint(f"PositionAbsolute_{idx}", lat, lon, alt))
            continue

        if fn_name == "makePositionFromOffset" and len(args) >= 5:
            base_lat = _resolve_numeric_value(args[0], numeric_vars, point_vars)
            base_lon = _resolve_numeric_value(args[1], numeric_vars, point_vars)
            north = _resolve_numeric_expr(args[2], numeric_vars)
            east = _resolve_numeric_expr(args[3], numeric_vars)
            alt = _resolve_numeric_value(args[4], numeric_vars, point_vars)
            if None in (base_lat, base_lon, north, east, alt):
                continue
            lat, lon = _add_vector_to_global_position(base_lat, base_lon, north, east)
            points.append(LatLonAltPoint(f"PositionFromOffset_{idx}", lat, lon, alt))

    position_literal_pattern = re.compile(r"MissionRoutePlanner::Position\s*\{([^{}]+)\}")
    for match in position_literal_pattern.finditer(src):
        parts = _split_top_level(match.group(1))
        if len(parts) < 3:
            continue
        lat = _resolve_numeric_value(parts[0], numeric_vars, point_vars)
        lon = _resolve_numeric_value(parts[1], numeric_vars, point_vars)
        alt = _resolve_numeric_value(parts[2], numeric_vars, point_vars)
        if None in (lat, lon, alt):
            continue
        points.append(LatLonAltPoint(f"RtlPosition_{len(points)}", lat, lon, alt))

    return points


def _parse_paths(
    src: str,
    numeric_vars: Dict[str, float],
    point_vars: Dict[str, Tuple[float, float, float]],
) -> List[PathCheck]:
    """Parse path check assignments (lat_from, lon_from, lat_to, lon_to)."""
    location_vars_map: Dict[str, Tuple[float, float, float]] = {}
    make_loc_var_pattern = re.compile(
        r'(?:^|\s)(?:const\s+)?(?:static\s+)?(?:constexpr\s+)?(?:auto|VehicleProjectionLocation)\s+'
        r'(\w+)\s*=\s*MakeLocation\s*\(\s*"[^"]+"\s*,\s*[^,]+\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)',
        re.MULTILINE,
    )
    for match in make_loc_var_pattern.finditer(src):
        lat = _resolve_numeric_expr(match.group(2), numeric_vars)
        lon = _resolve_numeric_expr(match.group(3), numeric_vars)
        alt = _resolve_numeric_expr(match.group(4), numeric_vars)
        if None in (lat, lon, alt):
            continue
        location_vars_map[match.group(1)] = (lat, lon, alt)

    def resolve_value(val_str: str) -> Optional[float]:
        return _resolve_numeric_value(val_str, numeric_vars, {**point_vars, **location_vars_map})

    path_assignments: Dict[str, Dict[str, float]] = defaultdict(dict)
    assign_pattern = re.compile(
        r"([\w]+(?:\[\d+\])?)\.(lat_from|lon_from|alt_from|lat_to|lon_to)\s*=\s*([^;]+)\s*;",
        re.MULTILINE,
    )
    for match in assign_pattern.finditer(src):
        value = resolve_value(match.group(3))
        if value is not None:
            path_assignments[match.group(1)][match.group(2)] = value

    paths: List[PathCheck] = []
    for name, data in path_assignments.items():
        if all(key in data for key in ("lat_from", "lon_from", "lat_to", "lon_to")):
            paths.append(PathCheck(
                name=name,
                lat_from=data["lat_from"],
                lon_from=data["lon_from"],
                alt_from=data.get("alt_from", 0.0),
                lat_to=data["lat_to"],
                lon_to=data["lon_to"],
            ))
    return paths


def _parse_projections(src: str, numeric_vars: Dict[str, float]) -> List[Projection]:
    """Parse MakeProjection() calls."""
    projections: List[Projection] = []
    for _, args, _ in _iter_function_calls(src, ("MakeProjection",)):
        if len(args) < 6:
            continue
        lat = _resolve_numeric_expr(args[3], numeric_vars)
        lon = _resolve_numeric_expr(args[4], numeric_vars)
        alt = _resolve_numeric_expr(args[5], numeric_vars)
        if None in (lat, lon, alt):
            continue
        projections.append(Projection(lat, lon, alt, f"Proj_{len(projections)}"))
    return projections


def parse_cpp_code(text: str) -> ParseResult:
    """Parse C++ unit-test source into structured visualization data."""
    src = _strip_cpp_comments(text or "")

    point_vars, numeric_vars, latlon_points = _extract_variable_definitions(src)
    missions = _parse_missions(src, numeric_vars)
    coord_arrays = _parse_coord_arrays(src, numeric_vars)
    rally_points = _parse_rally_points(src, coord_arrays, numeric_vars)
    polygons = _parse_polygons(src, coord_arrays)
    circles = _parse_circles(src, point_vars, numeric_vars)
    vehicles = _parse_vehicles(src, numeric_vars)
    latlon_points.extend(_parse_latlon_points(src, point_vars, numeric_vars))
    paths = _parse_paths(src, numeric_vars, point_vars)
    projections = _parse_projections(src, numeric_vars)

    return missions, polygons, circles, vehicles, latlon_points, paths, rally_points, projections


# ==========================================
# 5. VISUALIZATION (Folium & PyDeck)
# ==========================================

def get_viewer_center(parsed: Optional[ParseResult]) -> Tuple[float, float]:
    if not parsed:
        return DEFAULT_CENTER

    missions, polygons, circles, vehicles, latlon_points, paths, rally, proj = parsed
    if vehicles:
        return vehicles[0].lat, vehicles[0].lon
    if latlon_points:
        return latlon_points[0].lat, latlon_points[0].lon
    if paths:
        return paths[0].lat_from, paths[0].lon_from
    if missions:
        m = next(iter(missions.values()))
        if m:
            return m[0].lat, m[0].lon
    if polygons and polygons[0].vertices:
        return polygons[0].vertices[0][0], polygons[0].vertices[0][1]
    if circles:
        return circles[0].center[0], circles[0].center[1]
    if rally:
        return rally[0].lat, rally[0].lon
    return DEFAULT_CENTER


def create_3d_map(parsed: ParseResult) -> pdk.Deck:
    missions, polygons, circles, vehicles, latlon_points, paths, rally, proj = parsed
    center_lat, center_lon = get_viewer_center(parsed)
    layers = []

    # 3D Polygons
    poly_data = []
    for p in polygons:
        contour = [[pt[1], pt[0], pt[2]] for pt in p.vertices]
        color = [255, 0, 0, 80] if "Exclusion" in p.type else [0, 255, 0, 80]
        poly_data.append({"contour": contour, "name": p.name, "color": color})
    if poly_data:
        layers.append(pdk.Layer(
            "PolygonLayer", poly_data, get_polygon="contour", get_fill_color="color",
            extruded=True, get_elevation=50, pickable=True
        ))

    # 3D Paths
    path_data = []
    for name, wps in missions.items():
        path = [[wp.lon, wp.lat, wp.alt] for wp in wps]
        path_data.append({"path": path, "name": name})
    if path_data:
        layers.append(pdk.Layer(
            "PathLayer", path_data, get_path="path", get_color=[0, 0, 255],
            width_min_pixels=3, pickable=True
        ))

    # Scatter points
    pts = []
    vehicle_vel_lines = []
    for v in vehicles:
        pts.append({
            "pos": [v.lon, v.lat, v.alt],
            "c": [0, 0, 255],
            "info": f"{v.label} vel {v.vel_n:.1f}/{v.vel_e:.1f} m/s",
        })
        arrow = _compute_velocity_arrow(v.lat, v.lon, v.vel_n, v.vel_e, v.v_xy_valid)
        if arrow:
            vehicle_vel_lines.append({
                "source": [arrow["tail"][1], arrow["tail"][0], v.alt],
                "target": [arrow["tip"][1], arrow["tip"][0], v.alt],
                "c": [0, 0, 200],
                "info": f"{v.label} vel {v.vel_n:.1f}/{v.vel_e:.1f} m/s",
            })
    for p in latlon_points: pts.append({"pos": [p.lon, p.lat, p.alt], "c": [255,0,255], "info": p.label})
    for r in rally: pts.append({"pos": [r.lon, r.lat, r.alt], "c": [0,255,0], "info": f"Rally {r.id}"})
    if pts:
        layers.append(pdk.Layer(
            "ScatterplotLayer", pts, get_position="pos", get_fill_color="c",
            get_radius=10, pickable=True
        ))
    if vehicle_vel_lines:
        layers.append(pdk.Layer(
            "LineLayer", vehicle_vel_lines, get_source_position="source",
            get_target_position="target", get_color="c", pickable=True,
            width_min_pixels=2
        ))

    return pdk.Deck(
        layers=layers,
        initial_view_state=pdk.ViewState(latitude=center_lat, longitude=center_lon, zoom=13, pitch=45),
        tooltip={"text": "{info}\n{name}"},
        map_style="mapbox://styles/mapbox/light-v9"
    )


# ==========================================
# 6. UNIFIED MAP & GENERATOR LOGIC
# ==========================================

def get_creator_color(mode: str) -> str:
    if "Path" in mode: return "orange"
    if "Exclusion" in mode: return "red"
    if "Fence" in mode: return "green"
    if "Rally" in mode: return "purple"
    return "blue"


def _add_velocity_arrow_to_map(
    m: folium.Map,
    lat: float,
    lon: float,
    vel_n: float,
    vel_e: float,
    color: str,
    tooltip: str,
    *,
    weight: int = 2,
    opacity: float = 0.9,
    v_xy_valid: bool = True,
) -> None:
    arrow = _compute_velocity_arrow(lat, lon, vel_n, vel_e, v_xy_valid=v_xy_valid)
    if not arrow:
        return
    folium.PolyLine(
        locations=[arrow["tail"], arrow["tip"]],
        color=color,
        weight=weight,
        opacity=opacity,
        tooltip=tooltip,
    ).add_to(m)
    folium.PolyLine(
        locations=[arrow["tip"], arrow["left"]],
        color=color,
        weight=weight,
        opacity=opacity,
        tooltip=tooltip,
    ).add_to(m)
    folium.PolyLine(
        locations=[arrow["tip"], arrow["right"]],
        color=color,
        weight=weight,
        opacity=opacity,
        tooltip=tooltip,
    ).add_to(m)


def _add_parsed_layers_to_map(m: folium.Map, parsed: ParseResult):
    """Render the parsed C++ objects onto the folium map."""
    missions, polygons, circles, vehicles, latlon_points, paths, rally, projections = parsed

    # Polygons
    for poly in polygons:
        c = "red" if "Exclusion" in poly.type else "green"
        folium.Polygon(
            locations=[(p[0], p[1]) for p in poly.vertices],
            color=c, fill=True, fill_opacity=0.3, weight=2,
            popup=f"PARSED: {poly.name}\n{poly.type}"
        ).add_to(m)

    # Circles
    for circ in circles:
        c = "red" if "Exclusion" in circ.type else "green"
        folium.Circle(
            location=(circ.center[0], circ.center[1]),
            radius=circ.radius,
            color=c, fill=True, fill_opacity=0.3, weight=2,
            popup=f"PARSED: Circle\n{circ.type}"
        ).add_to(m)

    # Missions
    colors = ["blue", "orange", "purple", "black"]
    for i, (name, waypoints) in enumerate(missions.items()):
        pts = [(wp.lat, wp.lon) for wp in waypoints]
        c = colors[i % len(colors)]
        if len(pts) >= 2:
            folium.PolyLine(pts, color=c, weight=4, opacity=0.8, tooltip=f"PARSED: {name}").add_to(m)
        for idx, wp in enumerate(waypoints):
            folium.CircleMarker(
                location=(wp.lat, wp.lon), radius=3, color=c, fill=True,
                popup=f"{name}: WP{idx}\nAlt: {wp.alt}"
            ).add_to(m)

    # Paths
    for path in paths:
        pts = [(path.lat_from, path.lon_from), (path.lat_to, path.lon_to)]
        folium.PolyLine(
            pts, color="red", weight=3, dash_array="5, 10",
            tooltip=f"PARSED: {path.name}"
        ).add_to(m)
        folium.CircleMarker(
            pts[0], radius=3, color="green", fill=True,
            tooltip=f"{path.name} Start"
        ).add_to(m)
        folium.CircleMarker(
            pts[1], radius=3, color="red", fill=True,
            tooltip=f"{path.name} End"
        ).add_to(m)

    # Vehicles
    for v in vehicles:
        tooltip = (
            f"PARSED: {v.label}\n"
            f"Alt: {v.alt:.1f}m\n"
            f"Vel N/E: {v.vel_n:.1f}/{v.vel_e:.1f} m/s"
        )
        folium.Marker(
            [v.lat, v.lon], popup=tooltip,
            icon=folium.Icon(color="gray", icon="plane", prefix="fa")
        ).add_to(m)
        _add_velocity_arrow_to_map(
            m,
            v.lat,
            v.lon,
            v.vel_n,
            v.vel_e,
            "gray",
            tooltip,
            v_xy_valid=v.v_xy_valid,
        )

    # LatLonAlt points
    for p in latlon_points:
        folium.Marker(
            [p.lat, p.lon], popup=f"PARSED: {p.label}",
            icon=folium.Icon(color="blue", icon="map-marker", prefix="fa")
        ).add_to(m)

    # Rally Points
    for rp in rally:
        folium.Marker(
            [rp.lat, rp.lon], popup=f"PARSED: Rally {rp.id}",
            icon=folium.Icon(color="lightgray", icon="flag", prefix="fa")
        ).add_to(m)

    # Projections
    for p in projections:
        folium.Marker(
            [p.lat, p.lon], popup=f"PARSED: {p.label}",
            icon=folium.Icon(color="purple", icon="crosshairs", prefix="fa")
        ).add_to(m)


def _build_combined_map(
    parsed_data: Optional[ParseResult],
    creator_points: CreatorPointsMap,
    active_mode: str,
    center: Tuple[float, float],
    zoom: float
) -> folium.Map:

    m = folium.Map(location=center, zoom_start=zoom, tiles="OpenStreetMap")
    m.add_child(folium.LatLngPopup())

    # 1. Render Parsed Data (Background / Reference)
    if parsed_data:
        _add_parsed_layers_to_map(m, parsed_data)

    # 2. Render Creator Data (Interactive / New)
    for mode, points in creator_points.items():
        if not points:
            continue

        color = get_creator_color(mode)
        is_active = (mode == active_mode)
        opacity = 0.9 if is_active else 0.5
        weight = 3 if is_active else 2

        if mode == "Vehicle Location":
            vehicle_points, _, _ = _normalize_vehicle_creator_points(points)
            for i, (lat, lon, alt, vel_n, vel_e) in enumerate(vehicle_points):
                lbl = f"{mode} Pt{i}"
                tooltip_txt = (
                    f"NEW: {lbl}\n"
                    f"Alt: {alt:.1f}m\n"
                    f"Vel N/E: {vel_n:.1f}/{vel_e:.1f} m/s"
                )

                is_last = (is_active and i == len(vehicle_points)-1)
                radius_marker = 6 if is_last else 4

                folium.CircleMarker(
                    location=(lat, lon), radius=radius_marker, color=color, fill=True, fill_opacity=1.0,
                    tooltip=tooltip_txt
                ).add_to(m)
                _add_velocity_arrow_to_map(
                    m,
                    lat,
                    lon,
                    vel_n,
                    vel_e,
                    color,
                    tooltip_txt,
                    weight=weight,
                    opacity=opacity,
                )
            continue

        # Strip altitude/radius for geometry rendering
        coords_2d = [(p[0], p[1]) for p in points]

        # A. Geometry rendering
        if mode == "Mission" and len(points) >= 2:
            folium.PolyLine(coords_2d, color=color, weight=weight, opacity=opacity).add_to(m)

        elif "Polygon" in mode:
            if len(points) >= 3:
                folium.Polygon(coords_2d, color=color, fill=True, fill_opacity=0.2).add_to(m)
            elif len(points) >= 2:
                folium.PolyLine(coords_2d, color=color, weight=weight).add_to(m)

        elif "Circle" in mode:
            # Render all circles using their specific stored radius (index 3)
            for pt in points:
                lat, lon, alt, rad = pt
                folium.Circle(
                    location=(lat, lon), radius=rad, color=color, fill=True, fill_opacity=0.2
                ).add_to(m)

        elif "Path" in mode:
            # Render pairs
            for i in range(0, len(coords_2d)-1, 2):
                seg = [coords_2d[i], coords_2d[i+1]]
                folium.PolyLine(seg, color=color, weight=weight, dash_array="5, 10").add_to(m)

        # B. Markers for all points
        for i, (lat, lon, alt, rad) in enumerate(points):
            lbl = f"{mode} Pt{i}"
            if "Path" in mode:
                lbl = f"Path {i//2} {'Start' if i%2==0 else 'End'}"

            tooltip_txt = f"NEW: {lbl}\nAlt: {alt}m"
            if "Circle" in mode:
                tooltip_txt += f"\nRad: {rad}m"

            # Highlight last added point in active mode
            is_last = (is_active and i == len(points)-1)
            radius_marker = 6 if is_last else 4

            folium.CircleMarker(
                location=(lat, lon), radius=radius_marker, color=color, fill=True, fill_opacity=1.0,
                tooltip=tooltip_txt
            ).add_to(m)

    return m


def generate_full_cpp_snippet(
    points_by_mode: CreatorPointsMap,
    base_name: str
) -> str:
    """
    Generates a combined C++ snippet for ALL modes that have points.
    Uses per-point altitude and radius stored in the creator list.
    """
    sections = []

    # 1. Mission (new helper syntax)
    m_pts = points_by_mode.get("Mission", [])
    if m_pts:
        lines = [f"// --- Mission ({len(m_pts)} items) ---"]
        lines.append("std::vector<mission_item_s> mission{")
        for i, (lat, lon, alt, _) in enumerate(m_pts):
            if len(m_pts) == 1:
                line = f"    makePositionItem({lat:.7f}, {lon:.7f}, {alt:.1f}f),  // {i}"
            elif i == 0:
                line = f"    makeTakeoffItem({lat:.7f}, {lon:.7f}, {alt:.1f}f),  // {i}"
            elif i == len(m_pts) - 1:
                line = f"    makeLandItem({lat:.7f}, {lon:.7f}, {alt:.1f}f),  // {i}"
            else:
                line = f"    makePositionItem({lat:.7f}, {lon:.7f}, {alt:.1f}f),  // {i}"
            lines.append(line)
        lines.append("};")
        sections.append("\n".join(lines))

    # 2. Polygons
    for mode in points_by_mode:
        if "Polygon" in mode and points_by_mode[mode]:
            pts = points_by_mode[mode]
            is_excl = "Exclusion" in mode
            suffix = "_poly_excl" if is_excl else "_poly_incl"
            var = sanitize_cpp_identifier(base_name + suffix)

            lines = [f"// --- {mode} ---"]
            lines.append(f"const std::array<Mission::LatLonAlt, {len(pts)}> {var}{{{{")
            for lat, lon, alt, _ in pts:
                lines.append(f"    {{{lat:.7f}, {lon:.7f}, {alt:.1f}f}},")
            lines.append("}};")

            c_type = "NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION" if is_excl else "NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION"
            lines.append(f"writePolygonFence({var}, {c_type});")
            sections.append("\n".join(lines))

    # 3. Circles
    for mode in points_by_mode:
        if "Circle" in mode and points_by_mode[mode]:
            pts = points_by_mode[mode]
            is_excl = "Exclusion" in mode
            c_type = "NAV_CMD_FENCE_CIRCLE_EXCLUSION" if is_excl else "NAV_CMD_FENCE_CIRCLE_INCLUSION"

            lines = [f"// --- {mode} ({len(pts)} items) ---"]
            for i, (lat, lon, alt, rad) in enumerate(pts):
                suffix = f"_circ_{i}"
                var = sanitize_cpp_identifier(base_name + suffix)
                lines.append(f"const Mission::LatLonAlt {var}{{{lat:.7f}, {lon:.7f}, {alt:.1f}f}};")
                # Radius is taken from the point data
                lines.append(f"writeCircleFence({var}, {rad:.1f}f, {c_type});")
            sections.append("\n".join(lines))

    # 4. Vehicle Locations (new VehicleLocation struct syntax)
    v_pts = points_by_mode.get("Vehicle Location", [])
    if v_pts:
        lines = [f"// --- Vehicle Locations ({len(v_pts)} items) ---"]
        var = sanitize_cpp_identifier(base_name + "_locations")
        lines.append(f"static const VehicleLocation {var}[] = {{")
        for i, pt in enumerate(v_pts):
            if isinstance(pt, (list, tuple)) and len(pt) == 5:
                lat, lon, alt, vel_n, vel_e = pt
            elif isinstance(pt, (list, tuple)) and len(pt) == 4:
                lat, lon, alt, _ = pt
                vel_n, vel_e = 0.0, 0.0
            else:
                continue
            label = base_name if len(v_pts) == 1 else f"{base_name}_{i}"
            lines.append(
                f'    {{"{label}", {lat:.7f}, {lon:.7f}, {alt:.1f}f, {vel_n:.1f}f, {vel_e:.1f}f}},'
            )
        lines.append("};")
        sections.append("\n".join(lines))

    # 5. Paths
    p_pts = points_by_mode.get("Path Check", [])
    if p_pts:
        # Group into pairs
        pairs = []
        for i in range(0, len(p_pts) - 1, 2):
            pairs.append((p_pts[i], p_pts[i+1]))

        if pairs:
            lines = [f"// --- Path Checks ({len(pairs)} paths) ---"]
            var = sanitize_cpp_identifier(base_name + "_paths")
            lines.append(f"Geofence::PathCheck {var}[{len(pairs)}] {{}};")

            for i, (start, end) in enumerate(pairs):
                # Unpack start/end
                s_lat, s_lon, s_alt, _ = start
                e_lat, e_lon, _, _ = end

                idx_str = f"[{i}]"
                lines.append("")
                lines.append(f"{var}{idx_str}.lat_from = {s_lat:.7f};")
                lines.append(f"{var}{idx_str}.lon_from = {s_lon:.7f};")
                lines.append(f"{var}{idx_str}.alt_from = {s_alt:.1f}f;")
                lines.append(f"{var}{idx_str}.lat_to = {e_lat:.7f};")
                lines.append(f"{var}{idx_str}.lon_to = {e_lon:.7f};")

            sections.append("\n".join(lines))

    # 6. Rally Points (new helper syntax)
    r_pts = points_by_mode.get("Rally Points", [])
    if r_pts:
        lines = [f"// --- Rally / Safe Points ({len(r_pts)} items) ---"]
        lines.append("std::vector<mission_item_s> safe_points{")
        for i, (lat, lon, alt, _) in enumerate(r_pts):
            lines.append(f"    makeSafePointAbsolute({lat:.7f}, {lon:.7f}, {alt:.1f}f),  // {i}")
        lines.append("};")
        sections.append("\n".join(lines))

    if not sections:
        return "// No points defined on the map."

    return "\n\n".join(sections)


# ==========================================
# 7. STREAMLIT APP
# ==========================================

def init_session():
    if "parse_result" not in st.session_state:
        st.session_state["parse_result"] = None

    # Creator State
    if "creator_points" not in st.session_state:
        st.session_state["creator_points"] = {m: [] for m in CREATOR_MODES}

    # Normalize stored points to the current format.
    for m in CREATOR_MODES:
        pts = st.session_state["creator_points"][m]
        if not pts:
            continue
        if m == "Vehicle Location":
            normalized, dropped, legacy = _normalize_vehicle_creator_points(pts)
            st.session_state["creator_points"][m] = normalized
            if legacy:
                st.warning(f"Upgraded {legacy} legacy {m} point(s) with zero velocity.")
            if dropped:
                st.warning(f"Dropped {dropped} {m} point(s) due to incompatible format.")
        else:
            if len(pts[0]) != 4:
                st.session_state["creator_points"][m] = []
                st.warning(f"Cleared legacy {m} points due to version update (Added Radius support).")

    if "creator_map_center" not in st.session_state:
        st.session_state["creator_map_center"] = DEFAULT_CENTER
    if "creator_map_zoom" not in st.session_state:
        st.session_state["creator_map_zoom"] = DEFAULT_ZOOM

    # Defaults for inputs
    if "last_radius" not in st.session_state:
        st.session_state["last_radius"] = 100.0
    if "last_vehicle_vel_north_m_s" not in st.session_state:
        st.session_state["last_vehicle_vel_north_m_s"] = 0.0
    if "last_vehicle_vel_east_m_s" not in st.session_state:
        st.session_state["last_vehicle_vel_east_m_s"] = 0.0


def main():
    st.set_page_config(layout="wide", page_title="PX4 Mission Helper")
    st.title("PX4 Mission & Geofence Utility")
    init_session()
    # Sync view state before rendering the map to avoid zoom snapping back.
    _sync_map_view_state(st.session_state.get("combined_map"))

    main_tab, viewer_3d_tab = st.tabs(["Main Interface (2D Map)", "3D Viewer (View Only)"])

    # --- TAB 1: Main Unified Interface ---
    with main_tab:
        col_vis, col_map, col_creator = st.columns([1, 3, 1])

        with col_vis:
            # === SECTION A: VIEWER INPUT ===
            with st.expander("1. Visualize Existing C++ Code", expanded=False):
                raw_code = st.text_area("Paste C++", height=200, help="Paste unit test code here.")
                if st.button("Parse & Visualize", type="secondary"):
                    try:
                        parsed = parse_cpp_code(raw_code)
                        st.session_state["parse_result"] = parsed

                        # Auto-center map on parsed content
                        new_center = get_viewer_center(parsed)
                        st.session_state["creator_map_center"] = new_center
                        st.success(f"Parsed successfully! Map centered at {new_center}")
                    except Exception as e:
                        st.error(f"Error: {e}")

                if st.button("Clear Visualized Layer"):
                    st.session_state["parse_result"] = None
                    st.rerun()

            st.divider()

        with col_creator:
            # === SECTION B: CREATOR INPUT ===
            st.markdown("### 2. Add New Items")

            mode = st.radio("Item Type", CREATOR_MODES)
            base_name = st.text_input("Base Name", value="kItem")

            # Per-point parameters
            is_vehicle = (mode == "Vehicle Location")
            current_vel_north = 0.0
            current_vel_east = 0.0
            c1, c2 = st.columns(2)
            with c1:
                current_alt = st.number_input("Altitude (m)", value=500.0, help="Applies to next click")
            with c2:
                disable_rad = "Circle" not in mode
                current_rad = st.number_input(
                    "Radius (m)",
                    value=st.session_state["last_radius"],
                    disabled=disable_rad,
                    help="Applies to next click",
                )
                # Update session state to remember radius across re-runs
                if not disable_rad:
                    st.session_state["last_radius"] = current_rad

            if is_vehicle:
                v1, v2 = st.columns(2)
                with v1:
                    current_vel_north = st.number_input(
                        "Velocity North (m/s)",
                        value=st.session_state["last_vehicle_vel_north_m_s"],
                        help="Applies to next click",
                    )
                    st.session_state["last_vehicle_vel_north_m_s"] = current_vel_north
                with v2:
                    current_vel_east = st.number_input(
                        "Velocity East (m/s)",
                        value=st.session_state["last_vehicle_vel_east_m_s"],
                        help="Applies to next click",
                    )
                    st.session_state["last_vehicle_vel_east_m_s"] = current_vel_east

            st.markdown("---")

            # Action Buttons
            pts_map = st.session_state["creator_points"]
            curr_len = len(pts_map[mode])

            if st.button(f"Clear New {mode} ({curr_len})", use_container_width=True):
                pts_map[mode] = []
                st.rerun()

            if st.button("Clear ALL New Items", type="secondary", use_container_width=True):
                for m in CREATOR_MODES:
                    pts_map[m] = []
                st.rerun()

        with col_map:
            # Map Rendering
            # We strictly use session state for center/zoom to prevent jumping
            center = st.session_state["creator_map_center"]
            zoom = st.session_state["creator_map_zoom"]
            parsed_data = st.session_state["parse_result"]

            m_combined = _build_combined_map(
                parsed_data,
                st.session_state["creator_points"],
                mode,
                tuple(center),
                zoom
            )

            # Return objects: capture click and view state
            ret = st_folium_compat(
                m_combined, key="combined_map", height=600,
                returned_objects=["last_clicked", "last_object_clicked", "center", "zoom"]
            )

            # Update View State (seamless interaction)
            _sync_map_view_state(ret)

            # Handle Clicks (Add Point)
            click = ret.get("last_clicked") or ret.get("last_object_clicked")
            if click:
                lc = click
                current_list = st.session_state["creator_points"][mode]

                if mode == "Vehicle Location":
                    pt_vehicle = (
                        round(lc["lat"], 7),
                        round(lc["lng"], 7),
                        current_alt,
                        current_vel_north,
                        current_vel_east,
                    )
                    already_exists = any(
                        (p[0] == pt_vehicle[0] and p[1] == pt_vehicle[1]) for p in current_list
                    )
                    if not already_exists:
                        st.session_state["creator_points"][mode].append(pt_vehicle)
                        st.rerun()
                else:
                    pt_4d = (round(lc["lat"], 7), round(lc["lng"], 7), current_alt, current_rad)
                    already_exists = any(
                        (p[0] == pt_4d[0] and p[1] == pt_4d[1]) for p in current_list
                    )

                    if not already_exists:
                        if "Polygon" in mode:
                            st.session_state["creator_points"][mode] = _insert_point_on_polygon(current_list, pt_4d)
                        else:
                            st.session_state["creator_points"][mode].append(pt_4d)
                        st.rerun()

    st.write("") # Add a small spacer
    if st.button("Generate C++ for NEW Items", type="primary", use_container_width=True):
        code = generate_full_cpp_snippet(pts_map, base_name)
        st.session_state["generated_code"] = code

    if st.session_state.get("generated_code"):
        st.divider()
        st.markdown("### Generated Code")
        st.code(st.session_state["generated_code"], language="cpp")

    # --- TAB 2: 3D Viewer ---
    with viewer_3d_tab:
        if st.session_state["parse_result"]:
            try:
                st.pydeck_chart(create_3d_map(st.session_state["parse_result"]))
                st.info("Note: 3D view currently shows only the parsed C++ data, not newly added items.")
            except Exception as e:
                st.warning(f"3D Error: {e}")
        else:
            st.info("Paste and parse C++ code in the Main Interface to see the 3D visualization.")

if __name__ == "__main__":
    main()
