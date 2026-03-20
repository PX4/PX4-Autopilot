"""
PX4 Mission & Geofence Utility (Streamlit)

Features
- Unified Viewer & Creator: Paste C++ unit-test data to visualize it,
  then click to add new items (fences, missions, etc.) on the same map.
- Code Generator: Generates C++ snippets only for the newly added items.
- 3D Viewer: Optional pydeck visualization (View Only).
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import inspect
import math
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

DEFAULT_CENTER: Tuple[float, float] = (45.9766, 7.6585) # Matterhorn
DEFAULT_ZOOM: int = 12
EARTH_RADIUS_M: float = 6371000.0
BANK_TURN_MODE: str = "Bank Turn Zone"
BANK_TURN_FEASIBILITY_SAMPLES: int = 8
BANK_TURN_MAX_WIND_FALLBACK_M_S: float = 15.0
BANK_TURN_FIELD_NAMES: Tuple[str, ...] = (
    "current_lat",
    "current_lon",
    "current_alt",
    "turn_lat",
    "turn_lon",
    "turn_alt",
    "base_radius_m",
    "wind_north_m_s",
    "wind_east_m_s",
    "max_wind_m_s",
    "expected_safe",
)
BANK_TURN_LEGACY_FIELD_NAMES: Tuple[str, ...] = (
    "lat",
    "lon",
    "alt",
    "base_radius_m",
    "wind_north_m_s",
    "wind_east_m_s",
    "max_wind_m_s",
    "expected_safe",
)
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
    BANK_TURN_MODE,
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
class BankTurnCreatorPoint:
    current_lat: float
    current_lon: float
    current_alt: float
    turn_lat: float
    turn_lon: float
    turn_alt: float
    base_radius_m: float
    wind_north_m_s: float
    wind_east_m_s: float
    max_wind_m_s: float
    expected_safe: bool


@dataclass(frozen=True)
class BankTurnZone:
    name: str
    turn_point: Tuple[float, float, float]
    current_location: Tuple[float, float, float]
    center: Tuple[float, float, float]
    polygon: List[Tuple[float, float, float]]
    radius_m: float
    base_radius_m: float
    wind_north_m_s: float
    wind_east_m_s: float
    max_wind_m_s: float
    expected_safe: Optional[bool]


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
    List[BankTurnZone],
    List[Projection],
]


# ==========================================
# 3. MATH HELPERS (For Polygon Insertion)
# ==========================================

# Lat, Lon, Alt, Radius
CreatorPoint = Tuple[float, float, float, float]
VehicleCreatorPoint = Tuple[float, float, float, float, float]
CreatorItem = Union[CreatorPoint, VehicleCreatorPoint, BankTurnCreatorPoint]
CreatorPointsMap = Dict[str, List[CreatorItem]]

def _build_bank_turn_creator_point(
    current_lat: object,
    current_lon: object,
    current_alt: object,
    turn_lat: object,
    turn_lon: object,
    turn_alt: object,
    base_radius_m: object,
    wind_north_m_s: object,
    wind_east_m_s: object,
    max_wind_m_s: object,
    expected_safe: object,
) -> Optional[BankTurnCreatorPoint]:
    try:
        return BankTurnCreatorPoint(
            current_lat=float(current_lat),
            current_lon=float(current_lon),
            current_alt=float(current_alt),
            turn_lat=float(turn_lat),
            turn_lon=float(turn_lon),
            turn_alt=float(turn_alt),
            base_radius_m=float(base_radius_m),
            wind_north_m_s=float(wind_north_m_s),
            wind_east_m_s=float(wind_east_m_s),
            max_wind_m_s=float(max_wind_m_s),
            expected_safe=bool(expected_safe),
        )
    except Exception:
        return None


def _build_bank_turn_creator_point_from_turn(
    lat: object,
    lon: object,
    alt: object,
    base_radius_m: object,
    wind_north_m_s: object,
    wind_east_m_s: object,
    max_wind_m_s: object,
    expected_safe: object,
) -> Optional[BankTurnCreatorPoint]:
    return _build_bank_turn_creator_point(
        lat,
        lon,
        alt,
        lat,
        lon,
        alt,
        base_radius_m,
        wind_north_m_s,
        wind_east_m_s,
        max_wind_m_s,
        expected_safe,
    )


def _coerce_bank_turn_point(item: object) -> Optional[BankTurnCreatorPoint]:
    if isinstance(item, BankTurnCreatorPoint):
        return item

    if isinstance(item, dict):
        if all(k in item for k in BANK_TURN_FIELD_NAMES):
            return _build_bank_turn_creator_point(
                item["current_lat"],
                item["current_lon"],
                item["current_alt"],
                item["turn_lat"],
                item["turn_lon"],
                item["turn_alt"],
                item["base_radius_m"],
                item["wind_north_m_s"],
                item["wind_east_m_s"],
                item["max_wind_m_s"],
                item["expected_safe"],
            )
        if all(k in item for k in BANK_TURN_LEGACY_FIELD_NAMES):
            return _build_bank_turn_creator_point_from_turn(
                item["lat"],
                item["lon"],
                item["alt"],
                item["base_radius_m"],
                item["wind_north_m_s"],
                item["wind_east_m_s"],
                item["max_wind_m_s"],
                item["expected_safe"],
            )
        return None

    if all(hasattr(item, k) for k in BANK_TURN_FIELD_NAMES):
        return _build_bank_turn_creator_point(
            getattr(item, "current_lat"),
            getattr(item, "current_lon"),
            getattr(item, "current_alt"),
            getattr(item, "turn_lat"),
            getattr(item, "turn_lon"),
            getattr(item, "turn_alt"),
            getattr(item, "base_radius_m"),
            getattr(item, "wind_north_m_s"),
            getattr(item, "wind_east_m_s"),
            getattr(item, "max_wind_m_s"),
            getattr(item, "expected_safe"),
        )
    if all(hasattr(item, k) for k in BANK_TURN_LEGACY_FIELD_NAMES):
        return _build_bank_turn_creator_point_from_turn(
            getattr(item, "lat"),
            getattr(item, "lon"),
            getattr(item, "alt"),
            getattr(item, "base_radius_m"),
            getattr(item, "wind_north_m_s"),
            getattr(item, "wind_east_m_s"),
            getattr(item, "max_wind_m_s"),
            getattr(item, "expected_safe"),
        )

    if isinstance(item, (list, tuple)) and len(item) == len(BANK_TURN_FIELD_NAMES):
        return _build_bank_turn_creator_point(*item)
    if isinstance(item, (list, tuple)) and len(item) == len(BANK_TURN_LEGACY_FIELD_NAMES):
        return _build_bank_turn_creator_point_from_turn(*item)

    return None


def _normalize_bank_turn_points(
    points: Sequence[CreatorItem],
) -> Tuple[List[BankTurnCreatorPoint], int]:
    # Streamlit reruns re-create class objects; coerce by shape/attrs instead.
    normalized: List[BankTurnCreatorPoint] = []
    dropped = 0
    for item in points:
        pt = _coerce_bank_turn_point(item)
        if pt is None:
            dropped += 1
            continue
        normalized.append(pt)
    return normalized, dropped


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


def _compute_bank_turn_zone(
    turn_lat: float,
    turn_lon: float,
    turn_alt: float,
    base_radius_m: float,
    wind_north_m_s: float,
    wind_east_m_s: float,
    max_wind_m_s: float,
) -> Tuple[Tuple[float, float, float], float]:
    wind_speed = math.hypot(wind_north_m_s, wind_east_m_s)
    wind_valid = (
        math.isfinite(wind_north_m_s)
        and math.isfinite(wind_east_m_s)
        and math.isfinite(wind_speed)
    )

    max_wind = max_wind_m_s if (math.isfinite(max_wind_m_s) and max_wind_m_s > 0.0) else BANK_TURN_MAX_WIND_FALLBACK_M_S
    wind_ratio = min(max(wind_speed / max_wind, 0.0), 1.0) if wind_valid else 1.0

    delta = base_radius_m * wind_ratio
    radius_m = base_radius_m + delta
    center_lat, center_lon = turn_lat, turn_lon

    if wind_valid and wind_speed > 1e-6 and delta > 1e-6:
        shift_n = delta * wind_north_m_s / wind_speed
        shift_e = delta * wind_east_m_s / wind_speed
        center_lat, center_lon = _add_vector_to_global_position(turn_lat, turn_lon, shift_n, shift_e)

    return (center_lat, center_lon, turn_alt), radius_m


def _build_bank_turn_polygon(
    center_lat: float,
    center_lon: float,
    alt: float,
    radius_m: float,
    samples: int,
) -> List[Tuple[float, float, float]]:
    if samples < 3:
        return []

    step = (2.0 * math.pi) / float(samples)
    points = []
    for i in range(samples):
        angle = step * float(i)
        north_rel = radius_m * math.cos(angle)
        east_rel = radius_m * math.sin(angle)
        lat, lon = _add_vector_to_global_position(center_lat, center_lon, north_rel, east_rel)
        points.append((lat, lon, alt))
    return points


def _compute_bank_turn_geometry(
    turn_lat: float,
    turn_lon: float,
    turn_alt: float,
    base_radius_m: float,
    wind_north_m_s: float,
    wind_east_m_s: float,
    max_wind_m_s: float,
    samples: int,
) -> Tuple[Tuple[float, float, float], float, List[Tuple[float, float, float]]]:
    center, radius_m = _compute_bank_turn_zone(
        turn_lat,
        turn_lon,
        turn_alt,
        base_radius_m,
        wind_north_m_s,
        wind_east_m_s,
        max_wind_m_s,
    )
    polygon = _build_bank_turn_polygon(center[0], center[1], center[2], radius_m, samples)
    return center, radius_m, polygon


def _compute_bank_turn_geometry_for_zone(
    zone: BankTurnZone,
    samples: int,
) -> Tuple[Tuple[float, float, float], float, List[Tuple[float, float, float]]]:
    """Recompute bank turn geometry from zone inputs to respect current sampling."""
    return _compute_bank_turn_geometry(
        zone.turn_point[0],
        zone.turn_point[1],
        zone.turn_point[2],
        zone.base_radius_m,
        zone.wind_north_m_s,
        zone.wind_east_m_s,
        zone.max_wind_m_s,
        samples,
    )


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

LATLON_TOKEN_RE: str = r"(?:Mission::LatLonAlt\s*)?\{[^}]*\}|\w+(?:::\w+)*"

# Shared regex for coordinate triples {lat, lon, alt}
_COORD_ITEM_RE = re.compile(
    rf"\{{\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\}}",
    re.MULTILINE,
)

# Allowed nav commands for waypoint parsing
_NAV_CMDS = {"NAV_CMD_WAYPOINT", "NAV_CMD_TAKEOFF", "NAV_CMD_LAND"}


def _parse_latlon_token(
    token: str,
    vars_map: Dict[str, Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float]]:
    cleaned = token.strip()
    if not cleaned:
        return None
    if cleaned.startswith("Mission::LatLonAlt"):
        cleaned = cleaned[len("Mission::LatLonAlt"):].strip()
    if cleaned.startswith("{") and cleaned.endswith("}"):
        inner = cleaned[1:-1]
        parts = [p.strip() for p in inner.split(",") if p.strip()]
        if len(parts) < 2:
            return None
        lat = _safe_float(parts[0])
        lon = _safe_float(parts[1])
        alt = _safe_float(parts[2]) if len(parts) >= 3 else 0.0
        return lat, lon, alt
    var_name = cleaned.split("::")[-1]
    return vars_map.get(var_name)


def _resolve_float_token(token: str, float_vars_map: Dict[str, float]) -> Optional[float]:
    cleaned = token.strip()
    if cleaned.startswith("(") and cleaned.endswith(")"):
        cleaned = cleaned[1:-1].strip()
    if re.fullmatch(FLOAT_RE, cleaned):
        return _safe_float(cleaned)

    sign = 1.0
    if cleaned.startswith(("+", "-")):
        sign = -1.0 if cleaned[0] == "-" else 1.0
        cleaned = cleaned[1:].strip()
    if re.fullmatch(r"[A-Za-z_]\w*", cleaned):
        val = float_vars_map.get(cleaned)
        if val is None:
            return None
        return sign * val

    return None


def _resolve_bool_token(token: Optional[str]) -> bool:
    if token is None:
        return True
    cleaned = token.strip().lower()
    if cleaned in ("true", "false"):
        return cleaned == "true"
    return True


# ── Sub-parsers for parse_cpp_code ──────────────────────────────────────────


def _extract_variable_definitions(
    src: str,
) -> Tuple[Dict[str, Tuple[float, float, float]], Dict[str, float], List[LatLonAltPoint]]:
    """Extract Mission::LatLonAlt and float variable definitions from C++ source."""
    latlon_var_pattern = re.compile(
        rf"Mission::LatLonAlt\s+(\w+)\s*(?:=\s*)?\{{\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*(?:,\s*({FLOAT_RE}))?\s*\}}",
        re.MULTILINE,
    )
    vars_map: Dict[str, Tuple[float, float, float]] = {}
    latlon_points: List[LatLonAltPoint] = []

    for match in latlon_var_pattern.finditer(src):
        vars_map[match.group(1)] = (
            _safe_float(match.group(2)),
            _safe_float(match.group(3)),
            _safe_float(match.group(4) or "0"),
        )

    for name, (lat, lon, alt) in vars_map.items():
        latlon_points.append(LatLonAltPoint(name, lat, lon, alt))

    float_vars_map: Dict[str, float] = {}
    float_var_pattern = re.compile(
        rf"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?float\s+(\w+)\s*=\s*({FLOAT_RE})\s*;",
        re.MULTILINE,
    )
    for match in float_var_pattern.finditer(src):
        float_vars_map[match.group(1)] = _safe_float(match.group(2))

    return vars_map, float_vars_map, latlon_points


def _parse_missions(src: str) -> Dict[str, List[Waypoint]]:
    """Parse all mission item sources: arrays, legacy arrays, and inline helper calls."""
    missions: Dict[str, List[Waypoint]] = {}

    # MissionItemInput arrays
    mission_array_pattern = re.compile(
        rf"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::array<\s*MissionItemInput\s*,\s*\d+\s*>\s+(\w+)\s*\{{\s*\{{(.*?)\}}\s*\}}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    mission_item_pattern = re.compile(
        rf"\{{\s*(NAV_CMD_\w+)\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})",
        re.MULTILINE,
    )
    for match in mission_array_pattern.finditer(src):
        name = match.group(1)
        wps = [
            Waypoint(cmd=m.group(1), lat=_safe_float(m.group(2)),
                     lon=_safe_float(m.group(3)), alt=_safe_float(m.group(4)))
            for m in mission_item_pattern.finditer(match.group(2))
            if m.group(1) in _NAV_CMDS
        ]
        if wps:
            missions[name] = wps

    # LegacyMissionItem arrays
    legacy_array_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?LegacyMissionItem\s+(\w+)\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in legacy_array_pattern.finditer(src):
        name = match.group(1)
        wps = [
            Waypoint(cmd=m.group(1), lat=_safe_float(m.group(2)),
                     lon=_safe_float(m.group(3)), alt=_safe_float(m.group(4)))
            for m in mission_item_pattern.finditer(match.group(2))
            if m.group(1) in _NAV_CMDS
        ]
        if wps:
            missions[name] = wps

    # Inline helper calls
    _parse_inline_mission_helpers(src, missions)

    return missions


def _parse_inline_mission_helpers(src: str, missions: Dict[str, List[Waypoint]]) -> None:
    """Parse makePositionItem, makePositionItemFromOffset, makeTakeoffItem*, makeLandItem* calls."""
    # makePositionItem(lat, lon, alt, NAV_CMD_*)
    for match in re.finditer(
        rf"makePositionItem\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*(NAV_CMD_\w+)\s*\)",
        src,
    ):
        if match.group(4) in _NAV_CMDS:
            missions.setdefault("_inline_makePositionItem", []).append(
                Waypoint(cmd=match.group(4), lat=_safe_float(match.group(1)),
                         lon=_safe_float(match.group(2)), alt=_safe_float(match.group(3))))

    # makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_*)
    for match in re.finditer(
        rf"makePositionItemFromOffset\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,"
        rf"\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*(NAV_CMD_\w+)",
        src,
    ):
        if match.group(6) in _NAV_CMDS:
            lat_r, lon_r = _add_vector_to_global_position(
                _safe_float(match.group(1)), _safe_float(match.group(2)),
                _safe_float(match.group(3)), _safe_float(match.group(4)))
            missions.setdefault("_inline_makePositionItemFromOffset", []).append(
                Waypoint(cmd=match.group(6), lat=lat_r, lon=lon_r, alt=_safe_float(match.group(5))))

    # Offset helpers for takeoff / land
    _offset_helpers = [
        ("makeTakeoffItemFromOffset", "NAV_CMD_TAKEOFF", "_inline_makeTakeoffItemFromOffset"),
        ("makeLandItemFromOffset",    "NAV_CMD_LAND",    "_inline_makeLandItemFromOffset"),
    ]
    for func_name, cmd, key in _offset_helpers:
        for match in re.finditer(
            rf"{func_name}\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,"
            rf"\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\)",
            src,
        ):
            lat_r, lon_r = _add_vector_to_global_position(
                _safe_float(match.group(1)), _safe_float(match.group(2)),
                _safe_float(match.group(3)), _safe_float(match.group(4)))
            missions.setdefault(key, []).append(
                Waypoint(cmd=cmd, lat=lat_r, lon=lon_r, alt=_safe_float(match.group(5))))

    # Absolute helpers for takeoff / land
    _abs_helpers = [
        ("makeTakeoffItem", "NAV_CMD_TAKEOFF", "_inline_makeTakeoffItem"),
        ("makeLandItem",    "NAV_CMD_LAND",    "_inline_makeLandItem"),
    ]
    for func_name, cmd, key in _abs_helpers:
        for match in re.finditer(
            rf"{func_name}\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\)",
            src,
        ):
            missions.setdefault(key, []).append(
                Waypoint(cmd=cmd, lat=_safe_float(match.group(1)),
                         lon=_safe_float(match.group(2)), alt=_safe_float(match.group(3))))


def _parse_coord_arrays(src: str) -> Dict[str, List[Tuple[float, float, float]]]:
    """Parse std::array<Mission::LatLonAlt, N> coordinate arrays."""
    coord_arrays: Dict[str, List[Tuple[float, float, float]]] = {}
    coord_array_pattern = re.compile(
        rf"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?std::array<\s*Mission::LatLonAlt\s*,\s*\d+\s*>\s+(\w+)\s*\{{\s*\{{(.*?)\}}\s*\}}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in coord_array_pattern.finditer(src):
        items = [
            (_safe_float(m.group(1)), _safe_float(m.group(2)), _safe_float(m.group(3)))
            for m in _COORD_ITEM_RE.finditer(match.group(2))
        ]
        if items:
            coord_arrays[match.group(1)] = items
    return coord_arrays


def _parse_rally_points(
    src: str,
    coord_arrays: Dict[str, List[Tuple[float, float, float]]],
) -> List[RallyPoint]:
    """Parse rally points from coordinate arrays, legacy arrays, and helper calls."""
    rally_points: List[RallyPoint] = []

    # From coordinate arrays named *rally*
    for name, items in coord_arrays.items():
        if "rally" in name.lower():
            for i, (lat, lon, alt) in enumerate(items):
                rally_points.append(RallyPoint(lat, lon, alt, i))

    # LegacyRallyPoint arrays
    legacy_rally_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?LegacyRallyPoint\s+(\w+)\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    for match in legacy_rally_pattern.finditer(src):
        idx_base = len(rally_points)
        for i, item in enumerate(_COORD_ITEM_RE.finditer(match.group(2))):
            rally_points.append(RallyPoint(
                _safe_float(item.group(1)), _safe_float(item.group(2)),
                _safe_float(item.group(3)), idx_base + i))

    # makeSafePointAbsolute(lat, lon, alt)
    for match in re.finditer(
        rf"makeSafePointAbsolute\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\)", src,
    ):
        rally_points.append(RallyPoint(
            _safe_float(match.group(1)), _safe_float(match.group(2)),
            _safe_float(match.group(3)), len(rally_points)))

    # makeSafePointFromOffset(base_lat, base_lon, north_m, east_m, alt)
    for match in re.finditer(
        rf"makeSafePointFromOffset\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,"
        rf"\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\)", src,
    ):
        lat_r, lon_r = _add_vector_to_global_position(
            _safe_float(match.group(1)), _safe_float(match.group(2)),
            _safe_float(match.group(3)), _safe_float(match.group(4)))
        rally_points.append(RallyPoint(lat_r, lon_r, _safe_float(match.group(5)), len(rally_points)))

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
            f_type = "Exclusion" if "EXCLUSION" in cmd_type else "Inclusion"
            polygons.append(PolygonFence(var_name, coord_arrays[var_name], f_type))
    return polygons


def _parse_circles(
    src: str,
    vars_map: Dict[str, Tuple[float, float, float]],
    float_vars_map: Dict[str, float],
) -> List[CircleFence]:
    """Parse writeCircleFence() calls."""
    circles: List[CircleFence] = []
    for match in re.finditer(
        rf"writeCircleFence\(\s*(\w+)\s*,\s*({FLOAT_RE}|[A-Za-z_]\w*)\s*,\s*(NAV_CMD_\w+)\s*\)", src,
    ):
        var_name = match.group(1)
        radius_token = match.group(2).strip()
        radius = (_safe_float(radius_token)
                  if re.fullmatch(FLOAT_RE, radius_token) else float_vars_map.get(radius_token))
        center = vars_map.get(var_name)
        if center and radius is not None:
            f_type = "Exclusion" if "EXCLUSION" in match.group(3) else "Inclusion"
            circles.append(CircleFence(center, radius, f_type))
    return circles


def _parse_vehicles(src: str, float_vars_map: Dict[str, float]) -> List[VehicleLocation]:
    """Parse MakeLocation() calls and VehicleLocation struct arrays."""
    vehicles: List[VehicleLocation] = []

    # MakeLocation("label", ..., lat, lon, alt, vel_n, vel_e [, v_xy_valid])
    make_loc_pattern = re.compile(
        r'MakeLocation\s*\(\s*"([^"]+)"\s*,\s*[^,]+\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,\)]+)(?:\s*,\s*([^\)]+))?\s*\)',
        re.DOTALL | re.MULTILINE,
    )
    for match in make_loc_pattern.finditer(src):
        lat = _resolve_float_token(match.group(2), float_vars_map)
        lon = _resolve_float_token(match.group(3), float_vars_map)
        alt = _resolve_float_token(match.group(4), float_vars_map)
        if lat is None or lon is None or alt is None:
            continue
        vel_n = _resolve_float_token(match.group(5), float_vars_map)
        vel_e = _resolve_float_token(match.group(6), float_vars_map)
        vel_valid = (vel_n is not None) and (vel_e is not None)
        v_xy_valid = _resolve_bool_token(match.group(7)) and vel_valid
        vehicles.append(VehicleLocation(
            match.group(1), lat, lon, alt,
            vel_n if vel_n is not None else 0.0,
            vel_e if vel_e is not None else 0.0,
            v_xy_valid))

    # VehicleLocation struct arrays
    vehicle_loc_array_pattern = re.compile(
        r"(?:static\s+)?(?:constexpr\s+)?(?:const\s+)?VehicleLocation\s+(\w+)\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL | re.MULTILINE,
    )
    vehicle_loc_item_pattern = re.compile(
        rf'\{{\s*"([^"]+)"\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE}|[A-Za-z_]\w*)\s*,\s*({FLOAT_RE}|[A-Za-z_]\w*)\s*\}}',
        re.MULTILINE,
    )
    for match in vehicle_loc_array_pattern.finditer(src):
        for item in vehicle_loc_item_pattern.finditer(match.group(2)):
            vel_n = _resolve_float_token(item.group(5).strip(), float_vars_map)
            vel_e = _resolve_float_token(item.group(6).strip(), float_vars_map)
            vehicles.append(VehicleLocation(
                item.group(1), _safe_float(item.group(2)),
                _safe_float(item.group(3)), _safe_float(item.group(4)),
                vel_n if vel_n is not None else 0.0,
                vel_e if vel_e is not None else 0.0,
                True))

    return vehicles


def _parse_latlon_points(
    src: str,
    vars_map: Dict[str, Tuple[float, float, float]],
) -> List[LatLonAltPoint]:
    """Parse makePositionAbsolute, makePositionFromOffset, and RtlRoutePlanner::Position literals."""
    points: List[LatLonAltPoint] = []

    # makePositionAbsolute(lat, lon, alt)
    for i, match in enumerate(re.finditer(
        rf"makePositionAbsolute\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\)", src,
    )):
        points.append(LatLonAltPoint(
            f"PositionAbsolute_{i}",
            _safe_float(match.group(1)), _safe_float(match.group(2)), _safe_float(match.group(3))))

    # makePositionFromOffset(base_lat, base_lon, north_m, east_m, alt)
    for i, match in enumerate(re.finditer(
        rf"makePositionFromOffset\s*\(\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,"
        rf"\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\)", src,
    )):
        lat_r, lon_r = _add_vector_to_global_position(
            _safe_float(match.group(1)), _safe_float(match.group(2)),
            _safe_float(match.group(3)), _safe_float(match.group(4)))
        points.append(LatLonAltPoint(f"PositionFromOffset_{i}", lat_r, lon_r, _safe_float(match.group(5))))

    # RtlRoutePlanner::Position{lat, lon, alt}
    for i, match in enumerate(re.finditer(
        rf"RtlRoutePlanner::Position\s*\{{\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*\}}", src,
    )):
        points.append(LatLonAltPoint(
            f"RtlPosition_{i}",
            _safe_float(match.group(1)), _safe_float(match.group(2)), _safe_float(match.group(3))))

    return points


def _parse_paths(
    src: str,
    float_vars_map: Dict[str, float],
    vars_map: Dict[str, Tuple[float, float, float]],
) -> List[PathCheck]:
    """Parse path check assignments (lat_from, lon_from, lat_to, lon_to)."""
    # Build location vars map for resolving variable references
    location_vars_map: Dict[str, Tuple[float, float, float]] = {}
    make_loc_var_pattern = re.compile(
        rf'(?:^|\s)(?:const\s+)?(?:static\s+)?(?:constexpr\s+)?(?:auto|VehicleProjectionLocation)\s+'
        rf'(\w+)\s*=\s*MakeLocation\s*\(\s*"[^"]+"\s*,\s*[^,]+\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})',
        re.MULTILINE,
    )
    for match in make_loc_var_pattern.finditer(src):
        location_vars_map[match.group(1)] = (
            _safe_float(match.group(2)), _safe_float(match.group(3)), _safe_float(match.group(4)))

    def resolve_value(val_str: str) -> Optional[float]:
        cleaned = val_str.strip()
        if cleaned.startswith("(") and cleaned.endswith(")"):
            cleaned = cleaned[1:-1].strip()
        if re.fullmatch(FLOAT_RE, cleaned):
            return _safe_float(cleaned)
        expr_match = re.fullmatch(rf"(.+?)\s*([+-])\s*({FLOAT_RE})", cleaned)
        if expr_match:
            base = resolve_value(expr_match.group(1))
            if base is not None:
                delta = _safe_float(expr_match.group(3))
                return base + delta if expr_match.group(2) == "+" else base - delta
        if "." in cleaned:
            parts = cleaned.split(".")
            base_var = parts[0].split("::")[-1]
            field = parts[-1]
            source = location_vars_map.get(base_var) or vars_map.get(base_var)
            if source:
                lat, lon, alt = source
                if "lat" in field: return lat
                if "lon" in field: return lon
                if "alt" in field: return alt
        return None

    path_assignments: Dict[str, Dict[str, float]] = defaultdict(dict)
    assign_pattern = re.compile(
        r"([\w]+(?:\[\d+\])?)\.(lat_from|lon_from|alt_from|lat_to|lon_to)\s*=\s*([^;]+)\s*;",
        re.MULTILINE,
    )
    for match in assign_pattern.finditer(src):
        val = resolve_value(match.group(3))
        if val is not None:
            path_assignments[match.group(1)][match.group(2)] = val

    paths: List[PathCheck] = []
    for name, data in path_assignments.items():
        if all(k in data for k in ("lat_from", "lon_from", "lat_to", "lon_to")):
            paths.append(PathCheck(
                name=name, lat_from=data["lat_from"], lon_from=data["lon_from"],
                alt_from=data.get("alt_from", 0.0), lat_to=data["lat_to"], lon_to=data["lon_to"]))
    return paths


def _parse_projections(src: str) -> List[Projection]:
    """Parse MakeProjection() calls."""
    projections: List[Projection] = []
    proj_pattern = re.compile(
        rf"MakeProjection\s*\(\s*[^,]+,\s*[^,]+,\s*[^,]+,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})",
        re.DOTALL | re.MULTILINE,
    )
    for i, match in enumerate(proj_pattern.finditer(src)):
        projections.append(Projection(
            _safe_float(match.group(1)), _safe_float(match.group(2)),
            _safe_float(match.group(3)), f"Proj_{i}"))
    return projections


def _parse_bank_turn_scenarios(
    src: str,
    vars_map: Dict[str, Tuple[float, float, float]],
) -> List[BankTurnZone]:
    """Parse BankTurnGeofenceScenario arrays (current and legacy formats)."""
    bank_turn_zones: List[BankTurnZone] = []

    scenario_array_pattern = re.compile(
        r"BankTurnGeofenceScenario\s+\w+\s*\[\s*\]\s*=\s*\{(.*?)\};",
        re.DOTALL | re.MULTILINE,
    )
    scenario_item_pattern = re.compile(
        rf"\{{\s*\"([^\"]+)\"\s*,\s*({LATLON_TOKEN_RE})\s*,\s*({LATLON_TOKEN_RE})\s*,"
        rf"\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*(true|false)\s*,?\s*\}}",
        re.MULTILINE,
    )
    scenario_item_legacy_pattern = re.compile(
        rf"\{{\s*\"([^\"]+)\"\s*,\s*({LATLON_TOKEN_RE})\s*,"
        rf"\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*(true|false)\s*,?\s*\}}",
        re.MULTILINE,
    )

    def _make_zone(
        name: str,
        turn_point: Tuple[float, float, float],
        current_location: Tuple[float, float, float],
        base_radius: float,
        wind_n: float,
        wind_e: float,
        max_wind: float,
        expected_safe: bool,
    ) -> BankTurnZone:
        center, radius_m, polygon = _compute_bank_turn_geometry(
            turn_point[0], turn_point[1], turn_point[2],
            base_radius, wind_n, wind_e, max_wind, BANK_TURN_FEASIBILITY_SAMPLES)
        return BankTurnZone(
            name=name, turn_point=turn_point, current_location=current_location,
            center=center, polygon=polygon, radius_m=radius_m,
            base_radius_m=base_radius, wind_north_m_s=wind_n,
            wind_east_m_s=wind_e, max_wind_m_s=max_wind, expected_safe=expected_safe)

    for block in scenario_array_pattern.finditer(src):
        content = block.group(1)

        # Current format: name, turn_point, current_location, radius, wind_n, wind_e, max_wind, safe
        for match in scenario_item_pattern.finditer(content):
            turn_point = _parse_latlon_token(match.group(2), vars_map)
            current_location = _parse_latlon_token(match.group(3), vars_map)
            if not turn_point or not current_location:
                continue
            bank_turn_zones.append(_make_zone(
                match.group(1), turn_point, current_location,
                _safe_float(match.group(4)), _safe_float(match.group(5)),
                _safe_float(match.group(6)), _safe_float(match.group(7)),
                match.group(8) == "true"))

        # Legacy format: name, turn_point, radius, wind_n, wind_e, max_wind, safe
        for match in scenario_item_legacy_pattern.finditer(content):
            turn_point = _parse_latlon_token(match.group(2), vars_map)
            if not turn_point:
                continue
            bank_turn_zones.append(_make_zone(
                match.group(1), turn_point, turn_point,
                _safe_float(match.group(3)), _safe_float(match.group(4)),
                _safe_float(match.group(5)), _safe_float(match.group(6)),
                match.group(7) == "true"))

    return bank_turn_zones


def parse_cpp_code(text: str) -> ParseResult:
    """Parse C++ unit-test source into structured visualization data.

    Delegates to focused sub-parsers for each data type, then returns
    the combined result tuple.
    """
    src = text or ""

    # 1. Variable definitions (needed by several downstream parsers)
    vars_map, float_vars_map, latlon_points = _extract_variable_definitions(src)

    # 2. Missions
    missions = _parse_missions(src)

    # 3. Coordinate arrays (shared by polygons & rally points)
    coord_arrays = _parse_coord_arrays(src)

    # 4. Rally points
    rally_points = _parse_rally_points(src, coord_arrays)

    # 5. Polygons
    polygons = _parse_polygons(src, coord_arrays)

    # 6. Circle fences
    circles = _parse_circles(src, vars_map, float_vars_map)

    # 7. Vehicle locations
    vehicles = _parse_vehicles(src, float_vars_map)

    # 8. Additional LatLonAlt points (positions, offsets, RtlRoutePlanner structs)
    latlon_points.extend(_parse_latlon_points(src, vars_map))

    # 9. Path checks
    paths = _parse_paths(src, float_vars_map, vars_map)

    # 10. Projections
    projections = _parse_projections(src)

    # 11. Bank turn scenarios
    bank_turn_zones = _parse_bank_turn_scenarios(src, vars_map)

    return missions, polygons, circles, vehicles, latlon_points, paths, rally_points, bank_turn_zones, projections


# ==========================================
# 5. VISUALIZATION (Folium & PyDeck)
# ==========================================

def get_viewer_center(parsed: Optional[ParseResult]) -> Tuple[float, float]:
    if not parsed:
        return DEFAULT_CENTER

    missions, polygons, circles, vehicles, latlon_points, paths, rally, bank_turn_zones, proj = parsed
    # Priority order for centering the map
    if vehicles: return vehicles[0].lat, vehicles[0].lon
    if bank_turn_zones: return bank_turn_zones[0].center[0], bank_turn_zones[0].center[1]
    if latlon_points: return latlon_points[0].lat, latlon_points[0].lon
    if paths: return paths[0].lat_from, paths[0].lon_from
    if missions:
        m = next(iter(missions.values()))
        if m: return m[0].lat, m[0].lon
    if polygons and polygons[0].vertices:
        return polygons[0].vertices[0][0], polygons[0].vertices[0][1]
    if circles: return circles[0].center[0], circles[0].center[1]
    if rally: return rally[0].lat, rally[0].lon
    return DEFAULT_CENTER


def create_3d_map(parsed: ParseResult) -> pdk.Deck:
    missions, polygons, circles, vehicles, latlon_points, paths, rally, bank_turn_zones, proj = parsed
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

    # Bank turn zones
    zone_poly_data = []
    zone_marker_data = []
    zone_line_data = []
    for zone in bank_turn_zones:
        center, _, polygon = _compute_bank_turn_geometry_for_zone(zone, BANK_TURN_FEASIBILITY_SAMPLES)
        base_color = [0, 120, 255] if zone.expected_safe else [255, 140, 0]
        fill_color = [base_color[0], base_color[1], base_color[2], 80]
        contour = [[pt[1], pt[0], pt[2]] for pt in polygon]
        info = (
            f"{zone.name} (base {zone.base_radius_m:.1f}m, "
            f"wind {zone.wind_north_m_s:.1f}/{zone.wind_east_m_s:.1f} m/s, "
            f"max {zone.max_wind_m_s:.1f} m/s)"
        )
        if contour:
            zone_poly_data.append({
                "contour": contour,
                "color": fill_color,
                "info": info,
                "name": zone.name,
            })
        zone_marker_data.append({
            "pos": [zone.turn_point[1], zone.turn_point[0], zone.turn_point[2]],
            "c": base_color,
            "info": f"{zone.name} turn point",
            "name": zone.name,
        })
        if (zone.current_location[0], zone.current_location[1]) != (zone.turn_point[0], zone.turn_point[1]):
            zone_marker_data.append({
                "pos": [zone.current_location[1], zone.current_location[0], zone.current_location[2]],
                "c": base_color,
                "info": f"{zone.name} current location",
                "name": zone.name,
            })
        zone_marker_data.append({
            "pos": [center[1], center[0], center[2]],
            "c": base_color,
            "info": f"{zone.name} shifted center",
            "name": zone.name,
        })
        if (center[0], center[1]) != (zone.turn_point[0], zone.turn_point[1]):
            zone_line_data.append({
                "source": [zone.turn_point[1], zone.turn_point[0], zone.turn_point[2]],
                "target": [center[1], center[0], center[2]],
                "c": base_color,
                "info": f"{zone.name} shift",
                "name": zone.name,
            })
    if zone_poly_data:
        layers.append(pdk.Layer(
            "PolygonLayer", zone_poly_data, get_polygon="contour",
            get_fill_color="color", get_line_color="color",
            pickable=True, extruded=False, line_width_min_pixels=1
        ))
    if zone_line_data:
        layers.append(pdk.Layer(
            "LineLayer", zone_line_data, get_source_position="source",
            get_target_position="target", get_color="c", pickable=True,
            width_min_pixels=2
        ))
    if zone_marker_data:
        layers.append(pdk.Layer(
            "ScatterplotLayer", zone_marker_data, get_position="pos", get_fill_color="c",
            get_radius=8, pickable=True
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


def _add_bank_turn_zone_to_map(
    m: folium.Map,
    turn_point: Tuple[float, float, float],
    current_location: Optional[Tuple[float, float, float]],
    center: Tuple[float, float, float],
    polygon: Sequence[Tuple[float, float, float]],
    color: str,
    label: str,
    details: str,
    *,
    weight: int = 2,
    fill_opacity: float = 0.15,
) -> None:
    if polygon:
        folium.Polygon(
            locations=[(p[0], p[1]) for p in polygon],
            color=color,
            fill=True,
            fill_opacity=fill_opacity,
            weight=weight,
            smooth_factor=0.0,
            popup=f"{label}\n{details}",
        ).add_to(m)

    if (turn_point[0], turn_point[1]) != (center[0], center[1]):
        folium.PolyLine(
            locations=[(turn_point[0], turn_point[1]), (center[0], center[1])],
            color=color,
            weight=weight,
            dash_array="4, 6",
            tooltip=f"{label}\nShift vector",
        ).add_to(m)

    folium.Marker(
        location=(turn_point[0], turn_point[1]),
        icon=folium.Icon(color=color, icon="times", prefix="fa"),
        tooltip=f"{label}\nTurn point\nAlt: {turn_point[2]:.1f}m",
    ).add_to(m)

    folium.CircleMarker(
        location=(center[0], center[1]),
        radius=3,
        color=color,
        fill=True,
        fill_opacity=1.0,
        tooltip=f"{label}\nShifted center\nAlt: {center[2]:.1f}m",
    ).add_to(m)
    if current_location and (current_location[0], current_location[1]) != (turn_point[0], turn_point[1]):
        folium.CircleMarker(
            location=(current_location[0], current_location[1]),
            radius=5,
            color=color,
            fill=True,
            fill_opacity=1.0,
            tooltip=f"{label}\nCurrent location\nAlt: {current_location[2]:.1f}m",
        ).add_to(m)


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
    missions, polygons, circles, vehicles, latlon_points, paths, rally, bank_turn_zones, projections = parsed

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

    # Bank turn zones
    for zone in bank_turn_zones:
        center, _, polygon = _compute_bank_turn_geometry_for_zone(zone, BANK_TURN_FEASIBILITY_SAMPLES)
        c = "blue" if zone.expected_safe else "orange"
        label = f"PARSED: BankTurn {zone.name}"
        details = (
            f"Base {zone.base_radius_m:.1f}m | "
            f"Wind {zone.wind_north_m_s:.1f}/{zone.wind_east_m_s:.1f} m/s | "
            f"Max {zone.max_wind_m_s:.1f} m/s"
        )
        _add_bank_turn_zone_to_map(
            m,
            zone.turn_point,
            zone.current_location,
            center,
            polygon,
            c,
            label,
            details,
        )

    # Missions
    colors = ["blue", "orange", "purple", "black"]
    for i, (name, waypoints) in enumerate(missions.items()):
        pts = [(wp.lat, wp.lon) for wp in waypoints]
        c = colors[i % len(colors)]
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
    pending_bank_turn_current: Optional[Tuple[float, float, float]],
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

        if mode == BANK_TURN_MODE:
            bank_turn_points, _ = _normalize_bank_turn_points(points)
            for i, pt in enumerate(bank_turn_points):
                center, _, polygon = _compute_bank_turn_geometry(
                    pt.turn_lat,
                    pt.turn_lon,
                    pt.turn_alt,
                    pt.base_radius_m,
                    pt.wind_north_m_s,
                    pt.wind_east_m_s,
                    pt.max_wind_m_s,
                    BANK_TURN_FEASIBILITY_SAMPLES,
                )
                c = "blue" if pt.expected_safe else "orange"
                label = f"NEW: BankTurn Pt{i}"
                details = (
                    f"Base {pt.base_radius_m:.1f}m | "
                    f"Wind {pt.wind_north_m_s:.1f}/{pt.wind_east_m_s:.1f} m/s | "
                    f"Max {pt.max_wind_m_s:.1f} m/s"
                )
                _add_bank_turn_zone_to_map(
                    m,
                    (pt.turn_lat, pt.turn_lon, pt.turn_alt),
                    (pt.current_lat, pt.current_lon, pt.current_alt),
                    center,
                    polygon,
                    c,
                    label,
                    details,
                    weight=weight,
                    fill_opacity=0.2 if is_active else 0.1,
                )
            if is_active and pending_bank_turn_current:
                lat, lon, alt = pending_bank_turn_current
                folium.CircleMarker(
                    location=(lat, lon),
                    radius=6,
                    color=color,
                    fill=True,
                    fill_opacity=1.0,
                    tooltip=f"NEW: BankTurn Current\nAlt: {alt:.1f}m",
                ).add_to(m)
            continue

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
            cmd = "NAV_CMD_TAKEOFF" if i == 0 else ("NAV_CMD_LAND" if i == len(m_pts) - 1 else "NAV_CMD_WAYPOINT")
            lines.append(f"    makePositionItem({lat:.7f}, {lon:.7f}, {alt:.1f}f, {cmd}),  // {i}")
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

    # 3.5 Bank Turn Scenarios
    b_pts_raw = points_by_mode.get(BANK_TURN_MODE, [])
    b_pts, _ = _normalize_bank_turn_points(b_pts_raw)
    if b_pts:
        lines = [f"// --- Bank Turn Scenarios ({len(b_pts)} items) ---"]
        var = sanitize_cpp_identifier(base_name + "_bank_turn")
        lines.append(f"static constexpr BankTurnGeofenceScenario {var}[] = {{")
        for i, pt in enumerate(b_pts):
            name = base_name if len(b_pts) == 1 else f"{base_name}_{i}"
            lines.append("    {")
            lines.append(f'        "{name}",')
            lines.append(f"        {{{pt.turn_lat:.7f}, {pt.turn_lon:.7f}, {pt.turn_alt:.1f}f}},")
            lines.append(f"        {{{pt.current_lat:.7f}, {pt.current_lon:.7f}, {pt.current_alt:.1f}f}},")
            lines.append(f"        {pt.base_radius_m:.1f}f,")
            lines.append(f"        {pt.wind_north_m_s:.1f}f,")
            lines.append(f"        {pt.wind_east_m_s:.1f}f,")
            lines.append(f"        {pt.max_wind_m_s:.1f}f,")
            lines.append(f"        {'true' if pt.expected_safe else 'false'},")
            lines.append("    },")
        lines.append("};")
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
    if "bank_turn_pending_current" not in st.session_state:
        st.session_state["bank_turn_pending_current"] = None

    # Normalize stored points to the current format.
    for m in CREATOR_MODES:
        pts = st.session_state["creator_points"][m]
        if not pts:
            continue
        if m == BANK_TURN_MODE:
            normalized, dropped = _normalize_bank_turn_points(pts)
            st.session_state["creator_points"][m] = normalized
            if dropped:
                st.warning(f"Dropped {dropped} {m} point(s) due to incompatible format.")
        elif m == "Vehicle Location":
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
    if "last_wind_north_m_s" not in st.session_state:
        st.session_state["last_wind_north_m_s"] = 0.0
    if "last_wind_east_m_s" not in st.session_state:
        st.session_state["last_wind_east_m_s"] = 0.0
    if "last_max_wind_m_s" not in st.session_state:
        st.session_state["last_max_wind_m_s"] = BANK_TURN_MAX_WIND_FALLBACK_M_S
    if "last_expected_safe" not in st.session_state:
        st.session_state["last_expected_safe"] = True
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
            is_bank_turn = (mode == BANK_TURN_MODE)
            is_vehicle = (mode == "Vehicle Location")
            current_vel_north = 0.0
            current_vel_east = 0.0
            c1, c2 = st.columns(2)
            with c1:
                current_alt = st.number_input("Altitude (m)", value=500.0, help="Applies to next click")
            with c2:
                # Radius relevant for circles and bank turns.
                disable_rad = ("Circle" not in mode) and (mode != BANK_TURN_MODE)
                rad_label = "Base Radius (m)" if is_bank_turn else "Radius (m)"
                current_rad = st.number_input(rad_label, value=st.session_state["last_radius"], disabled=disable_rad, help="Applies to next click")
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

            if is_bank_turn:
                w1, w2 = st.columns(2)
                with w1:
                    wind_north = st.number_input(
                        "Wind North (m/s)",
                        value=st.session_state["last_wind_north_m_s"],
                        help="Applies to next click",
                    )
                    st.session_state["last_wind_north_m_s"] = wind_north
                with w2:
                    wind_east = st.number_input(
                        "Wind East (m/s)",
                        value=st.session_state["last_wind_east_m_s"],
                        help="Applies to next click",
                    )
                    st.session_state["last_wind_east_m_s"] = wind_east

                max_wind = st.number_input(
                    "Max Wind (m/s)",
                    value=st.session_state["last_max_wind_m_s"],
                    help="Applies to next click",
                )
                st.session_state["last_max_wind_m_s"] = max_wind

                expected_safe = st.checkbox(
                    "Expected Safe",
                    value=st.session_state["last_expected_safe"],
                    help="Applies to next click",
                )
                st.session_state["last_expected_safe"] = expected_safe

            st.markdown("---")

            # Action Buttons
            pts_map = st.session_state["creator_points"]
            curr_len = len(pts_map[mode])

            if st.button(f"Clear New {mode} ({curr_len})", use_container_width=True):
                pts_map[mode] = []
                if mode == BANK_TURN_MODE:
                    st.session_state["bank_turn_pending_current"] = None
                st.rerun()

            if st.button("Clear ALL New Items", type="secondary", use_container_width=True):
                for m in CREATOR_MODES:
                    pts_map[m] = []
                st.session_state["bank_turn_pending_current"] = None
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
                st.session_state.get("bank_turn_pending_current"),
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
                if mode == BANK_TURN_MODE:
                    pending = st.session_state.get("bank_turn_pending_current")
                    if pending is None:
                        st.session_state["bank_turn_pending_current"] = (
                            round(lc["lat"], 7),
                            round(lc["lng"], 7),
                            current_alt,
                        )
                        st.rerun()
                    else:
                        pt = BankTurnCreatorPoint(
                            current_lat=pending[0],
                            current_lon=pending[1],
                            current_alt=pending[2],
                            turn_lat=round(lc["lat"], 7),
                            turn_lon=round(lc["lng"], 7),
                            turn_alt=current_alt,
                            base_radius_m=current_rad,
                            wind_north_m_s=st.session_state["last_wind_north_m_s"],
                            wind_east_m_s=st.session_state["last_wind_east_m_s"],
                            max_wind_m_s=st.session_state["last_max_wind_m_s"],
                            expected_safe=st.session_state["last_expected_safe"],
                        )
                        st.session_state["creator_points"][mode].append(pt)
                        st.session_state["bank_turn_pending_current"] = None
                        st.rerun()
                else:
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
                        # Construct 4D point: Lat, Lon, Alt, Radius
                        pt_4d = (round(lc["lat"], 7), round(lc["lng"], 7), current_alt, current_rad)

                        # Check existence by matching lat/lon only
                        already_exists = any(
                            (p[0] == pt_4d[0] and p[1] == pt_4d[1]) for p in current_list
                        )

                        if not already_exists:
                            if "Polygon" in mode:
                                # Use smart insertion for polygons
                                st.session_state["creator_points"][mode] = _insert_point_on_polygon(current_list, pt_4d)
                            else:
                                # Append for missions, paths, etc.
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
