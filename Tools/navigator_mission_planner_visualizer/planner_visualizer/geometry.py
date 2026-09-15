"""Local tangent plane math, matching add_vector_to_global_position() in lib/geo."""

import math
from typing import List, Sequence, Tuple

EARTH_RADIUS_M = 6371000.0

LatLon = Tuple[float, float]


def offset_to_latlon(ref_lat: float, ref_lon: float, north_m: float, east_m: float) -> LatLon:
    """Move north_m/east_m away from a reference, exactly like the C++ helpers do."""
    lat = math.degrees(math.radians(ref_lat) + north_m / EARTH_RADIUS_M)
    lon = math.degrees(
        math.radians(ref_lon) + east_m / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat)))
    )
    return lat, lon


def latlon_to_offset(ref_lat: float, ref_lon: float, lat: float, lon: float) -> Tuple[float, float]:
    """Inverse of offset_to_latlon(): north and east metres from the reference."""
    north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    east = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    return north, east


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    return math.hypot(*latlon_to_offset(lat1, lon1, lat2, lon2))


def snap_to_grid(ref_lat: float, ref_lon: float, lat: float, lon: float, grid_m: float) -> LatLon:
    """Round the offset from the reference to whole grid cells."""
    if grid_m <= 0:
        return lat, lon
    north, east = latlon_to_offset(ref_lat, ref_lon, lat, lon)
    return offset_to_latlon(
        ref_lat, ref_lon, round(north / grid_m) * grid_m, round(east / grid_m) * grid_m
    )


def velocity_arrow(lat: float, lon: float, vel_n: float, vel_e: float) -> List[Tuple[LatLon, LatLon]]:
    """Shaft and two head strokes pointing along the velocity, as lat/lon segments."""
    speed = math.hypot(vel_n, vel_e)
    if not math.isfinite(speed) or speed < 0.1:
        return []
    # 2 m per m/s, clamped so the arrow stays visible at any test scale.
    length = min(max(2.0 * speed, 10.0), 200.0)
    head = min(max(0.25 * length, 5.0), 25.0)
    dir_n, dir_e = vel_n / speed, vel_e / speed
    tip = offset_to_latlon(lat, lon, dir_n * length, dir_e * length)
    segments = [((lat, lon), tip)]
    for sign in (1.0, -1.0):
        angle = sign * math.radians(25.0)
        # Each head stroke is the reversed direction rotated by +/- 25 degrees.
        head_n = -dir_n * math.cos(angle) - dir_e * math.sin(angle)
        head_e = -dir_e * math.cos(angle) + dir_n * math.sin(angle)
        segments.append((tip, offset_to_latlon(tip[0], tip[1], head_n * head, head_e * head)))
    return segments


def fit_view(points: Sequence[LatLon], map_height_px: int = 620) -> Tuple[LatLon, int]:
    """Center and web mercator zoom showing every point with some margin."""
    lats = [point[0] for point in points]
    lons = [point[1] for point in points]
    center = ((min(lats) + max(lats)) / 2, (min(lons) + max(lons)) / 2)
    span = max(
        distance_m(min(lats), center[1], max(lats), center[1]),
        distance_m(center[0], min(lons), center[0], max(lons)),
        50.0,
    )
    # 156543 m per pixel at zoom 0 on the equator, halving with every zoom level.
    meters_per_px = 156543.03 * math.cos(math.radians(center[0]))
    zoom = math.log2(meters_per_px * map_height_px * 0.6 / span)
    return center, int(max(3, min(19, zoom)))
