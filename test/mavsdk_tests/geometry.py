"""Coordinate and vector helpers for the MAVSDK integration tests.

CoordinateTransformation is a port of mavsdk::geometry from MAVSDK C++ (azimuthal equidistant
projection around a reference point).
"""

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]

WORLD_RADIUS_M = 6371000.0


class CoordinateTransformation:

    def __init__(self, ref_latitude_deg: float, ref_longitude_deg: float) -> None:
        self._ref_lat_rad = math.radians(ref_latitude_deg)
        self._ref_lon_rad = math.radians(ref_longitude_deg)

    def local_from_global(self, latitude_deg: float, longitude_deg: float) -> Tuple[float, float]:
        """Return (north_m, east_m) of a global position."""
        lat_rad = math.radians(latitude_deg)
        lon_rad = math.radians(longitude_deg)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        cos_d_lon = math.cos(lon_rad - self._ref_lon_rad)

        ref_sin_lat = math.sin(self._ref_lat_rad)
        ref_cos_lat = math.cos(self._ref_lat_rad)

        arg = ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon
        c = math.acos(min(max(arg, -1.0), 1.0))
        k = c / math.sin(c) if abs(c) > 0 else 1.0

        north_m = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * WORLD_RADIUS_M
        east_m = k * cos_lat * math.sin(lon_rad - self._ref_lon_rad) * WORLD_RADIUS_M
        return north_m, east_m

    def global_from_local(self, north_m: float, east_m: float) -> Tuple[float, float]:
        """Return (latitude_deg, longitude_deg) of a local position."""
        x_rad = north_m / WORLD_RADIUS_M
        y_rad = east_m / WORLD_RADIUS_M
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

        if abs(c) == 0:
            return math.degrees(self._ref_lat_rad), math.degrees(self._ref_lon_rad)

        sin_c = math.sin(c)
        cos_c = math.cos(c)
        ref_sin_lat = math.sin(self._ref_lat_rad)
        ref_cos_lat = math.cos(self._ref_lat_rad)

        lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
        lon_rad = self._ref_lon_rad + math.atan2(
            y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c)
        return math.degrees(lat_rad), math.degrees(lon_rad)


def norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def diff(a: Vec3, b: Vec3) -> Vec3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def normalized(v: Vec3) -> Vec3:
    n = norm(v)
    if n > 1e-6:
        return v[0] / n, v[1] / n, v[2] / n
    return 0.0, 0.0, 0.0


def point_to_line_distance(point: Vec3, line_start: Vec3, line_end: Vec3) -> float:
    """Shortest distance from a point to the infinite line through two points."""
    direction = normalized(diff(line_end, line_start))
    t = dot(direction, diff(point, line_start))
    closest = (line_start[0] + t * direction[0],
               line_start[1] + t * direction[1],
               line_start[2] + t * direction[2])
    return norm(diff(closest, point))
