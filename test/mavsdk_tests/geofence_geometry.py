"""Fence shapes shared by the geofence and RTL flight tests."""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple

MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003
MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004

Local2 = Tuple[float, float]


def rectangle(north_min_m: float, north_max_m: float, east_min_m: float,
              east_max_m: float) -> List[Local2]:
    """Corners of a rectangle with its edges along north and east."""
    return [(north_min_m, east_min_m), (north_max_m, east_min_m),
            (north_max_m, east_max_m), (north_min_m, east_max_m)]


class FenceKind(Enum):
    POLYGON_INCLUSION = 0
    POLYGON_EXCLUSION = 1
    CIRCLE_INCLUSION = 2
    CIRCLE_EXCLUSION = 3


@dataclass
class FenceShape:
    kind: FenceKind
    vertices: List[Local2] = field(default_factory=list)  # polygons only
    center: Local2 = (0.0, 0.0)  # circles only
    radius_m: float = 0.0  # circles only

    def is_breached_by(self, north_m: float, east_m: float) -> bool:
        """A breach is being inside an exclusion or outside an inclusion shape."""
        if self.kind in (FenceKind.POLYGON_INCLUSION, FenceKind.POLYGON_EXCLUSION):
            if len(self.vertices) < 3:
                return False
            inside = point_in_polygon(north_m, east_m, self.vertices)
            return inside == (self.kind == FenceKind.POLYGON_EXCLUSION)

        distance_m = math.hypot(north_m - self.center[0], east_m - self.center[1])
        if self.kind == FenceKind.CIRCLE_EXCLUSION:
            return distance_m < self.radius_m
        return distance_m > self.radius_m

    def clearance_m(self, north_m: float, east_m: float) -> float:
        """Distance to the boundary, positive on the permitted side."""
        if self.kind in (FenceKind.CIRCLE_INCLUSION, FenceKind.CIRCLE_EXCLUSION):
            distance = abs(math.hypot(north_m - self.center[0], east_m - self.center[1])
                           - self.radius_m)
        else:
            assert len(self.vertices) >= 3, "A polygon needs at least three vertices"
            edges = zip(self.vertices, self.vertices[1:] + self.vertices[:1])
            distance = min(distance_to_segment((north_m, east_m), start, end)
                           for start, end in edges)
        return -distance if self.is_breached_by(north_m, east_m) else distance


def point_in_polygon(north_m: float, east_m: float, polygon: List[Local2]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        ni, ei = polygon[i]
        nj, ej = polygon[j]
        if (ei > east_m) != (ej > east_m) and north_m < (nj - ni) * (east_m - ei) / (ej - ei) + ni:
            inside = not inside
        j = i
    return inside


def distance_to_segment(point: Local2, start: Local2, end: Local2) -> float:
    dn, de = end[0] - start[0], end[1] - start[1]
    length_squared = dn * dn + de * de
    fraction = 0.0
    if length_squared > 0.0:
        fraction = ((point[0] - start[0]) * dn + (point[1] - start[1]) * de) / length_squared
        fraction = max(0.0, min(1.0, fraction))
    return math.hypot(point[0] - start[0] - fraction * dn, point[1] - start[1] - fraction * de)
