"""Offset math must match the C++ helpers, and the view must fit the fixtures."""

import math
import unittest

from planner_visualizer.geometry import (
    EARTH_RADIUS_M,
    distance_m,
    fit_view,
    latlon_to_offset,
    offset_to_latlon,
    snap_to_grid,
    velocity_arrow,
)


class OffsetTest(unittest.TestCase):
    def test_offsets_follow_add_vector_to_global_position(self):
        lat, lon = offset_to_latlon(47.397742, 8.545594, 200.0, 100.0)
        self.assertAlmostEqual(lat, 47.397742 + math.degrees(200.0 / EARTH_RADIUS_M))
        self.assertAlmostEqual(
            lon, 8.545594 + math.degrees(100.0 / (EARTH_RADIUS_M * math.cos(math.radians(47.397742))))
        )

    def test_offsets_round_trip(self):
        lat, lon = offset_to_latlon(47.397742, 8.545594, -350.0, 1234.5)
        north, east = latlon_to_offset(47.397742, 8.545594, lat, lon)
        self.assertAlmostEqual(north, -350.0, places=6)
        self.assertAlmostEqual(east, 1234.5, places=6)
        self.assertAlmostEqual(distance_m(47.397742, 8.545594, lat, lon), math.hypot(350.0, 1234.5), places=5)

    def test_snapping(self):
        lat, lon = offset_to_latlon(47.0, 8.0, 203.7, -14.9)
        north, east = latlon_to_offset(47.0, 8.0, *snap_to_grid(47.0, 8.0, lat, lon, 10.0))
        self.assertEqual((round(north, 6), round(east, 6)), (200.0, -10.0))
        self.assertEqual(snap_to_grid(47.0, 8.0, lat, lon, 0.0), (lat, lon))


class ArrowTest(unittest.TestCase):
    def test_arrow_points_along_the_velocity_and_has_two_head_strokes(self):
        segments = velocity_arrow(47.0, 8.0, 10.0, 0.0)
        self.assertEqual(len(segments), 3)
        (tail, tip), (head_start, left), (_, right) = segments
        self.assertEqual(tail, (47.0, 8.0))
        self.assertEqual(head_start, tip)
        north, east = latlon_to_offset(47.0, 8.0, *tip)
        self.assertAlmostEqual(north, 20.0, places=6)
        self.assertAlmostEqual(east, 0.0, places=6)
        # Both head strokes go back from the tip, one to each side.
        self.assertLess(latlon_to_offset(*tip, *left)[0], 0.0)
        self.assertLess(latlon_to_offset(*tip, *right)[0], 0.0)
        self.assertLess(latlon_to_offset(*tip, *left)[1] * latlon_to_offset(*tip, *right)[1], 0.0)

    def test_arrow_is_clamped_and_absent_when_stopped(self):
        _, tip = velocity_arrow(47.0, 8.0, 0.0, 500.0)[0]
        self.assertAlmostEqual(latlon_to_offset(47.0, 8.0, *tip)[1], 200.0, places=5)
        self.assertEqual(velocity_arrow(47.0, 8.0, 0.0, 0.0), [])
        self.assertEqual(velocity_arrow(47.0, 8.0, float("nan"), 1.0), [])


class ViewTest(unittest.TestCase):
    def test_view_is_centered_and_zoomed_to_the_span(self):
        center, zoom = fit_view([(47.0, 8.0), (47.01, 8.0), (47.0, 8.02)])
        self.assertAlmostEqual(center[0], 47.005)
        self.assertAlmostEqual(center[1], 8.01)
        self.assertTrue(13 <= zoom <= 15, zoom)
        _, tight = fit_view([(47.0, 8.0), (47.0001, 8.0)])
        _, wide = fit_view([(46.0, 7.0), (48.0, 9.0)])
        self.assertGreater(tight, zoom)
        self.assertLess(wide, zoom)
        self.assertEqual(fit_view([(47.0, 8.0)]), ((47.0, 8.0), 19))


if __name__ == "__main__":
    unittest.main()
