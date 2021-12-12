/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file geo.c
 *
 * Geo / math functions to perform geodesic calculations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "geo.h"

#include <float.h>

using matrix::wrap_pi;
using matrix::wrap_2pi;

#ifndef hrt_absolute_time
# define hrt_absolute_time() (0)
#endif

/*
 * Azimuthal Equidistant Projection
 * formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html
 */

void MapProjection::initReference(double lat_0, double lon_0, uint64_t timestamp)
{
	_ref_timestamp = timestamp;
	_ref_lat = math::radians(lat_0);
	_ref_lon = math::radians(lon_0);
	_ref_sin_lat = sin(_ref_lat);
	_ref_cos_lat = cos(_ref_lat);
	_ref_init_done = true;
}

void MapProjection::project(double lat, double lon, float &x, float &y) const
{
	const double lat_rad = math::radians(lat);
	const double lon_rad = math::radians(lon);

	const double sin_lat = sin(lat_rad);
	const double cos_lat = cos(lat_rad);

	const double cos_d_lon = cos(lon_rad - _ref_lon);

	const double arg = math::constrain(_ref_sin_lat * sin_lat + _ref_cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
	const double c = acos(arg);

	double k = 1.0;

	if (fabs(c) > 0) {
		k = (c / sin(c));
	}

	x = static_cast<float>(k * (_ref_cos_lat * sin_lat - _ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	y = static_cast<float>(k * cos_lat * sin(lon_rad - _ref_lon) * CONSTANTS_RADIUS_OF_EARTH);
}

void MapProjection::reproject(float x, float y, double &lat, double &lon) const
{
	const double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
	const double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
	const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

	if (fabs(c) > 0) {
		const double sin_c = sin(c);
		const double cos_c = cos(c);

		const double lat_rad = asin(cos_c * _ref_sin_lat + (x_rad * sin_c * _ref_cos_lat) / c);
		const double lon_rad = (_ref_lon + atan2(y_rad * sin_c, c * _ref_cos_lat * cos_c - x_rad * _ref_sin_lat * sin_c));

		lat = math::degrees(lat_rad);
		lon = math::degrees(lon_rad);

	} else {
		lat = math::degrees(_ref_lat);
		lon = math::degrees(_ref_lon);
	}
}

float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = math::radians(lat_now);
	const double lat_next_rad = math::radians(lat_next);

	const double d_lat = lat_next_rad - lat_now_rad;
	const double d_lon = math::radians(lon_next) - math::radians(lon_now);

	const double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat_now_rad) * cos(
				 lat_next_rad);

	const double c = atan2(sqrt(a), sqrt(1.0 - a));

	return static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * 2.0 * c);
}

void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
					double *lat_target, double *lon_target)
{
	if (fabsf(dist) < FLT_EPSILON) {
		*lat_target = lat_A;
		*lon_target = lon_A;

	} else {
		float heading = get_bearing_to_next_waypoint(lat_A, lon_A, lat_B, lon_B);
		waypoint_from_heading_and_distance(lat_A, lon_A, heading, dist, lat_target, lon_target);
	}
}

void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
					double *lat_target, double *lon_target)
{
	bearing = wrap_2pi(bearing);
	double radius_ratio = static_cast<double>(dist) / CONSTANTS_RADIUS_OF_EARTH;

	double lat_start_rad = math::radians(lat_start);
	double lon_start_rad = math::radians(lon_start);

	*lat_target = asin(sin(lat_start_rad) * cos(radius_ratio) + cos(lat_start_rad) * sin(radius_ratio) * cos((
				   double)bearing));
	*lon_target = lon_start_rad + atan2(sin((double)bearing) * sin(radius_ratio) * cos(lat_start_rad),
					    cos(radius_ratio) - sin(lat_start_rad) * sin(*lat_target));

	*lat_target = math::degrees(*lat_target);
	*lon_target = math::degrees(*lon_target);
}

float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = math::radians(lat_now);
	const double lat_next_rad = math::radians(lat_next);

	const double cos_lat_next = cos(lat_next_rad);
	const double d_lon = math::radians(lon_next - lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	const float y = static_cast<float>(sin(d_lon) * cos_lat_next);
	const float x = static_cast<float>(cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon));

	return wrap_pi(atan2f(y, x));
}

void
get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n, float *v_e)
{
	const double lat_now_rad = math::radians(lat_now);
	const double lat_next_rad = math::radians(lat_next);
	const double d_lon = math::radians(lon_next) - math::radians(lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(
					  lat_next_rad) * cos(d_lon)));
	*v_e = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * sin(d_lon) * cos(lat_next_rad));
}

void
get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n,
				 float *v_e)
{
	double lat_now_rad = math::radians(lat_now);
	double lon_now_rad = math::radians(lon_now);
	double lat_next_rad = math::radians(lat_next);
	double lon_next_rad = math::radians(lon_next);

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * d_lat);
	*v_e = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * d_lon * cos(lat_now_rad));
}

void add_vector_to_global_position(double lat_now, double lon_now, float v_n, float v_e, double *lat_res,
				   double *lon_res)
{
	double lat_now_rad = math::radians(lat_now);
	double lon_now_rad = math::radians(lon_now);

	*lat_res = math::degrees(lat_now_rad + (double)v_n / CONSTANTS_RADIUS_OF_EARTH);
	*lon_res = math::degrees(lon_now_rad + (double)v_e / (CONSTANTS_RADIUS_OF_EARTH * cos(lat_now_rad)));
}

// Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>

int get_distance_to_line(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
			 double lat_start, double lon_start, double lat_end, double lon_end)
{
	// This function returns the distance to the nearest point on the track line.  Distance is positive if current
	// position is right of the track and negative if left of the track as seen from a point on the track line
	// headed towards the end point.

	int return_value = -1;	// Set error flag, cleared when valid result calculated.
	crosstrack_error->past_end = false;
	crosstrack_error->distance = 0.0f;
	crosstrack_error->bearing = 0.0f;

	float dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

	// Return error if arguments are bad
	if (dist_to_end < 0.1f) {
		return -1;
	}

	float bearing_end = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
	float bearing_track = get_bearing_to_next_waypoint(lat_start, lon_start, lat_end, lon_end);
	float bearing_diff = wrap_pi(bearing_track - bearing_end);

	// Return past_end = true if past end point of line
	if (bearing_diff > M_PI_2_F || bearing_diff < -M_PI_2_F) {
		crosstrack_error->past_end = true;
		return_value = 0;
		return return_value;
	}

	crosstrack_error->distance = (dist_to_end) * sinf(bearing_diff);

	if (sinf(bearing_diff) >= 0) {
		crosstrack_error->bearing = wrap_pi(bearing_track - M_PI_2_F);

	} else {
		crosstrack_error->bearing = wrap_pi(bearing_track + M_PI_2_F);
	}

	return_value = 0;

	return return_value;
}

int get_distance_to_arc(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
			double lat_center, double lon_center,
			float radius, float arc_start_bearing, float arc_sweep)
{
	// This function returns the distance to the nearest point on the track arc.  Distance is positive if current
	// position is right of the arc and negative if left of the arc as seen from the closest point on the arc and
	// headed towards the end point.

	// Determine if the current position is inside or outside the sector between the line from the center
	// to the arc start and the line from the center to the arc end
	float bearing_sector_start = 0.0f;
	float bearing_sector_end = 0.0f;
	float bearing_now = get_bearing_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

	int return_value = -1;		// Set error flag, cleared when valid result calculated.
	crosstrack_error->past_end = false;
	crosstrack_error->distance = 0.0f;
	crosstrack_error->bearing = 0.0f;

	// Return error if arguments are bad
	if (radius < 0.1f) {
		return return_value;
	}

	if (arc_sweep >= 0.0f) {
		bearing_sector_start = arc_start_bearing;
		bearing_sector_end = arc_start_bearing + arc_sweep;

		if (bearing_sector_end > 2.0f * M_PI_F) { bearing_sector_end -= (2 * M_PI_F); }

	} else {
		bearing_sector_end = arc_start_bearing;
		bearing_sector_start = arc_start_bearing - arc_sweep;

		if (bearing_sector_start < 0.0f) { bearing_sector_start += (2 * M_PI_F); }
	}

	bool in_sector = false;

	// Case where sector does not span zero
	if (bearing_sector_end >= bearing_sector_start && bearing_now >= bearing_sector_start
	    && bearing_now <= bearing_sector_end) {

		in_sector = true;
	}

	// Case where sector does span zero
	if (bearing_sector_end < bearing_sector_start && (bearing_now > bearing_sector_start
			|| bearing_now < bearing_sector_end)) {

		in_sector = true;
	}

	// If in the sector then calculate distance and bearing to closest point
	if (in_sector) {
		crosstrack_error->past_end = false;
		float dist_to_center = get_distance_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

		if (dist_to_center <= radius) {
			crosstrack_error->distance = radius - dist_to_center;
			crosstrack_error->bearing = bearing_now + M_PI_F;

		} else {
			crosstrack_error->distance = dist_to_center - radius;
			crosstrack_error->bearing = bearing_now;
		}

		// If out of the sector then calculate dist and bearing to start or end point

	} else {

		// Use the approximation  that 111,111 meters in the y direction is 1 degree (of latitude)
		// and 111,111 * cos(latitude) meters in the x direction is 1 degree (of longitude) to
		// calculate the position of the start and end points.  We should not be doing this often
		// as this function generally will not be called repeatedly when we are out of the sector.

		double start_disp_x = (double)radius * sin((double)arc_start_bearing);
		double start_disp_y = (double)radius * cos((double)arc_start_bearing);
		double end_disp_x = (double)radius * sin((double)wrap_pi(arc_start_bearing + arc_sweep));
		double end_disp_y = (double)radius * cos((double)wrap_pi(arc_start_bearing + arc_sweep));
		double lon_start = lon_now + start_disp_x / 111111.0;
		double lat_start = lat_now + start_disp_y * cos(lat_now) / 111111.0;
		double lon_end = lon_now + end_disp_x / 111111.0;
		double lat_end = lat_now + end_disp_y * cos(lat_now) / 111111.0;
		float dist_to_start = get_distance_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);
		float dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

		if (dist_to_start < dist_to_end) {
			crosstrack_error->distance = dist_to_start;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);

		} else {
			crosstrack_error->past_end = true;
			crosstrack_error->distance = dist_to_end;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
		}
	}

	crosstrack_error->bearing = wrap_pi(crosstrack_error->bearing);
	return_value = 0;

	return return_value;
}

float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		double lat_next, double lon_next, float alt_next,
		float *dist_xy, float *dist_z)
{
	double current_x_rad = math::radians(lat_next);
	double current_y_rad = math::radians(lon_next);
	double x_rad = math::radians(lat_now);
	double y_rad = math::radians(lon_now);

	double d_lat = x_rad - current_x_rad;
	double d_lon = y_rad - current_y_rad;

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(current_x_rad) * cos(x_rad);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	const float dxy = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * c);
	const float dz = static_cast<float>(alt_now - alt_next);

	*dist_xy = fabsf(dxy);
	*dist_z = fabsf(dz);

	return sqrtf(dxy * dxy + dz * dz);
}

float mavlink_wpm_distance_to_point_local(float x_now, float y_now, float z_now,
		float x_next, float y_next, float z_next,
		float *dist_xy, float *dist_z)
{
	float dx = x_now - x_next;
	float dy = y_now - y_next;
	float dz = z_now - z_next;

	*dist_xy = sqrtf(dx * dx + dy * dy);
	*dist_z = fabsf(dz);

	return sqrtf(dx * dx + dy * dy + dz * dz);
}
