/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 */

#include <systemlib/geo/geo.h>
#include <math.h>

__EXPORT float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	double lat_now_rad = lat_now / 180.0d * M_PI;
	double lon_now_rad = lon_now / 180.0d * M_PI;
	double lat_next_rad = lat_next / 180.0d * M_PI;
	double lon_next_rad = lon_next / 180.0d * M_PI;


	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	double a = sin(d_lat / 2.0d) * sin(d_lat / 2.0) + sin(d_lon / 2.0d) * sin(d_lon / 2.0d) * cos(lat_now_rad) * cos(lat_next_rad);
	double c = 2.0d * atan2(sqrt(a), sqrt(1.0d - a));

	const double radius_earth = 6371000.0d;

	return radius_earth * c;
}

__EXPORT float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;
	double lat_next_rad = lat_next * M_DEG_TO_RAD;
	double lon_next_rad = lon_next * M_DEG_TO_RAD;

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	float theta = atan2f(sin(d_lon) * cos(lat_next_rad) , cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

	theta = _wrapPI(theta);

	return theta;
}

// Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>

__EXPORT crosstrack_error_s get_distance_to_line(double lat_now, double lon_now, double lat_start, double lon_start, double lat_end, double lon_end)
{
// This function returns the distance to the nearest point on the track line.  Distance is positive if current
// position is right of the track and negative if left of the track as seen from a point on the track line
// headed towards the end point.

	crosstrack_error_s return_var;
	float dist_to_end;
	float bearing_end;
	float bearing_track;
	float bearing_diff;

	return_var.error = true;		// Set error flag, cleared when valid result calculated.
	return_var.past_end = false;
	return_var.distance = 0.0f;
	return_var.bearing = 0.0f;

	// Return error if arguments are bad
	if (lat_now == 0.0d || lon_now == 0.0d || lat_start == 0.0d || lon_start == 0.0d || lat_end == 0.0d || lon_end == 0.0d) return return_var;

	bearing_end = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
	bearing_track = get_bearing_to_next_waypoint(lat_start, lon_start, lat_end, lon_end);
	bearing_diff = bearing_track - bearing_end;
	bearing_diff = _wrapPI(bearing_diff);

	// Return past_end = true if past end point of line
	if (bearing_diff > M_PI_2_F || bearing_diff < -M_PI_2_F) {
		return_var.past_end = true;
		return_var.error = false;
		return return_var;
	}

	dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
	return_var.distance = (dist_to_end) * sin(bearing_diff);

	if (sin(bearing_diff) >= 0) {
		return_var.bearing = _wrapPI(bearing_track - M_PI_2_F);

	} else {
		return_var.bearing = _wrapPI(bearing_track + M_PI_2_F);
	}

	return_var.error = false;

	return return_var;

}


__EXPORT crosstrack_error_s get_distance_to_arc(double lat_now, double lon_now, double lat_center, double lon_center,
		float radius, float arc_start_bearing, float arc_sweep)
{
	// This function returns the distance to the nearest point on the track arc.  Distance is positive if current
	// position is right of the arc and negative if left of the arc as seen from the closest point on the arc and
	// headed towards the end point.
	crosstrack_error_s return_var;

	// Determine if the current position is inside or outside the sector between the line from the center
	// to the arc start and the line from the center to the arc end
	float	bearing_sector_start;
	float	bearing_sector_end;
	float	bearing_now = get_bearing_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);
	bool	in_sector;

	return_var.error = true;		// Set error flag, cleared when valid result calculated.
	return_var.past_end = false;
	return_var.distance = 0.0f;
	return_var.bearing = 0.0f;

	// Return error if arguments are bad
	if (lat_now == 0.0d || lon_now == 0.0d || lat_center == 0.0d || lon_center == 0.0d || radius == 0.0d) return return_var;


	if (arc_sweep >= 0) {
		bearing_sector_start = arc_start_bearing;
		bearing_sector_end = arc_start_bearing + arc_sweep;

		if (bearing_sector_end > 2.0f * M_PI_F) bearing_sector_end -= M_TWOPI_F;

	} else {
		bearing_sector_end = arc_start_bearing;
		bearing_sector_start = arc_start_bearing - arc_sweep;

		if (bearing_sector_start < 0.0) bearing_sector_start += M_TWOPI_F;
	}

	in_sector = false;

	// Case where sector does not span zero
	if (bearing_sector_end >= bearing_sector_start && bearing_now >= bearing_sector_start && bearing_now <= bearing_sector_end) in_sector = true;

	// Case where sector does span zero
	if (bearing_sector_end < bearing_sector_start && (bearing_now > bearing_sector_start || bearing_now < bearing_sector_end)) in_sector = true;

	// If in the sector then calculate distance and bearing to closest point
	if (in_sector) {
		return_var.past_end = false;
		float dist_to_center = get_distance_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

		if (dist_to_center <= radius) {
			return_var.distance = radius - dist_to_center;
			return_var.bearing = bearing_now + M_PI_F;

		} else {
			return_var.distance = dist_to_center - radius;
			return_var.bearing = bearing_now;
		}

		// If out of the sector then calculate dist and bearing to start or end point

	} else {

		// Use the approximation  that 111,111 meters in the y direction is 1 degree (of latitude)
		// and 111,111 * cos(latitude) meters in the x direction is 1 degree (of longitude) to
		// calculate the position of the start and end points.  We should not be doing this often
		// as this function generally will not be called repeatedly when we are out of the sector.

		// TO DO - this is messed up and won't compile
		float start_disp_x = radius * sin(arc_start_bearing);
		float start_disp_y = radius * cos(arc_start_bearing);
		float end_disp_x = radius * sin(_wrapPI(arc_start_bearing + arc_sweep));
		float end_disp_y = radius * cos(_wrapPI(arc_start_bearing + arc_sweep));
		float lon_start = lon_now + start_disp_x / 111111.0d;
		float lat_start = lat_now + start_disp_y * cos(lat_now) / 111111.0d;
		float lon_end = lon_now + end_disp_x / 111111.0d;
		float lat_end = lat_now + end_disp_y * cos(lat_now) / 111111.0d;
		float dist_to_start = get_distance_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);
		float dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

		if (dist_to_start < dist_to_end) {
			return_var.distance = dist_to_start;
			return_var.bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);

		} else {
			return_var.past_end = true;
			return_var.distance = dist_to_end;
			return_var.bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
		}

	}

	return_var.bearing = _wrapPI(return_var.bearing);
	return_var.error = false;
	return return_var;
}

float _wrapPI(float bearing)
{

	while (bearing > M_PI_F) {
		bearing = bearing - M_TWOPI_F;
	}

	while (bearing <=  -M_PI_F) {
		bearing = bearing + M_TWOPI_F;
	}

	return bearing;
}

float _wrap2PI(float bearing)
{

	while (bearing >= M_TWOPI_F) {
		bearing = bearing - M_TWOPI_F;
	}

	while (bearing <  0.0f) {
		bearing = bearing + M_TWOPI_F;
	}

	return bearing;
}

float _wrap180(float bearing)
{

	while (bearing > 180.0f) {
		bearing = bearing - 360.0f;
	}

	while (bearing <=  -180.0f) {
		bearing = bearing + 360.0f;
	}

	return bearing;
}

float _wrap360(float bearing)
{

	while (bearing >= 360.0f) {
		bearing = bearing - 360.0f;
	}

	while (bearing <  0.0f) {
		bearing = bearing + 360.0f;
	}

	return bearing;
}


