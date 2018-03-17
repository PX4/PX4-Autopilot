/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

#include <geo/geo.h>
#include <px4_config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <float.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

/*
 * Azimuthal Equidistant Projection
 * formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html
 */

static struct map_projection_reference_s mp_ref = {0.0, 0.0, 0.0, 0.0, false, 0};
static struct globallocal_converter_reference_s gl_ref = {0.0f, false};

__EXPORT bool map_projection_global_initialized()
{
	return map_projection_initialized(&mp_ref);
}

__EXPORT bool map_projection_initialized(const struct map_projection_reference_s *ref)
{
	return ref->init_done;
}

__EXPORT uint64_t map_projection_global_timestamp()
{
	return map_projection_timestamp(&mp_ref);
}

__EXPORT uint64_t map_projection_timestamp(const struct map_projection_reference_s *ref)
{
	return ref->timestamp;
}

__EXPORT int map_projection_global_init(double lat_0, double lon_0,
					uint64_t timestamp) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{
	return map_projection_init_timestamped(&mp_ref, lat_0, lon_0, timestamp);
}

__EXPORT int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0,
		uint64_t timestamp) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{

	ref->lat_rad = lat_0 * M_DEG_TO_RAD;
	ref->lon_rad = lon_0 * M_DEG_TO_RAD;
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);

	ref->timestamp = timestamp;
	ref->init_done = true;

	return 0;
}

__EXPORT int map_projection_init(struct map_projection_reference_s *ref, double lat_0,
				 double lon_0) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{
	return map_projection_init_timestamped(ref, lat_0, lon_0, hrt_absolute_time());
}

__EXPORT int map_projection_global_reference(double *ref_lat_rad, double *ref_lon_rad)
{
	return map_projection_reference(&mp_ref, ref_lat_rad, ref_lon_rad);
}

__EXPORT int map_projection_reference(const struct map_projection_reference_s *ref, double *ref_lat_rad,
				      double *ref_lon_rad)
{
	if (!map_projection_initialized(ref)) {
		return -1;
	}

	*ref_lat_rad = ref->lat_rad;
	*ref_lon_rad = ref->lon_rad;

	return 0;
}

__EXPORT int map_projection_global_project(double lat, double lon, float *x, float *y)
{
	return map_projection_project(&mp_ref, lat, lon, x, y);

}

__EXPORT int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x,
				    float *y)
{
	if (!map_projection_initialized(ref)) {
		return -1;
	}

	double lat_rad = lat * M_DEG_TO_RAD;
	double lon_rad = lon * M_DEG_TO_RAD;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);
	double cos_d_lon = cos(lon_rad - ref->lon_rad);

	double arg = ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon;

	if (arg > 1.0) {
		arg = 1.0;

	} else if (arg < -1.0) {
		arg = -1.0;
	}

	double c = acos(arg);
	double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

	*x = k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	*y = k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

	return 0;
}

__EXPORT int map_projection_global_reproject(float x, float y, double *lat, double *lon)
{
	return map_projection_reproject(&mp_ref, x, y, lat, lon);
}

__EXPORT int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat,
				      double *lon)
{
	if (!map_projection_initialized(ref)) {
		return -1;
	}

	double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
	double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
	double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
	double sin_c = sin(c);
	double cos_c = cos(c);

	double lat_rad;
	double lon_rad;

	if (fabs(c) > DBL_EPSILON) {
		lat_rad = asin(cos_c * ref->sin_lat + (x_rad * sin_c * ref->cos_lat) / c);
		lon_rad = (ref->lon_rad + atan2(y_rad * sin_c, c * ref->cos_lat * cos_c - x_rad * ref->sin_lat * sin_c));

	} else {
		lat_rad = ref->lat_rad;
		lon_rad = ref->lon_rad;
	}

	*lat = lat_rad * 180.0 / M_PI;
	*lon = lon_rad * 180.0 / M_PI;

	return 0;
}

__EXPORT int map_projection_global_getref(double *lat_0, double *lon_0)
{
	if (!map_projection_global_initialized()) {
		return -1;
	}

	if (lat_0 != NULL) {
		*lat_0 = M_RAD_TO_DEG * mp_ref.lat_rad;
	}

	if (lon_0 != NULL) {
		*lon_0 = M_RAD_TO_DEG * mp_ref.lon_rad;
	}

	return 0;

}
__EXPORT int globallocalconverter_init(double lat_0, double lon_0, float alt_0, uint64_t timestamp)
{
	gl_ref.alt = alt_0;

	if (!map_projection_global_init(lat_0, lon_0, timestamp)) {
		gl_ref.init_done = true;
		return 0;

	} else {
		gl_ref.init_done = false;
		return -1;
	}
}

__EXPORT bool globallocalconverter_initialized()
{
	return gl_ref.init_done && map_projection_global_initialized();
}

__EXPORT int globallocalconverter_tolocal(double lat, double lon, float alt, float *x, float *y, float *z)
{
	if (!map_projection_global_initialized()) {
		return -1;
	}

	map_projection_global_project(lat, lon, x, y);
	*z = gl_ref.alt - alt;

	return 0;
}

__EXPORT int globallocalconverter_toglobal(float x, float y, float z,  double *lat, double *lon, float *alt)
{
	if (!map_projection_global_initialized()) {
		return -1;
	}

	map_projection_global_reproject(x, y, lat, lon);
	*alt = gl_ref.alt - z;

	return 0;
}

__EXPORT int globallocalconverter_getref(double *lat_0, double *lon_0, float *alt_0)
{
	if (!map_projection_global_initialized()) {
		return -1;
	}

	if (map_projection_global_getref(lat_0, lon_0)) {
		return -1;
	}

	if (alt_0 != NULL) {
		*alt_0 = gl_ref.alt;
	}

	return 0;
}

__EXPORT float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	double lat_now_rad = lat_now / (double)180.0 * M_PI;
	double lon_now_rad = lon_now / (double)180.0 * M_PI;
	double lat_next_rad = lat_next / (double)180.0 * M_PI;
	double lon_next_rad = lon_next / (double)180.0 * M_PI;


	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	double a = sin(d_lat / (double)2.0) * sin(d_lat / (double)2.0) + sin(d_lon / (double)2.0) * sin(d_lon /
			(double)2.0) * cos(lat_now_rad) * cos(lat_next_rad);
	double c = (double)2.0 * atan2(sqrt(a), sqrt((double)1.0 - a));

	return CONSTANTS_RADIUS_OF_EARTH * c;
}

__EXPORT void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
		double *lat_target, double *lon_target)
{
	if (fabsf(dist) < FLT_EPSILON) {
		*lat_target = lat_A;
		*lon_target = lon_A;

	} else if (dist >= FLT_EPSILON) {
		float heading = get_bearing_to_next_waypoint(lat_A, lon_A, lat_B, lon_B);
		waypoint_from_heading_and_distance(lat_A, lon_A, heading, dist, lat_target, lon_target);

	} else {
		float heading = get_bearing_to_next_waypoint(lat_A, lon_A, lat_B, lon_B);
		heading = _wrap_2pi(heading + M_PI_F);
		waypoint_from_heading_and_distance(lat_A, lon_A, heading, dist, lat_target, lon_target);
	}
}

__EXPORT void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
		double *lat_target, double *lon_target)
{
	bearing = _wrap_2pi(bearing);
	double radius_ratio = fabs((double)dist) / CONSTANTS_RADIUS_OF_EARTH;

	double lat_start_rad = lat_start * M_DEG_TO_RAD;
	double lon_start_rad = lon_start * M_DEG_TO_RAD;

	*lat_target = asin(sin(lat_start_rad) * cos(radius_ratio) + cos(lat_start_rad) * sin(radius_ratio) * cos((
				   double)bearing));
	*lon_target = lon_start_rad + atan2(sin((double)bearing) * sin(radius_ratio) * cos(lat_start_rad),
					    cos(radius_ratio) - sin(lat_start_rad) * sin(*lat_target));

	*lat_target *= M_RAD_TO_DEG;
	*lon_target *= M_RAD_TO_DEG;
}
__EXPORT float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;
	double lat_next_rad = lat_next * M_DEG_TO_RAD;
	double lon_next_rad = lon_next * M_DEG_TO_RAD;

	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	float theta = atan2f(sin(d_lon) * cos(lat_next_rad),
			     cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

	theta = _wrap_pi(theta);

	return theta;
}

__EXPORT void get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n,
		float *v_e)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;
	double lat_next_rad = lat_next * M_DEG_TO_RAD;
	double lon_next_rad = lon_next * M_DEG_TO_RAD;

	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = CONSTANTS_RADIUS_OF_EARTH * (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(
			d_lon));
	*v_e = CONSTANTS_RADIUS_OF_EARTH * sin(d_lon) * cos(lat_next_rad);
}

__EXPORT void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next,
		float *v_n, float *v_e)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;
	double lat_next_rad = lat_next * M_DEG_TO_RAD;
	double lon_next_rad = lon_next * M_DEG_TO_RAD;

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = CONSTANTS_RADIUS_OF_EARTH * d_lat;
	*v_e = CONSTANTS_RADIUS_OF_EARTH * d_lon * cos(lat_now_rad);
}

__EXPORT void add_vector_to_global_position(double lat_now, double lon_now, float v_n, float v_e, double *lat_res,
		double *lon_res)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;

	*lat_res = (lat_now_rad + (double)v_n / CONSTANTS_RADIUS_OF_EARTH) * M_RAD_TO_DEG;
	*lon_res = (lon_now_rad + (double)v_e / (CONSTANTS_RADIUS_OF_EARTH * cos(lat_now_rad))) * M_RAD_TO_DEG;
}

// Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>

__EXPORT int get_distance_to_line(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
				  double lat_start, double lon_start, double lat_end, double lon_end)
{
// This function returns the distance to the nearest point on the track line.  Distance is positive if current
// position is right of the track and negative if left of the track as seen from a point on the track line
// headed towards the end point.

	float dist_to_end;
	float bearing_end;
	float bearing_track;
	float bearing_diff;

	int return_value = ERROR;	// Set error flag, cleared when valid result calculated.
	crosstrack_error->past_end = false;
	crosstrack_error->distance = 0.0f;
	crosstrack_error->bearing = 0.0f;

	dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

	// Return error if arguments are bad
	if (dist_to_end < 0.1f) {
		return ERROR;
	}

	bearing_end = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
	bearing_track = get_bearing_to_next_waypoint(lat_start, lon_start, lat_end, lon_end);
	bearing_diff = bearing_track - bearing_end;
	bearing_diff = _wrap_pi(bearing_diff);

	// Return past_end = true if past end point of line
	if (bearing_diff > M_PI_2_F || bearing_diff < -M_PI_2_F) {
		crosstrack_error->past_end = true;
		return_value = OK;
		return return_value;
	}

	crosstrack_error->distance = (dist_to_end) * sinf(bearing_diff);

	if (sinf(bearing_diff) >= 0) {
		crosstrack_error->bearing = _wrap_pi(bearing_track - M_PI_2_F);

	} else {
		crosstrack_error->bearing = _wrap_pi(bearing_track + M_PI_2_F);
	}

	return_value = OK;

	return return_value;

}


__EXPORT int get_distance_to_arc(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
				 double lat_center, double lon_center,
				 float radius, float arc_start_bearing, float arc_sweep)
{
	// This function returns the distance to the nearest point on the track arc.  Distance is positive if current
	// position is right of the arc and negative if left of the arc as seen from the closest point on the arc and
	// headed towards the end point.

	// Determine if the current position is inside or outside the sector between the line from the center
	// to the arc start and the line from the center to the arc end
	float	bearing_sector_start;
	float	bearing_sector_end;
	float	bearing_now = get_bearing_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);
	bool	in_sector;

	int return_value = ERROR;		// Set error flag, cleared when valid result calculated.
	crosstrack_error->past_end = false;
	crosstrack_error->distance = 0.0f;
	crosstrack_error->bearing = 0.0f;

	// Return error if arguments are bad
	if (radius < 0.1f) { return return_value; }


	if (arc_sweep >= 0.0f) {
		bearing_sector_start = arc_start_bearing;
		bearing_sector_end = arc_start_bearing + arc_sweep;

		if (bearing_sector_end > 2.0f * M_PI_F) { bearing_sector_end -= M_TWOPI_F; }

	} else {
		bearing_sector_end = arc_start_bearing;
		bearing_sector_start = arc_start_bearing - arc_sweep;

		if (bearing_sector_start < 0.0f) { bearing_sector_start += M_TWOPI_F; }
	}

	in_sector = false;

	// Case where sector does not span zero
	if (bearing_sector_end >= bearing_sector_start && bearing_now >= bearing_sector_start
	    && bearing_now <= bearing_sector_end) { in_sector = true; }

	// Case where sector does span zero
	if (bearing_sector_end < bearing_sector_start && (bearing_now > bearing_sector_start
			|| bearing_now < bearing_sector_end)) { in_sector = true; }

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
		double end_disp_x = (double)radius * sin((double)_wrap_pi((double)(arc_start_bearing + arc_sweep)));
		double end_disp_y = (double)radius * cos((double)_wrap_pi((double)(arc_start_bearing + arc_sweep)));
		double lon_start = lon_now + start_disp_x / 111111.0;
		double lat_start = lat_now + start_disp_y * cos(lat_now) / 111111.0;
		double lon_end = lon_now + end_disp_x / 111111.0;
		double lat_end = lat_now + end_disp_y * cos(lat_now) / 111111.0;
		double dist_to_start = get_distance_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);
		double dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);


		if (dist_to_start < dist_to_end) {
			crosstrack_error->distance = dist_to_start;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);

		} else {
			crosstrack_error->past_end = true;
			crosstrack_error->distance = dist_to_end;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
		}

	}

	crosstrack_error->bearing = _wrap_pi((double)crosstrack_error->bearing);
	return_value = OK;
	return return_value;
}

__EXPORT float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		double lat_next, double lon_next, float alt_next,
		float *dist_xy, float *dist_z)
{
	double current_x_rad = lat_next / 180.0 * M_PI;
	double current_y_rad = lon_next / 180.0 * M_PI;
	double x_rad = lat_now / 180.0 * M_PI;
	double y_rad = lon_now / 180.0 * M_PI;

	double d_lat = x_rad - current_x_rad;
	double d_lon = y_rad - current_y_rad;

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(current_x_rad) * cos(x_rad);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	float dxy = CONSTANTS_RADIUS_OF_EARTH * c;
	float dz = alt_now - alt_next;

	*dist_xy = fabsf(dxy);
	*dist_z = fabsf(dz);

	return sqrtf(dxy * dxy + dz * dz);
}


__EXPORT float mavlink_wpm_distance_to_point_local(float x_now, float y_now, float z_now,
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

__EXPORT float _wrap_pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= M_PI_F) {
		bearing -= M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < -M_PI_F) {
		bearing += M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

__EXPORT float _wrap_2pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= M_TWOPI_F) {
		bearing -= M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < 0.0f) {
		bearing += M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

__EXPORT float _wrap_180(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= 180.0f) {
		bearing -= 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < -180.0f) {
		bearing += 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

__EXPORT float _wrap_360(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= 360.0f) {
		bearing -= 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < 0.0f) {
		bearing += 360.0f;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}
