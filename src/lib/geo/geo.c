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

#include <geo/geo.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>


/* values for map projection */
static double phi_1;
static double sin_phi_1;
static double cos_phi_1;
static double lambda_0;
static double scale;

__EXPORT void map_projection_init(double lat_0, double lon_0) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{
	/* notation and formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html */
	phi_1 = lat_0 / 180.0 * M_PI;
	lambda_0 = lon_0 / 180.0 * M_PI;

	sin_phi_1 = sin(phi_1);
	cos_phi_1 = cos(phi_1);

	/* calculate local scale by using the relation of true distance and the distance on plane */ //TODO: this is a quick solution, there are probably easier ways to determine the scale

	/* 1) calculate true distance d on sphere to a point: http://www.movable-type.co.uk/scripts/latlong.html */
	const double r_earth = 6371000;

	double lat1 = phi_1;
	double lon1 = lambda_0;

	double lat2 = phi_1 + 0.5 / 180 * M_PI;
	double lon2 = lambda_0 + 0.5 / 180 * M_PI;
	double sin_lat_2 = sin(lat2);
	double cos_lat_2 = cos(lat2);
	double d = acos(sin(lat1) * sin_lat_2 + cos(lat1) * cos_lat_2 * cos(lon2 - lon1)) * r_earth;

	/* 2) calculate distance rho on plane */
	double k_bar = 0;
	double c =  acos(sin_phi_1 * sin_lat_2 + cos_phi_1 * cos_lat_2 * cos(lon2 - lambda_0));

	if (0 != c)
		k_bar = c / sin(c);

	double x2 = k_bar * (cos_lat_2 * sin(lon2 - lambda_0)); //Projection of point 2 on plane
	double y2 = k_bar * ((cos_phi_1 * sin_lat_2 - sin_phi_1 * cos_lat_2 * cos(lon2 - lambda_0)));
	double rho = sqrt(pow(x2, 2) + pow(y2, 2));

	scale = d / rho;

}

__EXPORT void map_projection_project(double lat, double lon, float *x, float *y)
{
	/* notation and formulas accoring to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html */
	double phi = lat / 180.0 * M_PI;
	double lambda = lon / 180.0 * M_PI;

	double sin_phi = sin(phi);
	double cos_phi = cos(phi);

	double k_bar = 0;
	/* using small angle approximation (formula in comment is without aproximation) */
	double c =  acos(sin_phi_1 * sin_phi + cos_phi_1 * cos_phi * (1 - pow((lambda - lambda_0), 2) / 2)); //double c =  acos( sin_phi_1 * sin_phi + cos_phi_1 * cos_phi * cos(lambda - lambda_0) );

	if (0 != c)
		k_bar = c / sin(c);

	/* using small angle approximation (formula in comment is without aproximation) */
	*y = k_bar * (cos_phi * (lambda - lambda_0)) * scale;//*y = k_bar * (cos_phi * sin(lambda - lambda_0)) * scale;
	*x = k_bar * ((cos_phi_1 * sin_phi - sin_phi_1 * cos_phi * (1 - pow((lambda - lambda_0), 2) / 2))) * scale; //	*x = k_bar * ((cos_phi_1 * sin_phi - sin_phi_1 * cos_phi * cos(lambda - lambda_0))) * scale;

//	printf("%phi_1=%.10f, lambda_0 =%.10f\n", phi_1, lambda_0);
}

__EXPORT void map_projection_reproject(float x, float y, double *lat, double *lon)
{
	/* notation and formulas accoring to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html */

	double x_descaled = x / scale;
	double y_descaled = y / scale;

	double c = sqrt(pow(x_descaled, 2) + pow(y_descaled, 2));
	double sin_c = sin(c);
	double cos_c = cos(c);

	double lat_sphere = 0;

	if (c != 0)
		lat_sphere = asin(cos_c * sin_phi_1 + (x_descaled * sin_c * cos_phi_1) / c);
	else
		lat_sphere = asin(cos_c * sin_phi_1);

//	printf("lat_sphere = %.10f\n",lat_sphere);

	double lon_sphere = 0;

	if (phi_1  == M_PI / 2) {
		//using small angle approximation (formula in comment is without aproximation)
		lon_sphere = (lambda_0 - y_descaled / x_descaled); //lon_sphere = (lambda_0 + atan2(-y_descaled, x_descaled));

	} else if (phi_1 == -M_PI / 2) {
		//using small angle approximation (formula in comment is without aproximation)
		lon_sphere = (lambda_0 + y_descaled / x_descaled); //lon_sphere = (lambda_0 + atan2(y_descaled, x_descaled));

	} else {

		lon_sphere = (lambda_0 + atan2(y_descaled * sin_c , c * cos_phi_1 * cos_c - x_descaled * sin_phi_1 * sin_c));
		//using small angle approximation
//    	double denominator = (c * cos_phi_1 * cos_c - x_descaled * sin_phi_1 * sin_c);
//    	if(denominator != 0)
//    	{
//    		lon_sphere = (lambda_0 + (y_descaled * sin_c) / denominator);
//    	}
//    	else
//    	{
//    	...
//    	}
	}

//	printf("lon_sphere = %.10f\n",lon_sphere);

	*lat = lat_sphere * 180.0 / M_PI;
	*lon = lon_sphere * 180.0 / M_PI;

}


__EXPORT float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	double lat_now_rad = lat_now / 180.0d * M_PI;
	double lon_now_rad = lon_now / 180.0d * M_PI;
	double lat_next_rad = lat_next / 180.0d * M_PI;
	double lon_next_rad = lon_next / 180.0d * M_PI;


	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	double a = sin(d_lat / 2.0d) * sin(d_lat / 2.0d) + sin(d_lon / 2.0d) * sin(d_lon / 2.0d) * cos(lat_now_rad) * cos(lat_next_rad);
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

	theta = _wrap_pi(theta);

	return theta;
}

__EXPORT void get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float* vx, float* vy)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;
	double lat_next_rad = lat_next * M_DEG_TO_RAD;
	double lon_next_rad = lon_next * M_DEG_TO_RAD;

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*vy = CONSTANTS_RADIUS_OF_EARTH * sin(d_lon) * cos(lat_next_rad);
	*vx = CONSTANTS_RADIUS_OF_EARTH * cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon);
}

__EXPORT void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next, float* vx, float* vy)
{
	double lat_now_rad = lat_now * M_DEG_TO_RAD;
	double lon_now_rad = lon_now * M_DEG_TO_RAD;
	double lat_next_rad = lat_next * M_DEG_TO_RAD;
	double lon_next_rad = lon_next * M_DEG_TO_RAD;

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*vy = CONSTANTS_RADIUS_OF_EARTH * d_lon;
	*vx = CONSTANTS_RADIUS_OF_EARTH * cos(lat_now_rad);
}

// Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>

__EXPORT int get_distance_to_line(struct crosstrack_error_s * crosstrack_error, double lat_now, double lon_now, double lat_start, double lon_start, double lat_end, double lon_end)
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

	// Return error if arguments are bad
	if (lat_now == 0.0d || lon_now == 0.0d || lat_start == 0.0d || lon_start == 0.0d || lat_end == 0.0d || lon_end == 0.0d) return return_value;

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

	dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
	crosstrack_error->distance = (dist_to_end) * sin(bearing_diff);

	if (sin(bearing_diff) >= 0) {
		crosstrack_error->bearing = _wrap_pi(bearing_track - M_PI_2_F);

	} else {
		crosstrack_error->bearing = _wrap_pi(bearing_track + M_PI_2_F);
	}

	return_value = OK;

	return return_value;

}


__EXPORT int get_distance_to_arc(struct crosstrack_error_s * crosstrack_error, double lat_now, double lon_now, double lat_center, double lon_center,
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
	if (lat_now == 0.0d || lon_now == 0.0d || lat_center == 0.0d || lon_center == 0.0d || radius == 0.0d) return return_value;


	if (arc_sweep >= 0) {
		bearing_sector_start = arc_start_bearing;
		bearing_sector_end = arc_start_bearing + arc_sweep;

		if (bearing_sector_end > 2.0f * M_PI_F) bearing_sector_end -= M_TWOPI_F;

	} else {
		bearing_sector_end = arc_start_bearing;
		bearing_sector_start = arc_start_bearing - arc_sweep;

		if (bearing_sector_start < 0.0f) bearing_sector_start += M_TWOPI_F;
	}

	in_sector = false;

	// Case where sector does not span zero
	if (bearing_sector_end >= bearing_sector_start && bearing_now >= bearing_sector_start && bearing_now <= bearing_sector_end) in_sector = true;

	// Case where sector does span zero
	if (bearing_sector_end < bearing_sector_start && (bearing_now > bearing_sector_start || bearing_now < bearing_sector_end)) in_sector = true;

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
			crosstrack_error->distance = dist_to_start;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);

		} else {
			crosstrack_error->past_end = true;
			crosstrack_error->distance = dist_to_end;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
		}

	}

	crosstrack_error->bearing = _wrapPI(crosstrack_error->bearing);
	return_value = OK;
	return return_value;
}

__EXPORT float _wrap_pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing) || bearing == 0) {
		return bearing;
	}

	int c = 0;

	while (bearing > M_PI_F && c < 30) {
		bearing -= M_TWOPI_F;
		c++;
	}

	c = 0;

	while (bearing <=  -M_PI_F && c < 30) {
		bearing += M_TWOPI_F;
		c++;
	}

	return bearing;
}

__EXPORT float _wrap_2pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	while (bearing >= M_TWOPI_F) {
		bearing = bearing - M_TWOPI_F;
	}

	while (bearing <  0.0f) {
		bearing = bearing + M_TWOPI_F;
	}

	return bearing;
}

__EXPORT float _wrap_180(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	while (bearing > 180.0f) {
		bearing = bearing - 360.0f;
	}

	while (bearing <=  -180.0f) {
		bearing = bearing + 360.0f;
	}

	return bearing;
}

__EXPORT float _wrap_360(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	while (bearing >= 360.0f) {
		bearing = bearing - 360.0f;
	}

	while (bearing <  0.0f) {
		bearing = bearing + 360.0f;
	}

	return bearing;
}


