/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *			 Thomas Gubler <thomasgubler@student.ethz.ch>
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
 * @file position_estimator_main.c
 * Model-identification based position estimator for multirotors
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <poll.h>

#define N_STATES 6
#define ERROR_COVARIANCE_INIT 3
#define R_EARTH 6371000.0

#define PROJECTION_INITIALIZE_COUNTER_LIMIT 5000
#define REPROJECTION_COUNTER_LIMIT 125

__EXPORT int position_estimator_main(int argc, char *argv[]);

static uint16_t position_estimator_counter_position_information;

/* values for map projection */
static double phi_1;
static double sin_phi_1;
static double cos_phi_1;
static double lambda_0;
static double scale;

/**
 * Initializes the map transformation.
 *
 * Initializes the transformation between the geographic coordinate system and the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
static void map_projection_init(double lat_0, double lon_0) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
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

/**
 * Transforms a point in the geographic coordinate system to the local azimuthal equidistant plane
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
static void map_projection_project(double lat, double lon, float *x, float *y)
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

/**
 * Transforms a point in the local azimuthal equidistant plane to the geographic coordinate system
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
static void map_projection_reproject(float x, float y, double *lat, double *lon)
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

/****************************************************************************
 * main
 ****************************************************************************/

int position_estimator_main(int argc, char *argv[])
{

	/* welcome user */
	printf("[multirotor position_estimator] started\n");

	/* initialize values */
	static float u[2] = {0, 0};
	static float z[3] = {0, 0, 0};
	static float xapo[N_STATES] = {0, 0, 0, 0, 0, 0};
	static float Papo[N_STATES * N_STATES] = {ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
			ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
			ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
			ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
			ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0,
			ERROR_COVARIANCE_INIT, 0, 0, 0, 0, 0
						 };

	static float xapo1[N_STATES];
	static float Papo1[36];

	static float gps_covariance[3] = {0.0f, 0.0f, 0.0f};

	static uint16_t counter = 0;
	position_estimator_counter_position_information = 0;

	uint8_t predict_only = 1;

	bool gps_valid = false;

	bool new_initialization = true;

	static double lat_current = 0.0d;//[°]] --> 47.0
	static double lon_current = 0.0d; //[°]] -->8.5
	float alt_current = 0.0f;


	//TODO: handle flight without gps but with estimator

	/* subscribe to vehicle status, attitude, gps */
	struct vehicle_gps_position_s gps;
	gps.fix_type = 0;
	struct vehicle_status_s vstatus;
	struct vehicle_attitude_s att;

	int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* subscribe to attitude at 100 Hz */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* wait until gps signal turns valid, only then can we initialize the projection */
	while (gps.fix_type < 3) {
		struct pollfd fds[1] = { {.fd = vehicle_gps_sub, .events = POLLIN} };

		/* wait for GPS updates, BUT READ VEHICLE STATUS (!)
		 * this choice is critical, since the vehicle status might not
		 * actually change, if this app is started after GPS lock was
		 * aquired.
		 */
		if (poll(fds, 1, 5000)) {
			/* Wait for the GPS update to propagate (we have some time) */
			usleep(5000);
			/* Read wether the vehicle status changed */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
			gps_valid = (gps.fix_type > 2);
		}
	}

	/* get gps value for first initialization */
	orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
	lat_current = ((double)(gps.lat)) * 1e-7;
	lon_current = ((double)(gps.lon)) * 1e-7;
	alt_current = gps.alt * 1e-3;

	/* initialize coordinates */
	map_projection_init(lat_current, lon_current);

	/* publish global position messages only after first GPS message */
	struct vehicle_local_position_s local_pos = {
		.x = 0,
		.y = 0,
		.z = 0
	};
	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

	printf("[multirotor position estimator] initialized projection with: lat: %.10f,  lon:%.10f\n", lat_current, lon_current);

	while (1) {

		/*This runs at the rate of the sensors, if we have also a new gps update this is used in the position_estimator function */
		struct pollfd fds[1] = { {.fd = vehicle_attitude_sub, .events = POLLIN} };

		if (poll(fds, 1, 5000) <= 0) {
			/* error / timeout */
		} else {

			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			/* got attitude, updating pos as well */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

			/*copy attitude */
			u[0] = att.roll;
			u[1] = att.pitch;

			/* initialize map projection with the last estimate (not at full rate) */
			if (gps.fix_type > 2) {
				/* Project gps lat lon (Geographic coordinate system) to plane*/
				map_projection_project(((double)(gps.lat)) * 1e-7, ((double)(gps.lon)) * 1e-7, &(z[0]), &(z[1]));

				local_pos.x = z[0];
				local_pos.y = z[1];
				/* negative offset from initialization altitude */
				local_pos.z = alt_current - (gps.alt) * 1e-3;


				orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);
			}


			// 	gps_covariance[0] = gps.eph; //TODO: needs scaling
			// 	gps_covariance[1] = gps.eph;
			// 	gps_covariance[2] = gps.epv;

			// } else {
			// 	/* we can not use the gps signal (it is of low quality) */
			// 	predict_only = 1;
			// }

			// //		predict_only = 0;																//TODO: only for testing, removeme, XXX
			// //		z[0] = sinf(((float)counter)/180.0f*3.14159265f);								//TODO: only for testing, removeme, XXX
			// //		usleep(100000);																	//TODO: only for testing, removeme, XXX


			// /*Get new estimation (this is calculated in the plane) */
			// //TODO: if new_initialization == true: use 0,0,0, else use xapo
			// if (true == new_initialization) {																								//TODO,XXX: uncomment!
			// 	xapo[0] = 0; //we have a new plane initialization. the current estimate is in the center of the plane
			// 	xapo[2] = 0;
			// 	xapo[4] = 0;
			// 	position_estimator(u, z, xapo, Papo, gps_covariance, predict_only, xapo1, Papo1);

			// } else {
			// 	position_estimator(u, z, xapo, Papo, gps_covariance, predict_only, xapo1, Papo1);
			// }



			// /* Copy values from xapo1 to xapo */
			// int i;

			// for (i = 0; i < N_STATES; i++) {
			// 	xapo[i] = xapo1[i];
			// }

			// if ((counter % REPROJECTION_COUNTER_LIMIT == 0) || (counter % (PROJECTION_INITIALIZE_COUNTER_LIMIT - 1) == 0)) {
			// 	/* Reproject from plane to geographic coordinate system */
			// 	//		map_projection_reproject(xapo1[0], xapo1[2], map_scale, phi_1, lambda_0, &lat_current, &lon_current) //TODO,XXX: uncomment!
			// 	map_projection_reproject(z[0], z[1], &lat_current, &lon_current); //do not use estimator for projection testing, removeme
			// 	//		//DEBUG
			// 	//			if(counter%500 == 0)
			// 	//			{
			// 	//				printf("phi_1: %.10f\n", phi_1);
			// 	//				printf("lambda_0: %.10f\n", lambda_0);
			// 	//				printf("lat_estimated: %.10f\n", lat_current);
			// 	//				printf("lon_estimated: %.10f\n", lon_current);
			// 	//				printf("z[0]=%.10f, z[1]=%.10f, z[2]=%f\n", z[0], z[1], z[2]);
			// 	//				fflush(stdout);
			// 	//
			// 	//			}

			// 	//			if(!isnan(lat_current) && !isnan(lon_current))// && !isnan(xapo1[4]) && !isnan(xapo1[1]) && !isnan(xapo1[3]) && !isnan(xapo1[5]))
			// 	//			{
			// 	/* send out */

			// 	global_pos.lat = lat_current;
			// 	global_pos.lon = lon_current;
			// 	global_pos.alt = xapo1[4];
			// 	global_pos.vx = xapo1[1];
			// 	global_pos.vy = xapo1[3];
			// 	global_pos.vz = xapo1[5];

				/* publish current estimate */
				// orb_publish(ORB_ID(vehicle_global_position), global_pos_pub, &global_pos);
				//			}
				//			else
				//			{
				//				printf("[position estimator] ERROR: nan values, lat_current=%.4f, lon_current=%.4f, z[0]=%.4f z[1]=%.4f\n", lat_current, lon_current, z[0], z[1]);
				//				fflush(stdout);
				//			}

			// }

			counter++;
		}

	}

	return 0;
}


