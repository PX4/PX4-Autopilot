/****************************************************************************
 * ardrone_control/position_control.c
 *
 *   Copyright (C) 2008, 2012 Thomas Gubler, Julian Oes, Lorenz Meier. All rights reserved.
 *   Author: Based on the pixhawk quadrotor controller
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include "position_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include "pid.h"

#ifndef FM_PI
#define FM_PI 3.1415926535897932384626433832795f
#endif



#define CONTROL_PID_POSITION_INTERVAL	0.020

int read_lock_position(global_data_position_t *position)
{
	static int ret;
	ret = global_data_wait(&global_data_position->access_conf);

	if (ret == 0) {
		memcpy(&position->lat, &global_data_position->lat, sizeof(global_data_position->lat));
		memcpy(&position->lon, &global_data_position->lon, sizeof(global_data_position->lon));
		memcpy(&position->alt, &global_data_position->alt, sizeof(global_data_position->alt));
		memcpy(&position->relative_alt, &global_data_position->relative_alt, sizeof(global_data_position->relative_alt));
		memcpy(&position->vx, &global_data_position->vx, sizeof(global_data_position->vx));
		memcpy(&position->vy, &global_data_position->vy, sizeof(global_data_position->vy));
		memcpy(&position->vz, &global_data_position->vz, sizeof(global_data_position->vz));
		memcpy(&position->hdg, &global_data_position->hdg, sizeof(global_data_position->hdg));


	} else {
		printf("Controller timeout, no new position values available\n");
	}

	global_data_unlock(&global_data_position->access_conf);
	return ret;
}

float get_distance_to_next_waypoint(float lat_now, float lon_now, float lat_next, float lon_next)
{
	double lat_now_rad = lat_now / 180.0 * M_PI;
	double lon_now_rad = lon_now / 180.0 * M_PI;
	double lat_next_rad = lat_next / 180.0 * M_PI;
	double lon_next_rad = lon_next / 180.0 * M_PI;


	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat_now_rad) * cos(lat_next_rad);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	const double radius_earth = 6371000.0;

	return radius_earth * c;
}

float get_bearing_to_next_waypoint(float lat_now, float lon_now, float lat_next, float lon_next)
{
	double lat_now_rad = lat_now / 180.0 * M_PI;
	double lon_now_rad = lon_now / 180.0 * M_PI;
	double lat_next_rad = lat_next / 180.0 * M_PI;
	double lon_next_rad = lon_next / 180.0 * M_PI;

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;


	float theta = atan2f(sin(d_lon) * cos(lat_next_rad) , cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

	if (theta < 0) {
		theta = theta + 2 * FM_PI;
	}

	return theta;
}

void control_position(void)
{
	static PID_t distance_controller;

	static int read_ret;
	static global_data_position_t position_estimated;

	static uint16_t counter;

	static bool initialized;
	static uint16_t pm_counter;

	static float lat_next;
	static float lon_next;

	static float pitch_current;

	static float thrust_total;


	if (initialized == false) {

		global_data_lock(&global_data_parameter_storage->access_conf);

		pid_init(&distance_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_POS_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_POS_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_POS_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_POS_AWU],
			 PID_MODE_DERIVATIV_CALC, 150);//150

//		pid_pos_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_LIM];
//		pid_pos_z_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_Z_LIM];

		pm_counter = global_data_parameter_storage->counter;

		global_data_unlock(&global_data_parameter_storage->access_conf);

		thrust_total = 0.0f;

		/* Position initialization */
		/* Wait for new position estimate */
		do {
			read_ret = read_lock_position(&position_estimated);
		} while (read_ret != 0);

		lat_next = position_estimated.lat;
		lon_next = position_estimated.lon;

		/* attitude initialization */
		global_data_lock(&global_data_attitude->access_conf);
		pitch_current = global_data_attitude->pitch;
		global_data_unlock(&global_data_attitude->access_conf);

		initialized = true;
	}

	/* load new parameters with 10Hz */
	if (counter % 50 == 0) {
		if (global_data_trylock(&global_data_parameter_storage->access_conf) == 0) {
			/* check whether new parameters are available */
			if (global_data_parameter_storage->counter > pm_counter) {
				pid_set_parameters(&distance_controller,
						   global_data_parameter_storage->pm.param_values[PARAM_PID_POS_P],
						   global_data_parameter_storage->pm.param_values[PARAM_PID_POS_I],
						   global_data_parameter_storage->pm.param_values[PARAM_PID_POS_D],
						   global_data_parameter_storage->pm.param_values[PARAM_PID_POS_AWU]);

//
//				pid_pos_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_LIM];
//				pid_pos_z_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_Z_LIM];

				pm_counter = global_data_parameter_storage->counter;
				printf("Position controller changed pid parameters\n");
			}
		}

		global_data_unlock(&global_data_parameter_storage->access_conf);
	}


	/* Wait for new position estimate */
	do {
		read_ret = read_lock_position(&position_estimated);
	} while (read_ret != 0);

	/* Get next waypoint */ //TODO: add local copy

	if (0 == global_data_trylock(&global_data_position_setpoint->access_conf)) {
		lat_next = global_data_position_setpoint->x;
		lon_next = global_data_position_setpoint->y;
		global_data_unlock(&global_data_position_setpoint->access_conf);
	}

	/* Get distance to waypoint */
	float distance_to_waypoint = get_distance_to_next_waypoint(position_estimated.lat , position_estimated.lon, lat_next, lon_next);
//	if(counter % 5 == 0)
//		printf("distance_to_waypoint: %.4f\n", distance_to_waypoint);

	/* Get bearing to waypoint (direction on earth surface to next waypoint) */
	float bearing = get_bearing_to_next_waypoint(position_estimated.lat, position_estimated.lon, lat_next, lon_next);

	if (counter % 5 == 0)
		printf("bearing: %.4f\n", bearing);

	/* Calculate speed in direction of bearing (needed for controller) */
	float speed_norm = sqrtf(position_estimated.vx  * position_estimated.vx + position_estimated.vy * position_estimated.vy);
//	if(counter % 5 == 0)
//		printf("speed_norm: %.4f\n", speed_norm);
	float speed_to_waypoint = 0; //(position_estimated.vx * cosf(bearing) + position_estimated.vy * sinf(bearing))/speed_norm; //FIXME, TODO: re-enable this once we have a full estimate of the speed, then we can do a PID for the distance controller

	/* Control Thrust in bearing direction  */
	float horizontal_thrust = -pid_calculate(&distance_controller, 0, distance_to_waypoint, speed_to_waypoint,
				  CONTROL_PID_POSITION_INTERVAL); //TODO: maybe this "-" sign is an error somewhere else

//	if(counter % 5 == 0)
//		printf("horizontal thrust: %.4f\n", horizontal_thrust);

	/* Get total thrust (from remote for now) */
	if (0 == global_data_trylock(&global_data_rc_channels->access_conf)) {
		thrust_total = (float)global_data_rc_channels->chan[THROTTLE].scale; 												//TODO: how should we use the RC_CHANNELS_FUNCTION enum?
		global_data_unlock(&global_data_rc_channels->access_conf);
	}

	const float max_gas = 500.0f;
	thrust_total *= max_gas / 20000.0f; //TODO: check this
	thrust_total += max_gas / 2.0f;


	if (horizontal_thrust > thrust_total) {
		horizontal_thrust = thrust_total;

	} else if (horizontal_thrust < -thrust_total) {
		horizontal_thrust = -thrust_total;
	}



	//TODO: maybe we want to add a speed controller later...

	/* Calclulate thrust in east and north direction */
	float thrust_north = cosf(bearing) * horizontal_thrust;
	float thrust_east = sinf(bearing) * horizontal_thrust;

	if (counter % 10 == 0) {
		printf("thrust north: %.4f\n", thrust_north);
		printf("thrust east: %.4f\n", thrust_east);
		fflush(stdout);
	}

	/* Get current attitude */
	if (0 == global_data_trylock(&global_data_attitude->access_conf)) {
		pitch_current = global_data_attitude->pitch;
		global_data_unlock(&global_data_attitude->access_conf);
	}

	/* Get desired pitch & roll */
	float pitch_desired = 0.0f;
	float roll_desired = 0.0f;

	if (thrust_total != 0) {
		float pitch_fraction = -thrust_north / thrust_total;
		float roll_fraction = thrust_east / (cosf(pitch_current) * thrust_total);

		if (roll_fraction < -1) {
			roll_fraction = -1;

		} else if (roll_fraction > 1) {
			roll_fraction = 1;
		}

//		if(counter % 5 == 0)
//		{
//			printf("pitch_fraction: %.4f, roll_fraction: %.4f\n",pitch_fraction, roll_fraction);
//			fflush(stdout);
//		}

		pitch_desired = asinf(pitch_fraction);
		roll_desired = asinf(roll_fraction);
	}

	/*Broadcast desired angles */
	global_data_lock(&global_data_ardrone_control->access_conf);
	global_data_ardrone_control->attitude_setpoint_navigationframe_from_positioncontroller[0] = roll_desired;
	global_data_ardrone_control->attitude_setpoint_navigationframe_from_positioncontroller[1] = pitch_desired;
	global_data_ardrone_control->attitude_setpoint_navigationframe_from_positioncontroller[2] = bearing; //TODO: add yaw setpoint
	global_data_unlock(&global_data_ardrone_control->access_conf);
	global_data_broadcast(&global_data_ardrone_control->access_conf);


	counter++;
}
