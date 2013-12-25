/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: 	Damian Aregger	<daregger@student.ethz.ch>
 *   			Tobias Naegeli <naegelit@student.ethz.ch>
* 				Lorenz Meier <lm@inf.ethz.ch>
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

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/geo/geo.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <drivers/drv_hrt.h>

#include "position_estimator_mc_params.h"
//#include <uORB/topics/debug_key_value.h>
#include "codegen/kalman_dlqe2.h"
#include "codegen/kalman_dlqe2_initialize.h"
#include "codegen/kalman_dlqe3.h"
#include "codegen/kalman_dlqe3_initialize.h"

static bool thread_should_exit = false;	/**< Deamon exit flag */
static bool thread_running = false;	/**< Deamon status flag */
static int position_estimator_mc_task;	/**< Handle of deamon task / thread */

__EXPORT int position_estimator_mc_main(int argc, char *argv[]);

int position_estimator_mc_thread_main(int argc, char *argv[]);
/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	warnx("usage: position_estimator_mc {start|stop|status}");
	exit(1);
}

/**
 * The position_estimator_mc_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_mc_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("position_estimator_mc already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		position_estimator_mc_task = task_spawn_cmd("position_estimator_mc",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 5,
					 4096,
					 position_estimator_mc_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("position_estimator_mc is running");
		} else {
			warnx("position_estimator_mc not started");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator_mc_thread_main(int argc, char *argv[])
{
	/* welcome user */
	warnx("[position_estimator_mc] started");
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[position_estimator_mc] started");

	/* initialize values */
	float z[3] = {0, 0, 0}; /* output variables from tangent plane mapping */
	// float rotMatrix[4] = {1.0f,  0.0f, 0.0f,  1.0f};
	float x_x_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_y_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_z_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_x_aposteriori[3] = {0.0f, 0.0f, 0.0f};
	float x_y_aposteriori[3] = {1.0f, 0.0f, 0.0f};
	float x_z_aposteriori[3] = {1.0f, 0.0f, 0.0f};

	// XXX this is terribly wrong and should actual dT instead
	const float dT_const_50 = 1.0f/50.0f;

	float addNoise = 0.0f;
	float sigma = 0.0f;
	//computed from dlqe in matlab
	const float K_vicon_50Hz[3] = {0.5297f, 0.9873f, 0.9201f};
	// XXX implement baro filter
	const float K_baro[3] = {0.0248f, 0.0377f, 0.0287f};
	float K[3] = {0.0f, 0.0f, 0.0f};
	int baro_loop_cnt = 0;
	int baro_loop_end = 70; /* measurement for 1 second */
	float p0_Pa = 0.0f; /* to determin while start up */
	float rho0 = 1.293f; /* standard pressure */
	const float const_earth_gravity = 9.81f;

	float posX = 0.0f;
	float posY = 0.0f;
	float posZ = 0.0f;

	double lat_current;
	double lon_current;
	float alt_current;

	float gps_origin_altitude = 0.0f;

	/* Initialize filter */
	kalman_dlqe2_initialize();
	kalman_dlqe3_initialize();

	/* declare and safely initialize all structs */
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status)); /* make sure that baroINITdone = false */
	struct vehicle_vicon_position_s vicon_pos;
	memset(&vicon_pos, 0, sizeof(vicon_pos));
	struct actuator_controls_effective_s act_eff;
	memset(&act_eff, 0, sizeof(act_eff));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_local_position_s local_pos_est;
	memset(&local_pos_est, 0, sizeof(local_pos_est));
	struct vehicle_global_position_s global_pos_est;
	memset(&global_pos_est, 0, sizeof(global_pos_est));

	/* subscribe */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	int actuator_eff_sub = orb_subscribe(ORB_ID(actuator_controls_effective_0));
	int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* advertise */
	orb_advert_t local_pos_est_pub = 0;
	orb_advert_t global_pos_est_pub = 0;

	struct position_estimator_mc_params pos1D_params;
	struct position_estimator_mc_param_handles pos1D_param_handles;
	/* initialize parameter handles */
	parameters_init(&pos1D_param_handles);

	bool flag_use_gps = false;
	bool flag_use_baro = false;
	bool flag_baro_initialized = false; /* in any case disable baroINITdone */
	/* FIRST PARAMETER READ at START UP*/
	struct parameter_update_s update;
	orb_copy(ORB_ID(parameter_update), sub_params, &update); /* read from param to clear updated flag */
	/* FIRST PARAMETER UPDATE */
	parameters_update(&pos1D_param_handles, &pos1D_params);
	flag_use_baro = pos1D_params.baro;
	sigma = pos1D_params.sigma;
	addNoise = pos1D_params.addNoise;
	/* END FIRST PARAMETER UPDATE */

	/* try to grab a vicon message - if it fails, go for GPS. */

	/* make sure the next orb_check() can't return true unless we get a timely update */
	orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);
	/* allow 200 ms for vicon to come in */
	usleep(200000);
	/* check if we got vicon */
	bool update_check;
	orb_check(vicon_pos_sub, &update_check);
	/* if no update was available, use GPS */
	flag_use_gps = !update_check;

	if (flag_use_gps) {
		mavlink_log_info(mavlink_fd, "[pos_est_mc] GPS locked");
		/* wait until gps signal turns valid, only then can we initialize the projection */

		// XXX magic number
		float hdop_threshold_m = 4.0f;
		float vdop_threshold_m = 8.0f;

		/*
		 * If horizontal dilution of precision (hdop / eph)
		 * and vertical diluation of precision (vdop / epv)
		 * are below a certain threshold (e.g. 4 m), AND
		 * home position is not yet set AND the last GPS
		 * GPS measurement is not older than two seconds AND
		 * the system is currently not armed, set home
		 * position to the current position.
		 */

		while (!(gps.fix_type == 3
			&& (gps.eph_m < hdop_threshold_m)
			&& (gps.epv_m < vdop_threshold_m)
			&& (hrt_absolute_time() - gps.timestamp_position < 2000000))) {

			struct pollfd fds1[2] = {
					{ .fd = vehicle_gps_sub, .events = POLLIN },
					{ .fd = sub_params,   .events = POLLIN },
			};

			/* wait for GPS updates, BUT READ VEHICLE STATUS (!)
			 * this choice is critical, since the vehicle status might not
			 * actually change, if this app is started after GPS lock was
			 * aquired.
			 */
			if (poll(fds1, 2, 5000)) {
				if (fds1[0].revents & POLLIN){
					/* Read gps position */
					orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				}
				if (fds1[1].revents & POLLIN){
					/* Read out parameters to check for an update there, e.g. useGPS variable */
					/* read from param to clear updated flag */
					struct parameter_update_s updated;
					orb_copy(ORB_ID(parameter_update), sub_params, &updated);
					/* update parameters */
					parameters_update(&pos1D_param_handles, &pos1D_params);
				}
			}
			static int printcounter = 0;
			if (printcounter == 100) {
				printcounter = 0;
				warnx("[pos_est_mc] wait for GPS fix");
			}
			printcounter++;
		}

		/* get gps value for first initialization */
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
		lat_current = ((double)(gps.lat)) * 1e-7d;
		lon_current = ((double)(gps.lon)) * 1e-7d;
		alt_current = gps.alt * 1e-3f;
		gps_origin_altitude = alt_current;
		/* initialize coordinates */
		map_projection_init(lat_current, lon_current);
		/* publish global position messages only after first GPS message */
		printf("[pos_est_mc] initialized projection with: lat: %.10f,  lon:%.10f\n", lat_current, lon_current);

	} else {
		mavlink_log_info(mavlink_fd, "[pos_est_mc] I'm NOT using GPS - I use VICON");
		/* onboard calculated position estimations */
	}
	thread_running = true;

	struct pollfd fds2[3] = {
		{ .fd = vehicle_gps_sub,   .events = POLLIN },
		{ .fd = vicon_pos_sub,   .events = POLLIN },
		{ .fd = sub_params,   .events = POLLIN },
	};

	bool vicon_updated = false;
	bool gps_updated = false;

	/**< main_loop */
	while (!thread_should_exit) {
		int ret = poll(fds2, 3, 20);  //wait maximal this 20 ms = 50 Hz minimum rate
		if (ret < 0) {
			/* poll error */
		} else {
			if (fds2[2].revents & POLLIN){
				/* new parameter */
				/* read from param to clear updated flag */
				struct parameter_update_s updated;
				orb_copy(ORB_ID(parameter_update), sub_params, &updated);
				/* update parameters */
				parameters_update(&pos1D_param_handles, &pos1D_params);
				flag_use_baro = pos1D_params.baro;
				sigma = pos1D_params.sigma;
				addNoise = pos1D_params.addNoise;
			}
			vicon_updated = false; /* default is no vicon_updated */
			if (fds2[1].revents & POLLIN) {
				/* new vicon position */
				orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);
				posX = vicon_pos.x;
				posY = vicon_pos.y;
				posZ = vicon_pos.z;
				vicon_updated = true; /* set flag for vicon update */
			} /* end of poll call for vicon updates */
			gps_updated = false;
			if (fds2[0].revents & POLLIN) {
				/* new GPS value */
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				/* Project gps lat lon (Geographic coordinate system) to plane*/
				map_projection_project(((double)(gps.lat)) * 1e-7d, ((double)(gps.lon)) * 1e-7d, &(z[0]), &(z[1]));
				posX = z[0];
				posY = z[1];
				posZ = (float)(gps.alt * 1e-3f);
				gps_updated = true;
			}

			/* Main estimator loop */
			orb_copy(ORB_ID(actuator_controls_effective_0), actuator_eff_sub, &act_eff);
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor);
			// barometric pressure estimation at start up
			if (!flag_baro_initialized){
				// mean calculation over several measurements
				if (baro_loop_cnt<baro_loop_end) {
					p0_Pa += (sensor.baro_pres_mbar*100);
					baro_loop_cnt++;
				} else {
					p0_Pa /= (float)(baro_loop_cnt);
					flag_baro_initialized = true;
					char *baro_m_start = "barometer initialized with p0 = ";
					char p0_char[15];
					sprintf(p0_char, "%8.2f", (double)(p0_Pa/100));
					char *baro_m_end = " mbar";
					char str[80];
					strcpy(str,baro_m_start);
					strcat(str,p0_char);
					strcat(str,baro_m_end);
					mavlink_log_info(mavlink_fd, str);
				}
			}
			if (flag_use_gps) {
				/* initialize map projection with the last estimate (not at full rate) */
				if (gps.fix_type > 2) {
					/* x-y-position/velocity estimation in earth frame = gps frame */
					kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_x_aposteriori_k,posX,gps_updated,0.0f,0.0f,x_x_aposteriori);
					memcpy(x_x_aposteriori_k, x_x_aposteriori, sizeof(x_x_aposteriori));
					kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_y_aposteriori_k,posY,gps_updated,0.0f,0.0f,x_y_aposteriori);
					memcpy(x_y_aposteriori_k, x_y_aposteriori, sizeof(x_y_aposteriori));
					/* z-position/velocity estimation in earth frame = vicon frame */
					float z_est = 0.0f;
					if (flag_baro_initialized && flag_use_baro) {
						z_est = -p0_Pa*logf(p0_Pa/(sensor.baro_pres_mbar*100))/(rho0*const_earth_gravity);
						K[0] = K_vicon_50Hz[0];
						K[1] = K_vicon_50Hz[1];
						K[2] = K_vicon_50Hz[2];
						gps_updated = 1.0f; /* always enable the update, cause baro update = 200 Hz */
					} else {
						z_est = posZ;
						K[0] = K_vicon_50Hz[0];
						K[1] = K_vicon_50Hz[1];
						K[2] = K_vicon_50Hz[2];
					}

					kalman_dlqe3(dT_const_50,K[0],K[1],K[2],x_z_aposteriori_k,z_est,gps_updated,0.0f,0.0f,x_z_aposteriori);
					memcpy(x_z_aposteriori_k, x_z_aposteriori, sizeof(x_z_aposteriori));
					local_pos_est.x = x_x_aposteriori_k[0];
					local_pos_est.vx = x_x_aposteriori_k[1];
					local_pos_est.y = x_y_aposteriori_k[0];
					local_pos_est.vy = x_y_aposteriori_k[1];
					local_pos_est.z = x_z_aposteriori_k[0];
					local_pos_est.vz = x_z_aposteriori_k[1];
					local_pos_est.timestamp = hrt_absolute_time();
					if ((isfinite(x_x_aposteriori_k[0])) && (isfinite(x_x_aposteriori_k[1])) && (isfinite(x_y_aposteriori_k[0])) && (isfinite(x_y_aposteriori_k[1])) && (isfinite(x_z_aposteriori_k[0])) && (isfinite(x_z_aposteriori_k[1]))) {
						/* publish local position estimate */
						if (local_pos_est_pub > 0) {
							orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
						} else {
							local_pos_est_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos_est);
						}
						/* publish on GPS updates */
						if (gps_updated) {

							double lat, lon;
							float alt = z_est + gps_origin_altitude;

							map_projection_reproject(local_pos_est.x, local_pos_est.y, &lat, &lon);

							global_pos_est.lat = lat;
							global_pos_est.lon = lon;
							global_pos_est.alt = alt;

							if (global_pos_est_pub > 0) {
								orb_publish(ORB_ID(vehicle_global_position), global_pos_est_pub, &global_pos_est);
							} else {
								global_pos_est_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos_est);
							}
						}
					}
				}
			} else {
				/* x-y-position/velocity estimation in earth frame = vicon frame */
				kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_x_aposteriori_k,posX,vicon_updated,addNoise,sigma,x_x_aposteriori);
				memcpy(x_x_aposteriori_k, x_x_aposteriori, sizeof(x_x_aposteriori));
				kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_y_aposteriori_k,posY,vicon_updated,addNoise,sigma,x_y_aposteriori);
				memcpy(x_y_aposteriori_k, x_y_aposteriori, sizeof(x_y_aposteriori));
				/* z-position/velocity estimation in earth frame = vicon frame */
				float z_est = 0.0f;
				float local_sigma = 0.0f;
				if (flag_baro_initialized && flag_use_baro) {
					z_est = -p0_Pa*logf(p0_Pa/(sensor.baro_pres_mbar*100.0f))/(rho0*const_earth_gravity);
					K[0] = K_vicon_50Hz[0];
					K[1] = K_vicon_50Hz[1];
					K[2] = K_vicon_50Hz[2];
					vicon_updated = 1; /* always enable the update, cause baro update = 200 Hz */
					local_sigma = 0.0f; /* don't add noise on barometer in any case */
				} else {
					z_est = posZ;
					K[0] = K_vicon_50Hz[0];
					K[1] = K_vicon_50Hz[1];
					K[2] = K_vicon_50Hz[2];
					local_sigma = sigma;
				}
				kalman_dlqe3(dT_const_50,K[0],K[1],K[2],x_z_aposteriori_k,z_est,vicon_updated,addNoise,local_sigma,x_z_aposteriori);
				memcpy(x_z_aposteriori_k, x_z_aposteriori, sizeof(x_z_aposteriori));
				local_pos_est.x = x_x_aposteriori_k[0];
				local_pos_est.vx = x_x_aposteriori_k[1];
				local_pos_est.y = x_y_aposteriori_k[0];
				local_pos_est.vy = x_y_aposteriori_k[1];
				local_pos_est.z = x_z_aposteriori_k[0];
				local_pos_est.vz = x_z_aposteriori_k[1];
				local_pos_est.timestamp = hrt_absolute_time();
				if ((isfinite(x_x_aposteriori_k[0])) && (isfinite(x_x_aposteriori_k[1])) && (isfinite(x_y_aposteriori_k[0])) && (isfinite(x_y_aposteriori_k[1])) && (isfinite(x_z_aposteriori_k[0])) && (isfinite(x_z_aposteriori_k[1]))){
					if(local_pos_est_pub > 0)
						orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
					else
						local_pos_est_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos_est);
					//char buf[0xff]; sprintf(buf,"[pos_est_mc] x:%f, y:%f, z:%f",x_x_aposteriori_k[0],x_y_aposteriori_k[0],x_z_aposteriori_k[0]);
					//mavlink_log_info(mavlink_fd, buf);
				}
			}
		} /* end of poll return value check */
	}

	printf("[pos_est_mc] exiting.\n");
	mavlink_log_info(mavlink_fd, "[pos_est_mc] exiting");
	thread_running = false;
	return 0;
}
