/****************************************************************************
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
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
 * @file position_estimator_inav_main.c
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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/optical_flow.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <systemlib/geo/geo.h>
#include <systemlib/systemlib.h>
#include <systemlib/conversions.h>
#include <drivers/drv_hrt.h>

#include "position_estimator_inav_params.h"
#include "inertial_filter.h"

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_inav_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;

__EXPORT int position_estimator_inav_main(int argc, char *argv[]);

int position_estimator_inav_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr,
		"usage: position_estimator_inav {start|stop|status} [-v]\n\n");
	exit(1);
}

/**
 * The position_estimator_inav_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_inav_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			printf("position_estimator_inav already running\n");
			/* this is not an error */
			exit(0);
		}

		if (argc > 1)
			if (!strcmp(argv[2], "-v"))
				verbose_mode = true;

		thread_should_exit = false;
		position_estimator_inav_task = task_spawn_cmd("position_estimator_inav",
					       SCHED_RR, SCHED_PRIORITY_MAX - 5, 4096,
					       position_estimator_inav_thread_main,
					       (argv) ? (const char **) &argv[2] : (const char **) NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tposition_estimator_inav is running\n");

		} else {
			printf("\tposition_estimator_inav not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator_inav_thread_main(int argc, char *argv[])
{
	warnx("started.");
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[inav] started");

	/* initialize values */
	float x_est[3] = { 0.0f, 0.0f, 0.0f };
	float y_est[3] = { 0.0f, 0.0f, 0.0f };
	float z_est[3] = { 0.0f, 0.0f, 0.0f };

	int baro_init_cnt = 0;
	int baro_init_num = 200;
	float baro_alt0 = 0.0f; /* to determine while start up */

	double lat_current = 0.0; //[°]] --> 47.0
	double lon_current = 0.0; //[°]] -->8.5
	double alt_current = 0.0; //[m] above MSL

	uint32_t accel_counter = 0;
	uint32_t baro_counter = 0;

	/* declare and safely initialize all structs */
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));
	/* make sure that baroINITdone = false */
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));

	/* subscribe */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	/* advertise */
	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
	orb_advert_t vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

	struct position_estimator_inav_params params;
	struct position_estimator_inav_param_handles pos_inav_param_handles;
	/* initialize parameter handles */
	parameters_init(&pos_inav_param_handles);

	/* first parameters read at start up */
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update); /* read from param topic to clear updated flag */
	/* first parameters update */
	parameters_update(&pos_inav_param_handles, &params);

	struct pollfd fds_init[2] = {
		{ .fd = sensor_combined_sub, .events = POLLIN },
		{ .fd = vehicle_gps_position_sub, .events = POLLIN }
	};

	/* wait for initial sensors values: baro, GPS fix, only then can we initialize the projection */
	bool wait_gps = params.use_gps;
	bool wait_baro = true;
	hrt_abstime wait_gps_start = 0;
	const hrt_abstime wait_gps_delay = 5000000;	// wait for 5s after 3D fix

	thread_running = true;

	while ((wait_gps || wait_baro) && !thread_should_exit) {
		int ret = poll(fds_init, params.use_gps ? 2 : 1, 1000);

		if (ret < 0) {
			/* poll error */
			errx(1, "subscriptions poll error on init.");

		} else if (ret > 0) {
			if (fds_init[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (wait_baro && sensor.baro_counter > baro_counter) {
					baro_counter = sensor.baro_counter;

					/* mean calculation over several measurements */
					if (baro_init_cnt < baro_init_num) {
						baro_alt0 += sensor.baro_alt_meter;
						baro_init_cnt++;

					} else {
						wait_baro = false;
						baro_alt0 /= (float) baro_init_cnt;
						warnx("init baro: alt = %.3f", baro_alt0);
						mavlink_log_info(mavlink_fd, "[inav] init baro: alt = %.3f", baro_alt0);
						local_pos.home_alt = baro_alt0;
						local_pos.home_timestamp = hrt_absolute_time();
					}
				}
			}

			if (params.use_gps && (fds_init[1].revents & POLLIN)) {
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				if (wait_gps && gps.fix_type >= 3) {
					hrt_abstime t = hrt_absolute_time();

					if (wait_gps_start == 0) {
						wait_gps_start = t;

					} else if (t - wait_gps_start > wait_gps_delay) {
						wait_gps = false;
						/* get GPS position for first initialization */
						lat_current = gps.lat * 1e-7;
						lon_current = gps.lon * 1e-7;
						alt_current = gps.alt * 1e-3;

						local_pos.home_lat = lat_current * 1e7;
						local_pos.home_lon = lon_current * 1e7;
						local_pos.home_hdg = 0.0f;
						local_pos.home_timestamp = t;

						/* initialize coordinates */
						map_projection_init(lat_current, lon_current);
						warnx("init GPS: lat = %.10f,  lon = %.10f", lat_current, lon_current);
						mavlink_log_info(mavlink_fd, "[inav] init GPS: %.7f, %.7f", lat_current, lon_current);
					}
				}
			}
		}
	}

	hrt_abstime t_prev = 0;

	uint16_t accel_updates = 0;
	uint16_t baro_updates = 0;
	uint16_t gps_updates = 0;
	uint16_t attitude_updates = 0;
	uint16_t flow_updates = 0;

	hrt_abstime updates_counter_start = hrt_absolute_time();
	uint32_t updates_counter_len = 1000000;

	hrt_abstime pub_last = hrt_absolute_time();
	uint32_t pub_interval = 4000;	// limit publish rate to 250 Hz

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float accel_corr[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float baro_corr = 0.0f;		// D
	float gps_corr[2][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
	};
	float sonar_corr = 0.0f;
	float sonar_corr_filtered = 0.0f;
	float flow_corr[] = { 0.0f, 0.0f };	// X, Y

	float sonar_prev = 0.0f;
	hrt_abstime sonar_time = 0;

	/* main loop */
	struct pollfd fds[6] = {
		{ .fd = parameter_update_sub, .events = POLLIN },
		{ .fd = vehicle_status_sub, .events = POLLIN },
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
		{ .fd = sensor_combined_sub, .events = POLLIN },
		{ .fd = optical_flow_sub, .events = POLLIN },
		{ .fd = vehicle_gps_position_sub, .events = POLLIN }
	};

	if (!thread_should_exit) {
		warnx("main loop started.");
	}

	while (!thread_should_exit) {
		int ret = poll(fds, params.use_gps ? 6 : 5, 10); // wait maximal this 10 ms = 100 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			warnx("subscriptions poll error.");
			thread_should_exit = true;
			continue;

		} else if (ret > 0) {
			/* parameter update */
			if (fds[0].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub,
					 &update);
				/* update parameters */
				parameters_update(&pos_inav_param_handles, &params);
			}

			/* vehicle status */
			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub,
					 &vehicle_status);
			}

			/* vehicle attitude */
			if (fds[2].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
				attitude_updates++;
			}

			/* sensor combined */
			if (fds[3].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.accelerometer_counter > accel_counter) {
					if (att.R_valid) {
						/* transform acceleration vector from body frame to NED frame */
						float accel_NED[3];

						for (int i = 0; i < 3; i++) {
							accel_NED[i] = 0.0f;

							for (int j = 0; j < 3; j++) {
								accel_NED[i] += att.R[i][j] * sensor.accelerometer_m_s2[j];
							}
						}

						accel_NED[2] += CONSTANTS_ONE_G;
						accel_corr[0] = accel_NED[0] - x_est[2];
						accel_corr[1] = accel_NED[1] - y_est[2];
						accel_corr[2] = accel_NED[2] - z_est[2];

					} else {
						memset(accel_corr, 0, sizeof(accel_corr));
					}

					accel_counter = sensor.accelerometer_counter;
					accel_updates++;
				}

				if (sensor.baro_counter > baro_counter) {
					baro_corr = - sensor.baro_alt_meter - z_est[0];
					baro_counter = sensor.baro_counter;
					baro_updates++;
				}
			}

			/* optical flow */
			if (fds[4].revents & POLLIN) {
				orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);

				if (flow.ground_distance_m > 0.31f && flow.ground_distance_m < 4.0f && (flow.ground_distance_m != sonar_prev || t - sonar_time < 150000)) {
					if (flow.ground_distance_m != sonar_prev) {
						sonar_time = t;
						sonar_prev = flow.ground_distance_m;
						sonar_corr = -flow.ground_distance_m - z_est[0];
						sonar_corr_filtered += (sonar_corr - sonar_corr_filtered) * params.sonar_filt;

						if (fabsf(sonar_corr) > params.sonar_err) {
							// correction is too large: spike or new ground level?
							if (fabsf(sonar_corr - sonar_corr_filtered) > params.sonar_err) {
								// spike detected, ignore
								sonar_corr = 0.0f;

							} else {
								// new ground level
								baro_alt0 += sonar_corr;
								warnx("new home: alt = %.3f", baro_alt0);
								mavlink_log_info(mavlink_fd, "[inav] new home: alt = %.3f", baro_alt0);
								local_pos.home_alt = baro_alt0;
								local_pos.home_timestamp = hrt_absolute_time();
								z_est[0] += sonar_corr;
								sonar_corr = 0.0f;
								sonar_corr_filtered = 0.0;
							}
						}
					}

				} else {
					sonar_corr = 0.0f;
				}

				flow_updates++;
			}

			if (params.use_gps && (fds[5].revents & POLLIN)) {
				/* vehicle GPS position */
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				if (gps.fix_type >= 3) {
					/* project GPS lat lon to plane */
					float gps_proj[2];
					map_projection_project(gps.lat * 1e-7, gps.lon * 1e-7, &(gps_proj[0]), &(gps_proj[1]));
					gps_corr[0][0] = gps_proj[0] - x_est[0];
					gps_corr[1][0] = gps_proj[1] - y_est[0];

					if (gps.vel_ned_valid) {
						gps_corr[0][1] = gps.vel_n_m_s;
						gps_corr[1][1] = gps.vel_e_m_s;

					} else {
						gps_corr[0][1] = 0.0f;
						gps_corr[1][1] = 0.0f;
					}

					gps_updates++;

				} else {
					memset(gps_corr, 0, sizeof(gps_corr));
				}
			}
		}

		/* end of poll return value check */

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		t_prev = t;

		/* inertial filter prediction for altitude */
		inertial_filter_predict(dt, z_est);

		/* inertial filter correction for altitude */
		baro_alt0 += sonar_corr * params.w_alt_sonar * dt;
		inertial_filter_correct(baro_corr + baro_alt0, dt, z_est, 0, params.w_alt_baro);
		inertial_filter_correct(sonar_corr, dt, z_est, 0, params.w_alt_sonar);
		inertial_filter_correct(accel_corr[2], dt, z_est, 2, params.w_alt_acc);

		/* dont't try to estimate position when no any position source available */
		bool can_estimate_pos = params.use_gps && gps.fix_type >= 3;

		if (can_estimate_pos) {
			/* inertial filter prediction for position */
			inertial_filter_predict(dt, x_est);
			inertial_filter_predict(dt, y_est);

			/* inertial filter correction for position */
			inertial_filter_correct(accel_corr[0], dt, x_est, 2, params.w_pos_acc);
			inertial_filter_correct(accel_corr[1], dt, y_est, 2, params.w_pos_acc);

			if (params.use_gps && gps.fix_type >= 3) {
				inertial_filter_correct(gps_corr[0][0], dt, x_est, 0, params.w_pos_gps_p);
				inertial_filter_correct(gps_corr[1][0], dt, y_est, 0, params.w_pos_gps_p);

				if (gps.vel_ned_valid) {
					inertial_filter_correct(gps_corr[0][1], dt, x_est, 1, params.w_pos_gps_v);
					inertial_filter_correct(gps_corr[1][1], dt, y_est, 1, params.w_pos_gps_v);
				}
			}
		}

		if (verbose_mode) {
			/* print updates rate */
			if (t - updates_counter_start > updates_counter_len) {
				float updates_dt = (t - updates_counter_start) * 0.000001f;
				warnx(
					"updates rate: accelerometer = %.1f/s, baro = %.1f/s, gps = %.1f/s, attitude = %.1f/s, flow = %.1f/s",
					accel_updates / updates_dt,
					baro_updates / updates_dt,
					gps_updates / updates_dt,
					attitude_updates / updates_dt,
					flow_updates / updates_dt);
				updates_counter_start = t;
				accel_updates = 0;
				baro_updates = 0;
				gps_updates = 0;
				attitude_updates = 0;
				flow_updates = 0;
			}
		}

		if (t - pub_last > pub_interval) {
			pub_last = t;
			local_pos.timestamp = t;
			local_pos.valid = can_estimate_pos;
			local_pos.x = x_est[0];
			local_pos.vx = x_est[1];
			local_pos.y = y_est[0];
			local_pos.vy = y_est[1];
			local_pos.z = z_est[0];
			local_pos.vz = z_est[1];
			local_pos.absolute_alt = local_pos.home_alt - local_pos.z;
			local_pos.hdg = att.yaw;

			if ((isfinite(local_pos.x)) && (isfinite(local_pos.vx))
			    && (isfinite(local_pos.y))
			    && (isfinite(local_pos.vy))
			    && (isfinite(local_pos.z))
			    && (isfinite(local_pos.vz))) {
				orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

				if (params.use_gps) {
					global_pos.valid = local_pos.valid;
					double est_lat, est_lon;
					map_projection_reproject(local_pos.x, local_pos.y, &est_lat, &est_lon);
					global_pos.lat = (int32_t)(est_lat * 1e7);
					global_pos.lon = (int32_t)(est_lon * 1e7);
					global_pos.alt = local_pos.home_alt - local_pos.z;
					global_pos.relative_alt = -local_pos.z;
					global_pos.vx = local_pos.vx;
					global_pos.vy = local_pos.vy;
					global_pos.vz = local_pos.vz;
					global_pos.hdg = local_pos.hdg;

					orb_publish(ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);
				}
			}
		}
	}

	warnx("exiting.");
	mavlink_log_info(mavlink_fd, "[inav] exiting");
	thread_running = false;
	return 0;
}
