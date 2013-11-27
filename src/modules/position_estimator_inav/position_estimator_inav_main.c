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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/optical_flow.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>

#include "position_estimator_inav_params.h"
#include "inertial_filter.h"

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_inav_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;

static const hrt_abstime gps_timeout = 1000000;	// GPS timeout = 1s
static const hrt_abstime flow_timeout = 1000000;	// optical flow timeout = 1s
static const uint32_t updates_counter_len = 1000000;
static const uint32_t pub_interval = 4000;	// limit publish rate to 250 Hz

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

		verbose_mode = false;

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
	float alt_avg = 0.0f;
	bool landed = true;
	hrt_abstime landed_time = 0;
	bool flag_armed = false;

	uint32_t accel_counter = 0;
	uint32_t baro_counter = 0;

	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator;
	memset(&actuator, 0, sizeof(actuator));
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
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
	int actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
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

	struct pollfd fds_init[1] = {
		{ .fd = sensor_combined_sub, .events = POLLIN },
	};

	/* wait for initial baro value */
	bool wait_baro = true;

	thread_running = true;

	while (wait_baro && !thread_should_exit) {
		int ret = poll(fds_init, 1, 1000);

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");

		} else if (ret > 0) {
			if (fds_init[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (wait_baro && sensor.baro_counter != baro_counter) {
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
						local_pos.ref_alt = baro_alt0;
						local_pos.ref_timestamp = hrt_absolute_time();
						local_pos.z_valid = true;
						local_pos.v_z_valid = true;
						local_pos.z_global = true;
					}
				}
			}
		}
	}

	bool ref_xy_inited = false;
	hrt_abstime ref_xy_init_start = 0;
	const hrt_abstime ref_xy_init_delay = 5000000;	// wait for 5s after 3D fix

	hrt_abstime t_prev = 0;

	uint16_t accel_updates = 0;
	uint16_t baro_updates = 0;
	uint16_t gps_updates = 0;
	uint16_t attitude_updates = 0;
	uint16_t flow_updates = 0;

	hrt_abstime updates_counter_start = hrt_absolute_time();
	hrt_abstime pub_last = hrt_absolute_time();

	/* acceleration in NED frame */
	float accel_NED[3] = { 0.0f, 0.0f, -CONSTANTS_ONE_G };

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float accel_corr[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float accel_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
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
	struct pollfd fds[7] = {
		{ .fd = parameter_update_sub, .events = POLLIN },
		{ .fd = actuator_sub, .events = POLLIN },
		{ .fd = armed_sub, .events = POLLIN },
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
		{ .fd = sensor_combined_sub, .events = POLLIN },
		{ .fd = optical_flow_sub, .events = POLLIN },
		{ .fd = vehicle_gps_position_sub, .events = POLLIN }
	};

	if (!thread_should_exit) {
		warnx("main loop started.");
	}

	while (!thread_should_exit) {
		int ret = poll(fds, 7, 10); // wait maximal this 10 ms = 100 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");
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

			/* actuator */
			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
			}

			/* armed */
			if (fds[2].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			}

			/* vehicle attitude */
			if (fds[3].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
				attitude_updates++;
			}

			/* sensor combined */
			if (fds[4].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.accelerometer_counter != accel_counter) {
					if (att.R_valid) {
						/* correct accel bias, now only for Z */
						sensor.accelerometer_m_s2[2] -= accel_bias[2];

						/* transform acceleration vector from body frame to NED frame */
						for (int i = 0; i < 3; i++) {
							accel_NED[i] = 0.0f;

							for (int j = 0; j < 3; j++) {
								accel_NED[i] += att.R[i][j] * sensor.accelerometer_m_s2[j];
							}
						}

						accel_corr[0] = accel_NED[0] - x_est[2];
						accel_corr[1] = accel_NED[1] - y_est[2];
						accel_corr[2] = accel_NED[2] + CONSTANTS_ONE_G - z_est[2];

					} else {
						memset(accel_corr, 0, sizeof(accel_corr));
					}

					accel_counter = sensor.accelerometer_counter;
					accel_updates++;
				}

				if (sensor.baro_counter != baro_counter) {
					baro_corr = - sensor.baro_alt_meter - z_est[0];
					baro_counter = sensor.baro_counter;
					baro_updates++;
				}
			}

			/* optical flow */
			if (fds[5].revents & POLLIN) {
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
								mavlink_log_info(mavlink_fd, "[inav] new home: alt = %.3f", baro_alt0);
								local_pos.ref_alt = baro_alt0;
								local_pos.ref_timestamp = hrt_absolute_time();
								z_est[0] += sonar_corr;
								sonar_corr = 0.0f;
								sonar_corr_filtered = 0.0f;
							}
						}
					}

				} else {
					sonar_corr = 0.0f;
				}

				flow_updates++;
			}

			/* vehicle GPS position */
			if (fds[6].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				if (gps.fix_type >= 3 && t < gps.timestamp_position + gps_timeout) {
					/* initialize reference position if needed */
					if (!ref_xy_inited) {
						/* require EPH < 10m */
						if (gps.eph_m < 10.0f) {
							if (ref_xy_init_start == 0) {
								ref_xy_init_start = t;

							} else if (t > ref_xy_init_start + ref_xy_init_delay) {
								ref_xy_inited = true;
								/* reference GPS position */
								double lat = gps.lat * 1e-7;
								double lon = gps.lon * 1e-7;

								local_pos.ref_lat = gps.lat;
								local_pos.ref_lon = gps.lon;
								local_pos.ref_timestamp = t;

								/* initialize projection */
								map_projection_init(lat, lon);
								warnx("init GPS: lat = %.10f,  lon = %.10f", lat, lon);
								mavlink_log_info(mavlink_fd, "[inav] init GPS: %.7f, %.7f", lat, lon);
							}
						} else {
							ref_xy_init_start = 0;
						}
					}

					if (ref_xy_inited) {
						/* project GPS lat lon to plane */
						float gps_proj[2];
						map_projection_project(gps.lat * 1e-7, gps.lon * 1e-7, &(gps_proj[0]), &(gps_proj[1]));
						/* calculate correction for position */
						gps_corr[0][0] = gps_proj[0] - x_est[0];
						gps_corr[1][0] = gps_proj[1] - y_est[0];

						/* calculate correction for velocity */
						if (gps.vel_ned_valid) {
							gps_corr[0][1] = gps.vel_n_m_s - x_est[1];
							gps_corr[1][1] = gps.vel_e_m_s - y_est[1];

						} else {
							gps_corr[0][1] = 0.0f;
							gps_corr[1][1] = 0.0f;
						}
					}

				} else {
					/* no GPS lock */
					memset(gps_corr, 0, sizeof(gps_corr));
					ref_xy_init_start = 0;
				}

				gps_updates++;
			}
		}

		/* end of poll return value check */

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		t_prev = t;

		/* reset ground level on arm */
		if (armed.armed && !flag_armed) {
			baro_alt0 -= z_est[0];
			z_est[0] = 0.0f;
			local_pos.ref_alt = baro_alt0;
			local_pos.ref_timestamp = hrt_absolute_time();
			mavlink_log_info(mavlink_fd, "[inav] new home on arm: alt = %.3f", baro_alt0);
		}

		/* accel bias correction, now only for Z
		 * not strictly correct, but stable and works */
		accel_bias[2] += (accel_NED[2] + CONSTANTS_ONE_G) * params.w_acc_bias * dt;

		/* inertial filter prediction for altitude */
		inertial_filter_predict(dt, z_est);

		/* inertial filter correction for altitude */
		baro_alt0 += sonar_corr * params.w_alt_sonar * dt;
		inertial_filter_correct(baro_corr + baro_alt0, dt, z_est, 0, params.w_alt_baro);
		inertial_filter_correct(sonar_corr, dt, z_est, 0, params.w_alt_sonar);
		inertial_filter_correct(accel_corr[2], dt, z_est, 2, params.w_alt_acc);

		bool gps_valid = ref_xy_inited && gps.fix_type >= 3 && t < gps.timestamp_position + gps_timeout;
		bool flow_valid = false;	// TODO implement opt flow

		/* try to estimate xy even if no absolute position source available,
		 * if using optical flow velocity will be correct in this case */
		bool can_estimate_xy = gps_valid || flow_valid;

		if (can_estimate_xy) {
			/* inertial filter prediction for position */
			inertial_filter_predict(dt, x_est);
			inertial_filter_predict(dt, y_est);

			/* inertial filter correction for position */
			inertial_filter_correct(accel_corr[0], dt, x_est, 2, params.w_pos_acc);
			inertial_filter_correct(accel_corr[1], dt, y_est, 2, params.w_pos_acc);

			if (gps_valid) {
				inertial_filter_correct(gps_corr[0][0], dt, x_est, 0, params.w_pos_gps_p);
				inertial_filter_correct(gps_corr[1][0], dt, y_est, 0, params.w_pos_gps_p);

				if (gps.vel_ned_valid && t < gps.timestamp_velocity + gps_timeout) {
					inertial_filter_correct(gps_corr[0][1], dt, x_est, 1, params.w_pos_gps_v);
					inertial_filter_correct(gps_corr[1][1], dt, y_est, 1, params.w_pos_gps_v);
				}
			}
		}

		/* detect land */
		alt_avg += (z_est[0] - alt_avg) * dt / params.land_t;
		float alt_disp = z_est[0] - alt_avg;
		alt_disp = alt_disp * alt_disp;
		float land_disp2 = params.land_disp * params.land_disp;
		/* get actual thrust output */
		float thrust = armed.armed ? actuator.control[3] : 0.0f;

		if (landed) {
			if (alt_disp > land_disp2 && thrust > params.land_thr) {
				landed = false;
				landed_time = 0;
			}

		} else {
			if (alt_disp < land_disp2 && thrust < params.land_thr) {
				if (landed_time == 0) {
					landed_time = t;    // land detected first time

				} else {
					if (t > landed_time + params.land_t * 1000000.0f) {
						landed = true;
						landed_time = 0;
					}
				}

			} else {
				landed_time = 0;
			}
		}

		if (verbose_mode) {
			/* print updates rate */
			if (t > updates_counter_start + updates_counter_len) {
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

		if (t > pub_last + pub_interval) {
			pub_last = t;
			/* publish local position */
			local_pos.timestamp = t;
			local_pos.xy_valid = can_estimate_xy && gps_valid;
			local_pos.v_xy_valid = can_estimate_xy;
			local_pos.xy_global = local_pos.xy_valid && gps_valid;	// will make sense when local position sources (e.g. vicon) will be implemented
			local_pos.x = x_est[0];
			local_pos.vx = x_est[1];
			local_pos.y = y_est[0];
			local_pos.vy = y_est[1];
			local_pos.z = z_est[0];
			local_pos.vz = z_est[1];
			local_pos.landed = landed;
			local_pos.yaw = att.yaw;

			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

			/* publish global position */
			global_pos.valid = local_pos.xy_global;

			if (local_pos.xy_global) {
				double est_lat, est_lon;
				map_projection_reproject(local_pos.x, local_pos.y, &est_lat, &est_lon);
				global_pos.lat = (int32_t)(est_lat * 1e7);
				global_pos.lon = (int32_t)(est_lon * 1e7);
				global_pos.time_gps_usec = gps.time_gps_usec;
			}

			/* set valid values even if position is not valid */
			if (local_pos.v_xy_valid) {
				global_pos.vx = local_pos.vx;
				global_pos.vy = local_pos.vy;
			}

			if (local_pos.z_valid) {
				global_pos.relative_alt = -local_pos.z;
			}

			if (local_pos.z_global) {
				global_pos.alt = local_pos.ref_alt - local_pos.z;
			}

			if (local_pos.v_z_valid) {
				global_pos.vz = local_pos.vz;
			}
			global_pos.yaw = local_pos.yaw;

			global_pos.timestamp = t;

			orb_publish(ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);
		}
		flag_armed = armed.armed;
	}

	warnx("exiting.");
	mavlink_log_info(mavlink_fd, "[inav] exiting");
	thread_running = false;
	return 0;
}
