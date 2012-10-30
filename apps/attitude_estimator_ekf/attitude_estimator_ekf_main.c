/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
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

/*
 * @file attitude_estimator_ekf_main.c
 * 
 * Extended Kalman Filter for Attitude Estimation.
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <v1.0/common/mavlink.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "codegen/attitudeKalmanfilter_initialize.h"
#include "codegen/attitudeKalmanfilter.h"
#include "attitude_estimator_ekf_params.h"

__EXPORT int attitude_estimator_ekf_main(int argc, char *argv[]);

static unsigned int loop_interval_alarm = 6500;	// loop interval in microseconds

static float dt = 0.005f;
/* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
static float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
static float x_aposteriori_k[12];		/**< states */
static float P_aposteriori_k[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
		0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
		0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
		0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
		0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
		0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
		0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
		0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
		0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
		0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
		0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
		0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f,
}; /**< init: diagonal matrix with big values */

static float x_aposteriori[12];
static float P_aposteriori[144];

/* output euler angles */
static float euler[3] = {0.0f, 0.0f, 0.0f};

static float Rot_matrix[9] = {1.f,  0,  0,
		0,  1.f,  0,
		0,  0,  1.f
};		/**< init: identity matrix */


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int attitude_estimator_ekf_task;				/**< Handle of deamon task / thread */

/**
 * Mainloop of attitude_estimator_ekf.
 */
int attitude_estimator_ekf_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: attitude_estimator_ekf {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The attitude_estimator_ekf app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int attitude_estimator_ekf_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("attitude_estimator_ekf already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		attitude_estimator_ekf_task = task_spawn("attitude_estimator_ekf",
				SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5,
				12000,
				attitude_estimator_ekf_thread_main,
				(argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tattitude_estimator_ekf app is running\n");
		} else {
			printf("\tattitude_estimator_ekf app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/*
 * [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */

/*
 * EKF Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int attitude_estimator_ekf_thread_main(int argc, char *argv[])
{
	// print text
	printf("Extended Kalman Filter Attitude Estimator initialized..\n\n");
	fflush(stdout);

	int overloadcounter = 19;

	/* Initialize filter */
	attitudeKalmanfilter_initialize();

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	uint64_t last_data = 0;
	uint64_t last_measurement = 0;

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* rate-limit raw data updates to 200Hz */
	orb_set_interval(sub_raw, 4);

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));

	/* advertise attitude */
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);


	int loopcounter = 0;
	int printcounter = 0;

	thread_running = true;

	/* advertise debug value */
	struct debug_key_value_s dbg = { .key = "", .value = 0.0f };
	orb_advert_t pub_dbg = -1;

	/* keep track of sensor updates */
	uint32_t sensor_last_count[3] = {0, 0, 0};
	uint64_t sensor_last_timestamp[3] = {0, 0, 0};
	float sensor_update_hz[3] = {0.0f, 0.0f, 0.0f};

	struct attitude_estimator_ekf_params ekf_params;

	struct attitude_estimator_ekf_param_handles ekf_param_handles;

	/* initialize parameter handles */
	parameters_init(&ekf_param_handles);

	uint64_t start_time = hrt_absolute_time();
	bool initialized = false;

	float gyro_offsets[3] = { 0.0f, 0.0f, 0.0f };
	unsigned offset_count = 0;

	/* Main loop*/
	while (!thread_should_exit) {

		struct pollfd fds[2] = {
				{ .fd = sub_raw,   .events = POLLIN },
				{ .fd = sub_params, .events = POLLIN }
		};
		int ret = poll(fds, 2, 1000);

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[attitude estimator ekf] WARNING: Not getting sensor data - sensor app running?\n");
		} else {

			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&ekf_param_handles, &ekf_params);
			}

			/* only run filter if sensor values changed */
			if (fds[0].revents & POLLIN) {

				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

				if (!initialized) {

					gyro_offsets[0] += raw.gyro_rad_s[0];
					gyro_offsets[1] += raw.gyro_rad_s[1];
					gyro_offsets[2] += raw.gyro_rad_s[2];
					offset_count++;

					if (hrt_absolute_time() - start_time > 3000000LL) {
						initialized = true;
						gyro_offsets[0] /= offset_count;
						gyro_offsets[1] /= offset_count;
						gyro_offsets[2] /= offset_count;
					}
				} else {

					/* Calculate data time difference in seconds */
					dt = (raw.timestamp - last_measurement) / 1000000.0f;
					last_measurement = raw.timestamp;
					uint8_t update_vect[3] = {0, 0, 0};

					/* Fill in gyro measurements */
					if (sensor_last_count[0] != raw.gyro_counter) {
						update_vect[0] = 1;
						sensor_last_count[0] = raw.gyro_counter;
						sensor_update_hz[0] = 1e6f / (raw.timestamp - sensor_last_timestamp[0]);
						sensor_last_timestamp[0] = raw.timestamp;
					}

					z_k[0] =  raw.gyro_rad_s[0] - gyro_offsets[0];
					z_k[1] =  raw.gyro_rad_s[1] - gyro_offsets[1];
					z_k[2] =  raw.gyro_rad_s[2] - gyro_offsets[2];

					/* update accelerometer measurements */
					if (sensor_last_count[1] != raw.accelerometer_counter) {
						update_vect[1] = 1;
						sensor_last_count[1] = raw.accelerometer_counter;
						sensor_update_hz[1] = 1e6f / (raw.timestamp - sensor_last_timestamp[1]);
						sensor_last_timestamp[1] = raw.timestamp;
					}
					z_k[3] = raw.accelerometer_m_s2[0];
					z_k[4] = raw.accelerometer_m_s2[1];
					z_k[5] = raw.accelerometer_m_s2[2];

					/* update magnetometer measurements */
					if (sensor_last_count[2] != raw.magnetometer_counter) {
						update_vect[2] = 1;
						sensor_last_count[2] = raw.magnetometer_counter;
						sensor_update_hz[2] = 1e6f / (raw.timestamp - sensor_last_timestamp[2]);
						sensor_last_timestamp[2] = raw.timestamp;
					}
					z_k[6] = raw.magnetometer_ga[0];
					z_k[7] = raw.magnetometer_ga[1];
					z_k[8] = raw.magnetometer_ga[2];

					uint64_t now = hrt_absolute_time();
					unsigned int time_elapsed = now - last_run;
					last_run = now;

					if (time_elapsed > loop_interval_alarm) {
						//TODO: add warning, cpu overload here
						// if (overloadcounter == 20) {
						// 	printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR EKF (%lu > %lu)\n", time_elapsed, loop_interval_alarm);
						// 	overloadcounter = 0;
						// }

						overloadcounter++;
					}

					int32_t z_k_sizes = 9;
					// float u[4] = {0.0f, 0.0f, 0.0f, 0.0f};

					static bool const_initialized = false;

					/* initialize with good values once we have a reasonable dt estimate */
					if (!const_initialized && dt < 0.05f && dt > 0.005f)
					{
						dt = 0.005f;
						parameters_update(&ekf_param_handles, &ekf_params);

						x_aposteriori_k[0] = z_k[0];
						x_aposteriori_k[1] = z_k[1];
						x_aposteriori_k[2] = z_k[2];
						x_aposteriori_k[3] = 0.0f;
						x_aposteriori_k[4] = 0.0f;
						x_aposteriori_k[5] = 0.0f;
						x_aposteriori_k[6] = z_k[3];
						x_aposteriori_k[7] = z_k[4];
						x_aposteriori_k[8] = z_k[5];
						x_aposteriori_k[9] = z_k[6];
						x_aposteriori_k[10] = z_k[7];
						x_aposteriori_k[11] = z_k[8];

						const_initialized = true;
					}

					/* do not execute the filter if not initialized */
					if (!const_initialized) {
						continue;
					}

					uint64_t timing_start = hrt_absolute_time();
					
					attitudeKalmanfilter(update_vect, dt, z_k, x_aposteriori_k, P_aposteriori_k, ekf_params.q, ekf_params.r,
							euler, Rot_matrix, x_aposteriori, P_aposteriori);
					/* swap values for next iteration, check for fatal inputs */
					if (isfinite(euler[0]) && isfinite(euler[1]) && isfinite(euler[2])) {
						memcpy(P_aposteriori_k, P_aposteriori, sizeof(P_aposteriori_k));
						memcpy(x_aposteriori_k, x_aposteriori, sizeof(x_aposteriori_k));
					} else {
						/* due to inputs or numerical failure the output is invalid, skip it */
						continue;
					}
					
					uint64_t timing_diff = hrt_absolute_time() - timing_start;

					// /* print rotation matrix every 200th time */
					if (printcounter % 200 == 0) {
						// printf("x apo:\n%8.4f\t%8.4f\t%8.4f\n%8.4f\t%8.4f\t%8.4f\n%8.4f\t%8.4f\t%8.4f\n",
						// 	x_aposteriori[0], x_aposteriori[1], x_aposteriori[2],
						// 	x_aposteriori[3], x_aposteriori[4], x_aposteriori[5],
						// 	x_aposteriori[6], x_aposteriori[7], x_aposteriori[8]);


						// }

						//printf("EKF attitude iteration: %d, runtime: %d us, dt: %d us (%d Hz)\n", loopcounter, (int)timing_diff, (int)(dt * 1000000.0f), (int)(1.0f / dt));
						//printf("roll: %8.4f\tpitch: %8.4f\tyaw:%8.4f\n", (double)euler[0], (double)euler[1], (double)euler[2]);
						//printf("update rates gyro: %8.4f\taccel: %8.4f\tmag:%8.4f\n", (double)sensor_update_hz[0], (double)sensor_update_hz[1], (double)sensor_update_hz[2]);
						// 	printf("\n%d\t%d\t%d\n%d\t%d\t%d\n%d\t%d\t%d\n", (int)(Rot_matrix[0] * 100), (int)(Rot_matrix[1] * 100), (int)(Rot_matrix[2] * 100),
						// 	       (int)(Rot_matrix[3] * 100), (int)(Rot_matrix[4] * 100), (int)(Rot_matrix[5] * 100),
						// 	       (int)(Rot_matrix[6] * 100), (int)(Rot_matrix[7] * 100), (int)(Rot_matrix[8] * 100));
					}

					// 				int i = printcounter % 9;

					// 	// for (int i = 0; i < 9; i++) {
					// 		char name[10];
					// 		sprintf(name, "xapo #%d", i);
					// 		memcpy(dbg.key, name, sizeof(dbg.key));
					// 		dbg.value = x_aposteriori[i];
					// if (pub_dbg < 0) {
							// pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
					// } else {
					// 		orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
					// }

					printcounter++;

					if (last_data > 0 && raw.timestamp - last_data > 12000) printf("[attitude estimator ekf] sensor data missed! (%llu)\n", raw.timestamp - last_data);

					last_data = raw.timestamp;

					/* send out */
					att.timestamp = raw.timestamp;
					att.roll = euler[0];
					att.pitch = euler[1];
					att.yaw = euler[2];

					att.rollspeed = x_aposteriori[0];
					att.pitchspeed = x_aposteriori[1];
					att.yawspeed = x_aposteriori[2];
					//att.yawspeed =z_k[2] ;
					/* copy offsets */
					memcpy(&att.rate_offsets, &(x_aposteriori[3]), sizeof(att.rate_offsets));

					/* copy rotation matrix */
					memcpy(&att.R, Rot_matrix, sizeof(Rot_matrix));
					att.R_valid = true;

					if (isfinite(att.roll) && isfinite(att.pitch) && isfinite(att.yaw)) {
						// Broadcast
						orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
					} else {
						warnx("NaN in roll/pitch/yaw estimate!");
					}
				}
			}
		}

		loopcounter++;
	}

	thread_running = false;

	return 0;
}
