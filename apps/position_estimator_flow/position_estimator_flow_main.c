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
 * @file position_estimator_flow_main.c
 * Model-identification based position estimator for multirotors
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/optical_flow.h>
#include <systemlib/perf_counter.h>
#include <poll.h>

__EXPORT int position_estimator_flow_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int position_estimator_flow_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_flow_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("flow position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("position_estimator_flow",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 5,
					 4096,
					 position_estimator_flow_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tflow position estimator is running\n");
		} else {
			printf("\tflow position estimator not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int position_estimator_flow_thread_main(int argc, char *argv[])
{

	/* welcome user */
	thread_running = true;
	printf("[flow position estimator] starting\n");

	/* FIXME should be a parameter */
	static const int8_t rotM_flow_sensor[3][3] =   {{  0, 1, 0 },
													{ -1, 0, 0 },
													{  0, 0, 1 }};
	const static float const_earth_gravity = 9.81f;

	static float u[2] = {0.0f, 0.0f};
	static float speed[3] = {0.0f, 0.0f, 0.0f}; // x,y
	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	static float global_speed[3] = {0.0f, 0.0f, 0.0f};
	static uint32_t counter = 0;
	static uint64_t time_last_flow = 0; // in ms
	static float dt = 0; // seconds
	static float time_scale = pow(10,-6);

	static int baro_loop_cnt = 0;
	static int baro_loop_end = 70; /* measurement for 1 second */
	static float p0_Pa = 0.0f; /* to determin while start up */
	static float rho0 = 1.293f;
	static bool baro_initialized = false;
	static float barometer_height_est = 0.0;
	static float sonar_last = 0;
	static bool sonar_valid = false;
//	static int sonar_unaccepted_count = 0;
//	static float sonar_barometer_offset = 0.0f;

	static float sonar_lp = 0.0f;

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));

	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	/* subscribe to attitude */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* subscribe to optical flow*/
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* publish global position messages */
	struct vehicle_local_position_s local_pos = {
		.x = 0.0f,
		.y = 0.0f,
		.z = 0.0f,
		.vx = 0.0f,
		.vy = 0.0f,
		.vz = 0.0f
	};
	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "position_estimator_flow_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "position_estimator_flow_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "position_estimator_flow_err");

	printf("[flow position estimator] initialized\n");

	while (!thread_should_exit) {

		/*This runs at the rate of the sensors */
		struct pollfd fds[2] = {{ .fd = optical_flow_sub, .events = POLLIN }};
		/* wait for a sensor update, check for exit condition every 1000 ms */
		int ret = poll(fds, 2, 1000);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);

		} else if (ret == 0) {
			/* check if we're in HIL - not getting sensor data is fine then */
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);

			if (!vstatus.flag_hil_enabled) {
				fprintf(stderr,
					"[att ekf] WARNING: Not getting sensors - sensor app running?\n");
			}
			/* no return value, ignore */
			//printf("[flow position estimator] no flow \n"); // XXX wrong position
		} else {

			/* only if flow data changed */
			if (fds[0].revents & POLLIN) {

				perf_begin(mc_loop_perf);

				orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);
				/* got flow, updating attitude and status as well */
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				/* sanitize flow data */
//				if (fabs(flow.flow_comp_x_m) > 5.0f || fabs(flow.flow_comp_y_m) > 5.0f ||
//						flow.ground_distance_m > 5.0f || flow.ground_distance_m < 0.0f ||
//						flow.quality <= 0)
//				{
//					continue;
//				}

				if (vstatus.state_machine == SYSTEM_STATE_AUTO || vstatus.state_machine == SYSTEM_STATE_STANDBY) {

					/*copy flow */
					flow_speed[0] = flow.flow_comp_x_m;
					flow_speed[1] = flow.flow_comp_y_m;
					flow_speed[2] = 0.0f;

					if (!baro_initialized){
						// mean calculation over several measurements
						if(baro_loop_cnt < baro_loop_end) {
							p0_Pa += (sensor.baro_pres_mbar * 100);
							baro_loop_cnt++;
						}else{
							p0_Pa /= (float)(baro_loop_cnt);
							baro_initialized = true;

							/* send log info */
//							char *baro_m_start = "barometer initialized with p0 = ";
//							char p0_char[15];
//							sprintf(p0_char, "%8.2f", p0_Pa/100);
//							char *baro_m_end = " mbar";
//							char str[80];
//							strcpy(str,baro_m_start);
//							strcat(str,p0_char);
//							strcat(str,baro_m_end);
//							mavlink_log_info(mavlink_fd, str);
						}
					}

					if(baro_initialized){
						barometer_height_est = -p0_Pa*log(p0_Pa/(sensor.baro_pres_mbar*100))/(rho0*const_earth_gravity);
					}

					/* ignore first flow msg */
					if(time_last_flow == 0)
					{
						time_last_flow = flow.timestamp;
						continue;
					}

					/* calc dt */
					dt = (float)(flow.timestamp - time_last_flow) * time_scale ;
					time_last_flow = flow.timestamp;

					/* convert to global velocity */
					for(uint8_t i = 0; i < 3; i++) {
						float sum = 0.0f;
						for(uint8_t j = 0; j < 3; j++) {
							sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];
						}
						speed[i] = sum;
					}

//					for(uint8_t i = 0; i < 3; i++) {
//						float sum = 0.0f;
//						for(uint8_t j = 0; j < 3; j++) {
//							sum = sum + speed[j] * att.R[i][j];
//						}
//						global_speed[i] = sum;
//					}
//					local_pos.x = local_pos.x + global_speed[0] * dt;
//					local_pos.y = local_pos.y + global_speed[1] * dt;


					local_pos.x = local_pos.x + speed[0] * dt;
					local_pos.y = local_pos.y + speed[1] * dt;


					local_pos.vx = speed[0];
					local_pos.vy = speed[1];

//					if (counter % 50 = 0)
//						printf("height_diff:%.3f", height_diff);
//
//					 not working/tested with barometer
//					int sonar_accepted = 0;
//					float height_diff = sonar - barometer_height_est;
//
//					if (fabsf(height_diff) < 0.5) {
//						sonar_unaccepted_count = 0;
//						sonar_barometer_offset = 0.0f;
//					} else {
//						sonar_unaccepted_count++;
//
//						if(sonar_unaccepted_count > 10) {
//							/* add/sub 1cm to offset */
//							if (height_diff < 0)
//								sonar_barometer_offset -= 0.01;
//							else
//								sonar_barometer_offset += 0.01;
//						}
//
//						local_pos.z = sonar_last + sonar_barometer_offset;
//
//					}

				} else {

					/* reset position */
					local_pos.x = 0.0f;
					local_pos.y = 0.0f;
					/* reset velocity */
//					local_pos.vx = 0.0f;
//					local_pos.vy = 0.0f;

				}

				/* filtering ground distance */
				float sonar = - flow.ground_distance_m;
				if (sonar_valid) {
					/* simple lowpass sonar filtering */

					/* if new value or with sonar update frequency */
					if (sonar != sonar_last || counter % 10 == 0) {
						sonar_lp = 0.05 * sonar + 0.95 * sonar_lp;
						sonar_last = sonar;
					}

					float height_diff = sonar - sonar_lp;

					/* if over 1/2m spike follow lowpass */
					if (height_diff < -0.5 || height_diff > 0.5)
					{
						local_pos.z = sonar_lp;

					} else
					{
						local_pos.z = sonar;
					}

				} else {
					/* first sonar input sanitation */
					if (sonar < 2.0f) {
						sonar_valid = true;
					}
					local_pos.z = -0.3f;

				}

				local_pos.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_pos);


				/* measure in what intervals the position estimator runs */
				perf_count(mc_interval_perf);
				perf_end(mc_loop_perf);

			}
		}

		counter++;
	}

	printf("[flow position estimator] exiting.\n");
	thread_running = false;

	close(vehicle_attitude_sub);
	close(vehicle_status_sub);
	close(optical_flow_sub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}


