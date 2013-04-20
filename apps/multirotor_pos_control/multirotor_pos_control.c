/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file multirotor_pos_control.c
 *
 * Multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <systemlib/systemlib.h>
#include <mavlink/mavlink_log.h>

#include "multirotor_pos_control_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int multirotor_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int multirotor_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason) {
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int multirotor_pos_control_main(int argc, char *argv[]) {
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("multirotor_pos_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("multirotor_pos_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 60,
					 4096,
					 multirotor_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmultirotor_pos_control app is running\n");
		} else {
			printf("\tmultirotor_pos_control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int multirotor_pos_control_thread_main(int argc, char *argv[]) {
	/* welcome user */
	printf("[multirotor_pos_control] started\n");
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[multirotor_pos_control] started");

	/* structures */
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));

	/* subscribe to attitude, motor setpoints and system state */
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	/* publish setpoint */
	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	bool reset_sp_alt = true;
	bool reset_sp_pos = true;
	hrt_abstime t_prev = 0;
	float alt_integral = 0.0f;
	float alt_ctl_dz = 0.2f;

	thread_running = true;

	struct multirotor_position_control_params params;
	struct multirotor_position_control_param_handles params_h;
	parameters_init(&params_h);
	parameters_update(&params_h, &params);

	int paramcheck_counter = 0;

	while (!thread_should_exit) {
		orb_copy(ORB_ID(vehicle_status), state_sub, &status);

		/* check parameters at 1 Hz*/
		paramcheck_counter++;
		if (paramcheck_counter == 50) {
			bool param_updated;
			orb_check(param_sub, &param_updated);
			if (param_updated) {
				parameters_update(&params_h, &params);
			}
			paramcheck_counter = 0;
		}

		/* Check if controller should act */
		bool act = status.flag_system_armed && (
		   		       /* SAS modes */
				   	   (
				   		   status.flag_control_manual_enabled &&
						   status.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_SAS && (
								   status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_ALTITUDE ||
								   status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE
						   )
				   	   ) ||
				   	   /* AUTO mode */
				   	   status.state_machine == SYSTEM_STATE_AUTO
				   );

		hrt_abstime t = hrt_absolute_time();
		float dt;
		if (t_prev != 0) {
			dt = (t - t_prev) * 0.000001f;
		} else {
			dt = 0.0f;
		}
		t_prev = t;
		if (act) {
			orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
			orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
			if (status.state_machine == SYSTEM_STATE_AUTO) {
				orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
			}

			if (reset_sp_alt) {
				reset_sp_alt = false;
				local_pos_sp.z = local_pos.z;
				alt_integral = manual.throttle;
				char str[80];
				sprintf(str, "reset alt setpoint: z = %.2f, throttle = %.2f", local_pos_sp.z, manual.throttle);
				mavlink_log_info(mavlink_fd, str);
			}

			if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE && reset_sp_pos) {
				reset_sp_pos = false;
				local_pos_sp.x = local_pos.x;
				local_pos_sp.y = local_pos.y;
				char str[80];
				sprintf(str, "reset pos setpoint: x = %.2f, y = %.2f", local_pos_sp.x, local_pos_sp.y);
				mavlink_log_info(mavlink_fd, str);
			}

			float err_linear_limit = params.alt_d / params.alt_p * params.alt_rate_max;

			if (status.flag_control_manual_enabled) {
				/* move altitude setpoint with manual controls */
				float alt_ctl = manual.throttle - 0.5f;
				if (fabs(alt_ctl) < alt_ctl_dz) {
					alt_ctl = 0.0f;
				} else {
					if (alt_ctl > 0.0f)
						alt_ctl = (alt_ctl - alt_ctl_dz) / (0.5f - alt_ctl_dz);
					else
						alt_ctl = (alt_ctl + alt_ctl_dz) / (0.5f - alt_ctl_dz);
					local_pos_sp.z -= alt_ctl * params.alt_rate_max * dt;
					if (local_pos_sp.z > local_pos.z + err_linear_limit)
						local_pos_sp.z = local_pos.z + err_linear_limit;
					else if (local_pos_sp.z < local_pos.z - err_linear_limit)
						local_pos_sp.z = local_pos.z - err_linear_limit;
				}

				if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE) {
					// TODO move position setpoint with manual controls
				}
			}

			/* PID for altitude */
			float alt_err = local_pos.z - local_pos_sp.z;
			/* don't accelerate more than ALT_RATE_MAX, limit error to corresponding value */
			if (alt_err > err_linear_limit) {
				alt_err = err_linear_limit;
			} else if (alt_err < -err_linear_limit) {
				alt_err = -err_linear_limit;
			}
			/* P and D components */
			float thrust_ctl_pd = alt_err * params.alt_p + local_pos.vz * params.alt_d;
			/* add I component */
			float thrust_ctl = thrust_ctl_pd + alt_integral;
			alt_integral += thrust_ctl_pd / params.alt_p * params.alt_i * dt;
			if (thrust_ctl < params.thr_min) {
				thrust_ctl = params.thr_min;
			} else if (thrust_ctl > params.thr_max) {
				thrust_ctl = params.thr_max;
			}
			if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE || status.state_machine == SYSTEM_STATE_AUTO) {
				// TODO add position controller
			} else {
				reset_sp_pos = true;
			}
			att_sp.thrust = thrust_ctl;
			att_sp.timestamp = hrt_absolute_time();
			if (status.flag_control_manual_enabled) {
				/* publish local position setpoint in manual mode */
				orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);
			}
			/* publish new attitude setpoint */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
		} else {
			reset_sp_alt = true;
			reset_sp_pos = true;
		}

		/* run at approximately 50 Hz */
		usleep(20000);

	}

	printf("[multirotor_pos_control] exiting\n");
	mavlink_log_info(mavlink_fd, "[multirotor_pos_control] exiting");

	thread_running = false;

	fflush(stdout);
	return 0;
}

