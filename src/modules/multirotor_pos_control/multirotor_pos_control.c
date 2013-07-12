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
 * @file multirotor_pos_control.c
 *
 * Multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <systemlib/systemlib.h>
#include <systemlib/pid/pid.h>
#include <mavlink/mavlink_log.h>

#include "multirotor_pos_control_params.h"
#include "thrust_pid.h"


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

static float scale_control(float ctl, float end, float dz);

static float norm(float x, float y);

static void usage(const char *reason)
{
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
int multirotor_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		warnx("start");
		thread_should_exit = false;
		deamon_task = task_spawn_cmd("multirotor_pos_control",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 60,
					     4096,
					     multirotor_pos_control_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		warnx("stop");
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("app is running");

		} else {
			warnx("app not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static float scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

static float norm(float x, float y)
{
	return sqrtf(x * x + y * y);
}

static int multirotor_pos_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	warnx("started");
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[mpc] started");

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
	struct vehicle_global_position_setpoint_s global_pos_sp;
	memset(&global_pos_sp, 0, sizeof(local_pos_sp));
	struct vehicle_global_velocity_setpoint_s global_vel_sp;
	memset(&global_vel_sp, 0, sizeof(global_vel_sp));

	/* subscribe to attitude, motor setpoints and system state */
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));

	/* publish setpoint */
	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &global_vel_sp);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	bool reset_sp_alt = true;
	bool reset_sp_pos = true;
	hrt_abstime t_prev = 0;
	/* integrate in NED frame to estimate wind but not attitude offset */
	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;
	float home_alt = 0.0f;
	hrt_abstime home_alt_t = 0;
	uint64_t local_home_timestamp = 0;

	static PID_t xy_pos_pids[2];
	static PID_t xy_vel_pids[2];
	static PID_t z_pos_pid;
	static thrust_pid_t z_vel_pid;

	thread_running = true;

	struct multirotor_position_control_params params;
	struct multirotor_position_control_param_handles params_h;
	parameters_init(&params_h);
	parameters_update(&params_h, &params);

	for (int i = 0; i < 2; i++) {
		pid_init(&(xy_pos_pids[i]), params.xy_p, 0.0f, params.xy_d, 1.0f, 0.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.02f);
		pid_init(&(xy_vel_pids[i]), params.xy_vel_p, params.xy_vel_i, params.xy_vel_d, 1.0f, params.slope_max, PID_MODE_DERIVATIV_CALC_NO_SP, 0.02f);
	}

	pid_init(&z_pos_pid, params.z_p, 0.0f, params.z_d, 1.0f, 0.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.02f);
	thrust_pid_init(&z_vel_pid, params.z_vel_p, params.z_vel_i, params.z_vel_d, -params.thr_max, -params.thr_min, PID_MODE_DERIVATIV_CALC_NO_SP, 0.02f);

	int paramcheck_counter = 0;
	bool global_pos_sp_updated = false;

	while (!thread_should_exit) {
		orb_copy(ORB_ID(vehicle_status), state_sub, &status);

		/* check parameters at 1 Hz*/
		paramcheck_counter++;

		if (paramcheck_counter == 50) {
			bool param_updated;
			orb_check(param_sub, &param_updated);

			if (param_updated) {
				parameters_update(&params_h, &params);

				for (int i = 0; i < 2; i++) {
					pid_set_parameters(&(xy_pos_pids[i]), params.xy_p, 0.0f, params.xy_d, 1.0f, 0.0f);
					pid_set_parameters(&(xy_vel_pids[i]), params.xy_vel_p, params.xy_vel_i, params.xy_vel_d, 1.0f, params.slope_max);
				}

				pid_set_parameters(&z_pos_pid, params.z_p, 0.0f, params.z_d, 1.0f, params.z_vel_max);
				thrust_pid_set_parameters(&z_vel_pid, params.z_vel_p, params.z_vel_i, params.z_vel_d, -params.thr_max, -params.thr_min);
			}

			paramcheck_counter = 0;
		}

		/* only check global position setpoint updates but not read */
		if (!global_pos_sp_updated) {
			orb_check(global_pos_sp_sub, &global_pos_sp_updated);
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
				//orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
				if (local_pos.home_timestamp != local_home_timestamp) {
					local_home_timestamp = local_pos.home_timestamp;
					/* init local projection using local position home */
					double lat_home = local_pos.home_lat * 1e-7;
					double lon_home = local_pos.home_lon * 1e-7;
					map_projection_init(lat_home, lon_home);
					warnx("local pos home: lat = %.10f,  lon = %.10f", lat_home, lon_home);
					mavlink_log_info(mavlink_fd, "local pos home: %.7f, %.7f", lat_home, lon_home);
				}

				if (global_pos_sp_updated) {
					global_pos_sp_updated = false;
					orb_copy(ORB_ID(vehicle_global_position_setpoint), global_pos_sp_sub, &global_pos_sp);
					double sp_lat = global_pos_sp.lat * 1e-7;
					double sp_lon = global_pos_sp.lon * 1e-7;
					/* project global setpoint to local setpoint */
					map_projection_project(sp_lat, sp_lon, &(local_pos_sp.x), &(local_pos_sp.y));

					if (global_pos_sp.altitude_is_relative) {
						local_pos_sp.z = -global_pos_sp.altitude;

					} else {
						local_pos_sp.z = local_pos.home_alt - global_pos_sp.altitude;
					}

					warnx("new setpoint: lat = %.10f,  lon = %.10f, x = %.2f, y = %.2f", sp_lat, sp_lon, local_pos_sp.x, local_pos_sp.y);
					mavlink_log_info(mavlink_fd, "new setpoint: %.7f, %.7f, %.2f, %.2f", sp_lat, sp_lon, local_pos_sp.x, local_pos_sp.y);
					/* publish local position setpoint as projection of global position setpoint */
					orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);
				}
			}

			if (reset_sp_alt) {
				reset_sp_alt = false;
				local_pos_sp.z = local_pos.z;
				z_vel_pid.integral = -manual.throttle;	// thrust PID uses Z downside
				mavlink_log_info(mavlink_fd, "reset alt setpoint: z = %.2f, throttle = %.2f", local_pos_sp.z, manual.throttle);
			}

			if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE && reset_sp_pos) {
				reset_sp_pos = false;
				local_pos_sp.x = local_pos.x;
				local_pos_sp.y = local_pos.y;
				xy_vel_pids[0].integral = 0.0f;
				xy_vel_pids[1].integral = 0.0f;
				mavlink_log_info(mavlink_fd, "reset pos setpoint: x = %.2f, y = %.2f", local_pos_sp.x, local_pos_sp.y);
			}

			float z_sp_offs_max = params.z_vel_max / params.z_p * 2.0f;
			float xy_sp_offs_max = params.xy_vel_max / params.xy_p * 2.0f;

			float sp_move_rate[3] = { 0.0f, 0.0f, 0.0f };

			/* manual control */
			if (status.flag_control_manual_enabled) {
				if (local_pos.home_timestamp != home_alt_t) {
					if (home_alt_t != 0) {
						/* home alt changed, don't follow large ground level changes in manual flight */
						local_pos_sp.z += local_pos.home_alt - home_alt;
					}

					home_alt_t = local_pos.home_timestamp;
					home_alt = local_pos.home_alt;
				}

				/* move altitude setpoint with manual controls */
				float z_sp_ctl = scale_control(manual.throttle - 0.5f, 0.5f, alt_ctl_dz);

				if (z_sp_ctl != 0.0f) {
					sp_move_rate[2] = -z_sp_ctl * params.z_vel_max;
					local_pos_sp.z += sp_move_rate[2] * dt;

					if (local_pos_sp.z > local_pos.z + z_sp_offs_max) {
						local_pos_sp.z = local_pos.z + z_sp_offs_max;

					} else if (local_pos_sp.z < local_pos.z - z_sp_offs_max) {
						local_pos_sp.z = local_pos.z - z_sp_offs_max;
					}
				}

				if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE) {
					/* move position setpoint with manual controls */
					float pos_pitch_sp_ctl = scale_control(-manual.pitch / params.rc_scale_pitch, 1.0f, pos_ctl_dz);
					float pos_roll_sp_ctl = scale_control(manual.roll / params.rc_scale_roll, 1.0f, pos_ctl_dz);

					if (pos_pitch_sp_ctl != 0.0f || pos_roll_sp_ctl != 0.0f) {
						/* calculate direction and increment of control in NED frame */
						float xy_sp_ctl_dir = att.yaw + atan2f(pos_roll_sp_ctl, pos_pitch_sp_ctl);
						float xy_sp_ctl_speed = norm(pos_pitch_sp_ctl, pos_roll_sp_ctl) * params.xy_vel_max;
						sp_move_rate[0] = cosf(xy_sp_ctl_dir) * xy_sp_ctl_speed;
						sp_move_rate[1] = sinf(xy_sp_ctl_dir) * xy_sp_ctl_speed;
						local_pos_sp.x += sp_move_rate[0] * dt;
						local_pos_sp.y += sp_move_rate[1] * dt;
						/* limit maximum setpoint from position offset and preserve direction */
						float pos_vec_x = local_pos_sp.x - local_pos.x;
						float pos_vec_y = local_pos_sp.y - local_pos.y;
						float pos_vec_norm = norm(pos_vec_x, pos_vec_y) / xy_sp_offs_max;

						if (pos_vec_norm > 1.0f) {
							local_pos_sp.x = local_pos.x + pos_vec_x / pos_vec_norm;
							local_pos_sp.y = local_pos.y + pos_vec_y / pos_vec_norm;
						}
					}
				}
			}

			/* run position & altitude controllers, calculate velocity setpoint */
			global_vel_sp.vz = pid_calculate(&z_pos_pid, local_pos_sp.z, local_pos.z, local_pos.vz, dt);

			if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE || status.state_machine == SYSTEM_STATE_AUTO) {
				/* calculate velocity set point in NED frame */
				global_vel_sp.vx = pid_calculate(&xy_pos_pids[0], local_pos_sp.x, local_pos.x, local_pos.vx, dt);
				global_vel_sp.vy = pid_calculate(&xy_pos_pids[1], local_pos_sp.y, local_pos.y, local_pos.vy, dt);

				/* limit horizontal speed */
				float xy_vel_sp_norm = norm(global_vel_sp.vx, global_vel_sp.vy) / params.xy_vel_max;
				if (xy_vel_sp_norm > 1.0f) {
					global_vel_sp.vx /= xy_vel_sp_norm;
					global_vel_sp.vy /= xy_vel_sp_norm;
				}

			} else {
				reset_sp_pos = true;
				global_vel_sp.vx = 0.0f;
				global_vel_sp.vy = 0.0f;
			}

			/* publish new velocity setpoint */
			orb_publish(ORB_ID(vehicle_global_velocity_setpoint), global_vel_sp_pub, &global_vel_sp);

			/* run velocity controllers, calculate thrust vector */
			float thrust_sp[3] = { 0.0f, 0.0f, 0.0f };
			thrust_sp[2] = thrust_pid_calculate(&z_vel_pid, global_vel_sp.vz, local_pos.vz, dt);

			if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE || status.state_machine == SYSTEM_STATE_AUTO) {
				/* calculate velocity set point in NED frame */
				thrust_sp[0] = pid_calculate(&xy_vel_pids[0], global_vel_sp.vx, local_pos.vx, 0.0f, dt);
				thrust_sp[1] = pid_calculate(&xy_vel_pids[1], global_vel_sp.vy, local_pos.vy, 0.0f, dt);
			}

			/* thrust_vector now contains desired acceleration (but not in m/s^2) in NED frame */
			/* limit horizontal part of thrust */
			float thrust_xy_dir = atan2f(thrust_sp[1], thrust_sp[0]);
			float thrust_xy_norm = norm(thrust_sp[0], thrust_sp[1]);

			if (thrust_xy_norm > params.slope_max) {
				thrust_xy_norm = params.slope_max;
			}

			/* use approximation: slope ~ sin(slope) = force */
			/* convert direction to body frame */
			thrust_xy_dir -= att.yaw;

			if (status.manual_sas_mode == VEHICLE_MANUAL_SAS_MODE_SIMPLE || status.state_machine == SYSTEM_STATE_AUTO) {
				/* calculate roll and pitch */
				att_sp.roll_body = sinf(thrust_xy_dir) * thrust_xy_norm;
				att_sp.pitch_body = -cosf(thrust_xy_dir) * thrust_xy_norm / cosf(att_sp.roll_body);	// reverse pitch
			}

			/* attitude-thrust compensation */
			float att_comp;

			if (att.R[2][2] > 0.8f)
				att_comp = 1.0f / att.R[2][2];
			else if (att.R[2][2] > 0.0f)
				att_comp = ((1.0f / 0.8f - 1.0f) / 0.8f) * att.R[2][2] + 1.0f;
			else
				att_comp = 1.0f;

			att_sp.thrust = -thrust_sp[2] * att_comp;
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

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[mpc] stopped");

	thread_running = false;

	fflush(stdout);
	return 0;
}

