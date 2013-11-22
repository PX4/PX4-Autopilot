/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
#include <uORB/topics/vehicle_control_mode.h>
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

	fprintf(stderr, "usage: multirotor_pos_control {start|stop|status}\n\n");
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
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
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
	memset(&global_pos_sp, 0, sizeof(global_pos_sp));
	struct vehicle_global_velocity_setpoint_s global_vel_sp;
	memset(&global_vel_sp, 0, sizeof(global_vel_sp));

	/* subscribe to attitude, motor setpoints and system state */
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
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

	bool reset_mission_sp = false;
	bool global_pos_sp_valid = false;
	bool reset_man_sp_z = true;
	bool reset_man_sp_xy = true;
	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool was_armed = false;
	bool reset_auto_sp_xy = true;
	bool reset_auto_sp_z = true;
	bool reset_takeoff_sp = true;

	hrt_abstime t_prev = 0;
	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;

	float ref_alt = 0.0f;
	hrt_abstime ref_alt_t = 0;
	uint64_t local_ref_timestamp = 0;

	PID_t xy_pos_pids[2];
	PID_t xy_vel_pids[2];
	PID_t z_pos_pid;
	thrust_pid_t z_vel_pid;

	thread_running = true;

	struct multirotor_position_control_params params;
	struct multirotor_position_control_param_handles params_h;
	parameters_init(&params_h);
	parameters_update(&params_h, &params);


	for (int i = 0; i < 2; i++) {
		pid_init(&(xy_pos_pids[i]), params.xy_p, 0.0f, params.xy_d, 1.0f, 0.0f, PID_MODE_DERIVATIV_SET, 0.02f);
		pid_init(&(xy_vel_pids[i]), params.xy_vel_p, params.xy_vel_i, params.xy_vel_d, 1.0f, params.tilt_max, PID_MODE_DERIVATIV_CALC_NO_SP, 0.02f);
	}

	pid_init(&z_pos_pid, params.z_p, 0.0f, params.z_d, 1.0f, params.z_vel_max, PID_MODE_DERIVATIV_SET, 0.02f);
	thrust_pid_init(&z_vel_pid, params.z_vel_p, params.z_vel_i, params.z_vel_d, -params.thr_max, -params.thr_min, PID_MODE_DERIVATIV_CALC_NO_SP, 0.02f);

	while (!thread_should_exit) {

		bool param_updated;
		orb_check(param_sub, &param_updated);

		if (param_updated) {
			/* clear updated flag */
			struct parameter_update_s ps;
			orb_copy(ORB_ID(parameter_update), param_sub, &ps);
			/* update params */
			parameters_update(&params_h, &params);

			for (int i = 0; i < 2; i++) {
				pid_set_parameters(&(xy_pos_pids[i]), params.xy_p, 0.0f, params.xy_d, 1.0f, 0.0f);
				/* use integral_limit_out = tilt_max / 2 */
				float i_limit;

				if (params.xy_vel_i > 0.0f) {
					i_limit = params.tilt_max / params.xy_vel_i / 2.0f;

				} else {
					i_limit = 0.0f;	// not used
				}

				pid_set_parameters(&(xy_vel_pids[i]), params.xy_vel_p, params.xy_vel_i, params.xy_vel_d, i_limit, params.tilt_max);
			}

			pid_set_parameters(&z_pos_pid, params.z_p, 0.0f, params.z_d, 1.0f, params.z_vel_max);
			thrust_pid_set_parameters(&z_vel_pid, params.z_vel_p, params.z_vel_i, params.z_vel_d, -params.thr_max, -params.thr_min);
		}

		bool updated;

		orb_check(control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
		}

		orb_check(global_pos_sp_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_global_position_setpoint), global_pos_sp_sub, &global_pos_sp);
			global_pos_sp_valid = true;
			reset_mission_sp = true;
		}

		hrt_abstime t = hrt_absolute_time();
		float dt;

		if (t_prev != 0) {
			dt = (t - t_prev) * 0.000001f;

		} else {
			dt = 0.0f;
		}

		if (control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			reset_man_sp_z = true;
			reset_man_sp_xy = true;
			reset_auto_sp_z = true;
			reset_auto_sp_xy = true;
			reset_takeoff_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
		}

		was_armed = control_mode.flag_armed;

		t_prev = t;

		if (control_mode.flag_control_altitude_enabled || control_mode.flag_control_velocity_enabled || control_mode.flag_control_position_enabled) {
			orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
			orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

			float z_sp_offs_max = params.z_vel_max / params.z_p * 2.0f;
			float xy_sp_offs_max = params.xy_vel_max / params.xy_p * 2.0f;
			float sp_move_rate[3] = { 0.0f, 0.0f, 0.0f };

			if (control_mode.flag_control_manual_enabled) {
				/* manual control */
				/* check for reference point updates and correct setpoint */
				if (local_pos.ref_timestamp != ref_alt_t) {
					if (ref_alt_t != 0) {
						/* home alt changed, don't follow large ground level changes in manual flight */
						local_pos_sp.z += local_pos.ref_alt - ref_alt;
					}

					ref_alt_t = local_pos.ref_timestamp;
					ref_alt = local_pos.ref_alt;
					// TODO also correct XY setpoint
				}

				/* reset setpoints to current position if needed */
				if (control_mode.flag_control_altitude_enabled) {
					if (reset_man_sp_z) {
						reset_man_sp_z = false;
						local_pos_sp.z = local_pos.z;
						mavlink_log_info(mavlink_fd, "[mpc] reset alt sp: %.2f", (double) - local_pos_sp.z);
					}

					/* move altitude setpoint with throttle stick */
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
				}

				if (control_mode.flag_control_position_enabled) {
					if (reset_man_sp_xy) {
						reset_man_sp_xy = false;
						local_pos_sp.x = local_pos.x;
						local_pos_sp.y = local_pos.y;
						pid_reset_integral(&xy_vel_pids[0]);
						pid_reset_integral(&xy_vel_pids[1]);
						mavlink_log_info(mavlink_fd, "[mpc] reset pos sp: %.2f, %.2f", (double)local_pos_sp.x, (double)local_pos_sp.y);
					}

					/* move position setpoint with roll/pitch stick */
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
						/* limit maximum setpoint from position offset and preserve direction
						 * fail safe, should not happen in normal operation */
						float pos_vec_x = local_pos_sp.x - local_pos.x;
						float pos_vec_y = local_pos_sp.y - local_pos.y;
						float pos_vec_norm = norm(pos_vec_x, pos_vec_y) / xy_sp_offs_max;

						if (pos_vec_norm > 1.0f) {
							local_pos_sp.x = local_pos.x + pos_vec_x / pos_vec_norm;
							local_pos_sp.y = local_pos.y + pos_vec_y / pos_vec_norm;
						}
					}
				}

				/* copy yaw setpoint to vehicle_local_position_setpoint topic */
				local_pos_sp.yaw = att_sp.yaw_body;

				/* local position setpoint is valid and can be used for auto loiter after position controlled mode */
				reset_auto_sp_xy = !control_mode.flag_control_position_enabled;
				reset_auto_sp_z = !control_mode.flag_control_altitude_enabled;
				reset_takeoff_sp = true;

				/* force reprojection of global setpoint after manual mode */
				reset_mission_sp = true;

			} else if (control_mode.flag_control_auto_enabled) {
				/* AUTO mode, use global setpoint */
				if (control_mode.auto_state == NAVIGATION_STATE_AUTO_READY) {
					reset_auto_sp_xy = true;
					reset_auto_sp_z = true;

				} else if (control_mode.auto_state == NAVIGATION_STATE_AUTO_TAKEOFF) {
					if (reset_takeoff_sp) {
						reset_takeoff_sp = false;
						local_pos_sp.x = local_pos.x;
						local_pos_sp.y = local_pos.y;
						local_pos_sp.z = - params.takeoff_alt - params.takeoff_gap;
						att_sp.yaw_body = att.yaw;
						mavlink_log_info(mavlink_fd, "[mpc] takeoff sp: %.2f %.2f %.2f", (double)local_pos_sp.x, (double)local_pos_sp.y, (double) - local_pos_sp.z);
					}

					reset_auto_sp_xy = false;
					reset_auto_sp_z = true;

				} else if (control_mode.auto_state == NAVIGATION_STATE_AUTO_RTL) {
					// TODO
					reset_auto_sp_xy = true;
					reset_auto_sp_z = true;

				} else if (control_mode.auto_state == NAVIGATION_STATE_AUTO_MISSION) {
					/* init local projection using local position ref */
					if (local_pos.ref_timestamp != local_ref_timestamp) {
						reset_mission_sp = true;
						local_ref_timestamp = local_pos.ref_timestamp;
						double lat_home = local_pos.ref_lat * 1e-7;
						double lon_home = local_pos.ref_lon * 1e-7;
						map_projection_init(lat_home, lon_home);
						mavlink_log_info(mavlink_fd, "[mpc] local pos ref: %.7f, %.7f", (double)lat_home, (double)lon_home);
					}

					if (reset_mission_sp) {
						reset_mission_sp = false;
						/* update global setpoint projection */

						if (global_pos_sp_valid) {
							/* global position setpoint valid, use it */
							double sp_lat = global_pos_sp.lat * 1e-7;
							double sp_lon = global_pos_sp.lon * 1e-7;
							/* project global setpoint to local setpoint */
							map_projection_project(sp_lat, sp_lon, &(local_pos_sp.x), &(local_pos_sp.y));

							if (global_pos_sp.altitude_is_relative) {
								local_pos_sp.z = -global_pos_sp.altitude;

							} else {
								local_pos_sp.z = local_pos.ref_alt - global_pos_sp.altitude;
							}
							/* update yaw setpoint only if value is valid */
							if (isfinite(global_pos_sp.yaw) && fabsf(global_pos_sp.yaw) < M_TWOPI) {
								att_sp.yaw_body = global_pos_sp.yaw;
							}

							mavlink_log_info(mavlink_fd, "[mpc] new sp: %.7f, %.7f (%.2f, %.2f)", (double)sp_lat, sp_lon, (double)local_pos_sp.x, (double)local_pos_sp.y);

						} else {
							if (reset_auto_sp_xy) {
								reset_auto_sp_xy = false;
								/* local position setpoint is invalid,
								 * use current position as setpoint for loiter */
								local_pos_sp.x = local_pos.x;
								local_pos_sp.y = local_pos.y;
								local_pos_sp.yaw = att.yaw;
							}

							if (reset_auto_sp_z) {
								reset_auto_sp_z = false;
								local_pos_sp.z = local_pos.z;
							}

							mavlink_log_info(mavlink_fd, "[mpc] no global pos sp, loiter: %.2f, %.2f", (double)local_pos_sp.x, (double)local_pos_sp.y);
						}
					}

					reset_auto_sp_xy = true;
					reset_auto_sp_z = true;
				}

				if (control_mode.auto_state != NAVIGATION_STATE_AUTO_TAKEOFF) {
					reset_takeoff_sp = true;
				}

				if (control_mode.auto_state != NAVIGATION_STATE_AUTO_MISSION) {
					reset_mission_sp = true;
				}

				/* copy yaw setpoint to vehicle_local_position_setpoint topic */
				local_pos_sp.yaw = att_sp.yaw_body;

				/* reset setpoints after AUTO mode */
				reset_man_sp_xy = true;
				reset_man_sp_z = true;

			} else {
				/* no control (failsafe), loiter or stay on ground */
				if (local_pos.landed) {
					/* landed: move setpoint down */
					/* in air: hold altitude */
					if (local_pos_sp.z < 5.0f) {
						/* set altitude setpoint to 5m under ground,
						 * don't set it too deep to avoid unexpected landing in case of false "landed" signal */
						local_pos_sp.z = 5.0f;
						mavlink_log_info(mavlink_fd, "[mpc] landed, set alt: %.2f", (double) - local_pos_sp.z);
					}

					reset_man_sp_z = true;

				} else {
					/* in air: hold altitude */
					if (reset_man_sp_z) {
						reset_man_sp_z = false;
						local_pos_sp.z = local_pos.z;
						mavlink_log_info(mavlink_fd, "[mpc] set loiter alt: %.2f", (double) - local_pos_sp.z);
					}

					reset_auto_sp_z = false;
				}

				if (control_mode.flag_control_position_enabled) {
					if (reset_man_sp_xy) {
						reset_man_sp_xy = false;
						local_pos_sp.x = local_pos.x;
						local_pos_sp.y = local_pos.y;
						local_pos_sp.yaw = att.yaw;
						att_sp.yaw_body = att.yaw;
						mavlink_log_info(mavlink_fd, "[mpc] set loiter pos: %.2f %.2f", (double)local_pos_sp.x, (double)local_pos_sp.y);
					}

					reset_auto_sp_xy = false;
				}
			}

			/* publish local position setpoint */
			orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);

			/* run position & altitude controllers, calculate velocity setpoint */
			if (control_mode.flag_control_altitude_enabled) {
				global_vel_sp.vz = pid_calculate(&z_pos_pid, local_pos_sp.z, local_pos.z, local_pos.vz - sp_move_rate[2], dt) + sp_move_rate[2];

			} else {
				reset_man_sp_z = true;
				global_vel_sp.vz = 0.0f;
			}

			if (control_mode.flag_control_position_enabled) {
				/* calculate velocity set point in NED frame */
				global_vel_sp.vx = pid_calculate(&xy_pos_pids[0], local_pos_sp.x, local_pos.x, local_pos.vx - sp_move_rate[0], dt) + sp_move_rate[0];
				global_vel_sp.vy = pid_calculate(&xy_pos_pids[1], local_pos_sp.y, local_pos.y, local_pos.vy - sp_move_rate[1], dt) + sp_move_rate[1];

				/* limit horizontal speed */
				float xy_vel_sp_norm = norm(global_vel_sp.vx, global_vel_sp.vy) / params.xy_vel_max;

				if (xy_vel_sp_norm > 1.0f) {
					global_vel_sp.vx /= xy_vel_sp_norm;
					global_vel_sp.vy /= xy_vel_sp_norm;
				}

			} else {
				reset_man_sp_xy = true;
				global_vel_sp.vx = 0.0f;
				global_vel_sp.vy = 0.0f;
			}

			/* publish new velocity setpoint */
			orb_publish(ORB_ID(vehicle_global_velocity_setpoint), global_vel_sp_pub, &global_vel_sp);
			// TODO subscribe to velocity setpoint if altitude/position control disabled

			if (control_mode.flag_control_climb_rate_enabled || control_mode.flag_control_velocity_enabled) {
				/* run velocity controllers, calculate thrust vector with attitude-thrust compensation */
				float thrust_sp[3] = { 0.0f, 0.0f, 0.0f };

				if (control_mode.flag_control_climb_rate_enabled) {
					if (reset_int_z) {
						reset_int_z = false;
						float i = params.thr_min;

						if (reset_int_z_manual) {
							i = manual.throttle;

							if (i < params.thr_min) {
								i = params.thr_min;

							} else if (i > params.thr_max) {
								i = params.thr_max;
							}
						}

						thrust_pid_set_integral(&z_vel_pid, -i);
						mavlink_log_info(mavlink_fd, "[mpc] reset hovering thrust: %.2f", (double)i);
					}

					thrust_sp[2] = thrust_pid_calculate(&z_vel_pid, global_vel_sp.vz, local_pos.vz, dt, att.R[2][2]);
					att_sp.thrust = -thrust_sp[2];

				} else {
					/* reset thrust integral when altitude control enabled */
					reset_int_z = true;
				}

				if (control_mode.flag_control_velocity_enabled) {
					/* calculate thrust set point in NED frame */
					if (reset_int_xy) {
						reset_int_xy = false;
						pid_reset_integral(&xy_vel_pids[0]);
						pid_reset_integral(&xy_vel_pids[1]);
						mavlink_log_info(mavlink_fd, "[mpc] reset pos integral");
					}

					thrust_sp[0] = pid_calculate(&xy_vel_pids[0], global_vel_sp.vx, local_pos.vx, 0.0f, dt);
					thrust_sp[1] = pid_calculate(&xy_vel_pids[1], global_vel_sp.vy, local_pos.vy, 0.0f, dt);

					/* thrust_vector now contains desired acceleration (but not in m/s^2) in NED frame */
					/* limit horizontal part of thrust */
					float thrust_xy_dir = atan2f(thrust_sp[1], thrust_sp[0]);
					/* assuming that vertical component of thrust is g,
					 * horizontal component = g * tan(alpha) */
					float tilt = atanf(norm(thrust_sp[0], thrust_sp[1]));

					if (tilt > params.tilt_max) {
						tilt = params.tilt_max;
					}

					/* convert direction to body frame */
					thrust_xy_dir -= att.yaw;
					/* calculate roll and pitch */
					att_sp.roll_body = sinf(thrust_xy_dir) * tilt;
					att_sp.pitch_body = -cosf(thrust_xy_dir) * tilt / cosf(att_sp.roll_body);

				} else {
					reset_int_xy = true;
				}

				att_sp.timestamp = hrt_absolute_time();

				/* publish new attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			}

		} else {
			/* position controller disabled, reset setpoints */
			reset_man_sp_z = true;
			reset_man_sp_xy = true;
			reset_int_z = true;
			reset_int_xy = true;
			reset_mission_sp = true;
			reset_auto_sp_xy = true;
			reset_auto_sp_z = true;
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = control_mode.flag_armed && control_mode.flag_control_manual_enabled && !control_mode.flag_control_climb_rate_enabled;

		/* run at approximately 50 Hz */
		usleep(20000);
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[mpc] stopped");

	thread_running = false;

	fflush(stdout);
	return 0;
}

