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

static void cross3(float a[3], float b[3], float res[3])
{
	res[0] = a[1] * b[2] - a[2] * b[1];
	res[1] = a[2] * b[0] - a[0] * b[2];
	res[2] = a[0] * b[1] - a[1] * b[0];
}

static float normalize3(float x[3])
{
	float n = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);

	if (n > 0.0f) {
		x[0] /= n;
		x[1] /= n;
		x[2] /= n;
	}

	return n;
}

static float rt_atan2f_snf(float u0, float u1)
{
	float y;
	int32_t b_u0;
	int32_t b_u1;

	if (isnanf(u0) || isnanf(u1)) {
		y = NAN;

	} else if (isinff(u0) && isinff(u1)) {
		if (u0 > 0.0f) {
			b_u0 = 1;

		} else {
			b_u0 = -1;
		}

		if (u1 > 0.0f) {
			b_u1 = 1;

		} else {
			b_u1 = -1;
		}

		y = atan2f((float)b_u0, (float)b_u1);

	} else if (u1 == 0.0f) {
		if (u0 > 0.0f) {
			y = M_PI_F / 2.0f;

		} else if (u0 < 0.0f) {
			y = -(M_PI_F / 2.0f);

		} else {
			y = 0.0F;
		}

	} else {
		y = atan2f(u0, u1);
	}

	return y;
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
	PID_t z_pos_pid;
	float thrust_int[3] = { 0.0f, 0.0f, 0.0f };

	thread_running = true;

	struct multirotor_position_control_params params;
	struct multirotor_position_control_param_handles params_h;
	parameters_init(&params_h);
	parameters_update(&params_h, &params);

	for (int i = 0; i < 2; i++) {
		pid_init(&(xy_pos_pids[i]), PID_MODE_DERIVATIV_SET, 0.02f);
	}

	pid_init(&z_pos_pid, PID_MODE_DERIVATIV_SET, 0.02f);

	bool param_updated = true;

	while (!thread_should_exit) {

		if (!param_updated)
			orb_check(param_sub, &param_updated);

		if (param_updated) {
			/* clear updated flag */
			struct parameter_update_s ps;
			orb_copy(ORB_ID(parameter_update), param_sub, &ps);

			/* update params */
			parameters_update(&params_h, &params);

			for (int i = 0; i < 2; i++) {
				pid_set_parameters(&(xy_pos_pids[i]), params.xy_p, 0.0f, params.xy_d, 0.0f, 0.0f);
			}

			pid_set_parameters(&z_pos_pid, params.z_p, 0.0f, params.z_d, 1.0f, params.z_vel_max);
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

		if (control_mode.flag_control_altitude_enabled || control_mode.flag_control_position_enabled || control_mode.flag_control_climb_rate_enabled || control_mode.flag_control_velocity_enabled) {
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
							if (isfinite(global_pos_sp.yaw) && fabsf(global_pos_sp.yaw) < M_TWOPI_F) {
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

				if (!control_mode.flag_control_manual_enabled) {
					/* limit horizontal speed only in AUTO mode */
					float xy_vel_sp_norm = norm(global_vel_sp.vx, global_vel_sp.vy) / params.xy_vel_max;

					if (xy_vel_sp_norm > 1.0f) {
						global_vel_sp.vx /= xy_vel_sp_norm;
						global_vel_sp.vy /= xy_vel_sp_norm;
					}
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
				/* calculate desired thrust vector in NED frame */
				float thrust_sp[3] = { 0.0f, 0.0f, 0.0f };

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

					thrust_int[2] = -i;
					mavlink_log_info(mavlink_fd, "[mpc] reset hovering thrust: %.2f", (double)i);
				}

				thrust_sp[2] = (global_vel_sp.vz - local_pos.vz) * params.z_vel_p + thrust_int[2];

				if (control_mode.flag_control_velocity_enabled) {
					if (reset_int_xy) {
						reset_int_xy = false;
						thrust_int[0] = 0.0f;
						thrust_int[1] = 0.0f;
						mavlink_log_info(mavlink_fd, "[mpc] reset xy vel integral");
					}

					thrust_sp[0] = (global_vel_sp.vx - local_pos.vx) * params.xy_vel_p + thrust_int[0];
					thrust_sp[1] = (global_vel_sp.vy - local_pos.vy) * params.xy_vel_p + thrust_int[1];

				} else {
					reset_int_xy = true;
				}

				/* limit thrust vector and check for saturation */
				bool saturation_xy = false;
				bool saturation_z = false;

				/* limit min lift */
				if (-thrust_sp[2] < params.thr_min) {
					thrust_sp[2] = -params.thr_min;
					saturation_z = true;
				}

				/* limit max tilt */
				float tilt = atan2f(norm(thrust_sp[0], thrust_sp[1]), -thrust_sp[2]);

				if (tilt > params.tilt_max && params.thr_min > 0.0f) {
					/* crop horizontal component */
					float k = tanf(params.tilt_max) / tanf(tilt);
					thrust_sp[0] *= k;
					thrust_sp[1] *= k;
					saturation_xy = true;
				}

				/* limit max thrust */
				float thrust_abs = sqrtf(thrust_sp[0] * thrust_sp[0] + thrust_sp[1] * thrust_sp[1] + thrust_sp[2] * thrust_sp[2]);

				if (thrust_abs > params.thr_max) {
					if (thrust_sp[2] < 0.0f) {
						if (-thrust_sp[2] > params.thr_max) {
							/* thrust Z component is too large, limit it */
							thrust_sp[0] = 0.0f;
							thrust_sp[1] = 0.0f;
							thrust_sp[2] = -params.thr_max;
							saturation_xy = true;
							saturation_z = true;

						} else {
							/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
							float thrust_xy_max = sqrtf(params.thr_max * params.thr_max - thrust_sp[2] * thrust_sp[2]);
							float thrust_xy_abs = norm(thrust_sp[0], thrust_sp[1]);
							float k = thrust_xy_max / thrust_xy_abs;
							thrust_sp[0] *= k;
							thrust_sp[1] *= k;
							saturation_xy = true;
						}

					} else {
						/* Z component is negative, going down, simply limit thrust vector */
						float k = params.thr_max / thrust_abs;
						thrust_sp[0] *= k;
						thrust_sp[1] *= k;
						thrust_sp[2] *= k;
						saturation_xy = true;
						saturation_z = true;
					}

					thrust_abs = params.thr_max;
				}

				/* update integrals */
				if (control_mode.flag_control_velocity_enabled && !saturation_xy) {
					thrust_int[0] += (global_vel_sp.vx - local_pos.vx) * params.xy_vel_i * dt;
					thrust_int[1] += (global_vel_sp.vy - local_pos.vy) * params.xy_vel_i * dt;
				}

				if (control_mode.flag_control_climb_rate_enabled && !saturation_z) {
					thrust_int[2] += (global_vel_sp.vz - local_pos.vz) * params.z_vel_i * dt;
				}

				/* calculate attitude and thrust from thrust vector */
				if (control_mode.flag_control_velocity_enabled) {
					/* desired body_z axis = -normalize(thrust_vector) */
					float body_x[3];
					float body_y[3];
					float body_z[3];

					if (thrust_abs > 0.0f) {
						body_z[0] = -thrust_sp[0] / thrust_abs;
						body_z[1] = -thrust_sp[1] / thrust_abs;
						body_z[2] = -thrust_sp[2] / thrust_abs;

					} else {
						body_z[0] = 0.0f;
						body_z[1] = 0.0f;
						body_z[2] = -1.0f;
					}

					/* vector of desired yaw direction in XY plane, rotated by PI/2 */
					float y_C[3];
					y_C[0] = -sinf(att_sp.yaw_body);
					y_C[1] = cosf(att_sp.yaw_body);
					y_C[2] = 0;

					/* desired body_x axis = cross(y_C, body_z) */
					cross3(y_C, body_z, body_x);
					float body_x_norm = normalize3(body_x);
					static const float body_x_norm_max = 0.5f;

					/* desired body_y axis = cross(body_z, body_x) */
					cross3(body_z, body_x, body_y);

					if (body_x_norm < body_x_norm_max) {
						/* roll is close to +/- PI/2, don't try to hold yaw exactly */
						float x_C[3];
						x_C[0] = cos(att_sp.yaw_body);
						x_C[1] = sinf(att_sp.yaw_body);
						x_C[2] = 0;

						float body_y_1[3];
						/* desired body_y axis for approximate yaw = cross(body_z, x_C) */
						cross3(body_z, x_C, body_y_1);

						float w = body_x_norm / body_x_norm_max;
						float w1 = 1.0f - w;

						/* mix two body_y axes */
						body_y[0] = body_y[0] * w + body_y_1[0] * w1;
						body_y[1] = body_y[1] * w + body_y_1[1] * w1;
						body_y[2] = body_y[2] * w + body_y_1[2] * w1;

						normalize3(body_y);

						/* desired body_x axis = cross(body_y, body_z) */
						cross3(body_y, body_z, body_x);
					}

					/* fill rotation matrix */
					for (int i = 0; i < 3; i++) {
						att_sp.R_body[i][0] = body_x[i];
						att_sp.R_body[i][1] = body_y[i];
						att_sp.R_body[i][2] = body_z[i];
					}

					att_sp.R_valid = true;

					/* calculate roll, pitch from rotation matrix */
					att_sp.roll_body = rt_atan2f_snf(att_sp.R_body[2][1], att_sp.R_body[2][2]);
					att_sp.pitch_body = -asinf(att_sp.R_body[2][0]);
					/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */
					//att_sp.yaw_body = rt_atan2f_snf(att_sp.R_body[1][0], att_sp.R_body[0][0]);

				} else {
					/* thrust compensation for altitude only control mode */
					float att_comp;

					if (att.R[2][2] > 0.8f)
						att_comp = 1.0f / att.R[2][2];
					else if (att.R[2][2] > 0.0f)
						att_comp = ((1.0f / 0.8f - 1.0f) / 0.8f) * att.R[2][2] + 1.0f;
					else
						att_comp = 1.0f;

					thrust_abs *= att_comp;
				}

				att_sp.thrust = thrust_abs;

				att_sp.timestamp = hrt_absolute_time();

				/* publish new attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			} else {
				reset_int_z = true;
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

