/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @file multirotor_att_control_main.c
 *
 * Implementation of multirotor attitude control main loop.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
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
#include <getopt.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>

#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "multirotor_attitude_control.h"
#include "multirotor_rate_control.h"

__EXPORT int multirotor_att_control_main(int argc, char *argv[]);

static bool thread_should_exit;
static int mc_task;
static bool motor_test_mode = false;
static const float min_takeoff_throttle = 0.3f;
static const float yaw_deadzone = 0.01f;

static int
mc_thread_main(int argc, char *argv[])
{
	/* declare and safely initialize all structs */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct offboard_control_setpoint_s offboard_sp;
	memset(&offboard_sp, 0, sizeof(offboard_sp));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* subscribe */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int offboard_control_setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	int vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	orb_advert_t rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_att_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "multirotor_att_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_att_control_err");

	warnx("starting");

	/* store last control mode to detect mode switches */
	bool control_yaw_position = true;
	bool reset_yaw_sp = true;

	struct pollfd fds[1] = {
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
	};

	while (!thread_should_exit) {

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);

		} else if (ret > 0) {
			/* only run controller if attitude changed */
			perf_begin(mc_loop_perf);

			/* attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

			bool updated;

			/* parameters */
			orb_check(parameter_update_sub, &updated);

			if (updated) {
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
				/* update parameters */
			}

			/* control mode */
			orb_check(vehicle_control_mode_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub, &control_mode);
			}

			/* manual control setpoint */
			orb_check(manual_control_setpoint_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual);
			}

			/* attitude setpoint */
			orb_check(vehicle_attitude_setpoint_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
			}

			/* offboard control setpoint */
			orb_check(offboard_control_setpoint_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(offboard_control_setpoint), offboard_control_setpoint_sub, &offboard_sp);
			}

			/* vehicle status */
			orb_check(vehicle_status_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);
			}

			/* sensors */
			orb_check(sensor_combined_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);
			}

			/* set flag to safe value */
			control_yaw_position = true;

			/* reset yaw setpoint if not armed */
			if (!control_mode.flag_armed) {
				reset_yaw_sp = true;
			}

			/* define which input is the dominating control input */
			if (control_mode.flag_control_offboard_enabled) {
				/* offboard inputs */
				if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_RATES) {
					rates_sp.roll = offboard_sp.p1;
					rates_sp.pitch = offboard_sp.p2;
					rates_sp.yaw = offboard_sp.p3;
					rates_sp.thrust = offboard_sp.p4;
					rates_sp.timestamp = hrt_absolute_time();
					orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);

				} else if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE) {
					att_sp.roll_body = offboard_sp.p1;
					att_sp.pitch_body = offboard_sp.p2;
					att_sp.yaw_body = offboard_sp.p3;
					att_sp.thrust = offboard_sp.p4;
					att_sp.timestamp = hrt_absolute_time();
					/* publish the result to the vehicle actuators */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
				}

				/* reset yaw setpoint after offboard control */
				reset_yaw_sp = true;

			} else if (control_mode.flag_control_manual_enabled) {
				/* manual input */
				if (control_mode.flag_control_attitude_enabled) {
					/* control attitude, update attitude setpoint depending on mode */
					if (att_sp.thrust < 0.1f) {
						/* no thrust, don't try to control yaw */
						rates_sp.yaw = 0.0f;
						control_yaw_position = false;

						if (status.condition_landed) {
							/* reset yaw setpoint if on ground */
							reset_yaw_sp = true;
						}

					} else {
						/* only move yaw setpoint if manual input is != 0 */
						if (manual.yaw < -yaw_deadzone || yaw_deadzone < manual.yaw) {
							/* control yaw rate */
							control_yaw_position = false;
							rates_sp.yaw = manual.yaw;
							reset_yaw_sp = true;	// has no effect on control, just for beautiful log

						} else {
							control_yaw_position = true;
						}
					}

					if (!control_mode.flag_control_velocity_enabled) {
						/* update attitude setpoint if not in position control mode */
						att_sp.roll_body = manual.roll;
						att_sp.pitch_body = manual.pitch;

						if (!control_mode.flag_control_climb_rate_enabled) {
							/* pass throttle directly if not in altitude control mode */
							att_sp.thrust = manual.throttle;
						}
					}

					/* reset yaw setpint to current position if needed */
					if (reset_yaw_sp) {
						att_sp.yaw_body = att.yaw;
						reset_yaw_sp = false;
					}

					if (motor_test_mode) {
						printf("testmode");
						att_sp.roll_body = 0.0f;
						att_sp.pitch_body = 0.0f;
						att_sp.yaw_body = 0.0f;
						att_sp.thrust = 0.1f;
					}

					att_sp.timestamp = hrt_absolute_time();

					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

				} else {
					/* manual rate inputs (ACRO), from RC control or joystick */
					if (control_mode.flag_control_rates_enabled) {
						rates_sp.roll = manual.roll;
						rates_sp.pitch = manual.pitch;
						rates_sp.yaw = manual.yaw;
						rates_sp.thrust = manual.throttle;
						rates_sp.timestamp = hrt_absolute_time();
					}

					/* reset yaw setpoint after ACRO */
					reset_yaw_sp = true;
				}

			} else {
				if (!control_mode.flag_control_auto_enabled) {
					/* no control, try to stay on place */
					if (!control_mode.flag_control_velocity_enabled) {
						/* no velocity control, reset attitude setpoint */
						att_sp.roll_body = 0.0f;
						att_sp.pitch_body = 0.0f;
						att_sp.timestamp = hrt_absolute_time();
						orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
					}
				}

				/* reset yaw setpoint after non-manual control */
				reset_yaw_sp = true;
			}

			/* check if we should we reset integrals */
			bool reset_integral = !control_mode.flag_armed || att_sp.thrust < 0.1f;	// TODO use landed status instead of throttle

			/* run attitude controller if needed */
			if (control_mode.flag_control_attitude_enabled) {
				multirotor_control_attitude(&att_sp, &att, &rates_sp, control_yaw_position, reset_integral);
				orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);
			}

			/* measure in what intervals the controller runs */
			perf_count(mc_interval_perf);

			/* run rates controller if needed */
			if (control_mode.flag_control_rates_enabled) {
				/* get current rate setpoint */
				bool rates_sp_updated = false;
				orb_check(vehicle_rates_setpoint_sub, &rates_sp_updated);

				if (rates_sp_updated) {
					orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rates_setpoint_sub, &rates_sp);
				}

				/* apply controller */
				float rates[3];
				rates[0] = att.rollspeed;
				rates[1] = att.pitchspeed;
				rates[2] = att.yawspeed;
				multirotor_control_rates(&rates_sp, rates, &actuators, reset_integral);

			} else {
				/* rates controller disabled, set actuators to zero for safety */
				actuators.control[0] = 0.0f;
				actuators.control[1] = 0.0f;
				actuators.control[2] = 0.0f;
				actuators.control[3] = 0.0f;
			}

			/* fill in manual control values */
			actuators.control[4] = manual.flaps;
			actuators.control[5] = manual.aux1;
			actuators.control[6] = manual.aux2;
			actuators.control[7] = manual.aux3;

			actuators.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

			perf_end(mc_loop_perf);
		}
	}

	warnx("stopping, disarming motors");

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	close(vehicle_attitude_sub);
	close(vehicle_control_mode_sub);
	close(manual_control_setpoint_sub);
	close(actuator_pub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	exit(0);
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: multirotor_att_control [-m <mode>] [-t] {start|status|stop}\n");
	fprintf(stderr, "    <mode> is 'rates' or 'attitude'\n");
	fprintf(stderr, "    -t enables motor test mode with 10%% thrust\n");
	exit(1);
}

int multirotor_att_control_main(int argc, char *argv[])
{
	int	ch;
	unsigned int optioncount = 0;

	while ((ch = getopt(argc, argv, "tm:")) != EOF) {
		switch (ch) {
		case 't':
			motor_test_mode = true;
			optioncount += 1;
			break;

		case ':':
			usage("missing parameter");
			break;

		default:
			fprintf(stderr, "option: -%c\n", ch);
			usage("unrecognized option");
			break;
		}
	}

	argc -= optioncount;
	//argv += optioncount;

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1 + optioncount], "start")) {

		thread_should_exit = false;
		mc_task = task_spawn_cmd("multirotor_att_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 15,
					 2048,
					 mc_thread_main,
					 NULL);
		exit(0);
	}

	if (!strcmp(argv[1 + optioncount], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
