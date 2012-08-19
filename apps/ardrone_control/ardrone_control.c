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

/*
 * @file Implementation of AR.Drone 1.0 / 2.0 control interface
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include "ardrone_control.h"
#include "attitude_control.h"
#include "rate_control.h"
#include "ardrone_motor_control.h"
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/ardrone_control.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/ardrone_motors_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int ardrone_control_main(int argc, char *argv[]);

// static void turn_xy_plane(const float_vect3 *vector, float yaw,
// 		   float_vect3 *result);
// static void navi2body_xy_plane(const float_vect3 *vector, const float yaw,
// 			float_vect3 *result);

// static void turn_xy_plane(const float_vect3 *vector, float yaw,
// 		   float_vect3 *result)
// {
// 	//turn clockwise
// 	static uint16_t counter;

// 	result->x = (cosf(yaw) * vector->x + sinf(yaw) * vector->y);
// 	result->y = (-sinf(yaw) * vector->x + cosf(yaw) * vector->y);
// 	result->z = vector->z; //leave direction normal to xy-plane untouched

// 	counter++;
// }

// static void navi2body_xy_plane(const float_vect3 *vector, const float yaw,
// 			float_vect3 *result)
// {
// 	turn_xy_plane(vector, yaw, result);
// //	result->x = vector->x;
// //	result->y = vector->y;
// //	result->z = vector->z;
// 	//	result->x = cos(yaw) * vector->x + sin(yaw) * vector->y;
// 	//	result->y = -sin(yaw) * vector->x + cos(yaw) * vector->y;
// 	//	result->z = vector->z; //leave direction normal to xy-plane untouched
// }

int ardrone_control_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[ardrone_control] Control started, taking over motors\n");

	/* default values for arguments */
	char *ardrone_uart_name = "/dev/ttyS1";

	/* File descriptors */
	int ardrone_write;
	int gpios;

	enum {
		CONTROL_MODE_RATES = 0,
		CONTROL_MODE_ATTITUDE = 1,
	} control_mode = CONTROL_MODE_ATTITUDE;

	char *commandline_usage = "\tusage: ardrone_control -d ardrone-devicename -m mode\n\tmodes are:\n\t\trates\n\t\tattitude\n";

	bool motor_test_mode = false;

	/* read commandline arguments */
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //ardrone set
			if (argc > i + 1) {
				ardrone_uart_name = argv[i + 1];
			} else {
				printf(commandline_usage);
				return ERROR;
			}

		} else if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
			if (argc > i + 1) {
				if (strcmp(argv[i + 1], "rates") == 0) {
					control_mode = CONTROL_MODE_RATES;

				} else if (strcmp(argv[i + 1], "attitude") == 0) {
					control_mode = CONTROL_MODE_ATTITUDE;

				} else {
					printf(commandline_usage);
					return ERROR;
				}

			} else {
				printf(commandline_usage);
				return ERROR;
			}

		} else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
			motor_test_mode = true;
		}
	}

	/* open uarts */
	printf("[ardrone_control] AR.Drone UART is %s\n", ardrone_uart_name);
	ardrone_write = open(ardrone_uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (ardrone_write < 0) {
		fprintf(stderr, "[ardrone_control] Failed opening AR.Drone UART, exiting.\n");
		exit(ERROR);
	}

	/* initialize motors */
	if (OK != ar_init_motors(ardrone_write, &gpios)) {
		close(ardrone_write);
		fprintf(stderr, "[ardrone_control] Failed initializing AR.Drone motors, exiting.\n");
		exit(ERROR);
	}

	/* Led animation */
	int counter = 0;
	int led_counter = 0;

	/* declare and safely initialize all structs */
	struct vehicle_status_s state;
	memset(&state, 0, sizeof(state));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct ardrone_motors_setpoint_s setpoint;
	memset(&setpoint, 0, sizeof(setpoint));
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int setpoint_sub = orb_subscribe(ORB_ID(ardrone_motors_setpoint));

	while (1) {

		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);

		if (state.state_machine == SYSTEM_STATE_MANUAL ||
			state.state_machine == SYSTEM_STATE_GROUND_READY ||
			state.state_machine == SYSTEM_STATE_STABILIZED ||
			state.state_machine == SYSTEM_STATE_AUTO ||
			state.state_machine == SYSTEM_STATE_MISSION_ABORT ||
			state.state_machine == SYSTEM_STATE_EMCY_LANDING ||
			motor_test_mode) {

			if (control_mode == CONTROL_MODE_RATES) {
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);
				orb_copy(ORB_ID(ardrone_motors_setpoint), setpoint_sub, &setpoint);
				control_rates(ardrone_write, &raw, &setpoint);

			} else if (control_mode == CONTROL_MODE_ATTITUDE) {

				// XXX Add failsafe logic for RC loss situations
				/* hardcore, last-resort safety checking */
				//if (status->rc_signal_lost) {

				/* get a local copy of manual setpoint */
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				/* get a local copy of attitude setpoint */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);

				att_sp.roll_body = -manual.roll * M_PI_F / 8.0f;
				att_sp.pitch_body = -manual.pitch * M_PI_F / 8.0f;
				att_sp.yaw_body = -manual.yaw * M_PI_F;
				if (motor_test_mode) {
					att_sp.roll_body = 0.0f;
					att_sp.pitch_body = 0.0f;
					att_sp.yaw_body = 0.0f;
					att_sp.thrust = 0.3f;
				} else {
					if (state.state_machine == SYSTEM_STATE_MANUAL ||
						state.state_machine == SYSTEM_STATE_GROUND_READY ||
						state.state_machine == SYSTEM_STATE_STABILIZED ||
						state.state_machine == SYSTEM_STATE_AUTO ||
						state.state_machine == SYSTEM_STATE_MISSION_ABORT ||
						state.state_machine == SYSTEM_STATE_EMCY_LANDING) {
						att_sp.thrust = manual.throttle;

					} else if (state.state_machine == SYSTEM_STATE_EMCY_CUTOFF) {
						/* immediately cut off motors */
						att_sp.thrust = 0.0f;

					} else {
						/* limit motor throttle to zero for an unknown mode */
						att_sp.thrust = 0.0f;
					}
					
				}

				float roll_control, pitch_control, yaw_control, thrust_control;

				multirotor_control_attitude(&att_sp, &att, &state, &actuator_controls, motor_test_mode);
				ardrone_mixing_and_output(ardrone_write, &actuator_controls, motor_test_mode);

			} else {
				/* invalid mode, complain */
				if (counter % 200 == 0) printf("[multirotor control] INVALID CONTROL MODE, locking down propulsion\n");
				ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);
			}
		} else {
			/* Silently lock down motor speeds to zero */
			ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);
		}

		if (counter % 30 == 0) {
			if (led_counter == 0) ar_set_leds(ardrone_write, 0, 1, 0, 0, 0, 0, 0 , 0);

			if (led_counter == 1) ar_set_leds(ardrone_write, 1, 1, 0, 0, 0, 0, 0 , 0);

			if (led_counter == 2) ar_set_leds(ardrone_write, 1, 0, 0, 0, 0, 0, 0 , 0);

			if (led_counter == 3) ar_set_leds(ardrone_write, 0, 0, 0, 1, 0, 0, 0 , 0);

			if (led_counter == 4) ar_set_leds(ardrone_write, 0, 0, 1, 1, 0, 0, 0 , 0);

			if (led_counter == 5) ar_set_leds(ardrone_write, 0, 0, 1, 0, 0, 0, 0 , 0);

			if (led_counter == 6) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 1, 0 , 0);

			if (led_counter == 7) ar_set_leds(ardrone_write, 0, 0, 0, 0, 1, 1, 0 , 0);

			if (led_counter == 8) ar_set_leds(ardrone_write, 0, 0, 0, 0, 1, 0, 0 , 0);

			if (led_counter == 9) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 0 , 1);

			if (led_counter == 10) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 1 , 1);

			if (led_counter == 11) ar_set_leds(ardrone_write, 0, 0, 0, 0, 0, 0, 1 , 0);

			led_counter++;

			if (led_counter == 12) led_counter = 0;
		}

		/* run at approximately 200 Hz */
		usleep(5000);

		// This is a hardcore debug code piece to validate
		// the motor interface
		// uint8_t motorSpeedBuf[5] = {1, 2, 3, 4, 5};
		// ar_get_motor_packet(motorSpeedBuf, 20, 20, 20, 20);
		// write(ardrone_write, motorSpeedBuf, 5);
		// usleep(15000);

		counter++;
	}

	/* close uarts */
	close(ardrone_write);
	ar_multiplexing_deinit(gpios);

	printf("[ardrone_control] ending now...\n");
	fflush(stdout);
	return OK;
}

