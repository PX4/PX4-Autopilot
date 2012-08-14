/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Laurens Mackay <mackayl@student.ethz.ch>
 *           @author Tobias Naegeli <naegelit@student.ethz.ch>
 *           @author Martin Rutschmann <rutmarti@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file attitude_control.c
 * Implementation of attitude controller
 */

#include "attitude_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ardrone_motor_control.h"
#include <float.h>
#include <math.h>
#include "pid.h"
#include <arch/board/up_hrt.h>

#define MAX_MOTOR_COUNT 16

void multirotor_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att, const struct vehicle_status_s *status,
	bool verbose, float* roll_output, float* pitch_output, float* yaw_output, float* thrust_output)
{
	static uint64_t last_run = 0;
	const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	static int motor_skip_counter = 0;

	static PID_t yaw_pos_controller;
	static PID_t yaw_speed_controller;
	static PID_t pitch_controller;
	static PID_t roll_controller;

	static float pid_yawpos_lim;
	static float pid_yawspeed_lim;
	static float pid_att_lim;

	static bool initialized = false;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false) {

		pid_init(&yaw_pos_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_AWU],
			 PID_MODE_DERIVATIV_CALC, 154);

		pid_init(&yaw_speed_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_AWU],
			 PID_MODE_DERIVATIV_CALC, 155);

		pid_init(&pitch_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU],
			 PID_MODE_DERIVATIV_SET, 156);

		pid_init(&roll_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU],
			 PID_MODE_DERIVATIV_SET, 157);

		pid_yawpos_lim = 	global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_LIM];
		pid_yawspeed_lim =	global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_LIM];
		pid_att_lim =	global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_LIM];

		initialized = true;
	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 50 == 0) {
		pid_set_parameters(&yaw_pos_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_AWU]);

		pid_set_parameters(&yaw_speed_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_AWU]);

		pid_set_parameters(&pitch_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU]);

		pid_set_parameters(&roll_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU]);

		pid_yawpos_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_LIM];
		pid_yawspeed_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_LIM];
		pid_att_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_LIM];
	}

	/*Calculate Controllers*/
	//control Nick
	float pitch_control = pid_calculate(&pitch_controller, att_sp->pitch_body + global_data_parameter_storage->pm.param_values[PARAM_ATT_YOFFSET],
					att->pitch, att->pitchspeed, deltaT);
	//control Roll
	float roll_control = pid_calculate(&roll_controller, att_sp->roll_body + global_data_parameter_storage->pm.param_values[PARAM_ATT_XOFFSET],
					att->roll, att->rollspeed, deltaT);
	//control Yaw Speed
	float yaw_rate_control = pid_calculate(&yaw_speed_controller, att_sp->yaw_body, att->yawspeed, 0.0f, deltaT); 	//attitude_setpoint_bodyframe.z is yaw speed!

	/*
	 * compensate the vertical loss of thrust
	 * when thrust plane has an angle.
	 * start with a factor of 1.0 (no change)
	 */
	float zcompensation = 1.0f;

	if (fabsf(att->roll) > 1.0f) {
		zcompensation *= 1.85081571768f;

	} else {
		zcompensation *= 1.0f / cosf(att->roll);
	}

	if (fabsf(att->pitch) > 1.0f) {
		zcompensation *= 1.85081571768f;

	} else {
		zcompensation *= 1.0f / cosf(att->pitch);
	}

	float motor_thrust = 0.0f;

	// FLYING MODES
	motor_thrust = att_sp->thrust;

	//printf("mot0: %3.1f\n", motor_thrust);

	/* compensate thrust vector for roll / pitch contributions */
	motor_thrust *= zcompensation;

	/* limit yaw rate output */
	if (yaw_rate_control > pid_yawspeed_lim) {
		yaw_rate_control = pid_yawspeed_lim;
		yaw_speed_controller.saturated = 1;
	}

	if (yaw_rate_control < -pid_yawspeed_lim) {
		yaw_rate_control = -pid_yawspeed_lim;
		yaw_speed_controller.saturated = 1;
	}

	if (pitch_control > pid_att_lim) {
		pitch_control = pid_att_lim;
		pitch_controller.saturated = 1;
	}

	if (pitch_control < -pid_att_lim) {
		pitch_control = -pid_att_lim;
		pitch_controller.saturated = 1;
	}


	if (roll_control > pid_att_lim) {
		roll_control = pid_att_lim;
		roll_controller.saturated = 1;
	}

	if (roll_control < -pid_att_lim) {
		roll_control = -pid_att_lim;
		roll_controller.saturated = 1;
	}

	*roll_output = roll_control;
	*pitch_output = pitch_control;
	*yaw_output = yaw_rate_control;
	*thrust_output = motor_thrust;
}

void ardrone_mixing_and_output(int ardrone_write, float roll_control, float pitch_control, float yaw_control, float motor_thrust, bool verbose) {

	unsigned int motor_skip_counter = 0;

	const float min_thrust = 0.02f;			/**< 2% minimum thrust */
	const float max_thrust = 1.0f;			/**< 100% max thrust */
	const float scaling = 512.0f;			/**< 100% thrust equals a value of 512 */

	const float min_gas = min_thrust * scaling;	/**< value range sent to motors, minimum */
	const float max_gas = max_thrust * scaling;	/**< value range sent to motors, maximum */

	/* initialize all fields to zero */
	uint16_t motor_pwm[MAX_MOTOR_COUNT] = {0};
	float motor_calc[MAX_MOTOR_COUNT] = {0};

	float output_band = 0.0f;
	float band_factor = 0.75f;
	const float startpoint_full_control = 0.25f;	/**< start full control at 25% thrust */
	float yaw_factor = 1.0f;

	if (motor_thrust <= min_thrust) {
		motor_thrust = min_thrust;
		output_band = 0.0f;

	} else if (motor_thrust < startpoint_full_control && motor_thrust > min_thrust) {
		output_band = band_factor * (motor_thrust - min_thrust);

	} else if (motor_thrust >= startpoint_full_control && motor_thrust < max_thrust - band_factor * startpoint_full_control) {
		output_band = band_factor * startpoint_full_control;

	} else if (motor_thrust >= max_thrust - band_factor * startpoint_full_control) {
		output_band = band_factor * (max_thrust - motor_thrust);
	}

	if (verbose && motor_skip_counter % 100 == 0) {
		printf("1: mot1: %3.1f band: %3.1f r: %3.1f n: %3.1f y: %3.1f\n", (double)motor_thrust, (double)output_band, (double)roll_control, (double)pitch_control, (double)yaw_control);
	}

	//add the yaw, nick and roll components to the basic thrust //TODO:this should be done by the mixer

	// FRONT (MOTOR 1)
	motor_calc[0] = motor_thrust + (roll_control / 2 + pitch_control / 2 - yaw_control);

	// RIGHT (MOTOR 2)
	motor_calc[1] = motor_thrust + (-roll_control / 2 + pitch_control / 2 + yaw_control);

	// BACK (MOTOR 3)
	motor_calc[2] = motor_thrust + (-roll_control / 2 - pitch_control / 2 - yaw_control);

	// LEFT (MOTOR 4)
	motor_calc[3] = motor_thrust + (roll_control / 2 - pitch_control / 2 + yaw_control);

	// if we are not in the output band
	if (!(motor_calc[0] < motor_thrust + output_band && motor_calc[0] > motor_thrust - output_band
	      && motor_calc[1] < motor_thrust + output_band && motor_calc[1] > motor_thrust - output_band
	      && motor_calc[2] < motor_thrust + output_band && motor_calc[2] > motor_thrust - output_band
	      && motor_calc[3] < motor_thrust + output_band && motor_calc[3] > motor_thrust - output_band)) {

		yaw_factor = 0.5f;
		// FRONT (MOTOR 1)
		motor_calc[0] = motor_thrust + (roll_control / 2 + pitch_control / 2 - yaw_control * yaw_factor);

		// RIGHT (MOTOR 2)
		motor_calc[1] = motor_thrust + (-roll_control / 2 + pitch_control / 2 + yaw_control * yaw_factor);

		// BACK (MOTOR 3)
		motor_calc[2] = motor_thrust + (-roll_control / 2 - pitch_control / 2 - yaw_control * yaw_factor);

		// LEFT (MOTOR 4)
		motor_calc[3] = motor_thrust + (roll_control / 2 - pitch_control / 2 + yaw_control * yaw_factor);
	}

	if (verbose && motor_skip_counter % 100 == 0) {
		printf("2: m1: %3.1f m2: %3.1f m3: %3.1f m4: %3.1f\n", (double)motor_calc[0], (double)motor_calc[1], (double)motor_calc[2], (double)motor_calc[3]);
	}

	for (int i = 0; i < 4; i++) {
		//check for limits
		if (motor_calc[i] < motor_thrust - output_band) {
			motor_calc[i] = motor_thrust - output_band;
		}

		if (motor_calc[i] > motor_thrust + output_band) {
			motor_calc[i] = motor_thrust + output_band;
		}
	}

	if (verbose && motor_skip_counter % 100 == 0) {
		printf("3: band lim: m1: %3.1f m2: %3.1f m3: %3.1f m4: %3.1f\n", (double)motor_calc[0], (double)motor_calc[1], (double)motor_calc[2], (double)motor_calc[3]);
	}

	/* set the motor values */

	/* scale up from 0..1 to 10..512) */
	motor_pwm[0] = (uint16_t) (motor_calc[0] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[1] = (uint16_t) (motor_calc[1] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[2] = (uint16_t) (motor_calc[2] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[3] = (uint16_t) (motor_calc[3] * ((float)max_gas - min_gas) + min_gas);

	if (verbose && motor_skip_counter % 100 == 0) {
		printf("4: scaled: m1: %d m2: %d m3: %d m4: %d\n", motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
	}

	/* Keep motors spinning while armed and prevent overflows */

	/* Failsafe logic - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] > 0) ? motor_pwm[0] : 10;
	motor_pwm[1] = (motor_pwm[1] > 0) ? motor_pwm[1] : 10;
	motor_pwm[2] = (motor_pwm[2] > 0) ? motor_pwm[2] : 10;
	motor_pwm[3] = (motor_pwm[3] > 0) ? motor_pwm[3] : 10;

	/* Failsafe logic - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] <= 512) ? motor_pwm[0] : 512;
	motor_pwm[1] = (motor_pwm[1] <= 512) ? motor_pwm[1] : 512;
	motor_pwm[2] = (motor_pwm[2] <= 512) ? motor_pwm[2] : 512;
	motor_pwm[3] = (motor_pwm[3] <= 512) ? motor_pwm[3] : 512;

	/* send motors via UART */
	if (verbose && motor_skip_counter % 100 == 0) printf("5: mot: %3.1f-%i-%i-%i-%i\n\n", (double)motor_thrust, motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
	ardrone_write_motor_commands(ardrone_write, motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);

	motor_skip_counter++;
}
