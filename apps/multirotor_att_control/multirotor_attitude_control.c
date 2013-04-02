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
 * @file multirotor_attitude_control.c
 * Implementation of attitude controller
 */

#include "multirotor_attitude_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include "multirotor_att_control_params.h"


void multirotor_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
		const struct vehicle_attitude_s *att, struct vehicle_rates_setpoint_s *rates_sp,
		bool control_yaw_position, struct multirotor_att_control_params *params, bool params_updated)
{
	static uint64_t last_run = 0;
	static uint64_t last_input = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	float dT_input = (hrt_absolute_time() - last_input) / 1000000.0f;
	last_run = hrt_absolute_time();

	if (last_input != att_sp->timestamp) {
		last_input = att_sp->timestamp;
	}

	static int sensor_delay;
	sensor_delay = hrt_absolute_time() - att->timestamp;

	static int motor_skip_counter = 0;

	static PID_t pitch_controller;
	static PID_t roll_controller;
	static PID_t yaw_controller;

	static bool initialized = false;

	static float yaw_error;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false) {

		pid_init(&pitch_controller, params->att_p, params->att_i, params->att_d, 1000.0f,
				1000.0f, PID_MODE_DERIVATIV_SET);
		pid_init(&roll_controller, params->att_p, params->att_i, params->att_d, 1000.0f,
				1000.0f, PID_MODE_DERIVATIV_SET);
		pid_init(&yaw_controller, params->yaw_p, params->yaw_i, params->yaw_d, params->yaw_intmax,
				1000.0f, PID_MODE_DERIVATIV_SET);

		initialized = true;
	}

	/* load new parameters with lower rate */
	if (params_updated) {
		/* apply parameters */
		pid_set_parameters(&pitch_controller, params->att_p, params->att_i, params->att_d, 1000.0f, 1000.0f);
		pid_set_parameters(&roll_controller, params->att_p, params->att_i, params->att_d, 1000.0f, 1000.0f);
		pid_set_parameters(&yaw_controller, params->yaw_p, params->yaw_i, params->yaw_d, params->yaw_intmax, 1000.0f);
	}

	/* reset integral if on ground */
	if (att_sp->thrust < 0.1f) {
		pid_reset_integral(&pitch_controller);
		pid_reset_integral(&roll_controller);
		pid_reset_integral(&yaw_controller);
	}

	/* calculate current control outputs */

	/* control pitch (forward) output */
	rates_sp->pitch = pid_calculate(&pitch_controller, att_sp->pitch_body ,
					att->pitch, att->pitchspeed, deltaT);

	/* control roll (left/right) output */
	rates_sp->roll = pid_calculate(&roll_controller, att_sp->roll_body ,
				       att->roll, att->rollspeed, deltaT);

	if (control_yaw_position) {
		/* control yaw rate */

		yaw_error = att->yaw - att_sp->yaw_body;

		if (yaw_error > M_PI_F) {
			yaw_error -= M_TWOPI_F;

		} else if (yaw_error < -M_PI_F) {
			yaw_error += M_TWOPI_F;
		}

		rates_sp->yaw = pid_calculate(&yaw_controller, 0.0f, yaw_error, att->yawspeed, deltaT);

	}

	rates_sp->thrust = att_sp->thrust;

	motor_skip_counter++;
}
