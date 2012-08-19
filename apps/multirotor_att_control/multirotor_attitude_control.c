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
#include <arch/board/up_hrt.h>

void multirotor_control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
	const struct vehicle_attitude_s *att, const struct vehicle_status_s *status,
	struct actuator_controls_s *actuators, bool verbose)
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

	actuators->control[0] = roll_control;
	actuators->control[1] = pitch_control;
	actuators->control[2] = yaw_rate_control;
	actuators->control[3] = motor_thrust;
}
