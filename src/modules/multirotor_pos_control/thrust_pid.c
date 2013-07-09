/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *           Tobias Naegeli <naegelit@student.ethz.ch>
 *           Martin Rutschmann <rutmarti@student.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
 *           Julian Oes <joes@student.ethz.ch>
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
 * @file thrust_pid.c
 *
 * Implementation of generic PID control interface.
 *
 * @author Laurens Mackay <mackayl@student.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Martin Rutschmann <rutmarti@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include "thrust_pid.h"
#include <math.h>

__EXPORT void thrust_pid_init(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max, uint8_t mode, float dt_min)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->limit_min = limit_min;
	pid->limit_max = limit_max;
	pid->mode = mode;
	pid->dt_min = dt_min;
	pid->last_output = 0.0f;
	pid->sp = 0.0f;
	pid->error_previous = 0.0f;
	pid->integral = 0.0f;
}

__EXPORT int thrust_pid_set_parameters(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max)
{
	int ret = 0;

	if (isfinite(kp)) {
		pid->kp = kp;

	} else {
		ret = 1;
	}

	if (isfinite(ki)) {
		pid->ki = ki;

	} else {
		ret = 1;
	}

	if (isfinite(kd)) {
		pid->kd = kd;

	} else {
		ret = 1;
	}

	if (isfinite(limit_min)) {
		pid->limit_min = limit_min;

	}  else {
		ret = 1;
	}

	if (isfinite(limit_max)) {
		pid->limit_max = limit_max;

	}  else {
		ret = 1;
	}

	return ret;
}

__EXPORT float thrust_pid_calculate(thrust_pid_t *pid, float sp, float val, float dt)
{
	/* Alternative integral component calculation
	 error = setpoint - actual_position
	 integral = integral + (Ki*error*dt)
	 derivative = (error - previous_error)/dt
	 output = (Kp*error) + integral + (Kd*derivative)
	 previous_error = error
	 wait(dt)
	 goto start
	 */

	if (!isfinite(sp) || !isfinite(val) || !isfinite(dt)) {
		return pid->last_output;
	}

	float i, d;
	pid->sp = sp;

	// Calculated current error value
	float error = pid->sp - val;

	// Calculate or measured current error derivative
	if (pid->mode == THRUST_PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;

	} else if (pid->mode == THRUST_PID_MODE_DERIVATIV_CALC_NO_SP) {
		d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;

	} else {
		d = 0.0f;
	}

	if (!isfinite(d)) {
		d = 0.0f;
	}

	// Calculate the error integral and check for saturation
	i = pid->integral + (pid->ki * error * dt);

	float output = (error * pid->kp) + i + (d * pid->kd);
	if (output < pid->limit_min || output > pid->limit_max) {
		i = pid->integral;		// If saturated then do not update integral value
		// recalculate output with old integral
		output = (error * pid->kp) + i + (d * pid->kd);

	} else {
		if (!isfinite(i)) {
			i = 0.0f;
		}

		pid->integral = i;
	}

	if (isfinite(output)) {
		if (output > pid->limit_max) {
			output = pid->limit_max;

		} else if (output < pid->limit_min) {
			output = pid->limit_min;
		}

		pid->last_output = output;
	}

	return pid->last_output;
}
