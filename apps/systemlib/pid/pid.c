/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Laurens Mackay <mackayl@student.ethz.ch>
 *           @author Tobias Naegeli <naegelit@student.ethz.ch>
 *           @author Martin Rutschmann <rutmarti@student.ethz.ch>
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
 * @file pid.c
 * Implementation of generic PID control interface
 */

#include "pid.h"
#include <math.h>

__EXPORT void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax,
	      uint8_t mode)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	pid->mode = mode;
	pid->count = 0;
	pid->saturated = 0;
	pid->last_output = 0;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;
}
__EXPORT int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax)
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

	if (isfinite(intmax)) {
		pid->intmax = intmax;
	}  else {
		ret = 1;
	}

	// pid->limit = limit;
	return ret;
}

//void pid_set(PID_t *pid, float sp)
//{
//	pid->sp = sp;
//	pid->error_previous = 0;
//	pid->integral = 0;
//}

/**
 *
 * @param pid
 * @param val
 * @param dt
 * @return
 */
__EXPORT float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	/*  error = setpoint - actual_position
	 integral = integral + (error*dt)
	 derivative = (error - previous_error)/dt
	 output = (Kp*error) + (Ki*integral) + (Kd*derivative)
	 previous_error = error
	 wait(dt)
	 goto start
	 */

	if (!isfinite(sp) || !isfinite(val) || !isfinite(val_dot) || !isfinite(dt))
	{
		return pid->last_output;
	}

	float i, d;
	pid->sp = sp;
	float error = pid->sp - val;

	if (pid->saturated && (pid->integral * error > 0)) {
		//Output is saturated and the integral would get bigger (positive or negative)
		i = pid->integral;

		//Reset saturation. If we are still saturated this will be set again at output limit check.
		pid->saturated = 0;

	} else {
		i = pid->integral + (error * dt);
	}

	// Anti-Windup. Needed if we don't use the saturation above.
	if (pid->intmax != 0.0f) {
		if (i > pid->intmax) {
			pid->integral = pid->intmax;

		} else if (i < -pid->intmax) {

			pid->integral = -pid->intmax;

		} else {
			pid->integral = i;
		}
	}

	if (pid->mode == PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / dt;

	} else if (pid->mode == PID_MODE_DERIVATIV_SET) {
		d = -val_dot;

	} else {
		d = 0.0f;
	}

	if (pid->kd == 0.0f) {
		d = 0.0f;
	}

	if (pid->ki == 0.0f) {
		i = 0;
	}

	float p;

	if (pid->kp == 0.0f) {
		p = 0.0f;
	} else {
		p = error;
	}

	if (isfinite(error)) {
		pid->error_previous = error;
	}

	float output = (error * pid->kp) + (i * pid->ki) + (d * pid->kd);

	if (isfinite(output)) {
		pid->last_output = output;
	}

	if (!isfinite(pid->integral)) {
		pid->integral = 0;
	}

	return pid->last_output;
}
