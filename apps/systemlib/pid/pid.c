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

__EXPORT void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax,
	      uint8_t mode, uint8_t plot_i)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	pid->mode = mode;
	pid->plot_i = plot_i;
	pid->count = 0;
	pid->saturated = 0;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;
}
__EXPORT void pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	//	pid->mode = mode;

	//	pid->sp = 0;
	//	pid->error_previous = 0;
	//	pid->integral = 0;
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
	if (pid->intmax != 0.0) {
		if (i > pid->intmax) {
			pid->integral = pid->intmax;

		} else if (i < -pid->intmax) {

			pid->integral = -pid->intmax;

		} else {
			pid->integral = i;
		}

		//Send Controller integrals
		//		Disabled because of new possibilities with debug_vect.
		//		Now sent in Main Loop at 5 Hz. 26.06.2010 Laurens
		//		if (pid->plot_i && (pid->count++ % 16 == 0)&&(global_data.param[PARAM_SEND_SLOT_DEBUG_2] == 1))
		//		{
		//			mavlink_msg_debug_send(MAVLINK_COMM_1, pid->plot_i, pid->integral);
		//		}
	}

	if (pid->mode == PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / dt;

	} else if (pid->mode == PID_MODE_DERIVATIV_SET) {
		d = -val_dot;

	} else {
		d = 0;
	}

	pid->error_previous = error;

	return (error * pid->kp) + (i * pid->ki) + (d * pid->kd);
}
