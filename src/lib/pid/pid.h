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
 * @file pid.h
 *
 * Definition of generic PID controller.
 *
 * @author Laurens Mackay <mackayl@student.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Martin Rutschmann <rutmarti@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

__BEGIN_DECLS

typedef enum PID_MODE {
	/* Use PID_MODE_DERIVATIV_NONE for a PI controller (vs PID) */
	PID_MODE_DERIVATIV_NONE = 0,
	/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error,
	 * val_dot in pid_calculate() will be ignored */
	PID_MODE_DERIVATIV_CALC,
	/* PID_MODE_DERIVATIV_CALC_NO_SP calculates discrete derivative from previous value,
	 * setpoint derivative will be ignored, val_dot in pid_calculate() will be ignored */
	PID_MODE_DERIVATIV_CALC_NO_SP,
	/* Use PID_MODE_DERIVATIV_SET if you have the derivative already (Gyros, Kalman) */
	PID_MODE_DERIVATIV_SET
} pid_mode_t;

typedef struct {
	pid_mode_t mode;
	float dt_min;
	float kp;
	float ki;
	float kd;
	float integral;
	float integral_limit;
	float output_limit;
	float error_previous;
	float last_output;
} PID_t;

__EXPORT void pid_init(PID_t *pid, pid_mode_t mode, float dt_min);
__EXPORT int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
__EXPORT float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);
__EXPORT void pid_reset_integral(PID_t *pid);

__END_DECLS

#endif /* PID_H_ */
