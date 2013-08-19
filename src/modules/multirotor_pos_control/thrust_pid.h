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
 * @file thrust_pid.h
 *
 * Definition of thrust control PID interface.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef THRUST_PID_H_
#define THRUST_PID_H_

#include <stdint.h>

__BEGIN_DECLS

/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error */
#define THRUST_PID_MODE_DERIVATIV_CALC	0
/* PID_MODE_DERIVATIV_CALC_NO_SP calculates discrete derivative from previous value, setpoint derivative is ignored */
#define THRUST_PID_MODE_DERIVATIV_CALC_NO_SP	1

typedef struct {
	float kp;
	float ki;
	float kd;
	float sp;
	float integral;
	float error_previous;
	float last_output;
	float limit_min;
	float limit_max;
	float dt_min;
	uint8_t mode;
} thrust_pid_t;

__EXPORT void thrust_pid_init(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max, uint8_t mode, float dt_min);
__EXPORT int thrust_pid_set_parameters(thrust_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max);
__EXPORT float thrust_pid_calculate(thrust_pid_t *pid, float sp, float val, float dt, float r22);
__EXPORT void thrust_pid_set_integral(thrust_pid_t *pid, float i);

__END_DECLS

#endif /* THRUST_PID_H_ */
