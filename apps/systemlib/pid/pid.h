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
 * @file pid.h
 * Definition of generic PID control interface
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error
 * val_dot in pid_calculate() will be ignored */
#define PID_MODE_DERIVATIV_CALC	0
/* Use PID_MODE_DERIVATIV_SET if you have the derivative already (Gyros, Kalman) */
#define PID_MODE_DERIVATIV_SET	1
// Use PID_MODE_DERIVATIV_NONE for a PI controller (vs PID)
#define PID_MODE_DERIVATIV_NONE 9

typedef struct {
	float kp;
	float ki;
	float kd;
	float intmax;
	float sp;
	float integral;
	float error_previous;
	float last_output;
	float limit;
	uint8_t mode;
	uint8_t count;
	uint8_t saturated;
} PID_t;

__EXPORT void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax, float limit, uint8_t mode);
__EXPORT int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax, float limit);
//void pid_set(PID_t *pid, float sp);
__EXPORT float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);



#endif /* PID_H_ */
