/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Julian Oes <joes@student.ethz.ch>
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
 * @file pwm_limit.c
 *
 * Lib to limit PWM output
 *
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include "pwm_limit.h"
#include <math.h>
#include <stdbool.h>
#include <drivers/drv_hrt.h>

__EXPORT void pwm_limit_init(pwm_limit_t *limit)
{
	limit->nchannels = 0;
	limit->state = LIMIT_STATE_OFF;
	limit->time_armed = 0;
	return;
}

__EXPORT void pwm_limit_calc(const bool armed, const uint16_t *disarmed_pwm, const uint16_t *min_pwm, const uint16_t *max_pwm, const float *output_requested, uint16_t *effective_pwm, pwm_limit_t *limit)
{
	/* first evaluate state changes */
	switch (limit->state) {
		case LIMIT_STATE_OFF:
			if (armed)
				limit->state = LIMIT_STATE_RAMP;
			limit->time_armed = hrt_absolute_time();
			break;
		case LIMIT_STATE_INIT:
			if (!armed)
				limit->state = LIMIT_STATE_OFF;
			else if (hrt_absolute_time() - limit->time_armed >= INIT_TIME_US)
				limit->state = LIMIT_STATE_RAMP;
			break;
		case LIMIT_STATE_RAMP:
			if (!armed)
				limit->state = LIMIT_STATE_OFF;
			else if (hrt_absolute_time() - limit->time_armed >= INIT_TIME_US + RAMP_TIME_US)
				limit->state = LIMIT_STATE_ON;
			break;
		case LIMIT_STATE_ON:
			if (!armed)
				limit->state = LIMIT_STATE_OFF;
			break;
		default:
			break;
	}

	unsigned progress;
	uint16_t temp_pwm;

	/* then set effective_pwm based on state */
	switch (limit->state) {
		case LIMIT_STATE_OFF:
		case LIMIT_STATE_INIT:
			for (unsigned i=0; i<limit->nchannels; i++) {
				effective_pwm[i] = disarmed_pwm[i];
			}
			break;
		case LIMIT_STATE_RAMP:

			progress = (hrt_absolute_time() - INIT_TIME_US - limit->time_armed)*10000 / RAMP_TIME_US;
			for (unsigned i=0; i<limit->nchannels; i++) {

				temp_pwm = output_requested[i] * (max_pwm[i] - min_pwm[i])/2 + (max_pwm[i] + min_pwm[i])/2;
				/* already follow user/controller input if higher than min_pwm */
				effective_pwm[i] = (disarmed_pwm[i]*(10000-progress) + (temp_pwm > min_pwm[i] ? temp_pwm : min_pwm[i])*progress)/10000;

			}
			break;
		case LIMIT_STATE_ON:
			for (unsigned i=0; i<limit->nchannels; i++) {
				effective_pwm[i] = output_requested[i] * (max_pwm[i] - min_pwm[i])/2 + (max_pwm[i] + min_pwm[i])/2;
			}
			break;
		default:
			break;
	}
	return;
}
