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

void pwm_limit_init(pwm_limit_t *limit)
{
	limit->state = LIMIT_STATE_OFF;
	limit->time_armed = 0;
	return;
}

void pwm_limit_calc(const bool armed, const unsigned num_channels, const uint16_t *disarmed_pwm, const uint16_t *min_pwm, const uint16_t *max_pwm, float *output, uint16_t *effective_pwm, pwm_limit_t *limit)
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
			for (unsigned i=0; i<num_channels; i++) {
				effective_pwm[i] = disarmed_pwm[i];
				output[i] = 0.0f;
			}
			break;
		case LIMIT_STATE_RAMP:

			progress = (hrt_absolute_time() - INIT_TIME_US - limit->time_armed)*10000 / RAMP_TIME_US;
			for (unsigned i=0; i<num_channels; i++) {
                
				uint16_t ramp_min_pwm;
                
				/* if a disarmed pwm value was set, blend between disarmed and min */
				if (disarmed_pwm[i] > 0) {

					/* safeguard against overflows */
					uint16_t disarmed = disarmed_pwm[i];
					if (disarmed > min_pwm[i])
						disarmed = min_pwm[i];

					uint16_t disarmed_min_diff = min_pwm[i] - disarmed;
					ramp_min_pwm = disarmed + (disarmed_min_diff * progress) / 10000;
				} else {
                    
					/* no disarmed pwm value set, choose min pwm */
					ramp_min_pwm = min_pwm[i];
				}

				effective_pwm[i] = output[i] * (max_pwm[i] - ramp_min_pwm)/2 + (max_pwm[i] + ramp_min_pwm)/2;
				output[i] = (float)progress/10000.0f * output[i];
			}
			break;
		case LIMIT_STATE_ON:
			for (unsigned i=0; i<num_channels; i++) {
				effective_pwm[i] = output[i] * (max_pwm[i] - min_pwm[i])/2 + (max_pwm[i] + min_pwm[i])/2;
				/* effective_output stays the same */
			}
			break;
		default:
			break;
	}
	return;
}
