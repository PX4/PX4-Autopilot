/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file output_limit.h
 *
 * Library for output limiting (PWM for example)
 *
 * @author Julian Oes <julian@px4.io>
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

__BEGIN_DECLS

/*
 * time for the ESCs to initialize
 * (this is not actually needed if the signal is sent right after boot)
 */
#define INIT_TIME_US 50000
/*
 * time to slowly ramp up the ESCs
 */
#define RAMP_TIME_US 500000

enum output_limit_state {
	OUTPUT_LIMIT_STATE_OFF = 0,
	OUTPUT_LIMIT_STATE_INIT,
	OUTPUT_LIMIT_STATE_RAMP,
	OUTPUT_LIMIT_STATE_ON
};

typedef struct {
	enum output_limit_state state;
	uint64_t time_armed;
	bool ramp_up; ///< if true, motors will ramp up from disarmed to min_output after arming
} output_limit_t;

__EXPORT void output_limit_init(output_limit_t *limit);

__EXPORT void output_limit_calc(const bool armed, const bool pre_armed, const unsigned num_channels,
				const uint16_t reverse_mask, const uint16_t *disarmed_output,
				const uint16_t *min_output, const uint16_t *max_output,
				const float *output, uint16_t *effective_output, output_limit_t *limit);

__END_DECLS
