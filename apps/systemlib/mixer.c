/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mixer.c
 *
 * Generic control value mixing library.
 *
 * This library implements a generic mixer function that can be used
 * by any driver or subsytem that wants to combine several control signals
 * into a single output.
 *
 * See mixer.h for more details.
 */

#include "mixer.h"

static int
scale_check(struct MixScaler *scale)
{
 	if (scale->offset > 1.0f)
 		return -1;
 	if (scale->offset > 1.0f)
 		return -1;
 	if (scale->lower_limit > scale->upper_limit)
 		return -1;
 	if (scale->lower_limit < -1.0f)
 		return -1;
 	if (scale->upper_limit > 1.0f)
 		return -1;
 	return 0;
}

int
mixer_check(struct MixMixer *mixer, unsigned control_count)
{
	if (mixer->control_count < 1)
		return -1;
	if (mixer->control_count > control_count)
		return -1;
	if (!scale_check(&mixer->output_scaler))
		return -1;

	for (unsigned i = 0; i < mixer->control_count; i++) {
		if (mixer->control_scaler[i].control >= control_count)
			return -1;
		if (!scale_check(&mixer->control_scaler[i]))
			return -1;
	}
	return 0;
}

static float
scale(struct MixScaler *scaler, float input)
{
	float output;

	if (input < 0.0f) {
		output = (input * scaler->negative_scale) + scaler->offset;
	} else {
		output = (input * scaler->positive_scale) + scaler->offset;
	}
	if (output > scaler->upper_limit) {
		output = scaler->upper_limit;
	} else if (output < scaler->lower_limit) {
		output = scaler->lower_limit;
	}

	return output;
}

float
mixer_mix(struct MixMixer *mixer, float *controls)
{
	struct MixScaler *scaler;
	float sum = 0.0f;

	for (unsigned i = 0; i < mixer->control_count; i++) {
		scaler = &mixer->control_scaler[i];
		sum += scale(scaler, controls[scaler->control]);
	}

	return scale(&mixer->output_scaler, sum);
}
