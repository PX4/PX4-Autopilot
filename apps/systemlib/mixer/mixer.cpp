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
 * @file mixer.cpp
 *
 * Programmable multi-channel mixer library.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "mixer.h"

Mixer::Mixer(ControlCallback control_cb, uintptr_t cb_handle) :
	_control_cb(control_cb),
	_cb_handle(cb_handle)
{
}

float
Mixer::get_control(uint8_t group, uint8_t index)
{
	float	value;

	_control_cb(_cb_handle, group, index, value);

	return value;
}


float
Mixer::scale(const mixer_scaler_s &scaler, float input)
{
	float output;

	if (input < 0.0f) {
		output = (input * scaler.negative_scale) + scaler.offset;

	} else {
		output = (input * scaler.positive_scale) + scaler.offset;
	}

	if (output > scaler.max_output) {
		output = scaler.max_output;

	} else if (output < scaler.min_output) {
		output = scaler.min_output;
	}

	return output;
}

int
Mixer::scale_check(struct mixer_scaler_s &scaler)
{
	if (scaler.offset > 1.001f)
		return 1;

	if (scaler.offset < -1.001f)
		return 2;

	if (scaler.min_output > scaler.max_output)
		return 3;

	if (scaler.min_output < -1.001f)
		return 4;

	if (scaler.max_output > 1.001f)
		return 5;

	return 0;
}

/****************************************************************************/

NullMixer::NullMixer() :
	Mixer(nullptr, 0)
{
}

unsigned
NullMixer::mix(float *outputs, unsigned space)
{
	if (space > 0) {
		*outputs = 0.0f;
		return 1;
	}

	return 0;
}

void
NullMixer::groups_required(uint32_t &groups)
{

}

/****************************************************************************/

SimpleMixer::SimpleMixer(ControlCallback control_cb,
			 uintptr_t cb_handle,
			 mixer_simple_s *mixinfo) :
	Mixer(control_cb, cb_handle),
	_info(mixinfo)
{
}

SimpleMixer::~SimpleMixer()
{
	if (_info != nullptr)
		free(_info);
}

unsigned
SimpleMixer::mix(float *outputs, unsigned space)
{
	float		sum = 0.0f;

	if (_info == nullptr)
		return 0;

	if (space < 1)
		return 0;

	for (unsigned i = 0; i < _info->control_count; i++) {
		float input;

		_control_cb(_cb_handle,
			    _info->controls[i].control_group,
			    _info->controls[i].control_index,
			    input);

		sum += scale(_info->controls[i].scaler, input);
	}

	*outputs = scale(_info->output_scaler, sum);
	return 1;
}

void
SimpleMixer::groups_required(uint32_t &groups)
{
	for (unsigned i = 0; i < _info->control_count; i++)
		groups |= 1 << _info->controls[i].control_group;
}

int
SimpleMixer::check()
{
	int ret;
	float junk;

	/* sanity that presumes that a mixer includes a control no more than once */
	/* max of 32 groups due to groups_required API */
	if (_info->control_count > 32)
		return -2;

	/* validate the output scaler */
	ret = scale_check(_info->output_scaler);

	if (ret != 0)
		return ret;

	/* validate input scalers */
	for (unsigned i = 0; i < _info->control_count; i++) {

		/* verify that we can fetch the control */
		if (_control_cb(_cb_handle,
				_info->controls[i].control_group,
				_info->controls[i].control_index,
				junk) != 0) {
			return -3;
		}

		/* validate the scaler */
		ret = scale_check(_info->controls[i].scaler);

		if (ret != 0)
			return (10 * i + ret);
	}

	return 0;
}
