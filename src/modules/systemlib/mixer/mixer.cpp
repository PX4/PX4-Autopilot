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
#include <ctype.h>
#include <systemlib/err.h>

#include "mixer.h"

Mixer::Mixer(ControlCallback control_cb, uintptr_t cb_handle) :
	_next(nullptr),
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
	if (scaler.offset > 1.001f) {
		return 1;
	}

	if (scaler.offset < -1.001f) {
		return 2;
	}

	if (scaler.min_output > scaler.max_output) {
		return 3;
	}

	if (scaler.min_output < -1.001f) {
		return 4;
	}

	if (scaler.max_output > 1.001f) {
		return 5;
	}

	return 0;
}

const char *
Mixer::findtag(const char *buf, unsigned &buflen, char tag)
{
	while (buflen >= 2) {
		if ((buf[0] == tag) && (buf[1] == ':')) {
			return buf;
		}

		buf++;
		buflen--;
	}

	return nullptr;
}

const char *
Mixer::skipline(const char *buf, unsigned &buflen)
{
	const char *p;

	/* if we can find a CR or NL in the buffer, skip up to it */
	if ((p = (const char *)memchr(buf, '\r', buflen)) || (p = (const char *)memchr(buf, '\n', buflen))) {
		/* skip up to it AND one beyond - could be on the NUL symbol now */
		buflen -= (p - buf) + 1;
		return p + 1;
	}

	return nullptr;
}

/****************************************************************************/

NullMixer::NullMixer() :
	Mixer(nullptr, 0)
{
}

unsigned
NullMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
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

NullMixer *
NullMixer::from_text(const char *buf, unsigned &buflen)
{
	NullMixer *nm = nullptr;

	/* enforce that the mixer ends with space or a new line */
	for (int i = buflen - 1; i >= 0; i--) {
		if (buf[i] == '\0') {
			continue;
		}

		/* require a space or newline at the end of the buffer, fail on printable chars */
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\r') {
			/* found a line ending or space, so no split symbols / numbers. good. */
			break;

		} else {
			return nm;
		}

	}

	if ((buflen >= 2) && (buf[0] == 'Z') && (buf[1] == ':')) {
		nm = new NullMixer;
		buflen -= 2;
	}

	return nm;
}
