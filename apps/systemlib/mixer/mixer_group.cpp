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
 * @file mixer_group.cpp
 *
 * Mixer collection.
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

namespace
{

/**
 * Effectively fdgets() with some extra smarts.
 */
static int
mixer_getline(int fd, char *line, unsigned maxlen)
{
	/* reduce line budget by 1 to account for terminal NUL */
	maxlen--;

	/* loop looking for a non-comment line */
	for (;;) {
		int	ret;
		char	c;
		char	*p = line;

		/* loop reading characters for this line */
		for (;;) {
			ret = read(fd, &c, 1);

			/* on error or EOF, return same */
			if (ret <= 0)
				return ret;

			/* ignore carriage returns */
			if (c == '\r')
				continue;

			/* line termination */
			if (c == '\n') {
				/* ignore malformed lines */
				if ((p - line) < 4)
					break;

				if (line[1] != ':')
					break;

				/* terminate line as string and return */
				*p = '\0';
				return 1;
			}

			/* if we have space, accumulate the byte and go on */
			if ((p - line) < maxlen)
				*p++ = c;
		}
	}
}

/**
 * Parse a scaler from the buffer.
 */
static int
mixer_parse_scaler(const char *buf, mixer_scaler_s &scaler, uint8_t &control_group, uint8_t &control_index)
{
	unsigned u[2];
	int s[5];

	if (sscanf(buf, "S: %u %u %d %d %d %d %d",
		   &u[0], &u[1], &s[0], &s[1], &s[2], &s[3], &s[4]) != 7)
		return -1;

	control_group	= u[0];
	control_index	= u[1];
	scaler.negative_scale	= s[0] / 10000.0f;
	scaler.positive_scale	= s[1] / 10000.0f;
	scaler.offset		= s[2] / 10000.0f;
	scaler.min_output	= s[3] / 10000.0f;
	scaler.max_output	= s[4] / 10000.0f;

	return 0;
}

SimpleMixer *
mixer_load_simple(Mixer::ControlCallback control_cb, uintptr_t cb_handle, int fd, unsigned inputs)
{
	mixer_simple_s	*mixinfo = nullptr;
	char		buf[60];
	uint8_t		control_group, control_index;
	int		ret;

	/* let's assume we're going to read a simple mixer */
	mixinfo = (mixer_simple_s *)malloc(MIXER_SIMPLE_SIZE(inputs));

	/* first, get the output scaler */
	ret = mixer_getline(fd, buf, sizeof(buf));
	if (ret < 1)
		goto fail;
	if (mixer_parse_scaler(buf, mixinfo->output_scaler, control_group, control_index))
		goto fail;

	/* now get any inputs */
	for (unsigned i = 0; i < inputs; i++) {
		ret = mixer_getline(fd, buf, sizeof(buf));
		if (ret < 1)
			goto fail;
		if (mixer_parse_scaler(buf, 
				       mixinfo->inputs[i].scaler,
				       mixinfo->inputs[i].control_group,
				       mixinfo->inputs[i].control_index)) {
			goto fail;
		}
	}

	/* XXX should be a factory that validates the mixinfo ... */
	return new SimpleMixer(control_cb, cb_handle, mixinfo);

fail:
	free(mixinfo);
	return nullptr;
}

int
mixer_load(Mixer::ControlCallback control_cb, uintptr_t cb_handle, int fd, Mixer *&mixer)
{
	int		ret;
	char		buf[60];
	unsigned	scalers;

	ret = mixer_getline(fd, buf, sizeof(buf));

	/* end of file or error ?*/
	if (ret < 1)
		return ret;

	/* slot is empty - allocate a null mixer */
	if (buf[0] == 'Z') {
		mixer = new NullMixer();
		return 0;
	}

	/* is it a simple mixer? */
	if (sscanf(buf, "M: %u", &scalers) == 1) {
		mixer = mixer_load_simple(control_cb, cb_handle, fd, scalers);
		return (mixer == nullptr) ? -1 : 0;
	}

	/* we don't recognise the mixer type */
	return -1;
}


} // namespace

MixerGroup::MixerGroup(ControlCallback control_cb, uintptr_t cb_handle) :
	Mixer(control_cb, cb_handle),
	_first(nullptr)
{
}

MixerGroup::~MixerGroup()
{
	Mixer *mixer;

	/* discard sub-mixers */
	while (_first != nullptr) {
		mixer = _first;
		_first = mixer->_next;
		delete mixer;
	}
}

void
MixerGroup::add_mixer(Mixer *mixer)
{
	Mixer **mpp;

	mpp = &_first;
	while (*mpp != nullptr)
		mpp = &((*mpp)->_next);
	*mpp = mixer;
	mixer->_next = nullptr;
}

unsigned
MixerGroup::mix(float *outputs, unsigned space)
{
	Mixer	*mixer = _first;
	unsigned index = 0;

	while ((mixer != nullptr) && (index < space)) {
		index += mixer->mix(outputs + index, space - index);
		mixer = mixer->_next;
	}
	return index;
}

void
MixerGroup::groups_required(uint32_t &groups)
{
	Mixer	*mixer = _first;

	while (mixer != nullptr) {
		mixer->groups_required(groups);
		mixer = mixer->_next;
	}
}

int
MixerGroup::load_from_file(const char *path)
{
	if (_first != nullptr)
		return -1;

	int fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;

	for (;;) {
		int	result;
		Mixer	*mixer;

		result = mixer_load(_control_cb,
				    _cb_handle,
				    fd,
				    mixer);

		/* error? */
		if (result < 0)
			return -1;

		/* EOF or error */
		if (result < 1)
			break;

		add_mixer(mixer);
	}

	close(fd);
	return 0;
}
