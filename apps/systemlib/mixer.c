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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "mixer.h"

static int
scale_check(struct scaler_s *scale)
{
	if (scale->offset > 1.1f)
		return 1;

	if (scale->offset < -1.1f)
		return 2;

	if (scale->lower_limit > scale->upper_limit)
		return 3;

	if (scale->lower_limit < -1.1f)
		return 4;

	if (scale->upper_limit > 1.1f)
		return 5;

	return 0;
}

int
mixer_check(struct mixer_s *mixer, unsigned group_count, unsigned control_count)
{
	int ret;

	/* sanity that presumes that a mixer includes a control no more than once */
	if (mixer->control_count > (group_count * control_count))
		return -2;

	/* validate the output scaler */
	ret = scale_check(&mixer->output_scaler);

	if (ret != 0)
		return ret;

	/* validate input scalers */
	for (unsigned i = 0; i < mixer->control_count; i++) {

		/* range-check input controls */
		if (mixer->control_scaler[i].control_group >= group_count)
			return -3;

		if (mixer->control_scaler[i].control_index >= control_count)
			return -3;

		/* validate the scaler */
		ret = scale_check(&mixer->control_scaler[i]);

		if (ret != 0)
			return (10 * i + ret);
	}

	return 0;
}

void
mixer_requires(struct mixer_s *mixer, uint32_t *groups)
{
	for (unsigned i = 0; i < mixer->control_count; i++)
		*groups |= 1 << mixer->control_scaler[i].control_group;
}

/**
 * Apply a scaler to a value.
 *
 * @param scaler		The applied scaler.
 * @param input			The value to scale.
 * @output			The scaled value.
 */
static float
scale(struct scaler_s *scaler, float input)
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
mixer_mix(struct mixer_s *mixer, float **controls)
{
	float sum = 0.0f;

	for (unsigned i = 0; i < mixer->control_count; i++) {

		struct scaler_s *scaler = &mixer->control_scaler[i];
		float *cg = controls[scaler->control_group];

		sum += scale(scaler, cg[scaler->control_index]);
	}

	return scale(&mixer->output_scaler, sum);
}

/**
 * Effectively fdgets()
 */
static int
mixer_getline(int fd, char *line, unsigned maxlen)
{
	int	ret;
	char	c;

	while (--maxlen) {
		ret = read(fd, &c, 1);

		if (ret <= 0)
			return ret;

		if (c == '\r')
			continue;

		if (c == '\n') {
			*line = '\0';
			return 1;
		}

		*line++ = c;
	}

	/* line too long */
	puts("line too long");
	return -1;
}

static int
mixer_load_scaler(const char *buf, struct scaler_s *scaler)
{
	unsigned u[2];
	int s[5];

	if (sscanf(buf, "S: %u %u %d %d %d %d %d",
		   &u[0], &u[1], &s[0], &s[1], &s[2], &s[3], &s[4]) != 7)
		return -1;

	scaler->control_group	= u[0];
	scaler->control_index	= u[1];
	scaler->negative_scale	= s[0] / 10000.0f;
	scaler->positive_scale	= s[1] / 10000.0f;
	scaler->offset		= s[2] / 10000.0f;
	scaler->lower_limit	= s[3] / 10000.0f;
	scaler->upper_limit	= s[4] / 10000.0f;

	return 0;
}

int
mixer_load(int fd, struct mixer_s **mp)
{
	int		ret, result = -1;
	struct mixer_s *mixer = NULL;
	char		buf[100];
	unsigned	scalers;

	ret = mixer_getline(fd, buf, sizeof(buf));

	/* end of file? */
	if (ret == 0)
		result = 0;

	/* can't proceed */
	if (ret < 1)
		goto out;

	/* get header */
	if (sscanf(buf, "M: %u", &scalers) != 1)
		goto out;

	/* if there are scalers, load them */
	if (scalers > 0) {

		/* allocate mixer */
		scalers--;
		mixer = (struct mixer_s *)malloc(MIXER_SIZE(scalers));

		if (mixer == NULL)
			goto out;

		mixer->control_count = scalers;

		ret = mixer_getline(fd, buf, sizeof(buf));

		if (ret < 1)
			goto out;

		if (mixer_load_scaler(buf, &mixer->output_scaler))
			goto out;

		for (unsigned i = 0; i < scalers; i++) {
			ret = mixer_getline(fd, buf, sizeof(buf));

			if (ret < 1)
				goto out;

			if (mixer_load_scaler(buf, &mixer->control_scaler[i]))
				goto out;
		}

	} else {
		/* we return NULL for the mixer, which is interpreted elsewhere as "no mixer" */
	}

	result = 1;

out:

	/* on error, discard allocated mixer */
	if ((result <= 0) && (mixer != NULL))
		free(mixer);

	*mp = mixer;
	return result;
}

static int
mixer_save_scaler(char *buf, struct scaler_s *scaler)
{
	int	s[5];

	s[0] = 10000.0f * scaler->negative_scale;
	s[1] = 10000.0f * scaler->positive_scale;
	s[2] = 10000.0f * scaler->offset;
	s[3] = 10000.0f * scaler->lower_limit;
	s[4] = 10000.0f * scaler->upper_limit;

	return sprintf(buf, "S: %u %u %d %d %d %d %d\n",
		       scaler->control_group, scaler->control_index,
		       s[0], s[1], s[2], s[3], s[4]);
}

int
mixer_save(int fd, struct mixer_s *mixer)
{
	char		buf[100];
	int		len, ret;

	/* write the mixer header */
	len = sprintf(buf, "M: %u\n", (mixer != NULL) ? mixer->control_count : 0);
	ret = write(fd, buf, len);

	if (ret != len)
		return -1;

	if (mixer != NULL) {
		/* write the output scaler */
		len = mixer_save_scaler(buf, &mixer->output_scaler);
		write(fd, buf, len);

		if (ret != len)
			return -1;

		/* write the control scalers */
		for (unsigned j = 0; j < mixer->control_count; j++) {
			len = mixer_save_scaler(buf, &mixer->control_scaler[j]);
			write(fd, buf, len);

			if (ret != len)
				return -1;
		}
	}

	return 0;
}