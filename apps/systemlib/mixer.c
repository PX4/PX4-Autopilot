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
scale_check(struct MixScaler *scale)
{
	if (scale->offset > 1.0f)
		return 1;

	if (scale->offset < -1.0f)
		return 2;

	if (scale->lower_limit > scale->upper_limit)
		return 3;

	if (scale->lower_limit < -1.0f)
		return 4;

	if (scale->upper_limit > 1.0f)
		return 5;

	return 0;
}

int
mixer_check(struct MixMixer *mixer, unsigned control_count)
{
	int ret;

	if (mixer->control_count < 1)
		return -1;

	if (mixer->control_count > control_count)
		return -2;

	ret = scale_check(&mixer->output_scaler);
	if (ret != 0)
		return ret;

	for (unsigned i = 0; i < mixer->control_count; i++) {
		if (mixer->control_scaler[i].control >= control_count)
			return -3;

		ret = scale_check(&mixer->control_scaler[i]);
		if (ret != 0)
			return (10 * i + ret);
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
mixer_load_scaler(const char *buf, struct MixScaler *scaler)
{
	if (sscanf(buf, "S: %u %f %f %f %f %f",
		   &scaler->control, &scaler->negative_scale, &scaler->positive_scale,
		   &scaler->offset, &scaler->lower_limit, &scaler->upper_limit) != 6)
		return -1;

	return 0;
}

int
mixer_load(int fd, struct MixMixer **mp)
{
	int		ret, result = -1;
	struct MixMixer *mixer = NULL;
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
		mixer = (struct MixMixer *)malloc(MIXER_SIZE(scalers));

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
mixer_save_scaler(char *buf, struct MixScaler *scaler)
{
	return sprintf(buf, "S: %u %f %f %f %f %f\n",
		       scaler->control, scaler->negative_scale, scaler->positive_scale,
		       scaler->offset, scaler->lower_limit, scaler->upper_limit);
}

int
mixer_save(int fd, struct MixMixer *mixer)
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