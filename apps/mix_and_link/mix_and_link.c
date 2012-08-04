/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Nils Wenzler <wenzlern@ethz.ch>
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

/*
 * @file Mixing / linking of RC channels
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdint.h>

#include "mix_and_link.h"

/*
 * Library function that mixes the abstract functions like ROLL, PITCH, THROTTLE
 * to one or several actuators.
 * @param *mixers is a pointer to the configuration struct, declared in mix_and_link.h
 * @param nr_mixers is a integer that denotes the number of mixers that should be processed
 * @param nr_actuators is a integer that denotes the max number of actuators that get linked to one source
 * @param *data is a pointer to the mixer data struct, declared in mix_and_link.h
 */
void mix_and_link(const mixer_conf_t *mixers, uint8_t nr_mixers, uint8_t nr_actuators, mixer_data_t *data)
{

	/* Reset the Output Array */
	uint8_t i, j;

	for (i = 0; i < nr_actuators; i++) {
		data->output[i] = 0;
	}

	/* Loop throug the given mixers */
	for (i = 0; i < nr_mixers; i++) {

		/* The actual mixing */
		for (j = 0; j < mixers[i].nr_actuators; j++) {
			data->output[mixers[i].dest[j]] += (int16_t)(data->input[mixers[i].source]
							   * mixers[i].dual_rate[j]);

			/* Saturate to +-10000 */
			if (data->output[mixers[i].dest[j]] > 10000)
				data->output[mixers[i].dest[j]] = 10000;
			else if (data->output[mixers[i].dest[j]] < -10000)
				data->output[mixers[i].dest[j]] = -10000;

		}
	}
}


