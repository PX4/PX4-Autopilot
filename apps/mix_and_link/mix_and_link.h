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


#ifndef _MIX_AND_LINK_H_
#define _MIX_AND_LINK_H_

#include <nuttx/config.h>
#include <unistd.h>
#include <stdint.h>

/**< The maximum number of actuators that get linked to one abstract function*/
#define NR_MAX_ACTUATORS 2

/**
 * The struct mixer_conf_t contains all the necessary information for one mixer.
 * Make an array of type mixer_conf_t for several mix functions.
 */
typedef struct {
	uint8_t source;                          /**< source RC channel index 0..n     		 */
	uint8_t nr_actuators;                    /**< number of actuators to output to 		 */
	uint8_t dest[NR_MAX_ACTUATORS];          /**< Which actuators to output to     		 */
	float   dual_rate[NR_MAX_ACTUATORS];     /**< Direction and rate of mixing. Range [-1,1] */
} mixer_conf_t; /**< Setup for one mixer */

/*
 * The struct mixer_data contains two int16_t arrays with the abstract  source functions
 * (Throttle, Roll,...) in the input, and an empty array output of the desired
 * size for the results.
 */
typedef struct {
	int16_t *input;
	int16_t *output;
} mixer_data_t;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Mixes the functions to the actuators.
 * e.g. Throttle, Roll,...
 *
 *  Each mixer_conf struct in the mixers array contains the source function (ex. THROTTLE), the number
 *  of actuators, an array with the desired actuators, Dual Rate determines which
 *  percentage of the source gets mixed to the destination and sets the direction.
 *  It's range is between -1 and 1.
 *
 *  EXAMPLE:  Deltamix for a wing plane:
 *
 *            mixer_data_t mixer_buffer;
 *            mixer_buffer.input  = buffer_rc;
 *            mixer_buffer.output = buffer_servo;
 *
 *
 *            mixer_conf_t mixers[3];
 *
 *            mixers[0].source = PITCH;
 *            mixers[0].nr_actuators = 2;
 *            mixers[0].dest[0] = AIL_1;
 *            mixers[0].dest[1] = AIL_2;
 *            mixers[0].dual_rate[0] = 1;
 *            mixers[0].dual_rate[1] = 1;
 *
 *            mixers[1].source = ROLL;
 *            mixers[1].nr_actuators = 2;
 *            mixers[1].dest[0] = AIL_1;
 *            mixers[1].dest[1] = AIL_2;
 *            mixers[1].dual_rate[0] = 1;
 *            mixers[1].dual_rate[0] = -1;
 *
 *            mixers[2].source = THROTTLE;
 *            mixers[2].nr_actuators = 1;
 *            mixers[2].dest[0] = MOT;
 *            mixers[2].dual_rate[0] = 1;
 *
 * @param mixers pointer to the mixer struct array
 * @param nr_mixers number of mixers in struct array
 * @param nr_actuators number of actuators
 * @param data pointer to the mixer_data struct.
 */
void mix_and_link(const mixer_conf_t *mixers, uint8_t nr_mixers, uint8_t nr_actuators, mixer_data_t *data);

#endif
