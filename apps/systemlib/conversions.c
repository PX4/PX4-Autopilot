/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file conversions.c
 * Implementation of commonly used conversions.
 */

#include <nuttx/config.h>
#include <float.h>

#include "conversions.h"

int16_t
int16_t_from_bytes(uint8_t bytes[])
{
	union {
		uint8_t    b[2];
		int16_t    w;
	} u;

	u.b[1] = bytes[0];
	u.b[0] = bytes[1];

	return u.w;
}

void rot2quat(const float R[9], float Q[4])
{
	float q0_2;
	float q1_2;
	float q2_2;
	float q3_2;
	int32_t idx;

	/* conversion of rotation matrix to quaternion
	 * choose the largest component to begin with */
	q0_2 = (((1.0F + R[0]) + R[4]) + R[8]) / 4.0F;
	q1_2 = (((1.0F + R[0]) - R[4]) - R[8]) / 4.0F;
	q2_2 = (((1.0F - R[0]) + R[4]) - R[8]) / 4.0F;
	q3_2 = (((1.0F - R[0]) - R[4]) + R[8]) / 4.0F;

	idx = 0;

	if (q0_2 < q1_2) {
		q0_2 = q1_2;

		idx = 1;
	}

	if (q0_2 < q2_2) {
		q0_2 = q2_2;
		idx = 2;
	}

	if (q0_2 < q3_2) {
		q0_2 = q3_2;
		idx = 3;
	}

	q0_2 = sqrtf(q0_2);

	/* solve for the remaining three components */
	if (idx == 0) {
		q1_2 = q0_2;
		q2_2 = (R[5] - R[7]) / 4.0F / q0_2;
		q3_2 = (R[6] - R[2]) / 4.0F / q0_2;
		q0_2 = (R[1] - R[3]) / 4.0F / q0_2;

	} else if (idx == 1) {
		q2_2 = q0_2;
		q1_2 = (R[5] - R[7]) / 4.0F / q0_2;
		q3_2 = (R[3] + R[1]) / 4.0F / q0_2;
		q0_2 = (R[6] + R[2]) / 4.0F / q0_2;

	} else if (idx == 2) {
		q3_2 = q0_2;
		q1_2 = (R[6] - R[2]) / 4.0F / q0_2;
		q2_2 = (R[3] + R[1]) / 4.0F / q0_2;
		q0_2 = (R[7] + R[5]) / 4.0F / q0_2;

	} else {
		q1_2 = (R[1] - R[3]) / 4.0F / q0_2;
		q2_2 = (R[6] + R[2]) / 4.0F / q0_2;
		q3_2 = (R[7] + R[5]) / 4.0F / q0_2;
	}

	/* return values */
	Q[0] = q1_2;
	Q[1] = q2_2;
	Q[2] = q3_2;
	Q[3] = q0_2;
}

void quat2rot(const float Q[4], float R[9])
{
	float q0_2;
	float q1_2;
	float q2_2;
	float q3_2;

	memset(&R[0], 0, 9U * sizeof(float));

	q0_2 = Q[0] * Q[0];
	q1_2 = Q[1] * Q[1];
	q2_2 = Q[2] * Q[2];
	q3_2 = Q[3] * Q[3];

	R[0] = ((q0_2 + q1_2) - q2_2) - q3_2;
	R[3] = 2.0F * (Q[1] * Q[2] - Q[0] * Q[3]);
	R[6] = 2.0F * (Q[1] * Q[3] + Q[0] * Q[2]);
	R[1] = 2.0F * (Q[1] * Q[2] + Q[0] * Q[3]);
	R[4] = ((q0_2 + q2_2) - q1_2) - q3_2;
	R[7] = 2.0F * (Q[2] * Q[3] - Q[0] * Q[1]);
	R[2] = 2.0F * (Q[1] * Q[3] - Q[0] * Q[2]);
	R[5] = 2.0F * (Q[2] * Q[3] + Q[0] * Q[1]);
	R[8] = ((q0_2 + q3_2) - q1_2) - q2_2;
}

float get_air_density(float static_pressure, float temperature_celsius)
{
	return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius + CONSTANTS_ABSOLUTE_NULL_KELVIN));
}
