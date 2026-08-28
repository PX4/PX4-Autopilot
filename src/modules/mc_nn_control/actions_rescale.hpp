/****************************************************************************
*
*   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file actions_rescale.hpp
 * Maps a network action to a normalized motor command. Header only and free of
 * module dependencies so it can be unit tested directly.
 */

#pragma once

#include <math.h>

namespace nn_control
{

/**
 * Map one network action in [-1, 1] to a normalized motor command.
 *
 * @param action network output, clamped to [-1, 1]
 * @param thrust_coeff_raw MC_NN_THRST_COEF as stored (scaled by 1e-5 internally)
 * @param min_rpm MC_NN_MIN_RPM
 * @param max_rpm MC_NN_MAX_RPM
 */
static inline float rescale_action(float action, float thrust_coeff_raw, float min_rpm, float max_rpm)
{
	const float thrust_coeff = thrust_coeff_raw / 100000.0f;
	const float a = 0.8f;
	const float b = (1.0f - 0.8f);
	const float tmp1 = b / (2.f * a);
	const float tmp2 = b * b / (4.f * a * a);

	if (action < -1.0f) {
		action = -1.0f;

	} else if (action > 1.0f) {
		action = 1.0f;
	}

	action = action + 1.0f;
	float rps = action / thrust_coeff;
	rps = sqrtf(rps);
	const float rpm = rps * 60.0f;
	const float scaled = (rpm * 2.0f - max_rpm - min_rpm) / (max_rpm - min_rpm);
	return a * (((scaled + 1.0f) / 2.0f + tmp1) * ((scaled + 1.0f) / 2.0f + tmp1) - tmp2);
}

} // namespace nn_control
