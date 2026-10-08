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
 *
 * The network acts in [-1, 1]. An action is read as a per motor thrust of
 * action + 1 in the units the thrust coefficient was fitted in, turned into an
 * rpm through that coefficient, and placed between the motor's rpm limits. The
 * last step compensates the motor's thrust curve.
 */

#pragma once

#include <math.h>

namespace nn_control
{

static constexpr float kThrustCoeffScale = 100000.0f;

// The part of the action range that reaches the motor has to be at least this
// wide, about a thousand representable actions at the edge of the range. Narrower
// than this and float rounding can leave a window with no action inside it, idle
// on one side and full scale on the other. It says nothing about how narrow a
// band a user may want, the range warning reports that.
static constexpr float kMinAchievableActionSpan = 1e-4f;

/**
 * The part of the action range the configured motor can reproduce. Actions below
 * lowest idle the motor, actions above highest run it at full scale.
 */
static inline void achievable_action_range(float thrust_coeff_raw, float min_rpm, float max_rpm,
		float &lowest, float &highest)
{
	const float thrust_coeff = thrust_coeff_raw / kThrustCoeffScale;
	lowest = thrust_coeff * (min_rpm / 60.f) * (min_rpm / 60.f) - 1.f;
	highest = thrust_coeff * (max_rpm / 60.f) * (max_rpm / 60.f) - 1.f;
}

/**
 * True when the limits describe a motor the mapping can work with: finite, the
 * coefficient above zero, 0 <= min < max, and a part of the action range at least
 * kMinAchievableActionSpan wide reaching something between idle and full scale.
 * The parameter metadata bounds each value on its own, this checks them together
 * and against raw writes.
 */
static inline bool valid_motor_limits(float thrust_coeff_raw, float min_rpm, float max_rpm)
{
	if (!isfinite(thrust_coeff_raw) || !isfinite(min_rpm) || !isfinite(max_rpm)) {
		return false;
	}

	if (!(thrust_coeff_raw > 0.f) || !(min_rpm >= 0.f) || !(max_rpm > min_rpm)) {
		return false;
	}

	float lowest = 0.f;
	float highest = 0.f;
	achievable_action_range(thrust_coeff_raw, min_rpm, max_rpm, lowest, highest);

	// values large enough to overflow the range would clip to a full span
	if (!isfinite(lowest) || !isfinite(highest)) {
		return false;
	}

	const float usable_low = (lowest > -1.f) ? lowest : -1.f;
	const float usable_high = (highest < 1.f) ? highest : 1.f;
	return (usable_high - usable_low) > kMinAchievableActionSpan;
}

/**
 * Map one network action in [-1, 1] to a normalized motor command in [0, 1].
 *
 * @param action network output, clamped to [-1, 1]
 * @param thrust_coeff_raw MC_NN_THRST_COEF as stored (scaled by 1e-5 internally)
 * @param min_rpm MC_NN_MIN_RPM
 * @param max_rpm MC_NN_MAX_RPM
 * @return the command, NAN for a non finite action or limits that fail valid_motor_limits()
 */
static inline float rescale_action(float action, float thrust_coeff_raw, float min_rpm, float max_rpm)
{
	if (!valid_motor_limits(thrust_coeff_raw, min_rpm, max_rpm) || !isfinite(action)) {
		return NAN;
	}

	const float thrust_coeff = thrust_coeff_raw / kThrustCoeffScale;

	if (action < -1.0f) {
		action = -1.0f;

	} else if (action > 1.0f) {
		action = 1.0f;
	}

	const float rpm = 60.0f * sqrtf((action + 1.0f) / thrust_coeff);

	// Where that rpm sits between the motor's limits. Below the minimum the motor
	// idles, above the maximum it is at full scale. Without the two bounds the
	// bottom of the action range went negative, which the motor output treats as
	// disarmed, and the top went past full scale.
	float x = (rpm - min_rpm) / (max_rpm - min_rpm);

	if (x < 0.f) {
		x = 0.f;

	} else if (x > 1.f) {
		x = 1.f;
	}

	// Thrust curve compensation, a * x^2 + (1 - a) * x, which is 0 at idle and 1
	// at full scale exactly. This is the same polynomial the module always used,
	// written out instead of in vertex form.
	const float a = 0.8f;
	return a * x * x + (1.0f - a) * x;
}

} // namespace nn_control
