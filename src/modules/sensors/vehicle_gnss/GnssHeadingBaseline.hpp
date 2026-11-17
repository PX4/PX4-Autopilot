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

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

namespace sensors
{
namespace gnss_heading
{

// SENS_GNSSn_HDG
enum class BaselineType : int32_t {
	Disabled = 0,
	MovingBase = 1,  // moving base rover: from the other slot's antenna to this slot's antenna
	DualAntenna = 2, // from this slot's antenna to its auxiliary antenna, SENS_GNSSn_AUXX/Y/Z
};

// ArduPilot's moving baseline length checks (AP_GPS_Backend::calculate_moving_base_yaw)
static constexpr float kMinAntennaSeparation = 0.05f; // m
static constexpr float kPermittedLengthError = 0.2f;  // fraction of the shorter of the configured and reported baselines

/**
 * Baseline from the antenna the heading is measured from to the antenna it points to, body frame (m).
 * Zero when the slot has no heading baseline.
 */
inline matrix::Vector3f configuredBaseline(int32_t type, const matrix::Vector3f &antenna,
		const matrix::Vector3f &other_antenna, const matrix::Vector3f &aux_antenna)
{
	switch (static_cast<BaselineType>(type)) {
	case BaselineType::MovingBase: return antenna - other_antenna;

	case BaselineType::DualAntenna: return aux_antenna - antenna;

	default: return {};
	}
}

/**
 * Whether a reported baseline matches the configured one. A baseline the receiver doesn't report (NAN) is not checked.
 * ArduPilot also checks the down component against the attitude; that is left out, since on a 0.35 m moving baseline
 * it scattered by up to 0.16 m while turning and rejected good headings, and caught nothing the length check missed.
 * @param configured_length length of the configured baseline (m)
 * @param reported_length reported baseline length (m)
 * @param reported_down reported down component of the baseline (m)
 */
inline bool baselineConsistent(float configured_length, float reported_length, float reported_down)
{
	if (!(configured_length >= kMinAntennaSeparation)) {
		return false;
	}

	if (!PX4_ISFINITE(reported_length)) {
		return true;
	}

	const float tolerance = kPermittedLengthError * fminf(configured_length, reported_length);

	if ((reported_length < kMinAntennaSeparation) || (fabsf(configured_length - reported_length) > tolerance)) {
		return false;
	}

	// the heading is the bearing of the horizontal projection, which a near vertical baseline doesn't have
	return !(reported_length * reported_length - reported_down * reported_down
		 < kMinAntennaSeparation * kMinAntennaSeparation);
}

} // namespace gnss_heading
} // namespace sensors
