/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file GroundImpactDetector.hpp
 * Detects a ground impact of a fixed-wing vehicle and holds ground contact until the vehicle is landed.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

namespace land_detector
{

using namespace time_literals;

class GroundImpactDetector
{
public:
	/**
	 * Report ground contact from a detected ground impact until the vehicle is landed.
	 *
	 * @param now current time
	 * @param specific_force_norm magnitude of the measured specific force [m/s^2]
	 * @param in_air vehicle is armed and not landed
	 * @param speed_indicates_flight ground speed and airspeed (where available) are too high for a vehicle on the ground
	 * @return true from a ground impact until the vehicle is landed, flies on or the timeout expires
	 */
	bool update(const hrt_abstime now, const float specific_force_norm, const bool in_air,
		    const bool speed_indicates_flight)
	{
		const float dt = (_time_last_update != 0) ? (now - _time_last_update) * 1e-6f : 0.f;
		const bool dt_valid = dt > 0.f && dt <= kUpdateIntervalMax;
		_time_last_update = now;

		if (!in_air) {
			_time_in_air_start = 0;
			_time_impact = 0;

		} else if (_time_in_air_start == 0) {
			_time_in_air_start = now;
		}

		// Only detect impacts after sustained flight, such that the acceleration during launch is not mistaken for one.
		const bool detection_active = _time_in_air_start != 0 && (now - _time_in_air_start) > kArmTime;

		if (!detection_active || !dt_valid) {
			_baseline.reset(specific_force_norm);
		}

		// An impact is a jump far above the slow baseline. A sustained high load factor is tracked by the baseline.
		if (detection_active && (specific_force_norm - _baseline.getState()) > kImpactJump) {
			_time_impact = now;
		}

		if (dt_valid) {
			_baseline.update(specific_force_norm, dt);
		}

		if (_time_impact != 0) {
			const hrt_abstime time_since_impact = now - _time_impact;

			// A fast pull-up also jumps above the baseline, but the vehicle keeps its speed afterwards.
			const bool flying_on = time_since_impact > kSpeedRejectDelay && speed_indicates_flight;

			if (flying_on || time_since_impact > kGroundContactTimeout) {
				_time_impact = 0;
			}
		}

		return _time_impact != 0;
	}

	static constexpr float kImpactJump = 4.f * CONSTANTS_ONE_G; ///< jump above the baseline regarded as impact [m/s^2]
	static constexpr float kBaselineTimeConstant = 0.5f; ///< time constant of the specific force baseline [s]
	static constexpr float kUpdateIntervalMax = 0.5f; ///< the baseline is reset after a longer gap between updates [s]
	static constexpr hrt_abstime kArmTime = 5_s; ///< continuous flight required before impacts are detected
	static constexpr hrt_abstime kSpeedRejectDelay = 2_s; ///< time after an impact before the speed can reject it
	static constexpr hrt_abstime kGroundContactTimeout = 3_s; ///< maximum time ground contact is held without landing

private:
	AlphaFilter<float> _baseline{kBaselineTimeConstant}; ///< low-pass filtered specific force magnitude [m/s^2]
	hrt_abstime _time_last_update{0};
	hrt_abstime _time_in_air_start{0}; ///< start of the current continuous flight, 0 if not flying
	hrt_abstime _time_impact{0}; ///< time of the last impact, 0 if there is no ground contact from an impact
};

} // namespace land_detector
