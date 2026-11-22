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

#include "FrontTransitionThrustCheck.hpp"

#include <cmath>
#include <mathlib/mathlib.h>

using namespace time_literals;

static constexpr float kMaxStartShare = 0.5f; // of the transition thrust
static constexpr hrt_abstime kMaxSampleGap = 100_ms;
static constexpr float kCheckTime = 1.f; // [s] at the transition thrust

void FrontTransitionThrustCheck::start(float start_share)
{
	_last_sample = 0;
	_start_share = start_share;
	_weighted_speed_gain = 0.f;
	_weighted_time = 0.f;
	_checking = start_share < kMaxStartShare;
}

bool FrontTransitionThrustCheck::update(hrt_abstime timestamp_sample, float thrust_share, float forward_acceleration,
					float min_acceleration)
{
	// integrate only between consecutive samples, not over gaps
	const hrt_abstime last_sample = _last_sample;
	_last_sample = timestamp_sample;
	const bool consecutive = (last_sample != 0) && (timestamp_sample > last_sample)
				 && (timestamp_sample - last_sample < kMaxSampleGap);

	if (!_checking || !consecutive || !std::isfinite(forward_acceleration)) {
		return false;
	}

	// the acceleration is thrust minus drag, divided by the mass: only the thrust added since the start counts
	const float added_share = math::max(thrust_share - _start_share, 0.f);
	const float dt = (timestamp_sample - last_sample) * 1e-6f;
	_weighted_speed_gain += added_share * forward_acceleration * dt;
	_weighted_time += added_share * added_share * dt;

	if (_weighted_time < math::sq(1.f - _start_share) * kCheckTime) {
		return false;
	}

	_checking = false;

	// fitted acceleration for the whole transition thrust below the minimum
	return _weighted_speed_gain < min_acceleration * _weighted_time;
}
