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

#include "ParachuteRelease.hpp"

#include <mathlib/mathlib.h>

float parachuteReleaseFloor(const float sink_rate)
{
	return math::max(sink_rate, kParachuteMinSinkRate) * kParachuteCanopyOpenTime;
}

float parachuteReleaseAltitude(const float release_alt_param, const float sink_rate)
{
	return math::max(release_alt_param, parachuteReleaseFloor(sink_rate));
}

matrix::Vector2f parachuteCrosswindAimShift(const matrix::Vector2f &wind_vel,
		const matrix::Vector2f &approach_direction, const float altitude_above_ground, const float release_alt,
		const float release_floor, const float sink_rate)
{
	const float release_alt_expected = math::constrain(altitude_above_ground, release_floor, release_alt);
	const matrix::Vector2f drift = wind_vel * release_alt_expected / math::max(sink_rate, kParachuteMinSinkRate);

	return drift - approach_direction * drift.dot(approach_direction);
}

float parachuteReleaseDistance(const matrix::Vector2f &ground_speed, const matrix::Vector2f &wind_vel,
			       const bool wind_valid, const matrix::Vector2f &approach_direction, const float altitude_above_ground,
			       const float sink_rate)
{
	const float ground_speed_along_track = math::max(ground_speed.dot(approach_direction), 0.f);
	const float wind_along_track = wind_valid ? wind_vel.dot(approach_direction) : 0.f;
	const float descent_time = altitude_above_ground / math::max(sink_rate, kParachuteMinSinkRate);

	return ground_speed_along_track * kParachuteDeploymentTime + wind_along_track * descent_time;
}
