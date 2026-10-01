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

#include <lib/geo/geo.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/sensor_gnss.h>

namespace sensors
{
namespace gnss_inconsistency
{

// Samples further apart are not compared: moving one along its velocity for longer would add more error than an
// RTK receiver's accuracy, and a receiver that far behind has missed samples
static constexpr float kMaxSampleTimeDifference = 0.5f; // s

/**
 * How far apart two receivers report the vehicle, beyond what their antenna positions explain (m): the difference
 * between the horizontal distance of their positions and the horizontal distance of their antennas. Without the
 * attitude this is the least the receivers disagree by at any heading, so a lever arm never shows as a disagreement.
 *
 * The other receiver's position is moved along its velocity to the time of the reference sample, as the receivers
 * don't sample at the same time.
 *
 * @param antenna antenna position of the reference receiver, body frame (m)
 * @param other_antenna antenna position of the other receiver, body frame (m)
 * @return NAN when either receiver has no horizontal position, or their samples are too far apart in time
 */
inline float horizontalInconsistency(const sensor_gnss_s &reference, const matrix::Vector3f &antenna,
				     const sensor_gnss_s &other, const matrix::Vector3f &other_antenna)
{
	if ((reference.fix_type < sensor_gnss_s::FIX_TYPE_2D) || (other.fix_type < sensor_gnss_s::FIX_TYPE_2D)) {
		return NAN;
	}

	const float dt = 1e-6f * static_cast<float>(static_cast<int64_t>(reference.timestamp_sample)
			 - static_cast<int64_t>(other.timestamp_sample));

	if (fabsf(dt) > kMaxSampleTimeDifference) {
		return NAN;
	}

	float north = 0.f;
	float east = 0.f;
	get_vector_to_next_waypoint(reference.latitude, reference.longitude, other.latitude, other.longitude, &north, &east);

	if (other.vel_ned_valid) {
		north += other.vel_north * dt;
		east += other.vel_east * dt;
	}

	const float distance = matrix::Vector2f(north, east).norm();
	const float antenna_distance = matrix::Vector2f(other_antenna(0) - antenna(0), other_antenna(1) - antenna(1)).norm();
	const float inconsistency = fabsf(distance - antenna_distance);

	return PX4_ISFINITE(inconsistency) ? inconsistency : NAN;
}

} // namespace gnss_inconsistency
} // namespace sensors
