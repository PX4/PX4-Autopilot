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

// Samples further apart are not compared: moving one along its velocity for longer adds more error than an RTK
// receiver's accuracy, and a receiver that far behind has missed samples
static constexpr float kMaxSampleTimeDifference = 0.5f; // s

struct Inconsistency {
	float horizontal{NAN}; // m
	float vertical{NAN};   // m
};

/**
 * How far another receiver is from where the reference receiver and the antenna positions put it.
 *
 * The other receiver's position is moved along its velocity to the time of the reference sample, as the receivers
 * don't sample at the same time. The antenna baseline is rotated into NED with the attitude. With an aligned heading
 * the horizontal error is the full error vector, which shows swapped or misplaced antennas. Without one the yaw is
 * arbitrary, so only the horizontal distances are compared: a lever arm then never shows as a disagreement, at any
 * heading. Tilt and the vertical error don't depend on the yaw.
 *
 * @param antenna antenna position of the reference receiver, body frame (m)
 * @param other_antenna antenna position of the other receiver, body frame (m)
 * @param attitude rotation from body to NED, identity when unknown
 * @param heading_aligned the yaw of the attitude is known
 * @return NAN horizontally without a horizontal position, vertically without a 3D fix, or both when the samples are too
 *         far apart in time
 */
inline Inconsistency inconsistency(const sensor_gnss_s &reference, const matrix::Vector3f &antenna,
				   const sensor_gnss_s &other, const matrix::Vector3f &other_antenna,
				   const matrix::Quatf &attitude, bool heading_aligned)
{
	Inconsistency result{};

	if ((reference.fix_type < sensor_gnss_s::FIX_TYPE_2D) || (other.fix_type < sensor_gnss_s::FIX_TYPE_2D)) {
		return result;
	}

	const float dt = 1e-6f * static_cast<float>(static_cast<int64_t>(reference.timestamp_sample)
			 - static_cast<int64_t>(other.timestamp_sample));

	if (fabsf(dt) > kMaxSampleTimeDifference) {
		return result;
	}

	matrix::Vector3f measured{};
	get_vector_to_next_waypoint(reference.latitude, reference.longitude, other.latitude, other.longitude,
				    &measured(0), &measured(1));
	measured(2) = -static_cast<float>(other.altitude_msl - reference.altitude_msl);

	if (other.vel_ned_valid) {
		measured += matrix::Vector3f(other.vel_north, other.vel_east, other.vel_down) * dt;
	}

	const matrix::Vector3f expected = attitude.rotateVector(other_antenna - antenna);
	const matrix::Vector2f measured_horizontal(measured(0), measured(1));
	const matrix::Vector2f expected_horizontal(expected(0), expected(1));

	result.horizontal = heading_aligned ? (measured_horizontal - expected_horizontal).norm()
			    : fabsf(measured_horizontal.norm() - expected_horizontal.norm());

	if ((reference.fix_type >= sensor_gnss_s::FIX_TYPE_3D) && (other.fix_type >= sensor_gnss_s::FIX_TYPE_3D)) {
		result.vertical = fabsf(measured(2) - expected(2));
	}

	return result;
}

} // namespace gnss_inconsistency
} // namespace sensors
