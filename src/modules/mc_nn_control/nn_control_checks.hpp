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
 * @file nn_control_checks.hpp
 * Checks the neural controller runs before it trusts its model, its observations
 * and its output. Header only and free of module dependencies so they can be unit
 * tested directly.
 */

#pragma once

#include <math.h>
#include <stdint.h>

namespace nn_control
{

static constexpr int kInputSize = 15;
static constexpr int kOutputSize = 4;

// Oldest observation the network is run on, in microseconds, the same limits the
// raptor module uses
static constexpr uint64_t kMaxLocalPositionAge = 100000;
static constexpr uint64_t kMaxAttitudeAge = 50000;
static constexpr uint64_t kMaxAngularVelocityAge = 10000;

// The controller runs on angular velocity updates. If they stop, a delayed run
// this long after the last one is what notices.
static constexpr uint64_t kAngularVelocityWatchdog = 2 * kMaxAngularVelocityAge;

enum class ObservationFault : uint8_t {
	None = 0,
	PositionInvalid = 1,
	PositionStale = 2,
	PositionNotFinite = 3,
	AttitudeStale = 4,
	AttitudeNotFinite = 5,
	AngularVelocityStale = 6,
	AngularVelocityNotFinite = 7,
};

// What the network is fed, copied out of the uORB topics so the check needs
// nothing from the module
struct ObservationSnapshot {
	uint64_t position_timestamp;
	bool xy_valid;
	bool z_valid;
	bool v_xy_valid;
	bool v_z_valid;
	float position[3];
	float velocity[3];
	uint64_t attitude_timestamp;
	float q[4];
	uint64_t angular_velocity_timestamp;
	float angular_velocity[3];
};

static inline bool all_finite(const float *values, int count)
{
	for (int i = 0; i < count; i++) {
		if (!isfinite(values[i])) {
			return false;
		}
	}

	return true;
}

// A stamp of zero has never been set. A stamp ahead of now is not stale.
static inline bool fresh(uint64_t now, uint64_t stamp, uint64_t max_age)
{
	return (stamp != 0) && ((now <= stamp) || ((now - stamp) <= max_age));
}

static inline ObservationFault check_observations(const ObservationSnapshot &o, uint64_t now)
{
	if (!o.xy_valid || !o.z_valid || !o.v_xy_valid || !o.v_z_valid) {
		return ObservationFault::PositionInvalid;
	}

	if (!fresh(now, o.position_timestamp, kMaxLocalPositionAge)) {
		return ObservationFault::PositionStale;
	}

	if (!all_finite(o.position, 3) || !all_finite(o.velocity, 3)) {
		return ObservationFault::PositionNotFinite;
	}

	if (!fresh(now, o.attitude_timestamp, kMaxAttitudeAge)) {
		return ObservationFault::AttitudeStale;
	}

	if (!all_finite(o.q, 4)) {
		return ObservationFault::AttitudeNotFinite;
	}

	if (!fresh(now, o.angular_velocity_timestamp, kMaxAngularVelocityAge)) {
		return ObservationFault::AngularVelocityStale;
	}

	if (!all_finite(o.angular_velocity, 3)) {
		return ObservationFault::AngularVelocityNotFinite;
	}

	return ObservationFault::None;
}

static inline bool outputs_finite(const float *outputs, int count)
{
	return all_finite(outputs, count);
}

/**
 * Why a loaded model cannot be run, or nullptr when it can. The documentation
 * tells users to swap the model array, so the shape is checked rather than
 * assumed.
 */
static inline const char *model_layout_problem(int32_t schema_version, int32_t expected_schema_version,
		int input_tensors, int input_count, bool input_is_float,
		int output_tensors, int output_count, bool output_is_float)
{
	if (schema_version != expected_schema_version) {
		return "schema version";
	}

	if (input_tensors != 1) {
		return "input tensors, expected one";
	}

	if (output_tensors != 1) {
		return "output tensors, expected one";
	}

	if (input_count != kInputSize) {
		return "input size, expected 15 elements";
	}

	if (!input_is_float) {
		return "input type, expected float32";
	}

	if (output_count != kOutputSize) {
		return "output size, expected 4 elements";
	}

	if (!output_is_float) {
		return "output type, expected float32";
	}

	return nullptr;
}

} // namespace nn_control
