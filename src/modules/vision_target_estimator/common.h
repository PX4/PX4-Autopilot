/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file common.h
 * @brief Definitions common to the position and orientation filters.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <cstdint>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Quaternion.hpp>
#include <matrix/math.hpp>
#include <uORB/topics/sensor_uwb.h>

namespace vision_target_estimator
{

using namespace time_literals;

/* Valid AoA measurement range between -60.00° and +60.00° for UWB*/
static constexpr float max_uwb_aoa_angle_degree = 60.0f;

static constexpr float kMinObservationNoise = 1e-2f;
static constexpr float kMinNisThreshold = 0.1f;

constexpr hrt_abstime kWarnThrottleIntervalUs = 20_s;

static inline bool hasTimedOut(const hrt_abstime ts, const uint32_t timeout_us)
{
	const hrt_abstime now = hrt_absolute_time();

	// Future timestamps are considered invalid -> timed out
	if (ts > now) {
		return true;
	}

	return (now - ts) >= static_cast<hrt_abstime>(timeout_us);
}

constexpr inline int64_t signedTimeDiffUs(const hrt_abstime newer, const hrt_abstime older)
{
	return static_cast<int64_t>(newer) - static_cast<int64_t>(older);
}

union SensorFusionMaskU {
	struct {
		uint16_t use_target_gps_pos   : 1;  ///< bit 0
		uint16_t use_uav_gps_vel      : 1;  ///< bit 1
		uint16_t use_vision_pos       : 1;  ///< bit 2
		uint16_t use_mission_pos      : 1;  ///< bit 3
		uint16_t use_target_gps_vel   : 1;  ///< bit 4
		uint16_t use_uwb              : 1;  ///< bit 5
		uint16_t reserved             : 10; ///< bits 6..15 (future use)
	} flags;

	uint16_t value;
};
static_assert(sizeof(SensorFusionMaskU) == 2, "Unexpected packing");

struct FloatStamped {
	hrt_abstime timestamp = 0;
	bool valid = false;
	float dist_bottom = 0.f;
};

struct Vector3fStamped {
	hrt_abstime timestamp = 0;
	bool valid = false;
	matrix::Vector3f xyz{};
};

inline bool uwbMeasurementToNed(const sensor_uwb_s &uwb_report,
				const matrix::Quaternionf &vehicle_att,
				matrix::Vector3f &relative_pos_ned)
{
	if (!PX4_ISFINITE(uwb_report.distance) || (uwb_report.distance <= 0.f)) {
		return false;
	}

	const float theta_rad = math::radians(uwb_report.aoa_azimuth_dev);
	const float phi_rad = math::radians(uwb_report.aoa_elevation_dev);

	const float distance = uwb_report.distance;

	const float delta_z = -distance * cosf(phi_rad) * cosf(theta_rad);
	const float delta_y = distance * cosf(phi_rad) * sinf(theta_rad);
	const float delta_x = -distance * sinf(phi_rad);

	const matrix::Vector3f relative_pos_sensor(uwb_report.offset_x + delta_x,
			uwb_report.offset_y + delta_y,
			uwb_report.offset_z + delta_z);

	const matrix::Quaternionf sensor_rotation = get_rot_quaternion(static_cast<enum Rotation>(uwb_report.orientation));
	const matrix::Quaternionf body_to_ned = vehicle_att * sensor_rotation;

	relative_pos_ned = body_to_ned.rotateVector(relative_pos_sensor);
	return true;
}

} // namespace vision_target_estimator
