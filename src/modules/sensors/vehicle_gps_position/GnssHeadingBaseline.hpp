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
#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/sensor_gnss.h>

namespace sensors
{
namespace gnss_heading
{

// SENS_GNSSn_HDG
enum class BaselineType : int32_t {
	Disabled = 0,
	MovingBase = 1,  // moving base rover: from the other slot's antenna to this slot's antenna
	DualAntenna = 2, // from this slot's antenna to its auxiliary antenna, SENS_GNSSn_AUXX/Y/Z
	TwoReceivers = 3, // independent RTK receivers: from the other slot's antenna to this slot's, by position difference
};

struct Baseline {
	float heading{NAN};          // bearing of the horizontal projection, NED (rad)
	float heading_accuracy{NAN}; // (rad)
	float length{NAN};           // (m)
	float down{NAN};             // down component (m)
	uint64_t time_utc_usec{0};   // GNSS epoch the baseline is measured at, the older of the two fixes' (us)
};

static constexpr uint64_t kMaxEpochMismatch = 50000; // (us) Max epoch mismatch between gnss samples for baseline measurement

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
	case BaselineType::MovingBase:
	case BaselineType::TwoReceivers: return antenna - other_antenna;

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

/**
 * Baseline from one receiver's antenna to another's, measured from their RTK fixed positions at the same GNSS epoch.
 * @param from_gnss fix of the receiver the baseline starts at
 * @param to_gnss fix of the receiver the baseline points to
 * @return false when computation cannot be done
 */
inline bool measureBaseline(const sensor_gnss_s &from_gnss, const sensor_gnss_s &to_gnss, Baseline &baseline)
{
	// Check that fix is correct
	if ((from_gnss.fix_type != sensor_gnss_s::FIX_TYPE_RTK_FIXED) || (to_gnss.fix_type != sensor_gnss_s::FIX_TYPE_RTK_FIXED)) {
		return false;
	}

	// Check that time is filled by the GNSS
	if ((from_gnss.time_utc_usec == 0) || (to_gnss.time_utc_usec == 0)) {
		return false;
	}

	// Time difference between reported gnss times
	const int64_t dt_us = static_cast<int64_t>(to_gnss.time_utc_usec) - static_cast<int64_t>(from_gnss.time_utc_usec);

	// Check dt_us is not too large
	if ((dt_us > static_cast<int64_t>(kMaxEpochMismatch)) || (-dt_us > static_cast<int64_t>(kMaxEpochMismatch))) {
		return false;
	}

	const sensor_gnss_s &old_gnss = dt_us > 0 ? from_gnss : to_gnss;
	const sensor_gnss_s &new_gnss = dt_us > 0 ? to_gnss : from_gnss;

	// Check velocity is valid when required
	if ((dt_us != 0) && !new_gnss.vel_ned_valid) {
		return false;
	}

	const float vel_north = new_gnss.vel_north;
	const float vel_east = new_gnss.vel_east;
	const float vel_down = new_gnss.vel_down;
	double compensated_lat = new_gnss.latitude;
	double compensated_lon = new_gnss.longitude;
	double compensated_alt = new_gnss.altitude_ellipsoid;

	const float dt = fabsf(static_cast<float>(dt_us) * 1e-6f); // absolute value of delta time

	// Rewind newest gnss position by dt using velocity
	add_vector_to_global_position(compensated_lat, compensated_lon, -vel_north * dt, -vel_east * dt,
				      &compensated_lat, &compensated_lon);
	compensated_alt += static_cast<double>(vel_down * dt);

	// The baseline runs from from_gnss to to_gnss, whichever of them was rewound
	const bool from_gnss_is_old = (dt_us > 0);
	const double from_lat = from_gnss_is_old ? old_gnss.latitude : compensated_lat;
	const double from_lon = from_gnss_is_old ? old_gnss.longitude : compensated_lon;
	const double from_alt = from_gnss_is_old ? old_gnss.altitude_ellipsoid : compensated_alt;
	const double to_lat = from_gnss_is_old ? compensated_lat : old_gnss.latitude;
	const double to_lon = from_gnss_is_old ? compensated_lon : old_gnss.longitude;
	const double to_alt = from_gnss_is_old ? compensated_alt : old_gnss.altitude_ellipsoid;

	float baseline_north = NAN;
	float baseline_east = NAN;
	get_vector_to_next_waypoint(from_lat, from_lon, to_lat, to_lon, &baseline_north, &baseline_east);
	const float baseline_down = static_cast<float>(from_alt - to_alt);

	const float horizontal_length = sqrtf(baseline_north * baseline_north + baseline_east * baseline_east);

	if (!(horizontal_length > FLT_EPSILON) || !PX4_ISFINITE(from_gnss.eph) || !PX4_ISFINITE(to_gnss.eph)) {
		return false;
	}

	// Both receivers' errors added as if independent. With corrections from the same base they are largely common
	// and cancel in the difference, so this overstates the error.
	const float extrapolation_error = PX4_ISFINITE(new_gnss.speed_accuracy) ? new_gnss.speed_accuracy * dt : 0.f;
	const float horizontal_error = sqrtf(from_gnss.eph * from_gnss.eph + to_gnss.eph * to_gnss.eph
					     + extrapolation_error * extrapolation_error);

	baseline.heading = atan2f(baseline_east, baseline_north);
	baseline.heading_accuracy = atan2f(horizontal_error, horizontal_length);
	baseline.length = sqrtf(horizontal_length * horizontal_length + baseline_down * baseline_down);
	baseline.down = baseline_down;
	baseline.time_utc_usec = old_gnss.time_utc_usec;
	return true;
}

} // namespace gnss_heading
} // namespace sensors
