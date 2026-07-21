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

#include <gtest/gtest.h>
#include <float.h>

#include <lib/geo/geo.h>

#include "FirstOrderHoldAltitude.hpp"

// run with: make tests TESTFILTER=FixedWingModeManager

// Target waypoint on the equator, ~1113 m away from the vehicle's start position.
static constexpr double kVehicleLat = 0.0;
static constexpr double kVehicleLon = 0.0;
static constexpr double kCurrLat = 0.0;
static constexpr double kCurrLon = 0.01;

static constexpr float kStartAlt = 100.0f; // altitude the vehicle is at when the target is received
static constexpr float kCurrAlt = 200.0f;  // target altitude

static position_setpoint_s makeCurr(uint8_t type = position_setpoint_s::SETPOINT_TYPE_POSITION,
				    float loiter_radius = 0.0f, float alt = kCurrAlt)
{
	position_setpoint_s sp{};
	sp.type = type;
	sp.lat = kCurrLat;
	sp.lon = kCurrLon;
	sp.alt = alt;
	sp.loiter_radius = loiter_radius;
	return sp;
}

TEST(FirstOrderHoldAltitudeTest, FirstCallWithoutHistoryRampsFromCurrentAltitude)
{
	// No altitude setpoint has been commanded yet: the ramp starts from the vehicle's current altitude, so at
	// the start distance the commanded altitude equals the current altitude.
	FirstOrderHoldAltitudeState state{};
	const float alt = calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, 0.0f, state);
	EXPECT_NEAR(alt, kStartAlt, 1e-2f);
}

TEST(FirstOrderHoldAltitudeTest, RampAlwaysStartsFromCurrentAltitude)
{
	// The ramp always starts from the current (measured) altitude, regardless of any prior ramp state.
	FirstOrderHoldAltitudeState state{};
	const float current_altitude = 90.0f;
	const float alt = calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, current_altitude, 0.0f,
			  state);
	EXPECT_NEAR(alt, current_altitude, 1e-2f);
}

TEST(FirstOrderHoldAltitudeTest, MidpointReturnsInterpolatedAltitude)
{
	// Start the ramp at the full leg length, then move the vehicle halfway to the target: the commanded
	// altitude should be the linear midpoint between the ramp start and target altitude.
	FirstOrderHoldAltitudeState state{};

	// First call captures the ramp start at the full leg length, ramping from kStartAlt.
	calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, 0.0f, state);

	// Move the vehicle to half the leg length from the target (same target altitude -> ramp keeps progressing).
	const double half_lon = kCurrLon / 2.0;
	const float alt = calculateFirstOrderHoldAltitude(makeCurr(), kCurrLat, half_lon, kStartAlt, 0.0f, state);
	EXPECT_NEAR(alt, 0.5f * (kStartAlt + kCurrAlt), 1e-2f);
}

TEST(FirstOrderHoldAltitudeTest, ReachesTargetAltitudeAtAcceptanceRadius)
{
	// Within the acceptance radius around the target the full target altitude is commanded.
	const float acc_rad = 50.0f;
	FirstOrderHoldAltitudeState state{};

	// Start the ramp far from the target.
	calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, acc_rad, state);

	// Place the vehicle on the target waypoint (distance 0, i.e. inside the acceptance radius).
	const float alt = calculateFirstOrderHoldAltitude(makeCurr(), kCurrLat, kCurrLon, kStartAlt, acc_rad, state);
	EXPECT_FLOAT_EQ(alt, kCurrAlt);
}

TEST(FirstOrderHoldAltitudeTest, MinDistanceOnlyDecreases)
{
	// Progress the vehicle close to the target, then move it back away: the commanded altitude must not fall
	// back toward the ramp start because the closest approach is latched.
	FirstOrderHoldAltitudeState state{};

	// Start the ramp at the full leg length.
	calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, 0.0f, state);

	// Move close to the target (small remaining distance).
	const double close_lon = kCurrLon * 0.99;
	const float alt_close = calculateFirstOrderHoldAltitude(makeCurr(), kCurrLat, close_lon, kStartAlt, 0.0f, state);

	// Move back to the start: altitude must not decrease again (closest approach is retained).
	const float alt_back = calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, 0.0f, state);
	EXPECT_FLOAT_EQ(alt_back, alt_close);
}

TEST(FirstOrderHoldAltitudeTest, SameTargetAltitudeDoesNotRestartRamp)
{
	// A position setpoint update that keeps the same target altitude must not restart the ramp: the ramp start
	// altitude and distance stay latched from the first call.
	FirstOrderHoldAltitudeState state{};

	calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, 0.0f, state);
	const float captured_start_alt = state.ramp_start_altitude;
	const float captured_start_dist = state.ramp_start_distance;

	// Call again from a different vehicle altitude but with the same target altitude.
	calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, /*current_altitude*/ 300.0f, 0.0f, state);

	EXPECT_FLOAT_EQ(state.ramp_start_altitude, captured_start_alt);
	EXPECT_FLOAT_EQ(state.ramp_start_distance, captured_start_dist);
}

TEST(FirstOrderHoldAltitudeTest, ChangedTargetAltitudeRestartsRamp)
{
	// A new target altitude restarts the ramp from the current altitude at the current distance.
	FirstOrderHoldAltitudeState state{};

	// First target: ramp toward kCurrAlt and drive it to completion.
	calculateFirstOrderHoldAltitude(makeCurr(), kVehicleLat, kVehicleLon, kStartAlt, 0.0f, state);
	const float alt_reached = calculateFirstOrderHoldAltitude(makeCurr(), kCurrLat, kCurrLon, kStartAlt, 0.0f, state);
	ASSERT_FLOAT_EQ(alt_reached, kCurrAlt);

	// New target altitude while far from the target again: the ramp restarts from the current altitude, which
	// here equals the previous target altitude (the vehicle reached it).
	const float new_target = 260.0f;
	const float alt = calculateFirstOrderHoldAltitude(makeCurr(position_setpoint_s::SETPOINT_TYPE_POSITION, 0.0f,
			  new_target), kVehicleLat, kVehicleLon, /*current_altitude*/ kCurrAlt, 0.0f, state);
	EXPECT_NEAR(alt, kCurrAlt, 1e-2f);
	EXPECT_FLOAT_EQ(state.target_altitude, new_target);
}

TEST(FirstOrderHoldAltitudeTest, LoiterRadiusActsAsAcceptanceRadius)
{
	// Inside the loiter circle of the target the full target altitude is commanded, even when the geometric
	// acceptance radius is zero.
	const float loiter_radius = 80.0f;
	FirstOrderHoldAltitudeState state{};

	// Start the ramp far from the target.
	calculateFirstOrderHoldAltitude(makeCurr(position_setpoint_s::SETPOINT_TYPE_LOITER, loiter_radius), kVehicleLat,
					kVehicleLon, kStartAlt, 0.0f, state);

	// Place the vehicle inside the loiter circle (distance 0 < loiter_radius).
	const float alt = calculateFirstOrderHoldAltitude(makeCurr(position_setpoint_s::SETPOINT_TYPE_LOITER, loiter_radius),
			  kCurrLat, kCurrLon, kStartAlt, 0.0f, state);
	EXPECT_FLOAT_EQ(alt, kCurrAlt);
}
