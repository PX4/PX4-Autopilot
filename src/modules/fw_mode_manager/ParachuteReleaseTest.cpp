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

#include "ParachuteRelease.hpp"

using matrix::Vector2f;

// approach flying due north, landing point ahead
static const Vector2f kApproachDirection(1.f, 0.f);

// defaults used throughout: 20 m release altitude, 5 m/s sink -> 15 m floor, 4 s descent from 20 m
static constexpr float kSink = 5.f;
static constexpr float kReleaseAlt = 20.f;

TEST(ParachuteRelease, releaseAltitudeIsFlooredForCanopyOpening)
{
	const float floor = parachuteReleaseFloor(kSink);
	EXPECT_FLOAT_EQ(floor, kSink * kParachuteCanopyOpenTime);

	// configured altitude above the floor is used as is
	EXPECT_FLOAT_EQ(parachuteReleaseAltitude(20.f, kSink), 20.f);

	// configured altitude below the floor is raised to it
	EXPECT_FLOAT_EQ(parachuteReleaseAltitude(5.f, kSink), floor);
}

TEST(ParachuteRelease, noWindReleaseDistanceIsDeploymentCarry)
{
	const Vector2f ground_speed(20.f, 0.f);

	const float dist = parachuteReleaseDistance(ground_speed, Vector2f(0.f, 0.f), true, kApproachDirection,
			   kReleaseAlt, kSink);

	EXPECT_FLOAT_EQ(dist, 20.f * kParachuteDeploymentTime);
}

TEST(ParachuteRelease, invalidWindIsIgnored)
{
	const Vector2f ground_speed(20.f, 0.f);
	const Vector2f wind(-15.f, 10.f);

	const float dist = parachuteReleaseDistance(ground_speed, wind, false, kApproachDirection, kReleaseAlt, kSink);

	EXPECT_FLOAT_EQ(dist, 20.f * kParachuteDeploymentTime);
}

TEST(ParachuteRelease, headwindMovesReleasePastLandingPoint)
{
	// strong headwind: barely progressing at 5 m/s ground speed
	const Vector2f ground_speed(5.f, 0.f);
	const Vector2f wind(-15.f, 0.f);

	const float dist = parachuteReleaseDistance(ground_speed, wind, true, kApproachDirection, kReleaseAlt, kSink);

	// 5 m carry, 60 m drift back during the 4 s descent: release 55 m past the point
	EXPECT_FLOAT_EQ(dist, 5.f - 60.f);
}

TEST(ParachuteRelease, windAboveGroundSpeedStaysFinite)
{
	// wind pushes the vehicle backwards: the carry is clamped to zero, no release before the point
	const Vector2f ground_speed(-2.f, 0.f);
	const Vector2f wind(-25.f, 0.f);

	const float dist = parachuteReleaseDistance(ground_speed, wind, true, kApproachDirection, kReleaseAlt, kSink);

	EXPECT_TRUE(std::isfinite(dist));
	EXPECT_FLOAT_EQ(dist, -25.f * 4.f);
}

TEST(ParachuteRelease, tailwindReleasesEarlier)
{
	const Vector2f ground_speed(25.f, 0.f);
	const Vector2f wind(5.f, 0.f);

	const float dist = parachuteReleaseDistance(ground_speed, wind, true, kApproachDirection, kReleaseAlt, kSink);

	EXPECT_FLOAT_EQ(dist, 25.f + 20.f);
}

TEST(ParachuteRelease, pureCrosswindDoesNotChangeReleaseDistance)
{
	const Vector2f ground_speed(20.f, 0.f);
	const Vector2f wind(0.f, 8.f);

	const float dist = parachuteReleaseDistance(ground_speed, wind, true, kApproachDirection, kReleaseAlt, kSink);

	EXPECT_FLOAT_EQ(dist, 20.f * kParachuteDeploymentTime);
}

TEST(ParachuteRelease, crosswindAimShiftIsPerpendicularAndDownwind)
{
	const float floor = parachuteReleaseFloor(kSink);
	const Vector2f wind(0.f, 5.f); // pure crosswind from the west, drifting the vehicle east

	// holding the release altitude
	const float altitude_above_ground = kReleaseAlt;
	const Vector2f shift = parachuteCrosswindAimShift(wind, kApproachDirection, altitude_above_ground, kReleaseAlt,
			       floor, kSink);

	// no along-track component: that part is handled by the release timing
	EXPECT_FLOAT_EQ(shift.dot(kApproachDirection), 0.f);

	// 4 s descent at 5 m/s crosswind: 20 m drift east; subtracting the shift aims 20 m upwind (west)
	EXPECT_FLOAT_EQ(shift(1), 20.f);
}

TEST(ParachuteRelease, diagonalWindSplitsIntoTimingAndAimShift)
{
	const float floor = parachuteReleaseFloor(kSink);
	const Vector2f ground_speed(20.f, 0.f);
	const Vector2f wind(-3.f, 4.f);

	// the along-track component only affects the release distance
	const float dist = parachuteReleaseDistance(ground_speed, wind, true, kApproachDirection, kReleaseAlt, kSink);
	EXPECT_FLOAT_EQ(dist, 20.f - 3.f * 4.f);

	// the cross component only affects the aim shift, holding the release altitude
	const float altitude_above_ground = kReleaseAlt;
	const Vector2f shift = parachuteCrosswindAimShift(wind, kApproachDirection, altitude_above_ground, kReleaseAlt,
			       floor, kSink);
	EXPECT_FLOAT_EQ(shift(0), 0.f);
	EXPECT_FLOAT_EQ(shift(1), 4.f * 4.f);
}

TEST(ParachuteRelease, aimShiftUsesExpectedReleaseAltitude)
{
	const float floor = parachuteReleaseFloor(kSink);
	const Vector2f wind(0.f, 5.f);

	// above the release altitude: the release will happen at the release altitude
	const Vector2f shift_above = parachuteCrosswindAimShift(wind, kApproachDirection, 60.f, kReleaseAlt, floor, kSink);
	EXPECT_FLOAT_EQ(shift_above(1), 5.f * kReleaseAlt / kSink);

	// inside the band (e.g. a glider sinking through): the release happens at the current altitude
	const Vector2f shift_inside = parachuteCrosswindAimShift(wind, kApproachDirection, 17.f, kReleaseAlt, floor, kSink);
	EXPECT_FLOAT_EQ(shift_inside(1), 5.f * 17.f / kSink);

	// below the floor: the release cannot happen lower than the floor
	const Vector2f shift_below = parachuteCrosswindAimShift(wind, kApproachDirection, 5.f, kReleaseAlt, floor, kSink);
	EXPECT_FLOAT_EQ(shift_below(1), 5.f * floor / kSink);
}

TEST(ParachuteRelease, zeroSinkRateIsGuarded)
{
	const Vector2f ground_speed(20.f, 0.f);
	const Vector2f wind(-5.f, 5.f);

	const float dist = parachuteReleaseDistance(ground_speed, wind, true, kApproachDirection, kReleaseAlt, 0.f);
	EXPECT_TRUE(std::isfinite(dist));

	const Vector2f shift = parachuteCrosswindAimShift(wind, kApproachDirection, kReleaseAlt, kReleaseAlt,
			       parachuteReleaseFloor(0.f), 0.f);
	EXPECT_TRUE(std::isfinite(shift(0)));
	EXPECT_TRUE(std::isfinite(shift(1)));
}
