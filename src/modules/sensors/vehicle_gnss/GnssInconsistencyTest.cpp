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

#include <cmath>
#include <lib/mathlib/mathlib.h>

#include "GnssInconsistency.hpp"

using namespace sensors::gnss_inconsistency;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static constexpr double kLatitude = 47.397742;
static constexpr double kLongitude = 8.545594;
static constexpr double kAltitude = 488.0;

static constexpr float kTolerance = 1e-3f;

// Antennas 0.7 m apart along the body x axis
static const Vector3f kFront(0.35f, 0.f, 0.f);
static const Vector3f kBack(-0.35f, 0.f, 0.f);

static const Quatf kLevelNorth{};

// A receiver with an RTK fixed solution, the given distance north, east and down of the reference point
static sensor_gnss_s receiverAt(float north, float east, float down = 0.f, uint64_t timestamp_sample = 1'000'000)
{
	sensor_gnss_s gnss{};
	gnss.timestamp_sample = timestamp_sample;
	gnss.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	add_vector_to_global_position(kLatitude, kLongitude, north, east, &gnss.latitude, &gnss.longitude);
	gnss.altitude_msl = kAltitude - static_cast<double>(down);
	return gnss;
}

static Inconsistency compare(const sensor_gnss_s &other, const Vector3f &antenna = Vector3f(),
			     const Vector3f &other_antenna = Vector3f(), const Quatf &attitude = kLevelNorth, bool heading_aligned = true)
{
	return inconsistency(receiverAt(0.f, 0.f), antenna, other, other_antenna, attitude, heading_aligned);
}

TEST(GnssInconsistency, ZeroForReceiversThatAgree)
{
	const Inconsistency result = compare(receiverAt(0.f, 0.f));
	EXPECT_NEAR(result.horizontal, 0.f, kTolerance);
	EXPECT_NEAR(result.vertical, 0.f, kTolerance);
}

TEST(GnssInconsistency, DistanceOfReceiversWithoutLeverArm)
{
	EXPECT_NEAR(compare(receiverAt(2.f, 0.f)).horizontal, 2.f, kTolerance);
	EXPECT_NEAR(compare(receiverAt(-1.f, 1.f)).horizontal, sqrtf(2.f), kTolerance);
	EXPECT_NEAR(compare(receiverAt(0.f, 0.f, -1.5f)).vertical, 1.5f, kTolerance);
}

TEST(GnssInconsistency, LeverArmRotatedWithTheHeading)
{
	// Facing north the front antenna is north of the back one, facing east it is east of it
	EXPECT_NEAR(compare(receiverAt(0.7f, 0.f), kBack, kFront).horizontal, 0.f, kTolerance);
	EXPECT_NEAR(compare(receiverAt(0.f, 0.7f), kBack, kFront, Quatf(Eulerf(0.f, 0.f, M_PI_2_F))).horizontal, 0.f,
		    kTolerance);
	EXPECT_NEAR(compare(receiverAt(-0.7f, 0.f), kBack, kFront, Quatf(Eulerf(0.f, 0.f, M_PI_F))).horizontal, 0.f,
		    kTolerance);
}

TEST(GnssInconsistency, SwappedAntennasWithAnAlignedHeading)
{
	// Facing north, the front receiver reports the back antenna's position: off by twice the baseline
	EXPECT_NEAR(compare(receiverAt(-0.7f, 0.f), kBack, kFront).horizontal, 1.4f, kTolerance);
}

TEST(GnssInconsistency, OnlyDistancesWithoutAnAlignedHeading)
{
	// The estimate says north while the vehicle faces south or east: the lever arm still explains the distance
	EXPECT_NEAR(compare(receiverAt(-0.7f, 0.f), kBack, kFront, kLevelNorth, false).horizontal, 0.f, kTolerance);
	EXPECT_NEAR(compare(receiverAt(0.f, 0.7f), kBack, kFront, kLevelNorth, false).horizontal, 0.f, kTolerance);

	// Receivers closer or further apart than their antennas still disagree
	EXPECT_NEAR(compare(receiverAt(0.f, 0.f), kBack, kFront, kLevelNorth, false).horizontal, 0.7f, kTolerance);
	EXPECT_NEAR(compare(receiverAt(5.f, 0.f), kBack, kFront, kLevelNorth, false).horizontal, 4.3f, kTolerance);
}

TEST(GnssInconsistency, TiltApplies)
{
	// Pitched 30 degrees nose up, the front antenna is 0.61 m ahead of and 0.35 m above the back one
	const Quatf pitched_up(Eulerf(0.f, math::radians(30.f), 0.f));
	const sensor_gnss_s front = receiverAt(0.7f * cosf(math::radians(30.f)), 0.f, -0.35f);

	for (bool heading_aligned : {true, false}) {
		const Inconsistency result = compare(front, kBack, kFront, pitched_up, heading_aligned);
		EXPECT_NEAR(result.horizontal, 0.f, kTolerance);
		EXPECT_NEAR(result.vertical, 0.f, kTolerance);
	}

	// Read as level, the same positions are 0.35 m off vertically
	EXPECT_NEAR(compare(front, kBack, kFront).vertical, 0.35f, kTolerance);
}

TEST(GnssInconsistency, AntennaHeightsApply)
{
	// The other antenna is mounted 1 m higher but reports 1.5 m lower: 2.5 m off
	const Vector3f above(0.f, 0.f, -1.f);
	EXPECT_NEAR(compare(receiverAt(0.f, 0.f, -1.f), Vector3f(), above).vertical, 0.f, kTolerance);
	EXPECT_NEAR(compare(receiverAt(0.f, 0.f, 1.5f), Vector3f(), above).vertical, 2.5f, kTolerance);
}

TEST(GnssInconsistency, OtherReceiverMovedToTheReferenceSampleTime)
{
	// At 20 m/s north and 1 m/s up, the other receiver's sample, 0.1 s older, lies 2 m behind and 0.1 m lower
	sensor_gnss_s other = receiverAt(-2.f, 0.f, 0.1f, 900'000);
	other.vel_ned_valid = true;
	other.vel_north = 20.f;
	other.vel_down = -1.f;

	Inconsistency result = compare(other);
	EXPECT_NEAR(result.horizontal, 0.f, kTolerance);
	EXPECT_NEAR(result.vertical, 0.f, kTolerance);

	// Without a velocity it can't be moved
	other.vel_ned_valid = false;
	result = compare(other);
	EXPECT_NEAR(result.horizontal, 2.f, kTolerance);
	EXPECT_NEAR(result.vertical, 0.1f, kTolerance);
}

TEST(GnssInconsistency, UnknownForSamplesFarApart)
{
	// A receiver that missed its samples for a second is not compared, either way round
	Inconsistency result = inconsistency(receiverAt(0.f, 0.f, 0.f, 2'000'000), Vector3f(), receiverAt(0.f, 0.f),
					     Vector3f(), kLevelNorth, true);
	EXPECT_TRUE(std::isnan(result.horizontal));
	EXPECT_TRUE(std::isnan(result.vertical));

	result = compare(receiverAt(0.f, 0.f, 0.f, 2'000'000));
	EXPECT_TRUE(std::isnan(result.horizontal));
	EXPECT_TRUE(std::isnan(result.vertical));

	EXPECT_NEAR(compare(receiverAt(0.f, 0.f, 0.f, 1'400'000)).horizontal, 0.f, kTolerance);
}

TEST(GnssInconsistency, UnknownWithoutAPosition)
{
	sensor_gnss_s no_fix = receiverAt(0.f, 0.f);
	no_fix.fix_type = sensor_gnss_s::FIX_TYPE_NONE;

	EXPECT_TRUE(std::isnan(compare(no_fix).horizontal));
	EXPECT_TRUE(std::isnan(inconsistency(no_fix, Vector3f(), receiverAt(0.f, 0.f), Vector3f(), kLevelNorth,
					     true).horizontal));

	// A 2D fix has no height to compare
	sensor_gnss_s fix_2d = receiverAt(0.f, 0.f);
	fix_2d.fix_type = sensor_gnss_s::FIX_TYPE_2D;
	EXPECT_NEAR(compare(fix_2d).horizontal, 0.f, kTolerance);
	EXPECT_TRUE(std::isnan(compare(fix_2d).vertical));
}
