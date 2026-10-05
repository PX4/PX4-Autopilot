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

#include "GnssInconsistency.hpp"

using namespace sensors::gnss_inconsistency;
using matrix::Vector3f;

static constexpr double kLatitude = 47.397742;
static constexpr double kLongitude = 8.545594;

// A receiver with a fix, the given distance north and east of the reference point
static sensor_gnss_s receiverAt(float north, float east, uint64_t timestamp_sample = 1'000'000)
{
	sensor_gnss_s gnss{};
	gnss.timestamp_sample = timestamp_sample;
	gnss.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	add_vector_to_global_position(kLatitude, kLongitude, north, east, &gnss.latitude, &gnss.longitude);
	return gnss;
}

TEST(GnssInconsistency, ZeroForReceiversThatAgree)
{
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), receiverAt(0.f, 0.f), Vector3f()), 0.f, 1e-3f);
}

TEST(GnssInconsistency, DistanceOfReceiversWithoutLeverArm)
{
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), receiverAt(2.f, 0.f), Vector3f()), 2.f, 1e-3f);
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), receiverAt(-1.f, 1.f), Vector3f()), sqrtf(2.f),
		    1e-3f);
}

TEST(GnssInconsistency, LeverArmExplainsTheDistanceAtAnyHeading)
{
	// antennas 0.7 m apart along the body x axis
	const Vector3f front(0.35f, 0.f, 0.f);
	const Vector3f back(-0.35f, 0.f, 0.f);

	// facing north, east, or anywhere between: the attitude isn't needed
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), back, receiverAt(0.7f, 0.f), front), 0.f, 1e-3f);
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), back, receiverAt(0.f, 0.7f), front), 0.f, 1e-3f);
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), back, receiverAt(-0.495f, -0.495f), front), 0.f, 1e-3f);

	// the height of an antenna doesn't count
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(0.f, 0.f, -0.5f), receiverAt(0.f, 0.f), Vector3f()),
		    0.f, 1e-3f);
}

TEST(GnssInconsistency, ReceiversCloserThanTheirAntennasDisagree)
{
	// 0.7 m apart antennas that report the same position
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(-0.35f, 0.f, 0.f), receiverAt(0.f, 0.f),
					    Vector3f(0.35f, 0.f, 0.f)), 0.7f, 1e-3f);
}

TEST(GnssInconsistency, OtherReceiverMovedToTheReferenceSampleTime)
{
	// at 20 m/s north the other receiver's sample, 0.1 s older, lies 2 m behind
	sensor_gnss_s other = receiverAt(-2.f, 0.f, 900'000);
	other.vel_ned_valid = true;
	other.vel_north = 20.f;

	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), other, Vector3f()), 0.f, 1e-3f);

	// without a velocity it can't be moved
	other.vel_ned_valid = false;
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), other, Vector3f()), 2.f, 1e-3f);
}

TEST(GnssInconsistency, UnknownForSamplesFarApart)
{
	// a receiver that missed its samples for a second is not compared, either way round
	EXPECT_TRUE(std::isnan(horizontalInconsistency(receiverAt(0.f, 0.f, 2'000'000), Vector3f(), receiverAt(0.f, 0.f),
			       Vector3f())));
	EXPECT_TRUE(std::isnan(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), receiverAt(0.f, 0.f, 2'000'000),
			       Vector3f())));
	EXPECT_NEAR(horizontalInconsistency(receiverAt(0.f, 0.f, 1'400'000), Vector3f(), receiverAt(0.f, 0.f), Vector3f()),
		    0.f, 1e-3f);
}

TEST(GnssInconsistency, UnknownWithoutAPosition)
{
	sensor_gnss_s no_fix = receiverAt(0.f, 0.f);
	no_fix.fix_type = sensor_gnss_s::FIX_TYPE_NONE;

	EXPECT_TRUE(std::isnan(horizontalInconsistency(receiverAt(0.f, 0.f), Vector3f(), no_fix, Vector3f())));
	EXPECT_TRUE(std::isnan(horizontalInconsistency(no_fix, Vector3f(), receiverAt(0.f, 0.f), Vector3f())));
}
