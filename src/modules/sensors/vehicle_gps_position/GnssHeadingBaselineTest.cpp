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

#include "GnssHeadingBaseline.hpp"

using namespace sensors::gnss_heading;
using matrix::Vector3f;

namespace
{

static constexpr double kLatitude = 47.397742;  // deg
static constexpr double kLongitude = 8.545594;  // deg
static constexpr double kAltitude = 488.0;      // m
static constexpr uint64_t kEpoch = 1760000000000000; // us
static constexpr float kTolerance = 1e-4f;

// RTK fixed receiver displaced from the reference point by north/east/down (m)
sensor_gnss_s rtkFix(float north, float east, float down = 0.f, uint64_t time_utc_usec = kEpoch)
{
	sensor_gnss_s gnss{};
	gnss.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	add_vector_to_global_position(kLatitude, kLongitude, north, east, &gnss.latitude, &gnss.longitude);
	gnss.altitude_ellipsoid = kAltitude - static_cast<double>(down);
	gnss.time_utc_usec = time_utc_usec;
	gnss.eph = 0.01f;
	gnss.vel_ned_valid = true;
	gnss.speed_accuracy = NAN;
	return gnss;
}

// the receiver's fix dt_us later, having moved with the given velocity (m/s)
sensor_gnss_s later(const sensor_gnss_s &gnss, uint64_t dt_us, float vel_north, float vel_east, float vel_down)
{
	const float dt = static_cast<float>(dt_us) * 1e-6f;
	sensor_gnss_s moved = gnss;
	add_vector_to_global_position(gnss.latitude, gnss.longitude, vel_north * dt, vel_east * dt,
				      &moved.latitude, &moved.longitude);
	moved.altitude_ellipsoid = gnss.altitude_ellipsoid - static_cast<double>(vel_down * dt);
	moved.vel_north = vel_north;
	moved.vel_east = vel_east;
	moved.vel_down = vel_down;
	moved.time_utc_usec = gnss.time_utc_usec + dt_us;
	return moved;
}

} // namespace

TEST(GnssHeadingBaseline, MovingBaseRunsFromTheOtherReceiver)
{
	// rover (this slot) 0.17 m right of the centre line, moving base (other slot) 0.17 m left
	const Vector3f baseline = configuredBaseline(static_cast<int32_t>(BaselineType::MovingBase),
				  Vector3f(0.f, 0.17f, 0.f), Vector3f(0.f, -0.17f, 0.f), Vector3f(1.f, 0.f, 0.f));

	// the rover is on the right: a measured heading of 90 deg means the vehicle faces north
	EXPECT_FLOAT_EQ(atan2f(baseline(1), baseline(0)), M_PI_F / 2.f);
	EXPECT_FLOAT_EQ(baseline.norm(), 0.34f);

	EXPECT_EQ(configuredBaseline(static_cast<int32_t>(BaselineType::Disabled), Vector3f(0.f, 0.17f, 0.f),
				     Vector3f(0.f, -0.17f, 0.f), Vector3f(0.3f, 0.f, 0.f)), Vector3f());
}

TEST(GnssHeadingBaseline, DualAntennaRunsFromTheMainToTheAuxAntenna)
{
	// main antenna 0.1 m behind the centre of gravity, auxiliary antenna 0.2 m in front; the other slot is ignored
	const Vector3f baseline = configuredBaseline(static_cast<int32_t>(BaselineType::DualAntenna),
				  Vector3f(-0.1f, 0.f, 0.f), Vector3f(0.f, -0.17f, 0.f), Vector3f(0.2f, 0.f, 0.f));

	EXPECT_FLOAT_EQ(atan2f(baseline(1), baseline(0)), 0.f);
	EXPECT_FLOAT_EQ(baseline.norm(), 0.3f);
}

TEST(GnssHeadingBaseline, RejectsFloatSolutions)
{
	// ARK G5 moving-base pair, 0.347 m apart: the rover's float solutions reported 10.9 m and 1.04 m
	EXPECT_FALSE(baselineConsistent(0.347f, 10.9f, 7.52f));
	EXPECT_FALSE(baselineConsistent(0.347f, 1.04f, -0.71f));
	EXPECT_TRUE(baselineConsistent(0.347f, 0.345f, -0.02f));

	// 20% of the shorter baseline
	EXPECT_TRUE(baselineConsistent(0.4f, 0.47f, NAN));
	EXPECT_FALSE(baselineConsistent(0.4f, 0.49f, NAN));
	EXPECT_FALSE(baselineConsistent(0.4f, 0.32f, NAN));
}

TEST(GnssHeadingBaseline, RejectsAVerticalBaseline)
{
	// the reported baseline is close to vertical, so it has no heading
	EXPECT_FALSE(baselineConsistent(0.35f, 0.35f, 0.348f));
	EXPECT_TRUE(baselineConsistent(0.35f, 0.35f, 0.16f));
}

TEST(GnssHeadingBaseline, NeedsAConfiguredBaseline)
{
	// a receiver that doesn't report its baseline is not checked against it, but still needs one to rotate its heading
	EXPECT_TRUE(baselineConsistent(0.35f, NAN, NAN));
	EXPECT_FALSE(baselineConsistent(0.f, NAN, NAN));
	EXPECT_FALSE(baselineConsistent(0.04f, 0.04f, 0.f));
}

TEST(GnssHeadingBaseline, TwoReceiversRunsFromTheOtherReceiver)
{
	// same convention as a moving base pair: from the other slot's antenna to this slot's
	const Vector3f baseline = configuredBaseline(static_cast<int32_t>(BaselineType::TwoReceivers),
				  Vector3f(0.5f, 0.f, 0.f), Vector3f(-0.5f, 0.f, -0.1f), Vector3f(1.f, 1.f, 0.f));

	EXPECT_EQ(baseline, Vector3f(1.f, 0.f, 0.1f));
	EXPECT_FLOAT_EQ(atan2f(baseline(1), baseline(0)), 0.f);
}

TEST(GnssHeadingBaseline, MeasuresTheBearingFromOneReceiverToTheOther)
{
	const sensor_gnss_s from = rtkFix(0.f, 0.f);
	Baseline baseline{};

	ASSERT_TRUE(measureBaseline(from, rtkFix(1.f, 0.f), baseline));
	EXPECT_NEAR(baseline.heading, 0.f, kTolerance);
	EXPECT_NEAR(baseline.length, 1.f, kTolerance);
	EXPECT_NEAR(baseline.down, 0.f, kTolerance);
	EXPECT_EQ(baseline.time_utc_usec, kEpoch);

	ASSERT_TRUE(measureBaseline(from, rtkFix(0.f, 1.f), baseline));
	EXPECT_NEAR(baseline.heading, M_PI_F / 2.f, kTolerance);

	ASSERT_TRUE(measureBaseline(from, rtkFix(0.f, -1.f), baseline));
	EXPECT_NEAR(baseline.heading, -M_PI_F / 2.f, kTolerance);

	ASSERT_TRUE(measureBaseline(from, rtkFix(-1.f, 0.f), baseline));
	EXPECT_NEAR(fabsf(baseline.heading), M_PI_F, kTolerance);

	ASSERT_TRUE(measureBaseline(from, rtkFix(0.5f, 0.5f), baseline));
	EXPECT_NEAR(baseline.heading, M_PI_F / 4.f, kTolerance);
	EXPECT_NEAR(baseline.length, sqrtf(0.5f), kTolerance);
}

TEST(GnssHeadingBaseline, SwappingTheReceiversReversesTheBaseline)
{
	const sensor_gnss_s a = rtkFix(0.3f, -0.2f, 0.f);
	const sensor_gnss_s b = rtkFix(-0.4f, 0.6f, 0.05f);
	Baseline a_to_b{};
	Baseline b_to_a{};

	ASSERT_TRUE(measureBaseline(a, b, a_to_b));
	ASSERT_TRUE(measureBaseline(b, a, b_to_a));
	EXPECT_NEAR(matrix::wrap_pi(a_to_b.heading - b_to_a.heading - M_PI_F), 0.f, kTolerance);
	EXPECT_NEAR(a_to_b.length, b_to_a.length, kTolerance);
	EXPECT_NEAR(a_to_b.down, -b_to_a.down, kTolerance);
}

TEST(GnssHeadingBaseline, BaselineDownIsTowardsTheGround)
{
	Baseline baseline{};

	// the antenna the baseline points to is 0.2 m higher
	ASSERT_TRUE(measureBaseline(rtkFix(0.f, 0.f), rtkFix(0.f, 1.f, -0.2f), baseline));
	EXPECT_NEAR(baseline.down, -0.2f, kTolerance);
	EXPECT_NEAR(baseline.length, sqrtf(1.04f), kTolerance);
	EXPECT_NEAR(baseline.heading, M_PI_F / 2.f, kTolerance);
}

TEST(GnssHeadingBaseline, NeedsRtkFixedOnBothReceivers)
{
	Baseline baseline{};
	sensor_gnss_s from = rtkFix(0.f, 0.f);
	sensor_gnss_s to = rtkFix(1.f, 0.f);

	from.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FLOAT;
	EXPECT_FALSE(measureBaseline(from, to, baseline));
	EXPECT_FALSE(measureBaseline(to, from, baseline));

	from.fix_type = sensor_gnss_s::FIX_TYPE_3D;
	EXPECT_FALSE(measureBaseline(from, to, baseline));

	// a receiver that never published is zeroed
	EXPECT_FALSE(measureBaseline(sensor_gnss_s{}, to, baseline));
}

TEST(GnssHeadingBaseline, NeedsTheGnssTime)
{
	Baseline baseline{};
	EXPECT_FALSE(measureBaseline(rtkFix(0.f, 0.f, 0.f, 0), rtkFix(1.f, 0.f), baseline));
	EXPECT_FALSE(measureBaseline(rtkFix(0.f, 0.f), rtkFix(1.f, 0.f, 0.f, 0), baseline));
}

TEST(GnssHeadingBaseline, RejectsFixesOfDifferentEpochs)
{
	Baseline baseline{};
	const sensor_gnss_s from = rtkFix(0.f, 0.f);

	EXPECT_TRUE(measureBaseline(from, later(rtkFix(1.f, 0.f), kMaxEpochMismatch, 0.f, 0.f, 0.f), baseline));
	EXPECT_FALSE(measureBaseline(from, later(rtkFix(1.f, 0.f), kMaxEpochMismatch + 1, 0.f, 0.f, 0.f), baseline));
	EXPECT_FALSE(measureBaseline(later(from, kMaxEpochMismatch + 1, 0.f, 0.f, 0.f), rtkFix(1.f, 0.f), baseline));

	// a receiver's previous fix, 200 ms (10 Hz) old
	EXPECT_FALSE(measureBaseline(from, later(rtkFix(1.f, 0.f), 200000, 0.f, 0.f, 0.f), baseline));
}

TEST(GnssHeadingBaseline, RewindsTheNewerFixToTheOlderEpoch)
{
	// vehicle flying north at 15 m/s and climbing at 2 m/s, the receivers 1 m apart east-west and 10 ms apart: unless
	// rewound, the newer fix is 0.15 m ahead, an 8.5 deg heading error
	static constexpr uint64_t dt_us = 10000;
	Baseline baseline{};

	const sensor_gnss_s from = rtkFix(0.f, 0.f);
	const sensor_gnss_s to_newer = later(rtkFix(0.f, 1.f), dt_us, 15.f, 0.f, -2.f);
	ASSERT_TRUE(measureBaseline(from, to_newer, baseline));
	EXPECT_NEAR(baseline.heading, M_PI_F / 2.f, 1e-3f);
	EXPECT_NEAR(baseline.length, 1.f, 1e-3f);
	EXPECT_NEAR(baseline.down, 0.f, 1e-3f);
	EXPECT_EQ(baseline.time_utc_usec, kEpoch);

	const sensor_gnss_s from_newer = later(rtkFix(0.f, 0.f), dt_us, 15.f, 0.f, -2.f);
	ASSERT_TRUE(measureBaseline(from_newer, rtkFix(0.f, 1.f), baseline));
	EXPECT_NEAR(baseline.heading, M_PI_F / 2.f, 1e-3f);
	EXPECT_NEAR(baseline.length, 1.f, 1e-3f);
	EXPECT_NEAR(baseline.down, 0.f, 1e-3f);
	EXPECT_EQ(baseline.time_utc_usec, kEpoch);
}

TEST(GnssHeadingBaseline, RewindingNeedsTheNewerFixVelocity)
{
	Baseline baseline{};
	sensor_gnss_s to = later(rtkFix(0.f, 1.f), 10000, 1.f, 0.f, 0.f);
	to.vel_ned_valid = false;
	EXPECT_FALSE(measureBaseline(rtkFix(0.f, 0.f), to, baseline));

	// the older fix's velocity isn't used
	sensor_gnss_s from = rtkFix(0.f, 0.f);
	from.vel_ned_valid = false;
	to.vel_ned_valid = true;
	EXPECT_TRUE(measureBaseline(from, to, baseline));

	// fixes of the same epoch need no velocity
	sensor_gnss_s to_same_epoch = rtkFix(0.f, 1.f);
	to_same_epoch.vel_ned_valid = false;
	EXPECT_TRUE(measureBaseline(from, to_same_epoch, baseline));
}

TEST(GnssHeadingBaseline, RejectsCoincidentAntennas)
{
	Baseline baseline{};
	EXPECT_FALSE(measureBaseline(rtkFix(0.f, 0.f), rtkFix(0.f, 0.f), baseline));

	// vertical baseline: no horizontal projection to take the bearing of
	EXPECT_FALSE(measureBaseline(rtkFix(0.f, 0.f), rtkFix(0.f, 0.f, -0.5f), baseline));
}

TEST(GnssHeadingBaseline, NeedsTheHorizontalAccuracy)
{
	Baseline baseline{};
	sensor_gnss_s from = rtkFix(0.f, 0.f);
	from.eph = NAN;
	EXPECT_FALSE(measureBaseline(from, rtkFix(1.f, 0.f), baseline));
	EXPECT_FALSE(measureBaseline(rtkFix(1.f, 0.f), from, baseline));
}

TEST(GnssHeadingBaseline, HeadingAccuracyFromBothReceivers)
{
	Baseline baseline{};
	sensor_gnss_s from = rtkFix(0.f, 0.f);
	sensor_gnss_s to = rtkFix(0.f, 2.f);
	from.eph = 0.03f;
	to.eph = 0.04f;

	// 0.05 m across a 2 m baseline
	ASSERT_TRUE(measureBaseline(from, to, baseline));
	EXPECT_NEAR(baseline.heading_accuracy, atan2f(0.05f, 2.f), 1e-5f);

	// a longer baseline is more accurate
	Baseline longer{};
	sensor_gnss_s to_longer = rtkFix(0.f, 4.f);
	to_longer.eph = 0.04f;
	ASSERT_TRUE(measureBaseline(from, to_longer, longer));
	EXPECT_LT(longer.heading_accuracy, baseline.heading_accuracy);
}

TEST(GnssHeadingBaseline, HeadingAccuracyIncludesTheRewind)
{
	Baseline baseline{};
	const sensor_gnss_s from = rtkFix(0.f, 0.f);
	sensor_gnss_s to = later(rtkFix(0.f, 1.f), 20000, 0.f, 0.f, 0.f);
	to.eph = 0.f;
	to.speed_accuracy = 1.5f;

	sensor_gnss_s from_exact = from;
	from_exact.eph = 0.f;

	// 1.5 m/s over 20 ms is 0.03 m
	ASSERT_TRUE(measureBaseline(from_exact, to, baseline));
	EXPECT_NEAR(baseline.heading_accuracy, atan2f(0.03f, 1.f), 1e-5f);

	// unknown speed accuracy adds nothing
	to.speed_accuracy = NAN;
	ASSERT_TRUE(measureBaseline(from_exact, to, baseline));
	EXPECT_NEAR(baseline.heading_accuracy, 0.f, 1e-5f);
}

TEST(GnssHeadingBaseline, MeasuredBaselineMatchesTheConfiguredOne)
{
	// receivers 0.5 m either side of the centre line, the vehicle facing north east
	const Vector3f configured = configuredBaseline(static_cast<int32_t>(BaselineType::TwoReceivers),
				    Vector3f(0.f, 0.5f, 0.f), Vector3f(0.f, -0.5f, 0.f), Vector3f());
	const float heading_offset = atan2f(configured(1), configured(0));
	const float yaw = M_PI_F / 4.f;

	// this slot's antenna is on the right, which faces south east
	const float right_north = cosf(yaw + M_PI_F / 2.f) * 0.5f;
	const float right_east = sinf(yaw + M_PI_F / 2.f) * 0.5f;

	Baseline baseline{};
	ASSERT_TRUE(measureBaseline(rtkFix(-right_north, -right_east), rtkFix(right_north, right_east), baseline));
	EXPECT_TRUE(baselineConsistent(configured.norm(), baseline.length, baseline.down));
	EXPECT_NEAR(matrix::wrap_pi(baseline.heading - heading_offset), yaw, kTolerance);

	// a float solution 0.3 m off doesn't match
	ASSERT_TRUE(measureBaseline(rtkFix(-right_north, -right_east), rtkFix(right_north + 0.3f, right_east), baseline));
	EXPECT_FALSE(baselineConsistent(configured.norm(), baseline.length, baseline.down));
}
