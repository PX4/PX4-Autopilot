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

TEST(GnssHeadingBaseline, MovingBaseRunsFromTheOtherReceiver)
{
	// rover (this slot) 0.17 m right of the centre line, moving base (other slot) 0.17 m left
	const Vector3f baseline = configuredBaseline(static_cast<int32_t>(BaselineType::OtherReceiver),
				  Vector3f(0.f, 0.17f, 0.f), Vector3f(0.f, -0.17f, 0.f), Vector3f(1.f, 0.f, 0.f));

	// the rover is on the right: a measured heading of 90 deg means the vehicle faces north
	EXPECT_FLOAT_EQ(atan2f(baseline(1), baseline(0)), M_PI_F / 2.f);
	EXPECT_FLOAT_EQ(baseline.norm(), 0.34f);

	EXPECT_EQ(configuredBaseline(static_cast<int32_t>(BaselineType::Custom), Vector3f(), Vector3f(),
				     Vector3f(0.3f, 0.f, 0.f)), Vector3f(0.3f, 0.f, 0.f));
	EXPECT_EQ(configuredBaseline(static_cast<int32_t>(BaselineType::Disabled), Vector3f(0.f, 0.17f, 0.f),
				     Vector3f(0.f, -0.17f, 0.f), Vector3f(0.3f, 0.f, 0.f)), Vector3f());
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
