/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include "GroundImpactDetector.hpp"

using namespace land_detector;

class GroundImpactDetectorTest : public ::testing::Test
{
public:
	bool update(float load_factor, bool in_air = true, bool speed_indicates_flight = false)
	{
		_now += kUpdateInterval;
		return _detector.update(_now, load_factor * CONSTANTS_ONE_G, in_air, speed_indicates_flight);
	}

	bool updateFor(hrt_abstime duration, float load_factor, bool in_air = true, bool speed_indicates_flight = false)
	{
		bool ground_contact = false;

		for (hrt_abstime t = 0; t < duration; t += kUpdateInterval) {
			ground_contact = update(load_factor, in_air, speed_indicates_flight);
		}

		return ground_contact;
	}

	void cruise()
	{
		// level flight for longer than the arm time
		ASSERT_FALSE(updateFor(10'000'000, 1.f, true, true));
	}

	static constexpr hrt_abstime kUpdateInterval = 50'000; // slowest rate the land detector runs at

	GroundImpactDetector _detector;
	hrt_abstime _now{1'000'000};
};

TEST_F(GroundImpactDetectorTest, noGroundContactInLevelFlight)
{
	cruise();
}

TEST_F(GroundImpactDetectorTest, impactHeldUntilLanded)
{
	// GIVEN: cruise flight
	cruise();

	// WHEN: there is an impact of a single sample and the vehicle then lies still
	EXPECT_TRUE(update(6.f));

	// THEN: ground contact is held while the vehicle is not landed
	for (hrt_abstime t = 0; t < GroundImpactDetector::kTimeout - 2 * kUpdateInterval; t += kUpdateInterval) {
		EXPECT_TRUE(update(1.f));
	}

	// AND: it ends once the vehicle is landed
	EXPECT_FALSE(update(1.f, false));
}

TEST_F(GroundImpactDetectorTest, impactTimesOutWithoutLanding)
{
	// GIVEN: an impact after cruise flight
	cruise();
	EXPECT_TRUE(update(6.f));

	// WHEN: the vehicle is never detected as landed
	EXPECT_TRUE(updateFor(GroundImpactDetector::kTimeout - 2 * kUpdateInterval, 1.f));

	// THEN: ground contact ends after the timeout
	EXPECT_FALSE(updateFor(4 * kUpdateInterval, 1.f));
}

TEST_F(GroundImpactDetectorTest, fastPullUpIsNoImpact)
{
	// GIVEN: cruise flight
	cruise();

	// WHEN: the load factor rises to 5 g within 0.3 s
	for (int i = 1; i <= 6; i++) {
		// THEN: no impact is detected
		EXPECT_FALSE(update(1.f + 4.f * i / 6.f, true, true));
	}

	EXPECT_FALSE(updateFor(1'000'000, 5.f, true, true));
}

TEST_F(GroundImpactDetectorTest, jumpInFlightIsRejectedBySpeed)
{
	// GIVEN: cruise flight
	cruise();

	// WHEN: the specific force jumps like in an impact but the vehicle keeps flying
	EXPECT_TRUE(update(6.f, true, true));

	// THEN: it is rejected after the confirmation time because the speed still indicates flight
	EXPECT_TRUE(updateFor(GroundImpactDetector::kConfirmTime - 2 * kUpdateInterval, 1.f, true, true));
	EXPECT_FALSE(updateFor(4 * kUpdateInterval, 1.f, true, true));
}

TEST_F(GroundImpactDetectorTest, impactKeptIfSpeedDropsBeforeConfirmation)
{
	// GIVEN: an impact after cruise flight
	cruise();
	EXPECT_TRUE(update(6.f, true, true));

	// WHEN: the vehicle is still fast for a moment and then comes to a stop
	EXPECT_TRUE(updateFor(1'000'000, 1.f, true, true));

	// THEN: ground contact is kept beyond the confirmation time
	EXPECT_TRUE(updateFor(GroundImpactDetector::kConfirmTime - 500'000, 1.f, true, false));
}

TEST_F(GroundImpactDetectorTest, slowLoadFactorRampIsNoImpact)
{
	// GIVEN: cruise flight
	cruise();

	// WHEN: the load factor ramps up to 5 g over 2 s and stays there
	for (int i = 1; i <= 40; i++) {
		EXPECT_FALSE(update(1.f + 0.1f * i, true, true));
	}

	// THEN: no impact is detected
	EXPECT_FALSE(updateFor(2'000'000, 5.f, true, true));
}

TEST_F(GroundImpactDetectorTest, launchAccelerationIsNoImpact)
{
	// GIVEN: the vehicle just left the ground
	EXPECT_FALSE(updateFor(1'000'000, 1.f, false));
	EXPECT_FALSE(update(1.f));

	// WHEN: there is a large acceleration before the arm time has passed
	EXPECT_FALSE(update(10.f));

	// THEN: no impact is detected
	EXPECT_FALSE(updateFor(GroundImpactDetector::kArmTime - 1'000'000, 1.f));
}

TEST_F(GroundImpactDetectorTest, secondImpactRestartsTimeout)
{
	// GIVEN: an impact after cruise flight
	cruise();
	EXPECT_TRUE(update(6.f));

	// WHEN: the vehicle bounces and hits the ground again shortly before the timeout
	EXPECT_TRUE(updateFor(GroundImpactDetector::kTimeout - 1'000'000, 1.f));
	EXPECT_TRUE(update(6.f));

	// THEN: ground contact is held beyond the timeout of the first impact
	EXPECT_TRUE(updateFor(2'000'000, 1.f));
}
