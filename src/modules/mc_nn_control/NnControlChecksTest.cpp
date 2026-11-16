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
 * @file NnControlChecksTest.cpp
 * Unit tests for the checks the neural controller runs before it trusts its model,
 * its observations and its output.
 *
 * to run, on a config that enables the module:
 *   cmake -DCMAKE_TESTING=ON build/px4_sitl_neural
 *   ninja -C build/px4_sitl_neural unit-NnControlChecks
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

#include "nn_control_checks.hpp"

using namespace nn_control;

static ObservationSnapshot good(uint64_t now)
{
	ObservationSnapshot o{};
	o.position_timestamp = now - 1000;
	o.xy_valid = o.z_valid = o.v_xy_valid = o.v_z_valid = true;
	o.position[0] = 1.f;
	o.position[1] = -2.f;
	o.position[2] = -3.f;
	o.velocity[0] = 0.1f;
	o.velocity[1] = 0.f;
	o.velocity[2] = -0.2f;
	o.attitude_timestamp = now - 1000;
	o.q[0] = 1.f;
	o.angular_velocity_timestamp = now - 500;
	o.angular_velocity[1] = 0.01f;
	return o;
}

static constexpr uint64_t kNow = 10000000;

TEST(NnControlChecksTest, staleLimitsAreTheRaptorModulesLimits)
{
	// the contract, not a restatement of the constants
	EXPECT_EQ(kMaxLocalPositionAge, 100000u);
	EXPECT_EQ(kMaxAttitudeAge, 50000u);
	EXPECT_EQ(kMaxAngularVelocityAge, 10000u);
	EXPECT_EQ(kAngularVelocityWatchdog, 20000u);
}

TEST(NnControlChecksTest, freshObservationsPass)
{
	EXPECT_EQ(check_observations(good(kNow), kNow), ObservationFault::None);
}

TEST(NnControlChecksTest, everyInvalidPositionFlagIsCaught)
{
	for (int flag = 0; flag < 4; flag++) {
		ObservationSnapshot o = good(kNow);
		(flag == 0 ? o.xy_valid : flag == 1 ? o.z_valid : flag == 2 ? o.v_xy_valid : o.v_z_valid) = false;
		EXPECT_EQ(check_observations(o, kNow), ObservationFault::PositionInvalid) << "flag " << flag;
	}
}

TEST(NnControlChecksTest, staleObservationsAreCaughtAtTheirOwnLimits)
{
	ObservationSnapshot o = good(kNow);
	o.position_timestamp = kNow - kMaxLocalPositionAge;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::None);
	o.position_timestamp = kNow - kMaxLocalPositionAge - 1;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::PositionStale);

	o = good(kNow);
	o.attitude_timestamp = kNow - kMaxAttitudeAge - 1;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::AttitudeStale);

	o = good(kNow);
	o.angular_velocity_timestamp = kNow - kMaxAngularVelocityAge - 1;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::AngularVelocityStale);
}

TEST(NnControlChecksTest, neverSetObservationsAreStale)
{
	ObservationSnapshot o = good(kNow);
	o.position_timestamp = 0;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::PositionStale);

	o = good(kNow);
	o.attitude_timestamp = 0;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::AttitudeStale);
}

TEST(NnControlChecksTest, observationsFromTheFutureAreNotStale)
{
	ObservationSnapshot o = good(kNow);
	o.position_timestamp = kNow + 5000;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::None);
}

TEST(NnControlChecksTest, nonFiniteObservationsAreCaught)
{
	ObservationSnapshot o = good(kNow);
	o.position[2] = NAN;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::PositionNotFinite);

	o = good(kNow);
	o.velocity[0] = INFINITY;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::PositionNotFinite);

	o = good(kNow);
	o.q[3] = NAN;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::AttitudeNotFinite);

	o = good(kNow);
	o.angular_velocity[2] = -INFINITY;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::AngularVelocityNotFinite);
}

TEST(NnControlChecksTest, positionProblemsAreReportedBeforeAttitudeProblems)
{
	ObservationSnapshot o = good(kNow);
	o.xy_valid = false;
	o.attitude_timestamp = 0;
	EXPECT_EQ(check_observations(o, kNow), ObservationFault::PositionInvalid);
}

TEST(NnControlChecksTest, outputsHaveToBeFiniteInEveryChannel)
{
	float out[kOutputSize] = {0.5f, 0.5f, 0.5f, 0.5f};
	EXPECT_TRUE(outputs_finite(out, kOutputSize));

	for (int i = 0; i < kOutputSize; i++) {
		float bad[kOutputSize] = {0.5f, 0.5f, 0.5f, 0.5f};
		bad[i] = (i % 2 == 0) ? NAN : INFINITY;
		EXPECT_FALSE(outputs_finite(bad, kOutputSize)) << "channel " << i;
	}
}

TEST(NnControlChecksTest, theExpectedModelLayoutIsAccepted)
{
	EXPECT_EQ(model_layout_problem(3, 3, 1, kInputSize, true, 1, kOutputSize, true), nullptr);
}

TEST(NnControlChecksTest, everyModelLayoutMismatchIsNamed)
{
	EXPECT_STREQ(model_layout_problem(2, 3, 1, kInputSize, true, 1, kOutputSize, true), "schema version");
	EXPECT_STREQ(model_layout_problem(3, 3, 2, kInputSize, true, 1, kOutputSize, true), "input tensors, expected one");
	EXPECT_STREQ(model_layout_problem(3, 3, 1, kInputSize, true, 2, kOutputSize, true), "output tensors, expected one");
	EXPECT_STREQ(model_layout_problem(3, 3, 1, 12, true, 1, kOutputSize, true), "input size, expected 15 elements");
	EXPECT_STREQ(model_layout_problem(3, 3, 1, kInputSize, false, 1, kOutputSize, true), "input type, expected float32");
	EXPECT_STREQ(model_layout_problem(3, 3, 1, kInputSize, true, 1, 6, true), "output size, expected 4 elements");
	EXPECT_STREQ(model_layout_problem(3, 3, 1, kInputSize, true, 1, kOutputSize, false), "output type, expected float32");
}
