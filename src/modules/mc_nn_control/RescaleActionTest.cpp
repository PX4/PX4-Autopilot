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
 * @file RescaleActionTest.cpp
 * Unit tests for the network action to motor command mapping. They assert the
 * properties the mapping has to keep rather than any particular output value,
 * and derive every threshold from the mapping's own achievable range, so a
 * change to the defaults only fails them when a property is broken.
 *
 * to run, on a config that enables the module:
 *   cmake -DCMAKE_TESTING=ON build/px4_sitl_neural
 *   ninja -C build/px4_sitl_neural unit-RescaleAction
 *   ctest --test-dir build/px4_sitl_neural -R RescaleAction
 */

#include <gtest/gtest.h>
#include <cmath>

#include "actions_rescale.hpp"

using nn_control::rescale_action;
using nn_control::achievable_action_range;
using nn_control::valid_motor_limits;

struct Params {
	float thrust_coeff;
	float min_rpm;
	float max_rpm;
};

// The shipped defaults from mc_nn_control_params.yaml, a smaller motor whose
// minimum rpm is above a ninth of its maximum, and a motor whose limits cover
// the whole action range. The properties below hold for all of them.
static constexpr Params kDefaults{1.2f, 1000.f, 22000.f};
static constexpr Params kSmallMotor{3.0f, 2000.f, 12000.f};
static constexpr Params kMatchedMotor{1.2f, 0.f, 24495.f};
static constexpr Params kParamSets[] = {kDefaults, kSmallMotor, kMatchedMotor};

static float map(float action, const Params &p)
{
	return rescale_action(action, p.thrust_coeff, p.min_rpm, p.max_rpm);
}

static void range(const Params &p, float &lowest, float &highest)
{
	achievable_action_range(p.thrust_coeff, p.min_rpm, p.max_rpm, lowest, highest);
}

TEST(RescaleActionTest, actionsBeyondOneClampToTheBoundary)
{
	for (const Params &p : kParamSets) {
		EXPECT_FLOAT_EQ(map(-5.f, p), map(-1.f, p));
		EXPECT_FLOAT_EQ(map(5.f, p), map(1.f, p));
	}
}

TEST(RescaleActionTest, commandsStayWithinZeroAndOne)
{
	for (const Params &p : kParamSets) {
		for (float action = -1.f; action <= 1.f; action += 0.01f) {
			const float cmd = map(action, p);
			EXPECT_GE(cmd, 0.f) << "below zero at action " << action;
			EXPECT_LE(cmd, 1.f) << "above one at action " << action;
		}
	}
}

TEST(RescaleActionTest, lowestActionIdlesTheMotorAndHighestRunsItAtFullScale)
{
	for (const Params &p : kParamSets) {
		EXPECT_FLOAT_EQ(map(-1.f, p), 0.f);
		// a motor whose maximum rpm is only just reached at action one lands a
		// rounding step short of it
		EXPECT_NEAR(map(1.f, p), 1.f, 1e-4f);
	}
}

TEST(RescaleActionTest, mappingDoesNotDecrease)
{
	for (const Params &p : kParamSets) {
		float prev = map(-1.f, p);

		for (float action = -0.99f; action <= 1.f; action += 0.01f) {
			const float cmd = map(action, p);
			EXPECT_GE(cmd, prev) << "decreases at action " << action;
			prev = cmd;
		}
	}
}

TEST(RescaleActionTest, mappingRisesInsideTheAchievableRange)
{
	for (const Params &p : kParamSets) {
		float lowest, highest;
		range(p, lowest, highest);
		const float start = fmaxf(lowest, -1.f);
		const float end = fminf(highest, 1.f);
		float prev = map(start, p);

		for (float action = start + 0.01f; action < end; action += 0.01f) {
			const float cmd = map(action, p);
			EXPECT_GT(cmd, prev) << "flat at action " << action;
			prev = cmd;
		}
	}
}

TEST(RescaleActionTest, actionsOutsideTheAchievableRangeSitOnTheBounds)
{
	for (const Params &p : kParamSets) {
		float lowest, highest;
		range(p, lowest, highest);

		if (lowest > -1.f) {
			EXPECT_FLOAT_EQ(map(lowest - 0.001f, p), 0.f);
		}

		if (highest < 1.f) {
			EXPECT_FLOAT_EQ(map(highest + 0.001f, p), 1.f);
			EXPECT_LT(map(highest - 0.01f, p), 1.f);
		}
	}
}

TEST(RescaleActionTest, achievableRangeFollowsTheLimits)
{
	float lowest, highest;
	range(kMatchedMotor, lowest, highest);
	EXPECT_NEAR(lowest, -1.f, 1e-3f);
	EXPECT_NEAR(highest, 1.f, 1e-3f);

	range(kDefaults, lowest, highest);
	EXPECT_GT(lowest, -1.f);
	EXPECT_LT(highest, 1.f);
}

TEST(RescaleActionTest, outputsRemainFiniteForNormalInputs)
{
	for (const Params &p : kParamSets) {
		for (float action = -1.f; action <= 1.f; action += 0.01f) {
			EXPECT_TRUE(std::isfinite(map(action, p))) << "not finite at action " << action;
		}
	}
}

// Every accepted set of limits has to give a real command somewhere inside the
// action range, and every rejected set has to give none. The intermediate command
// check does not go through the validator, so it is an independent oracle for it.
static void check_limits(float thrust_coeff, float min_rpm, float max_rpm, int &valid_sets, int &rejected_sets)
{
	const bool valid = valid_motor_limits(thrust_coeff, min_rpm, max_rpm);
	valid ? valid_sets++ : rejected_sets++;

	for (float action = -1.f; action <= 1.f; action += 0.25f) {
		EXPECT_EQ(std::isfinite(rescale_action(action, thrust_coeff, min_rpm, max_rpm)), valid)
				<< "thrust_coeff " << thrust_coeff << " min " << min_rpm << " max " << max_rpm << " action " << action;
	}

	if (valid) {
		float lowest, highest;
		achievable_action_range(thrust_coeff, min_rpm, max_rpm, lowest, highest);
		const float middle = (fmaxf(lowest, -1.f) + fminf(highest, 1.f)) / 2.f;
		const float cmd = rescale_action(middle, thrust_coeff, min_rpm, max_rpm);
		EXPECT_GT(cmd, 0.f) << "thrust_coeff " << thrust_coeff << " min " << min_rpm << " max " << max_rpm;
		EXPECT_LT(cmd, 1.f) << "thrust_coeff " << thrust_coeff << " min " << min_rpm << " max " << max_rpm;
	}
}

TEST(RescaleActionTest, acceptedLimitsCommandSomethingBetweenIdleAndFullScale)
{
	int valid_sets = 0;
	int rejected_sets = 0;

	// the limits from mc_nn_control_params.yaml, with the rpm pair kept apart
	for (float thrust_coeff = 0.01f; thrust_coeff <= 5.f; thrust_coeff += 0.05f) {
		for (float min_rpm = 0.f; min_rpm < 80000.f; min_rpm += 8000.f) {
			for (float max_rpm = min_rpm + 500.f; max_rpm <= 80000.f; max_rpm += 8000.f) {
				check_limits(thrust_coeff, min_rpm, max_rpm, valid_sets, rejected_sets);
			}
		}
	}

	// narrow windows near the bottom of the range, where float rounding once let a
	// set through whose only outputs were the two clamps, up to bands wide enough
	// to be accepted again
	for (float max_rpm = 1.f; max_rpm <= 4000.f; max_rpm += 1.f) {
		check_limits(0.01f, 0.f, max_rpm, valid_sets, rejected_sets);
	}

	EXPECT_GT(valid_sets, 0);
	EXPECT_GT(rejected_sets, 0);
}

TEST(RescaleActionTest, nonFiniteActionGivesNoCommand)
{
	// the caller maps a non finite command to a stopped motor, so the mapping
	// must not turn a bad action into a number
	for (const Params &p : kParamSets) {
		EXPECT_FALSE(std::isfinite(map(NAN, p)));
	}
}

TEST(RescaleActionTest, invalidLimitsAreRejected)
{
	// ordering and sign
	EXPECT_FALSE(valid_motor_limits(1.2f, 5000.f, 5000.f));
	EXPECT_FALSE(valid_motor_limits(1.2f, 6000.f, 5000.f));
	EXPECT_FALSE(valid_motor_limits(0.f, 1000.f, 22000.f));
	EXPECT_FALSE(valid_motor_limits(-1.2f, 1000.f, 22000.f));
	EXPECT_FALSE(valid_motor_limits(1.2f, -1.f, 22000.f));

	// raw parameter writes are not bounded by the metadata, so non finite values
	// have to be caught here. An infinite coefficient would map every action to
	// idle while looking valid.
	EXPECT_FALSE(valid_motor_limits(INFINITY, 1000.f, 22000.f));
	EXPECT_FALSE(valid_motor_limits(NAN, 1000.f, 22000.f));
	EXPECT_FALSE(valid_motor_limits(1.2f, NAN, 22000.f));
	EXPECT_FALSE(valid_motor_limits(1.2f, 1000.f, INFINITY));

	// limits the action range cannot reach: idle already above action one, or
	// full scale still below action minus one, or a window too narrow to hold a
	// representable action
	EXPECT_FALSE(valid_motor_limits(5.f, 79999.f, 80000.f));
	EXPECT_FALSE(valid_motor_limits(0.01f, 0.f, 1.f));
	EXPECT_FALSE(valid_motor_limits(0.01f, 0.f, 33.f));

	// values that overflow the achievable range would otherwise clip to a full span
	EXPECT_FALSE(valid_motor_limits(1e38f, 0.f, 1e38f));

	// a narrow band is still a band. This motor answers to five percent of the
	// action range and has to be accepted, the range warning is what tells the
	// user about it.
	EXPECT_TRUE(valid_motor_limits(1.2f, 1000.f, 3999.f));

	for (const Params &p : kParamSets) {
		EXPECT_TRUE(valid_motor_limits(p.thrust_coeff, p.min_rpm, p.max_rpm));
	}

	EXPECT_FALSE(std::isfinite(rescale_action(0.f, 1.2f, 5000.f, 5000.f)));
	EXPECT_FALSE(std::isfinite(rescale_action(0.f, INFINITY, 1000.f, 22000.f)));
	EXPECT_FALSE(std::isfinite(rescale_action(0.f, 5.f, 79999.f, 80000.f)));
}
