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
 * @file FrontTransitionThrustCheckTest.cpp
 *
 * Front transitions sampled at 200 Hz. The thrust share is 1 at the transition throttle, and grows with the throttle
 * squared while the throttle ramps up.
 */

#include <gtest/gtest.h>
#include "FrontTransitionThrustCheck.hpp"

#include <cmath>
#include <functional>

using namespace time_literals;

namespace
{
constexpr hrt_abstime kSampleInterval = 5_ms;
constexpr float kMinAcceleration = 0.8f; // [m/s^2] VT_EF_FT_ACC default

struct Sample {
	float thrust_share;
	float forward_acceleration; ///< [m/s^2]
};

// Flies a front transition for duration [s], with the sample at each time [s] since its start.
// Returns the time of the failure, or NAN.
float failureTime(FrontTransitionThrustCheck &check, float duration, const std::function<Sample(float)> &sample_at,
		  float min_acceleration = kMinAcceleration)
{
	const hrt_abstime start = 10_s;

	for (hrt_abstime elapsed = 0; elapsed < (hrt_abstime)(duration * 1e6f); elapsed += kSampleInterval) {
		const float time = elapsed * 1e-6f;
		const Sample sample = sample_at(time);

		if (check.update(start + elapsed, sample.thrust_share, sample.forward_acceleration, min_acceleration)) {
			return time;
		}
	}

	return NAN;
}

// The throttle steps to the transition throttle, as with VT_PSHER_SLEW 0
std::function<Sample(float)> atTransitionThrottle(float forward_acceleration)
{
	return [forward_acceleration](float) { return Sample{1.f, forward_acceleration}; };
}

// The throttle ramps up to the transition throttle in ramp_time [s]. A working pusher accelerates with its thrust.
std::function<Sample(float)> throttleRamp(float ramp_time, float acceleration_at_transition_throttle)
{
	return [ = ](float time) {
		const float throttle = fminf(time / ramp_time, 1.f);
		const float thrust_share = throttle * throttle;
		const float forward_acceleration = acceleration_at_transition_throttle * thrust_share;
		return Sample{thrust_share, forward_acceleration};
	};
}
} // namespace

TEST(FrontTransitionThrustCheckTest, NoAccelerationFailsAfterOneSecond)
{
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_NEAR(failureTime(check, 3.f, atTransitionThrottle(0.f)), 1.f, 0.01f);
	EXPECT_FALSE(check.isChecking()); // decided once
}

TEST(FrontTransitionThrustCheckTest, AccelerationAboveTheMinimumPasses)
{
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(1.f))));
	EXPECT_FALSE(check.isChecking());
}

TEST(FrontTransitionThrustCheckTest, MinimumAccelerationIsAnInput)
{
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_FALSE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(1.5f), 2.f)));

	check.start(0.f);
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(2.5f), 2.f)));
}

TEST(FrontTransitionThrustCheckTest, OnlyTheStartOfTheTransitionIsChecked)
{
	// after the decision, the drag decelerates the vehicle
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, [](float time) { return Sample{1.f, time < 1.7f ? 3.f : -2.f}; })));
}

TEST(FrontTransitionThrustCheckTest, SlowSpinUpPasses)
{
	// a motor that needs 0.25 s to spin up after the throttle step, with twice the minimum acceleration
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, [](float time) { return Sample{1.f, time < 0.25f ? 0.f : 1.6f}; })));
}

TEST(FrontTransitionThrustCheckTest, SamplesWithGapsAreNotIntegrated)
{
	// samples 150 ms apart are not consecutive: nothing is integrated, so there is no decision
	FrontTransitionThrustCheck check;
	check.start(0.f);

	for (hrt_abstime timestamp = 10_s; timestamp < 13_s; timestamp += 150_ms) {
		EXPECT_FALSE(check.update(timestamp, 1.f, 0.f, kMinAcceleration));
	}

	EXPECT_TRUE(check.isChecking());
}

TEST(FrontTransitionThrustCheckTest, NonFiniteAccelerationIsSkipped)
{
	// no acceleration from 0.5 s on: the check decides 1 s later
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_NEAR(failureTime(check, 3.f, [](float time) { return Sample{1.f, time < 0.5f ? NAN : 0.f}; }), 1.5f, 0.01f);
}

TEST(FrontTransitionThrustCheckTest, ThrustAtTheStartLowersTheRequiredAcceleration)
{
	// pusher assist at 0.49 of the transition throttle: the transition adds 1 - 0.49^2 = 76% of the transition
	// thrust, so it has to accelerate by at least 0.76 * 0.8 = 0.61 m/s^2
	const float start_share = 0.49f * 0.49f;
	FrontTransitionThrustCheck check;
	check.start(start_share);
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(0.7f))));

	check.start(start_share);
	EXPECT_NEAR(failureTime(check, 3.f, atTransitionThrottle(0.5f)), 1.f, 0.01f);
}

TEST(FrontTransitionThrustCheckTest, HalfTheTransitionThrustAtTheStartIsNotChecked)
{
	FrontTransitionThrustCheck check;
	check.start(0.5f);
	EXPECT_FALSE(check.isChecking());
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(-1.f))));

	check.start(0.49f);
	EXPECT_FALSE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(-1.f))));
}

TEST(FrontTransitionThrustCheckTest, WeakPusherFailsWhileTheThrottleRampsUp)
{
	// a 2 s ramp weighs as much as 2 s / 5 = 0.4 s at the transition throttle, so the check decides 0.6 s after it
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_NEAR(failureTime(check, 4.f, throttleRamp(2.f, 0.4f)), 2.6f, 0.01f);
}

TEST(FrontTransitionThrustCheckTest, WorkingPusherPassesWhileTheThrottleRampsUp)
{
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_TRUE(std::isnan(failureTime(check, 4.f, throttleRamp(2.f, 1.6f))));
	EXPECT_FALSE(check.isChecking());
}

TEST(FrontTransitionThrustCheckTest, EarlyAccelerationDoesNotHideALaterFailure)
{
	// more acceleration at low throttle than the thrust gives, then the pusher stops: samples count with the
	// thrust, so the start of the ramp weighs little
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_FALSE(std::isnan(failureTime(check, 4.f, [](float time) {
		const float throttle = fminf(time / 2.f, 1.f);
		return Sample{throttle * throttle, time < 0.5f ? 3.f : 0.f};
	})));
}

TEST(FrontTransitionThrustCheckTest, SkippedTransitionIsNotChecked)
{
	FrontTransitionThrustCheck check;
	check.start(0.f);
	check.skip();
	EXPECT_TRUE(std::isnan(failureTime(check, 3.f, atTransitionThrottle(0.f))));
}

TEST(FrontTransitionThrustCheckTest, EachTransitionIsCheckedAgain)
{
	FrontTransitionThrustCheck check;
	check.start(0.f);
	EXPECT_TRUE(std::isnan(failureTime(check, 2.f, atTransitionThrottle(1.f))));

	check.start(0.f);
	EXPECT_NEAR(failureTime(check, 2.f, atTransitionThrottle(0.f)), 1.f, 0.01f);
}
