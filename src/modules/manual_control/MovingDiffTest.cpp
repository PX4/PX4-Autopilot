/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "MovingDiff.hpp"
#include <gtest/gtest.h>

// Sustained movement in one direction: filter converges to true velocity
// and consecutiveSameSign accumulates past the override threshold (3).
TEST(MovingDiffTest, ConsistentMovement)
{
	MovingDiff diff;
	diff.update(0.f, 1.f); // first sample, no diff yet
	EXPECT_NEAR(diff.update(1.f, 1.f), 1.f, .1f); // diff=+1, ~0.952
	EXPECT_NEAR(diff.update(2.f, 1.f), 1.f, .01f); // ~0.998
	EXPECT_NEAR(diff.update(3.f, 1.f), 1.f, .001f); // ~0.9999
	EXPECT_NEAR(diff.update(4.f, 1.f), 1.f, .001f);
	EXPECT_NEAR(diff.update(5.f, 1.f), 1.f, .001f);
	EXPECT_FLOAT_EQ(diff.update(6.f, 1.f), 1.f); // fully settled
	EXPECT_GE(diff.consecutiveSameSign(), 6);
}

// Erratic same-sign signal at realistic RC rate (50 Hz): every 3rd sample
// spikes to +3/s, the rest are near-zero (+0.05/s), all positive. Raw diff
// peaks above the override threshold (1/s) yet the filter never crosses it,
// which shows the magnitude check adds necessary protection over sign alone.
TEST(MovingDiffTest, ErraticSameSignFilterStaysBelowThreshold)
{
	MovingDiff diff;
	const float dt = 0.02f; // 50 Hz
	diff.update(0.f, dt); // first sample

	float val = 0.f;

	for (int i = 0; i < 100; i++) {
		val += (i % 3 == 0) ? 0.04f : 0.001f; // diffs sometimes exceed with +2/s but are usually +0.05/s
		EXPECT_LT(diff.update(val, dt), 1.f); // filter never reaches override threshold
	}

	EXPECT_GT(diff.consecutiveSameSign(), 3); // consistently same sign throughout
}

// A single-sample outlier (spike up, immediately returns) must not build up
// consecutiveSameSign past 1, so it cannot trigger the override gate (threshold = 3).
TEST(MovingDiffTest, SingleSampleOutlier)
{
	MovingDiff diff;
	diff.update(0.f, 1.f); // first sample, no diff yet
	EXPECT_EQ(diff.consecutiveSameSign(), 0);

	diff.update(0.f, 1.f); // diff=0, sign=0, no update
	EXPECT_EQ(diff.consecutiveSameSign(), 0);

	diff.update(1.f, 1.f); // outlier spike: diff=+1, consecutive=1
	EXPECT_EQ(diff.consecutiveSameSign(), 1);

	diff.update(0.f, 1.f); // returns: diff=-1, opposite sign resets consecutive to 1
	EXPECT_EQ(diff.consecutiveSameSign(), 1);

	diff.update(0.f, 1.f); // diff=0, no update
	EXPECT_EQ(diff.consecutiveSameSign(), 1);
}

// Switching RC source to a joystick held at a different position produces a single large
// diff followed by zero-diffs (static hold). consecutiveSameSign must stay at 1 so the
// step does not trigger the override gate (threshold = 3).
TEST(MovingDiffTest, SignalStepOnSourceSwitch)
{
	MovingDiff diff;
	diff.update(0.2f, 1.f); // first sample, no diff yet
	EXPECT_EQ(diff.consecutiveSameSign(), 0);

	diff.update(0.2f, 1.f); // diff=0
	EXPECT_EQ(diff.consecutiveSameSign(), 0);

	diff.update(0.2f, 1.f); // diff=0
	EXPECT_EQ(diff.consecutiveSameSign(), 0);

	diff.update(-0.8f, 1.f); // source switch: single large step diff=-1.0, consecutive=1
	EXPECT_EQ(diff.consecutiveSameSign(), 1);

	diff.update(-0.8f, 1.f); // joystick static: diff=0, no sign update
	EXPECT_EQ(diff.consecutiveSameSign(), 1);

	diff.update(-0.8f, 1.f); // diff=0
	EXPECT_EQ(diff.consecutiveSameSign(), 1);

	diff.update(-0.8f, 1.f); // diff=0
	EXPECT_EQ(diff.consecutiveSameSign(), 1);
}
