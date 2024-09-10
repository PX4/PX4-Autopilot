/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * Test code for the Heading Smoothing library
 * Run this test only using make tests TESTFILTER=HeadingSmoothing
 */

#include "mathlib/math/Limits.hpp"
#include <gtest/gtest.h>

#include <motion_planning/HeadingSmoothing.hpp>

using namespace matrix;

class HeadingSmoothingTest : public ::testing::Test
{
public:
	HeadingSmoothing _smoothing;
};

TEST_F(HeadingSmoothingTest, convergence)
{
	const float dt = 0.1f;
	const float heading_corrent = math::radians(0.f);
	const float heading_target = math::radians(5.f);
	_smoothing.reset(heading_corrent, 0.f);
	_smoothing.setMaxHeadingRate(15.f);
	_smoothing.setMaxHeadingAccel(1.f);

	const int nb_steps = ceilf(1.f / dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, dt);
	}

	const float heading = _smoothing.getSmoothedHeading();
	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading, heading_target) << "heading (deg): " << math::degrees(heading);
	EXPECT_EQ(heading_rate, 0.f);
}

TEST_F(HeadingSmoothingTest, zero_crossing)
{
	const float dt = 0.1f;
	const float heading_corrent = math::radians(-95.f);
	const float heading_target = math::radians(5.f);
	_smoothing.reset(heading_corrent, 0.f);
	_smoothing.setMaxHeadingRate(15.f);
	_smoothing.setMaxHeadingAccel(1.f);

	const int nb_steps = ceilf(4.f / dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, dt);
	}

	const float heading = _smoothing.getSmoothedHeading();
	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading, heading_target) << "heading (deg): " << math::degrees(heading);
	EXPECT_EQ(heading_rate, 0.f);
}

TEST_F(HeadingSmoothingTest, wrap_pi)
{
	const float dt = 0.1f;
	const float heading_corrent = math::radians(-170.f);
	const float heading_target = math::radians(170.f);
	_smoothing.reset(heading_corrent, 0.f);
	_smoothing.setMaxHeadingRate(15.f);
	_smoothing.setMaxHeadingAccel(1.f);

	const int nb_steps = ceilf(2.f / dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, dt);
	}

	const float heading = _smoothing.getSmoothedHeading();
	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	printf("heading: %f, rate: %f\n", (double)math::degrees(heading), (double)heading_rate);
	EXPECT_EQ(heading, heading_target) << "heading (deg): " << math::degrees(heading);
	EXPECT_EQ(heading_rate, 0.f);
}

TEST_F(HeadingSmoothingTest, positive_rate)
{
	const float dt = 0.1f;
	const float heading_corrent = math::radians(-170.f);
	const float heading_target = math::radians(-20.f);
	_smoothing.reset(heading_corrent, 0.f);
	const float max_heading_rate = 0.15;
	_smoothing.setMaxHeadingRate(max_heading_rate);
	_smoothing.setMaxHeadingAccel(1.f);

	const int nb_steps = ceilf(2.f / dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, dt);
	}

	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading_rate, max_heading_rate);
}

TEST_F(HeadingSmoothingTest, negative_rate)
{
	const float dt = 0.1f;
	const float heading_corrent = math::radians(-20.f);
	const float heading_target = math::radians(-140.f);
	_smoothing.reset(heading_corrent, 0.f);
	const float max_heading_rate = 0.15;
	_smoothing.setMaxHeadingRate(max_heading_rate);
	_smoothing.setMaxHeadingAccel(1.f);

	const int nb_steps = ceilf(2.f / dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, dt);
	}

	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading_rate, -max_heading_rate);
}
