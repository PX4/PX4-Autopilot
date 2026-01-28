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
 * Run exclusively this test with the command "make tests TESTFILTER=HeadingSmoothing"
 */

#include "mathlib/math/Limits.hpp"
#include <gtest/gtest.h>

#include <motion_planning/HeadingSmoothing.hpp>

using namespace matrix;

class HeadingSmoothingTest : public ::testing::Test
{
public:
	HeadingSmoothingTest()
	{
		_smoothing.setMaxHeadingRate(15.f);
		_smoothing.setMaxHeadingAccel(1.f);
	}

	HeadingSmoothing _smoothing;
	float _dt{.1f};
};

TEST_F(HeadingSmoothingTest, convergence)
{
	const float heading_current = math::radians(0.f);
	const float heading_target = math::radians(5.f);
	_smoothing.reset(heading_current, 0.f);

	const int nb_steps = ceilf(1.f / _dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, _dt);
	}

	const float heading = _smoothing.getSmoothedHeading();
	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading, heading_target) << "heading (deg): " << math::degrees(heading);
	EXPECT_EQ(heading_rate, 0.f);
}

TEST_F(HeadingSmoothingTest, zero_crossing)
{
	const float heading_current = math::radians(-95.f);
	const float heading_target = math::radians(5.f);
	_smoothing.reset(heading_current, 0.f);

	const int nb_steps = ceilf(4.f / _dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, _dt);
	}

	const float heading = _smoothing.getSmoothedHeading();
	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading, heading_target) << "heading (deg): " << math::degrees(heading);
	EXPECT_EQ(heading_rate, 0.f);
}

TEST_F(HeadingSmoothingTest, wrap_pi)
{
	const float heading_current = math::radians(-170.f);
	const float heading_target = math::radians(170.f);
	_smoothing.reset(heading_current, 0.f);

	const int nb_steps = ceilf(2.f / _dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, _dt);
	}

	const float heading = _smoothing.getSmoothedHeading();
	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading, heading_target) << "heading (deg): " << math::degrees(heading);
	EXPECT_EQ(heading_rate, 0.f);
}

TEST_F(HeadingSmoothingTest, positive_rate)
{
	const float max_heading_rate = .15f;
	_smoothing.setMaxHeadingRate(max_heading_rate);
	const float heading_current = math::radians(-170.f);
	const float heading_target = math::radians(-20.f);
	_smoothing.reset(heading_current, 0.f);

	const int nb_steps = ceilf(2.f / _dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, _dt);
	}

	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading_rate, max_heading_rate);
}

TEST_F(HeadingSmoothingTest, negative_rate)
{
	const float max_heading_rate = 0.15;
	_smoothing.setMaxHeadingRate(max_heading_rate);
	const float heading_current = math::radians(-20.f);
	const float heading_target = math::radians(-140.f);
	_smoothing.reset(heading_current, 0.f);

	const int nb_steps = ceilf(2.f / _dt);

	for (int i = 0; i < nb_steps; i++) {
		_smoothing.update(heading_target, _dt);
	}

	const float heading_rate = _smoothing.getSmoothedHeadingRate();
	EXPECT_EQ(heading_rate, -max_heading_rate);
}
