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

/**
 * @file signal_generator_test.cpp
 * Unit tests for linear/log sine-sweep helpers used by fw autotune sys-id.
 */

#include <gtest/gtest.h>
#include <cmath>

#ifndef M_TWOPI_F
	#define M_TWOPI_F 6.283185307179586f
#endif

#include "signal_generator.hpp"

TEST(SignalGeneratorTest, LinearPastDurationIsZero)
{
	const float y = signal_generator::getLinearSineSweep(1.f, 10.f, 2.f, 2.5f);
	EXPECT_FLOAT_EQ(y, 0.f);
}

TEST(SignalGeneratorTest, LinearAtStartIsZero)
{
	const float y = signal_generator::getLinearSineSweep(1.f, 10.f, 2.f, 0.f);
	EXPECT_NEAR(y, 0.f, 1e-6f);
}

TEST(SignalGeneratorTest, LinearMidSampleBoundedAndFinite)
{
	const float y = signal_generator::getLinearSineSweep(1.f, 10.f, 2.f, 1.f);
	EXPECT_TRUE(std::isfinite(y));
	EXPECT_LE(y, 1.f);
	EXPECT_GE(y, -1.f);
}

TEST(SignalGeneratorTest, LinearConstantFrequencyQuarterPeriod)
{
	// f_start == f_end => pure sine at that frequency; at t = 0.25/f expect ~1
	const float f = 2.f;
	const float t = 0.25f / f;
	const float y = signal_generator::getLinearSineSweep(f, f, 10.f, t);
	EXPECT_NEAR(y, 1.f, 1e-4f);
}

TEST(SignalGeneratorTest, LogPastDurationIsZero)
{
	const float y = signal_generator::getLogSineSweep(1.f, 10.f, 2.f, 3.f);
	EXPECT_FLOAT_EQ(y, 0.f);
}

TEST(SignalGeneratorTest, LogAtStartIsZero)
{
	const float y = signal_generator::getLogSineSweep(1.f, 10.f, 2.f, 0.f);
	EXPECT_NEAR(y, 0.f, 1e-5f);
}

TEST(SignalGeneratorTest, LogMidSampleBoundedAndFinite)
{
	const float y = signal_generator::getLogSineSweep(1.f, 20.f, 4.f, 1.5f);
	EXPECT_TRUE(std::isfinite(y));
	EXPECT_LE(y, 1.f);
	EXPECT_GE(y, -1.f);
}
