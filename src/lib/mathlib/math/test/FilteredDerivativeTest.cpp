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
#include <mathlib/math/filter/FilteredDerivative.hpp>

TEST(FilteredDerivativeTest, firstSampleDoesNotUpdate)
{
	FilteredDerivative<float> fd;
	fd.setParameters(0.01f, 0.05f);
	fd.reset(0.f);

	// First update only arms the filter (no derivative yet)
	EXPECT_FLOAT_EQ(fd.update(1.f), 0.f);
}

TEST(FilteredDerivativeTest, constantSignalYieldsZeroDerivative)
{
	FilteredDerivative<float> fd;
	fd.setParameters(0.01f, 0.05f);
	fd.reset(0.f);

	// First sample initializes
	(void)fd.update(5.f);

	// Constant input -> raw derivative 0; filtered state stays ~0
	for (int i = 0; i < 50; i++) {
		EXPECT_NEAR(fd.update(5.f), 0.f, 1e-5f);
	}
}

TEST(FilteredDerivativeTest, rampApproachesSlope)
{
	FilteredDerivative<float> fd;
	const float dt = 0.01f;
	fd.setParameters(dt, 0.02f);
	fd.reset(0.f);

	const float slope = 3.f;
	float x = 0.f;
	(void)fd.update(x);

	for (int i = 0; i < 200; i++) {
		x += slope * dt;
		(void)fd.update(x);
	}

	// After sufficient samples the filtered derivative should approach slope
	EXPECT_NEAR(fd.getState(), slope, 0.15f);
}
