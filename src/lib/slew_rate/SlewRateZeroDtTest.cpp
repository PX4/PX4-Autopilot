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
#include "SlewRate.hpp"

TEST(SlewRateZeroDtTest, ZeroDtHoldsValue)
{
	SlewRate<float> sr;
	sr.setSlewRate(10.f);
	sr.setForcedValue(2.5f);

	EXPECT_FLOAT_EQ(sr.update(100.f, 0.f), 2.5f);
	EXPECT_FLOAT_EQ(sr.getState(), 2.5f);
}

TEST(SlewRateZeroDtTest, ForcedValueBypassesSlew)
{
	SlewRate<float> sr;
	sr.setSlewRate(0.1f);
	sr.setForcedValue(-7.f);
	EXPECT_FLOAT_EQ(sr.getState(), -7.f);

	// one small step toward target
	EXPECT_FLOAT_EQ(sr.update(0.f, 1.f), -6.9f);
}

TEST(SlewRateZeroDtTest, AlreadyAtTargetStays)
{
	SlewRate<float> sr;
	sr.setSlewRate(5.f);
	sr.setForcedValue(1.25f);
	EXPECT_FLOAT_EQ(sr.update(1.25f, 0.5f), 1.25f);
	EXPECT_FLOAT_EQ(sr.update(1.25f, 0.5f), 1.25f);
}
