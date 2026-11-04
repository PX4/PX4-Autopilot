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

#include "SkidToTurnAllocator.hpp"

static constexpr float kDt = 0.02f;           // [s]
static constexpr float kFastSlewRate = 1e6f;  // [m/s^3] effectively no slew rate limit

TEST(SkidToTurnAllocator, zeroRatioIsBankOnly)
{
	SkidToTurnAllocator allocator;
	allocator.setSlewRate(kFastSlewRate);

	const SkidToTurnAllocator::Output out = allocator.update(4.f, 0.f, 2.f, kDt);
	EXPECT_FLOAT_EQ(out.bank, 4.f);
	EXPECT_FLOAT_EQ(out.skid, 0.f);
}

TEST(SkidToTurnAllocator, ratioSplitsLateralAcceleration)
{
	SkidToTurnAllocator allocator;
	allocator.setSlewRate(kFastSlewRate);

	const SkidToTurnAllocator::Output out = allocator.update(-4.f, 0.25f, 2.f, kDt);
	EXPECT_FLOAT_EQ(out.skid, -1.f);
	EXPECT_FLOAT_EQ(out.bank, -3.f);
}

TEST(SkidToTurnAllocator, bankFillsWhatSkidCannotGenerate)
{
	SkidToTurnAllocator allocator;
	allocator.setSlewRate(kFastSlewRate);

	const SkidToTurnAllocator::Output out = allocator.update(4.f, 1.f, 1.5f, kDt);
	EXPECT_FLOAT_EQ(out.skid, 1.5f);
	EXPECT_FLOAT_EQ(out.bank, 2.5f);
	EXPECT_FLOAT_EQ(out.bank + out.skid, 4.f);
}

TEST(SkidToTurnAllocator, skidIsSlewRateLimitedAndBankCompensates)
{
	static constexpr float kSlewRate = 5.f; // [m/s^3]
	SkidToTurnAllocator allocator;
	allocator.setSlewRate(kSlewRate);

	const SkidToTurnAllocator::Output out = allocator.update(4.f, 1.f, 2.f, kDt);
	EXPECT_FLOAT_EQ(out.skid, kSlewRate * kDt);
	EXPECT_FLOAT_EQ(out.bank + out.skid, 4.f);

	for (int i = 0; i < 100; i++) {
		allocator.update(4.f, 1.f, 2.f, kDt);
	}

	EXPECT_FLOAT_EQ(allocator.getSkid(), 2.f);

	// on a reversal (figure-8) the skid still opposes the new turn direction, so bank briefly exceeds the total
	SkidToTurnAllocator::Output reversed = allocator.update(-4.f, 1.f, 2.f, kDt);
	EXPECT_FLOAT_EQ(reversed.skid, 2.f - kSlewRate * kDt);
	EXPECT_FLOAT_EQ(reversed.bank, -4.f - reversed.skid);

	for (int i = 0; i < 100; i++) {
		reversed = allocator.update(-4.f, 1.f, 2.f, kDt);
	}

	EXPECT_FLOAT_EQ(reversed.skid, -2.f);
	EXPECT_FLOAT_EQ(reversed.bank, -2.f);
}

TEST(SkidToTurnAllocator, invalidInputsFallBackToBankOnly)
{
	SkidToTurnAllocator allocator;
	allocator.setSlewRate(kFastSlewRate);

	SkidToTurnAllocator::Output out = allocator.update(4.f, NAN, 2.f, kDt);
	EXPECT_FLOAT_EQ(out.skid, 0.f);
	EXPECT_FLOAT_EQ(out.bank, 4.f);

	out = allocator.update(4.f, 1.f, NAN, kDt);
	EXPECT_FLOAT_EQ(out.skid, 0.f);
	EXPECT_FLOAT_EQ(out.bank, 4.f);

	out = allocator.update(NAN, 1.f, 2.f, kDt);
	EXPECT_FLOAT_EQ(out.skid, 0.f);
	EXPECT_FLOAT_EQ(out.bank, 0.f);

	out = allocator.update(4.f, 3.f, -1.f, kDt);
	EXPECT_FLOAT_EQ(out.skid, 0.f);
	EXPECT_FLOAT_EQ(out.bank, 4.f);
}

TEST(SkidToTurnAllocator, resetClearsSkid)
{
	static constexpr float kSlewRate = 5.f; // [m/s^3]
	SkidToTurnAllocator allocator;
	allocator.setSlewRate(kFastSlewRate);
	allocator.update(4.f, 1.f, 2.f, kDt);
	EXPECT_FLOAT_EQ(allocator.getSkid(), 2.f);

	allocator.reset();
	allocator.setSlewRate(kSlewRate);
	EXPECT_FLOAT_EQ(allocator.getSkid(), 0.f);
	EXPECT_FLOAT_EQ(allocator.update(4.f, 1.f, 2.f, kDt).skid, kSlewRate * kDt);
}
