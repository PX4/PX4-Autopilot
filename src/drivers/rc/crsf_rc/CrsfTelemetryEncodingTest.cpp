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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "CrsfTelemetryEncoding.hpp"

#include <cmath>
#include <limits>

TEST(CrsfTelemetryEncodingTest, BoundsFallbackRssi)
{
	EXPECT_EQ(crsfRssiDbmToPercent(-200.f, 100), 0);
	EXPECT_EQ(crsfRssiDbmToPercent(-130.f, 100), 0);
	EXPECT_EQ(crsfRssiDbmToPercent(-65.f, 100), 50);
	EXPECT_EQ(crsfRssiDbmToPercent(-64.f, 100), 50);
	EXPECT_EQ(crsfRssiDbmToPercent(0.f, 100), 100);
	EXPECT_EQ(crsfRssiDbmToPercent(20.f, 100), 100);
	EXPECT_EQ(crsfRssiDbmToPercent(std::numeric_limits<float>::quiet_NaN(), 100), 0);
}

TEST(CrsfTelemetryEncodingTest, SaturatesGpsAltitudeAfterOffset)
{
	EXPECT_EQ(crsfGpsAltitudeToWire(-2000.), 0);
	EXPECT_EQ(crsfGpsAltitudeToWire(-1000.), 0);
	EXPECT_EQ(crsfGpsAltitudeToWire(-999.5), 1);
	EXPECT_EQ(crsfGpsAltitudeToWire(0.), 1000);
	EXPECT_EQ(crsfGpsAltitudeToWire(12.75), 1012);
	EXPECT_EQ(crsfGpsAltitudeToWire(64535.), UINT16_MAX);
	EXPECT_EQ(crsfGpsAltitudeToWire(100000.), UINT16_MAX);
	EXPECT_EQ(crsfGpsAltitudeToWire(std::numeric_limits<double>::quiet_NaN()), 0);
	EXPECT_EQ(crsfGpsAltitudeToWire(std::numeric_limits<double>::infinity()), UINT16_MAX);
}

TEST(CrsfTelemetryEncodingTest, SaturatesUnsignedFuelField)
{
	EXPECT_EQ(crsfFuelToWire(-1.f), 0u);
	EXPECT_EQ(crsfFuelToWire(0.f), 0u);
	EXPECT_EQ(crsfFuelToWire(42.75f), 42u);
	EXPECT_EQ(crsfFuelToWire(0xFFFFFF), 0xFFFFFFu);
	EXPECT_EQ(crsfFuelToWire(0xFFFFFF + 1), 0xFFFFFFu);
	EXPECT_EQ(crsfFuelToWire(std::numeric_limits<float>::quiet_NaN()), 0u);
	EXPECT_EQ(crsfFuelToWire(std::numeric_limits<float>::infinity()), 0xFFFFFFu);
}

TEST(CrsfTelemetryEncodingTest, RequiresACompleteWrite)
{
	EXPECT_FALSE(crsfWriteComplete(-1, 16));
	EXPECT_FALSE(crsfWriteComplete(0, 16));
	EXPECT_FALSE(crsfWriteComplete(15, 16));
	EXPECT_TRUE(crsfWriteComplete(16, 16));
	EXPECT_FALSE(crsfWriteComplete(17, 16));
}
