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
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Fix2.hpp>

#include "gnss_time.hpp"

namespace
{
constexpr uint64_t kUtcUsec = 1'700'000'000'123'456ULL;
constexpr uint64_t kMaxTimestampUsec = (1ULL << 56) - 1; // uavcan.Timestamp is uint56.
constexpr uint8_t kLeapSeconds = 27;
}

template<typename FixType>
class GnssTimeTest : public ::testing::Test
{
protected:
	static uint64_t convert(uint64_t timestamp_usec, uint8_t standard, uint8_t leap_seconds = kLeapSeconds)
	{
		FixType msg{};
		msg.gnss_timestamp.usec = timestamp_usec;
		msg.gnss_time_standard = standard;
		msg.num_leap_seconds = leap_seconds;
		return gnssTimeToUtc(msg);
	}
};

using FixTypes = ::testing::Types<uavcan::equipment::gnss::Fix, uavcan::equipment::gnss::Fix2>;
TYPED_TEST_SUITE(GnssTimeTest, FixTypes);

TYPED_TEST(GnssTimeTest, GpsToUtc)
{
	EXPECT_EQ(TestFixture::convert(kUtcUsec + 18'000'000, TypeParam::GNSS_TIME_STANDARD_GPS), kUtcUsec);
}

TYPED_TEST(GnssTimeTest, TaiToUtc)
{
	EXPECT_EQ(TestFixture::convert(kUtcUsec + 37'000'000, TypeParam::GNSS_TIME_STANDARD_TAI), kUtcUsec);
}

TYPED_TEST(GnssTimeTest, UtcUnchanged)
{
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_UTC), kUtcUsec);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_UTC,
				       TypeParam::NUM_LEAP_SECONDS_UNKNOWN), kUtcUsec);
	EXPECT_EQ(TestFixture::convert(kMaxTimestampUsec, TypeParam::GNSS_TIME_STANDARD_UTC), kMaxTimestampUsec);
}

TYPED_TEST(GnssTimeTest, UnknownTimeStandard)
{
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_NONE), 0u);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, 7), 0u); // Undefined uint3 time standard.
}

TYPED_TEST(GnssTimeTest, UnknownLeapSeconds)
{
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_GPS,
				       TypeParam::NUM_LEAP_SECONDS_UNKNOWN), 0u);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_TAI,
				       TypeParam::NUM_LEAP_SECONDS_UNKNOWN), 0u);
}

TYPED_TEST(GnssTimeTest, LeapSecondBoundaries)
{
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_GPS, 1), kUtcUsec + 8'000'000);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_GPS, 9), kUtcUsec);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_GPS, 255), kUtcUsec - 246'000'000);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_TAI, 1), kUtcUsec - 11'000'000);
	EXPECT_EQ(TestFixture::convert(kUtcUsec, TypeParam::GNSS_TIME_STANDARD_TAI, 255), kUtcUsec - 265'000'000);
}

TYPED_TEST(GnssTimeTest, UnknownTimestamp)
{
	EXPECT_EQ(TestFixture::convert(0, TypeParam::GNSS_TIME_STANDARD_UTC), 0u);
	EXPECT_EQ(TestFixture::convert(0, TypeParam::GNSS_TIME_STANDARD_GPS), 0u);
	EXPECT_EQ(TestFixture::convert(0, TypeParam::GNSS_TIME_STANDARD_TAI), 0u);
	EXPECT_EQ(TestFixture::convert(0, TypeParam::GNSS_TIME_STANDARD_GPS, 1), 0u);
}

TYPED_TEST(GnssTimeTest, SubtractionBoundary)
{
	EXPECT_EQ(TestFixture::convert(17'999'999, TypeParam::GNSS_TIME_STANDARD_GPS), 0u);
	EXPECT_EQ(TestFixture::convert(18'000'000, TypeParam::GNSS_TIME_STANDARD_GPS), 0u);
	EXPECT_EQ(TestFixture::convert(18'000'001, TypeParam::GNSS_TIME_STANDARD_GPS), 1u);
	EXPECT_EQ(TestFixture::convert(36'999'999, TypeParam::GNSS_TIME_STANDARD_TAI), 0u);
	EXPECT_EQ(TestFixture::convert(37'000'000, TypeParam::GNSS_TIME_STANDARD_TAI), 0u);
	EXPECT_EQ(TestFixture::convert(37'000'001, TypeParam::GNSS_TIME_STANDARD_TAI), 1u);
}

TYPED_TEST(GnssTimeTest, AdditionBoundary)
{
	// Exercise the uint64 guard as well as the protocol's narrower uint56 range.
	EXPECT_EQ(TestFixture::convert(UINT64_MAX - 8'000'000, TypeParam::GNSS_TIME_STANDARD_GPS, 1), UINT64_MAX);
	EXPECT_EQ(TestFixture::convert(UINT64_MAX - 7'999'999, TypeParam::GNSS_TIME_STANDARD_GPS, 1), 0u);
}

TYPED_TEST(GnssTimeTest, MaximumDronecanTimestamp)
{
	EXPECT_EQ(TestFixture::convert(kMaxTimestampUsec, TypeParam::GNSS_TIME_STANDARD_GPS), kMaxTimestampUsec - 18'000'000);
	EXPECT_EQ(TestFixture::convert(kMaxTimestampUsec, TypeParam::GNSS_TIME_STANDARD_TAI), kMaxTimestampUsec - 37'000'000);
	EXPECT_EQ(TestFixture::convert(kMaxTimestampUsec, TypeParam::GNSS_TIME_STANDARD_GPS, 1), kMaxTimestampUsec + 8'000'000);
}
