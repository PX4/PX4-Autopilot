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

#include "mavlink_command_params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

using mavlink_cmd_params::decode_scaled_int32_field;

TEST(MavlinkActuatorParam, ScalesByDivisor)
{
	EXPECT_FLOAT_EQ(decode_scaled_int32_field(5000000, 1e7), 0.5);
	EXPECT_FLOAT_EQ(decode_scaled_int32_field(-10000000, 1e7), -1.0);
	EXPECT_FLOAT_EQ(decode_scaled_int32_field(0, 1e7), 0.0);

	EXPECT_FLOAT_EQ(decode_scaled_int32_field(15000, 1e4), 1.5);
	EXPECT_FLOAT_EQ(decode_scaled_int32_field(-5000, 1e4), -0.5);

	EXPECT_FLOAT_EQ(decode_scaled_int32_field(42, 1.0), 42.0);
	EXPECT_FLOAT_EQ(decode_scaled_int32_field(-7, 1.0), -7.0);
}

TEST(MavlinkActuatorParam, Int32MaxIsIgnored)
{
	EXPECT_TRUE(std::isnan(decode_scaled_int32_field(INT32_MAX, 1e7)));
	EXPECT_TRUE(std::isnan(decode_scaled_int32_field(INT32_MAX, 1e4)));
	EXPECT_TRUE(std::isnan(decode_scaled_int32_field(INT32_MAX, 1.0)));
}

TEST(MavlinkActuatorParam, Int32MinIsIgnored)
{
	// Some ground stations convert an unused NaN float param into INT32_MIN
	// when casting to int32; this must be treated as "not used" too.
	EXPECT_TRUE(std::isnan(decode_scaled_int32_field(INT32_MIN, 1e7)));
}

TEST(MavlinkActuatorParam, IgnoreSentinelIsPerFieldIndependent)
{
	// x=INT32_MAX, y=5000000 must ignore only the first field, not both.
	const double param5 = decode_scaled_int32_field(INT32_MAX, 1e7);
	const double param6 = decode_scaled_int32_field(5000000, 1e7);

	EXPECT_TRUE(std::isnan(param5));
	EXPECT_FLOAT_EQ(param6, 0.5);
}
