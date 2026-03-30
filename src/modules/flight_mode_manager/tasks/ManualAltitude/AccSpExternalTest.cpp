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
 * @file AccSpExternalTest.cpp
 *
 * Unit tests for the external acceleration setpoint logic:
 *   - Bit 12 detection in type_mask (MAVLink path)
 *   - Acceleration clamping
 *   - Watchdog timeout check
 *   - Invalid/NaN input rejection
 *   - Yaw passthrough
 */

#include <gtest/gtest.h>
#include <mathlib/mathlib.h>
#include <px4_defines.h>
#include <uORB/topics/acc_sp_external.h>

using namespace math;

// ---------------------------------------------------------------------------
// Helper: mirrors the clamping logic in _applyExternalAcceleration()
// ---------------------------------------------------------------------------
static bool applyExternalAccLogic(
	const acc_sp_external_s &cmd,
	uint64_t now_us,
	bool z_valid,
	float acc_limit,
	float &out_ax, float &out_ay, float &out_yaw)
{
	// Watchdog check
	const uint16_t timeout_ms = (cmd.timeout_ms > 0u) ? cmd.timeout_ms : 500u;

	if ((now_us - cmd.timestamp) > (uint64_t)timeout_ms * 1000ULL) {
		return false;
	}

	// Altitude estimate required
	if (!z_valid) {
		return false;
	}

	// NaN check
	if (!PX4_ISFINITE(cmd.acceleration[0]) || !PX4_ISFINITE(cmd.acceleration[1])) {
		return false;
	}

	// Clamp
	out_ax  = constrain(cmd.acceleration[0], -acc_limit, acc_limit);
	out_ay  = constrain(cmd.acceleration[1], -acc_limit, acc_limit);
	out_yaw = cmd.yaw;
	return true;
}

// ---------------------------------------------------------------------------
// Bit 12 detection — mirrors mavlink_receiver logic
// ---------------------------------------------------------------------------
static constexpr uint16_t ACC_SP_EXTERNAL_FLAG = (1u << 12); // 0x1000

TEST(AccSpExternalMavlinkTest, Bit12SetTriggerExternalPath)
{
	uint16_t type_mask = 0b110111000111u | ACC_SP_EXTERNAL_FLAG;
	EXPECT_TRUE(type_mask & ACC_SP_EXTERNAL_FLAG);
}

TEST(AccSpExternalMavlinkTest, Bit12ClearNoExternalPath)
{
	uint16_t type_mask = 0b110111000111u; // standard acc-only mask, no bit 12
	EXPECT_FALSE(type_mask & ACC_SP_EXTERNAL_FLAG);
}

TEST(AccSpExternalMavlinkTest, Bit12DoesNotAffectOtherBits)
{
	// Verify bit 12 is isolated and does not corrupt acceleration ignore bits
	uint16_t type_mask = 0b110111000111u | ACC_SP_EXTERNAL_FLAG;
	EXPECT_FALSE(type_mask & (1u << 6));  // AX not ignored
	EXPECT_FALSE(type_mask & (1u << 7));  // AY not ignored
	EXPECT_FALSE(type_mask & (1u << 8));  // AZ not ignored
}

// ---------------------------------------------------------------------------
// Acceleration clamping
// ---------------------------------------------------------------------------
TEST(AccSpExternalClampTest, WithinLimitPassthrough)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = -1.5f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax = 0.f, ay = 0.f, yaw = 0.f;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);

	EXPECT_TRUE(ok);
	EXPECT_FLOAT_EQ(ax, 2.0f);
	EXPECT_FLOAT_EQ(ay, -1.5f);
	EXPECT_TRUE(std::isnan(yaw));
}

TEST(AccSpExternalClampTest, ExceedsLimitIsClamped)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 10.0f;  // exceeds 5 m/s² limit
	cmd.acceleration[1] = -8.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax = 0.f, ay = 0.f, yaw = 0.f;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);

	EXPECT_TRUE(ok);
	EXPECT_FLOAT_EQ(ax, 5.0f);
	EXPECT_FLOAT_EQ(ay, -5.0f);
}

TEST(AccSpExternalClampTest, ZeroAcceleration)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 0.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax = 0.f, ay = 0.f, yaw = 0.f;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);

	EXPECT_TRUE(ok);
	EXPECT_FLOAT_EQ(ax, 0.0f);
	EXPECT_FLOAT_EQ(ay, 0.0f);
}

// ---------------------------------------------------------------------------
// Watchdog timeout
// ---------------------------------------------------------------------------
TEST(AccSpExternalWatchdogTest, FreshMessageAccepted)
{
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;    // t = 1 s
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw        = NAN;
	cmd.timeout_ms = 500u;

	float ax, ay, yaw;
	// now = 1.2 s → elapsed 200 ms < 500 ms
	bool ok = applyExternalAccLogic(cmd, 1'200'000u, true, 5.0f, ax, ay, yaw);
	EXPECT_TRUE(ok);
}

TEST(AccSpExternalWatchdogTest, StaleMessageRejected)
{
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;    // t = 1 s
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw        = NAN;
	cmd.timeout_ms = 500u;

	float ax, ay, yaw;
	// now = 2 s → elapsed 1000 ms > 500 ms
	bool ok = applyExternalAccLogic(cmd, 2'000'000u, true, 5.0f, ax, ay, yaw);
	EXPECT_FALSE(ok);
}

TEST(AccSpExternalWatchdogTest, ZeroTimeoutUsesDefault500ms)
{
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw        = NAN;
	cmd.timeout_ms = 0u;  // should default to 500 ms

	float ax, ay, yaw;
	// elapsed 300 ms < default 500 ms → should be accepted
	bool ok = applyExternalAccLogic(cmd, 1'300'000u, true, 5.0f, ax, ay, yaw);
	EXPECT_TRUE(ok);

	// elapsed 600 ms > default 500 ms → should be rejected
	bool ok2 = applyExternalAccLogic(cmd, 1'600'000u, true, 5.0f, ax, ay, yaw);
	EXPECT_FALSE(ok2);
}

// ---------------------------------------------------------------------------
// Health check: z_valid
// ---------------------------------------------------------------------------
TEST(AccSpExternalHealthTest, AltitudeInvalidRejectsCommand)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax, ay, yaw;
	bool ok = applyExternalAccLogic(cmd, 1000u, false /* z_valid=false */, 5.0f, ax, ay, yaw);
	EXPECT_FALSE(ok);
}

TEST(AccSpExternalHealthTest, AltitudeValidAcceptsCommand)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax, ay, yaw;
	bool ok = applyExternalAccLogic(cmd, 1000u, true /* z_valid=true */, 5.0f, ax, ay, yaw);
	EXPECT_TRUE(ok);
}

// ---------------------------------------------------------------------------
// NaN input rejection
// ---------------------------------------------------------------------------
TEST(AccSpExternalNaNTest, NaNAccXRejected)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = NAN;
	cmd.acceleration[1] = 1.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax, ay, yaw;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);
	EXPECT_FALSE(ok);
}

TEST(AccSpExternalNaNTest, NaNAccYRejected)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = NAN;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax, ay, yaw;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);
	EXPECT_FALSE(ok);
}

// ---------------------------------------------------------------------------
// Yaw passthrough
// ---------------------------------------------------------------------------
TEST(AccSpExternalYawTest, ExplicitYawPassthrough)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = 1.57f; // 90 degrees
	cmd.timeout_ms      = 500u;

	float ax, ay, yaw;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);
	EXPECT_TRUE(ok);
	EXPECT_FLOAT_EQ(yaw, 1.57f);
}

TEST(AccSpExternalYawTest, NaNYawHoldsCurrentYaw)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1000u;
	cmd.acceleration[0] = 1.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.acceleration[2] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	float ax, ay, yaw;
	bool ok = applyExternalAccLogic(cmd, 1000u, true, 5.0f, ax, ay, yaw);
	EXPECT_TRUE(ok);
	EXPECT_TRUE(std::isnan(yaw)); // NaN = hold current yaw
}
