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
// Mirrors the failsafe constants defined in FlightTaskManualAltitude.hpp
// ---------------------------------------------------------------------------
static constexpr float     EXT_ACC_BRAKE_VEL_THRESHOLD = 1.0f;
static constexpr float     EXT_ACC_VEL_LIMIT           = 5.0f;
static constexpr float     EXT_ACC_VEL_WARN_RATIO      = 0.8f;
static constexpr float     EXT_ACC_LAND_SPEED          = 0.7f;
static constexpr uint64_t  EXT_ACC_FORCE_LAND_US       = 30ULL * 1000000ULL;

// ---------------------------------------------------------------------------
// Helper: mirrors the full _applyExternalAcceleration() logic
// ---------------------------------------------------------------------------
struct ExtAccResult {
	bool  accepted;      // fresh command applied (not stale/invalid)
	float ax;            // final horizontal acc setpoint X
	float ay;            // final horizontal acc setpoint Y
	float yaw;           // yaw setpoint
	bool  braking;       // F2: active brake applied after timeout
	bool  vel_limited;   // F3: velocity limit modified acc
	bool  force_land;    // F8: force-land triggered
	float vz_setpoint;   // F8: descent rate when force-landing
};

static ExtAccResult applyExternalAccLogicFull(
	const acc_sp_external_s &cmd,
	uint64_t now_us,
	bool z_valid,
	bool v_xy_valid,
	float acc_limit,
	float vel_x, float vel_y,
	uint64_t last_valid_us)   // timestamp of last valid command (0 = never)
{
	ExtAccResult r{};
	r.vz_setpoint = 0.0f;

	// F6: reject if horizontal velocity estimate is invalid
	if (!v_xy_valid) {
		return r;
	}

	// Watchdog
	const uint16_t timeout_ms = (cmd.timeout_ms > 0u) ? cmd.timeout_ms : 500u;
	const bool cmd_fresh = ((now_us - cmd.timestamp) <= (uint64_t)timeout_ms * 1000ULL);

	if (!cmd_fresh) {
		// F2: active brake if still moving after stream loss
		const float vel_mag = sqrtf(vel_x * vel_x + vel_y * vel_y);

		if (vel_mag > EXT_ACC_BRAKE_VEL_THRESHOLD) {
			r.braking = true;
			r.ax = -vel_x / vel_mag * acc_limit;
			r.ay = -vel_y / vel_mag * acc_limit;
		}

		// F8: force land if stream absent > 30 s
		if (last_valid_us > 0 && (now_us - last_valid_us) > EXT_ACC_FORCE_LAND_US) {
			r.force_land  = true;
			r.vz_setpoint = EXT_ACC_LAND_SPEED;
		}

		return r;
	}

	// Fresh command — validate
	if (!z_valid) { return r; }

	if (!PX4_ISFINITE(cmd.acceleration[0]) || !PX4_ISFINITE(cmd.acceleration[1])) {
		return r;
	}

	float ax = constrain(cmd.acceleration[0], -acc_limit, acc_limit);
	float ay = constrain(cmd.acceleration[1], -acc_limit, acc_limit);

	// F3: velocity limiting
	const float vel_mag = sqrtf(vel_x * vel_x + vel_y * vel_y);

	if (vel_mag > EXT_ACC_VEL_LIMIT) {
		ax = -vel_x / vel_mag * acc_limit;
		ay = -vel_y / vel_mag * acc_limit;
		r.vel_limited = true;

	} else if (vel_mag > EXT_ACC_VEL_LIMIT * EXT_ACC_VEL_WARN_RATIO) {
		const float scale = 1.0f - (vel_mag - EXT_ACC_VEL_LIMIT * EXT_ACC_VEL_WARN_RATIO)
				    / (EXT_ACC_VEL_LIMIT * (1.0f - EXT_ACC_VEL_WARN_RATIO));
		ax *= scale;
		ay *= scale;
		r.vel_limited = true;
	}

	r.accepted = true;
	r.ax  = ax;
	r.ay  = ay;
	r.yaw = cmd.yaw;
	return r;
}

// ---------------------------------------------------------------------------
// Helper: backward-compatible wrapper (existing tests unchanged)
// ---------------------------------------------------------------------------
static bool applyExternalAccLogic(
	const acc_sp_external_s &cmd,
	uint64_t now_us,
	bool z_valid,
	float acc_limit,
	float &out_ax, float &out_ay, float &out_yaw)
{
	auto r = applyExternalAccLogicFull(cmd, now_us, z_valid, true, acc_limit,
					   0.f, 0.f, 0ULL);
	out_ax  = r.ax;
	out_ay  = r.ay;
	out_yaw = r.yaw;
	return r.accepted;
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

// ---------------------------------------------------------------------------
// F2: Active brake after watchdog timeout
// ---------------------------------------------------------------------------
TEST(AccSpExternalF2BrakeTest, NoBrakeWhenVelocityLow)
{
	// Stale command, velocity below threshold → no braking
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.timeout_ms      = 500u;

	// Elapsed 1 s > 500 ms → stale. vel = 0.5 m/s < 1.0 m/s threshold
	auto r = applyExternalAccLogicFull(cmd, 2'000'000u, true, true, 5.0f,
					   0.5f, 0.0f, 1'000'000u);
	EXPECT_FALSE(r.accepted);
	EXPECT_FALSE(r.braking);
}

TEST(AccSpExternalF2BrakeTest, BrakeAppliedWhenVelocityHigh)
{
	// Stale command, velocity above threshold → braking acc applied
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.timeout_ms      = 500u;

	// vel = 3.0 m/s North → brake should be South (negative ax)
	auto r = applyExternalAccLogicFull(cmd, 2'000'000u, true, true, 5.0f,
					   3.0f, 0.0f, 1'000'000u);
	EXPECT_FALSE(r.accepted);
	EXPECT_TRUE(r.braking);
	EXPECT_LT(r.ax, 0.0f);  // braking opposes velocity direction
	EXPECT_FLOAT_EQ(r.ay, 0.0f);
}

TEST(AccSpExternalF2BrakeTest, BrakeDirectionOppositeToDiagonalVelocity)
{
	// Drone moving NE → brake should push SW
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;
	cmd.timeout_ms = 500u;

	auto r = applyExternalAccLogicFull(cmd, 2'000'000u, true, true, 5.0f,
					   2.0f, 2.0f, 1'000'000u);
	EXPECT_TRUE(r.braking);
	EXPECT_LT(r.ax, 0.0f);
	EXPECT_LT(r.ay, 0.0f);
}

// ---------------------------------------------------------------------------
// F3: Velocity limiting during normal operation
// ---------------------------------------------------------------------------
TEST(AccSpExternalF3VelLimitTest, BelowWarnZoneNoScaling)
{
	// vel = 3.0 m/s < 4.0 m/s (80 % of 5.0) → acc unchanged
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = 1.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	auto r = applyExternalAccLogicFull(cmd, 1'200'000u, true, true, 5.0f,
					   3.0f, 0.0f, 1'000'000u);
	EXPECT_TRUE(r.accepted);
	EXPECT_FALSE(r.vel_limited);
	EXPECT_FLOAT_EQ(r.ax, 2.0f);
	EXPECT_FLOAT_EQ(r.ay, 1.0f);
}

TEST(AccSpExternalF3VelLimitTest, InWarnZoneAccScaledDown)
{
	// vel = 4.5 m/s — in warn zone [4.0, 5.0] → acc scaled down
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	auto r = applyExternalAccLogicFull(cmd, 1'200'000u, true, true, 5.0f,
					   4.5f, 0.0f, 1'000'000u);
	EXPECT_TRUE(r.accepted);
	EXPECT_TRUE(r.vel_limited);
	EXPECT_GT(r.ax, 0.0f);   // still positive (not brake)
	EXPECT_LT(r.ax, 2.0f);   // but reduced
}

TEST(AccSpExternalF3VelLimitTest, AboveHardLimitActiveBrake)
{
	// vel = 6.0 m/s > 5.0 m/s hard limit → brake override
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 3.0f;  // commanded forward, but must be overridden
	cmd.acceleration[1] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	auto r = applyExternalAccLogicFull(cmd, 1'200'000u, true, true, 5.0f,
					   6.0f, 0.0f, 1'000'000u);
	EXPECT_TRUE(r.accepted);
	EXPECT_TRUE(r.vel_limited);
	EXPECT_LT(r.ax, 0.0f);   // braking: opposite to velocity direction
}

TEST(AccSpExternalF3VelLimitTest, BrakeMagnitudeEqualsAccLimit)
{
	// Braking acc magnitude must equal acc_limit (= 5.0)
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 3.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	auto r = applyExternalAccLogicFull(cmd, 1'200'000u, true, true, 5.0f,
					   7.0f, 0.0f, 1'000'000u);
	const float brake_mag = sqrtf(r.ax * r.ax + r.ay * r.ay);
	EXPECT_NEAR(brake_mag, 5.0f, 1e-4f);
}

// ---------------------------------------------------------------------------
// F6: EKF horizontal velocity health check
// ---------------------------------------------------------------------------
TEST(AccSpExternalF6EkfTest, XYInvalidRejectsCommand)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	// v_xy_valid = false → must reject
	auto r = applyExternalAccLogicFull(cmd, 1'200'000u, true, false, 5.0f,
					   0.0f, 0.0f, 1'000'000u);
	EXPECT_FALSE(r.accepted);
	EXPECT_FALSE(r.braking);
}

TEST(AccSpExternalF6EkfTest, XYValidAcceptsCommand)
{
	acc_sp_external_s cmd{};
	cmd.timestamp       = 1'000'000u;
	cmd.acceleration[0] = 2.0f;
	cmd.acceleration[1] = 0.0f;
	cmd.yaw             = NAN;
	cmd.timeout_ms      = 500u;

	auto r = applyExternalAccLogicFull(cmd, 1'200'000u, true, true, 5.0f,
					   0.0f, 0.0f, 1'000'000u);
	EXPECT_TRUE(r.accepted);
}

// ---------------------------------------------------------------------------
// F8: Force land after prolonged stream absence
// ---------------------------------------------------------------------------
TEST(AccSpExternalF8ForceLandTest, NoForceLandBefore30s)
{
	// Stream lost at t=1s, check at t=20s (< 30s) → no force land
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;
	cmd.timeout_ms = 500u;

	const uint64_t last_valid  = 1'000'000u;
	const uint64_t now_20s     = 21'000'000u; // 20 s after last valid

	auto r = applyExternalAccLogicFull(cmd, now_20s, true, true, 5.0f,
					   0.0f, 0.0f, last_valid);
	EXPECT_FALSE(r.force_land);
}

TEST(AccSpExternalF8ForceLandTest, ForceLandAfter30s)
{
	// Stream lost at t=1s, check at t=32s (> 30s) → force land
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;
	cmd.timeout_ms = 500u;

	const uint64_t last_valid  = 1'000'000u;
	const uint64_t now_32s     = 33'000'000u; // 32 s after last valid

	auto r = applyExternalAccLogicFull(cmd, now_32s, true, true, 5.0f,
					   0.0f, 0.0f, last_valid);
	EXPECT_TRUE(r.force_land);
	EXPECT_NEAR(r.vz_setpoint, EXT_ACC_LAND_SPEED, 1e-4f);
}

TEST(AccSpExternalF8ForceLandTest, NoForceLandIfNeverHadValidCommand)
{
	// last_valid = 0 → stream never established → no force land
	acc_sp_external_s cmd{};
	cmd.timestamp  = 1'000'000u;
	cmd.timeout_ms = 500u;

	auto r = applyExternalAccLogicFull(cmd, 33'000'000u, true, true, 5.0f,
					   0.0f, 0.0f, 0ULL);
	EXPECT_FALSE(r.force_land);
}
