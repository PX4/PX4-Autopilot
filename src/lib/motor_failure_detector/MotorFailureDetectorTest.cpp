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
#include <MotorFailureDetector.hpp>

#include <cmath>
#include <functional>

namespace
{
constexpr float kDt = 1.f / 184.f;          // esc_status rate in the logs
constexpr float kBase = 21.f * 0.3f;         // healthy current at u=0.3: model_b*u

hrt_abstime us(float seconds) { return (hrt_abstime)(seconds * 1e6f); }

// Config from a healthy hexrotor fit (linear, slope ~21 A/command).
MotorFailureDetector::Config makeConfig()
{
	MotorFailureDetector::Config cfg{};
	cfg.model_b = 21.f;
	cfg.residual_lpf_tau_s = 0.2f;
	cfg.threshold_a = 5.f;
	cfg.persistence_s = 0.5f;
	return cfg;
}

// Drive motor 0 for `seconds` at fixed dt, advancing the timestamp `now`, with command u
// and current current_fn(t). The first update() of a fresh detector just seeds the clock.
void run(MotorFailureDetector &d, hrt_abstime &now, float seconds, float dt, float u,
	 const std::function<float(float)> &current_fn, bool reversible = false)
{
	const float cmd[1] = {u};
	const bool rev[1] = {reversible};

	for (float t = 0.f; t < seconds; t += dt) {
		now += us(dt);
		const float cur[1] = {current_fn(t)};
		d.update(1, now, cmd, cur, rev);
	}
}

// Sustained open failure at fixed dt for >> persistence; did it latch? Isolates the kMaxGap boundary.
bool latchesAtCadence(float dt)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                             // persistence 0.5 s
	hrt_abstime now = 0;
	run(d, now, 3.f, dt, 0.3f, [](float) { return 0.f; }); // 3 s >> persistence
	return d.anyFailed();
}
} // namespace

// Current matches the model exactly -> residual ~0 -> never trips (the false-positive guard).
TEST(MotorFailureDetectorTest, HealthyNeverTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 5.f, kDt, 0.3f, [](float) { return kBase; });
	EXPECT_FALSE(d.anyFailed());
	EXPECT_LT(std::fabs(d.status(0).residual_lpf), 0.1f);
}

// Commanded high but current collapses to ~0 -> trips after persistence.
TEST(MotorFailureDetectorTest, OpenFailureTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });
	EXPECT_TRUE(d.anyFailed());
	EXPECT_EQ(d.firstFailed(), 0);
	EXPECT_LT(d.status(0).residual_lpf, -5.f);
}

// A fault that crosses the threshold but recovers before the persistence window must not latch
// (exercises the debounce-reject path: the residual goes over-threshold, then clears).
TEST(MotorFailureDetectorTest, ShortSpikeDoesNotLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 0.4f, kDt, 0.3f, [](float) { return 0.f; });   // residual crosses threshold ~0.1 s < 0.5 s persistence
	ASSERT_GT(std::fabs(d.status(0).residual_lpf), makeConfig().threshold_a); // genuinely over-threshold...
	run(d, now, 3.f, kDt, 0.3f, [](float) { return kBase; });  // ...then recovers before latching
	EXPECT_FALSE(d.anyFailed());
}

// A winding short / mechanical jam draws far MORE current than commanded -> large
// positive residual. The |.| threshold catches the over-current side, not just open circuits.
TEST(MotorFailureDetectorTest, OvercurrentTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 2.5f * kBase; }); // ~2.5x expected
	EXPECT_TRUE(d.anyFailed());
	EXPECT_EQ(d.firstFailed(), 0);
	EXPECT_GT(d.status(0).residual_lpf, 5.f);                   // positive residual trips
}

// A fully shed propeller unloads the motor: current collapses well below the model
// (~15% here) -> negative residual past the threshold. This is the case a current-vs-rpm
// model would miss (rpm stays high while thrust is gone); command-residual catches it.
TEST(MotorFailureDetectorTest, PropLossSevereTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.15f * kBase; });
	EXPECT_TRUE(d.anyFailed());
	EXPECT_LT(d.status(0).residual_lpf, -5.f);
}

// A ~50% partial current loss sits inside the dead band (|residual| < threshold) and is
// NOT caught with a global threshold -- the known partial-fault limit. Lowering the
// threshold only helps down to the in-flight maneuver noise floor.
TEST(MotorFailureDetectorTest, PartialLossNearFloorMissed)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 5.f, kDt, 0.3f, [](float) { return 0.5f * kBase; }); // 50% loss
	EXPECT_FALSE(d.anyFailed());
	EXPECT_GT(std::fabs(d.status(0).residual_lpf), 0.f);                       // residual present...
	EXPECT_LT(std::fabs(d.status(0).residual_lpf), makeConfig().threshold_a);  // ...but below trip
}

// A data gap mid-build must RESET the persistence debounce, not just avoid latching in one step.
TEST(MotorFailureDetectorTest, GapResetsPersistenceTimer)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                                  // persistence 0.5 s
	hrt_abstime now = 0;
	run(d, now, 0.45f, kDt, 0.3f, [](float) { return 0.f; });   // fault building, < 0.5 s, not latched
	ASSERT_FALSE(d.anyFailed());

	const float cmd[1] = {0.3f}, cur[1] = {0.f};
	const bool rev[1] = {false};
	now += us(0.5f);                                            // > kMaxGap -> gap resets filters + debounce
	d.update(1, now, cmd, cur, rev);

	run(d, now, 0.45f, kDt, 0.3f, [](float) { return 0.f; });   // rebuild from zero, < 0.5 s -> must NOT latch
	EXPECT_FALSE(d.anyFailed());
}

// kMaxGap must clear the ~10 Hz feed interval: at kMaxGap=0.1 the feed (dt~=0.1 s) hit the gap-reset
// every tick, so a sustained fault never latched (replay: 0/100 phases). A real outage still resets.
TEST(MotorFailureDetectorTest, TenHzFeedLatchesButRealGapStillResets)
{
	static_assert(MotorFailureDetector::kMaxGap > 0.1f, "kMaxGap must exceed the ~10 Hz feed interval");
	static_assert(MotorFailureDetector::kMaxGap < 0.4f, "kMaxGap must stay below the 400 ms ESC-offline timeout");

	EXPECT_TRUE(latchesAtCadence(0.105f));  // 10 Hz feed
	EXPECT_FALSE(latchesAtCadence(0.4f));   // real outage
}

// Exclusion gates: a reversible or NaN (motor-off) command is never evaluated.
TEST(MotorFailureDetectorTest, ExcludedGates)
{
	for (auto kind : {0, 1}) {
		MotorFailureDetector d;
		d.configure(makeConfig());
		hrt_abstime now = 0;
		const float u = (kind == 1) ? NAN : 0.3f;
		run(d, now, 5.f, kDt, u, [](float) { return 150.f; }, /*reversible=*/kind == 0);
		EXPECT_FALSE(d.anyFailed());
		EXPECT_TRUE(d.status(0).excluded);
	}
}

// threshold_a <= 0 (incl. a default-constructed Config) disables the check: never latches.
TEST(MotorFailureDetectorTest, MonitorOnlyNeverLatches)
{
	MotorFailureDetector unconfigured;                    // zero Config
	hrt_abstime now0 = 0;
	run(unconfigured, now0, 5.f, kDt, 0.5f, [](float) { return 123.f; });
	EXPECT_FALSE(unconfigured.anyFailed());

	MotorFailureDetector::Config cfg = makeConfig();
	cfg.threshold_a = 0.f;
	MotorFailureDetector d;
	d.configure(cfg);
	hrt_abstime now = 0;
	run(d, now, 5.f, kDt, 0.3f, [](float) { return 0.f; });
	EXPECT_FALSE(d.anyFailed());
}

// Telemetry dropout (NaN current) holds the residual: the debounce keeps running on the
// held value (so a fault that also drops telemetry still latches) and state stays finite.
TEST(MotorFailureDetectorTest, DropoutHoldsTimerAndStaysFinite)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                                // persistence 0.5 s
	hrt_abstime now = 0;
	run(d, now, 0.45f, kDt, 0.3f, [](float) { return 0.f; });  // fault building, not yet latched
	ASSERT_FALSE(d.anyFailed());
	run(d, now, 1.0f, kDt, 0.3f, [](float) { return NAN; });   // dropout: held residual keeps the debounce running
	EXPECT_TRUE(d.anyFailed());
	EXPECT_TRUE(std::isfinite(d.status(0).residual_lpf));
}

// A single step longer than kMaxGap is a data gap: it must reset, not latch in one jump.
TEST(MotorFailureDetectorTest, LargeDtGapNoLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                            // persistence 0.5 s
	const float cmd[1] = {0.3f}, cur[1] = {0.f};
	const bool rev[1] = {false};
	hrt_abstime now = 0;
	d.update(1, now, cmd, cur, rev);                  // seed the clock
	now += us(10.f);                                      // 10 s >> kMaxGap, single step -> gap, no latch
	d.update(1, now, cmd, cur, rev);
	EXPECT_FALSE(d.anyFailed());
}

// A residual built up before exclusion must not survive it and trip on re-entry.
TEST(MotorFailureDetectorTest, ExclusionReentryNoStaleTrip)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 0.3f, kDt, 0.3f, [](float) { return 0.f; });   // big residual (< persistence)
	run(d, now, 1.f, kDt, NAN, [](float) { return 0.f; });     // excluded (motor commanded off)
	ASSERT_TRUE(d.status(0).excluded);
	run(d, now, 3.f, kDt, 0.3f, [](float) { return kBase; });  // healthy again
	EXPECT_FALSE(d.anyFailed());
	EXPECT_LT(std::fabs(d.status(0).residual_lpf), 0.5f); // reseeded, not stale
}

// reset() clears the sticky failure latch and the firstFailed() index (re-arm).
TEST(MotorFailureDetectorTest, ResetClearsLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });
	ASSERT_TRUE(d.anyFailed());
	d.reset();
	EXPECT_FALSE(d.anyFailed());
	EXPECT_EQ(d.firstFailed(), -1);
}

// firstFailed() returns the lowest index among multiple failed motors.
TEST(MotorFailureDetectorTest, MultiMotorFirstFailed)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	const int N = 3;
	const float cmd[N] = {0.3f, 0.3f, 0.3f};
	const bool rev[N] = {false, false, false};
	hrt_abstime now = 0;

	for (float t = 0.f; t < 3.f; t += kDt) {
		now += us(kDt);
		const float cur[N] = {kBase, 0.f, 0.f};       // 0 healthy, 1 & 2 open
		d.update(N, now, cmd, cur, rev);
	}

	EXPECT_FALSE(d.status(0).failed);
	EXPECT_TRUE(d.status(1).failed);
	EXPECT_TRUE(d.status(2).failed);
	EXPECT_EQ(d.firstFailed(), 1);
}
