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
using Failure = MotorFailureDetector::Failure;

constexpr float kDt = 1.f / 184.f;          // esc_status rate in the logs
constexpr float kBase = 21.f * 0.3f;         // healthy current at u=0.3: current_slope_a*u

hrt_abstime us(float seconds) { return (hrt_abstime)(seconds * 1e6f); }

// Config from a healthy hexrotor fit (linear, slope ~21 A/command).
MotorFailureDetector::Config makeConfig()
{
	MotorFailureDetector::Config cfg{};
	cfg.current_slope_a = 21.f;
	cfg.overcurrent_threshold_a = 5.f;
	cfg.overcurrent_trigger_time_s = 0.5f;
	cfg.undercurrent_threshold_a = 5.f;
	cfg.undercurrent_trigger_time_s = 0.5f;
	return cfg;
}

// Drive motor 0 for `seconds` at fixed dt, advancing the timestamp `now`, with command u
// and current current_fn(t). The first update() of a fresh detector just seeds the clock.
void run(MotorFailureDetector &d, hrt_abstime &now, float seconds, float dt, float u,
	 const std::function<float(float)> &current_fn)
{
	for (float t = 0.f; t < seconds; t += dt) {
		now += us(dt);
		d.update(0, now, u, current_fn(t));
	}
}

// Sustained open failure at fixed dt for >> trigger time; did it latch? Isolates the kMaxGap boundary.
bool latchesAtCadence(float dt)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                             // trigger time 0.5 s
	hrt_abstime now = 0;
	run(d, now, 3.f, dt, 0.3f, [](float) { return 0.f; }); // 3 s >> trigger time
	return d.failure(0) != Failure::None;
}
} // namespace

// Current matches the model exactly -> residual ~0 -> never trips (the false-positive guard).
TEST(MotorFailureDetectorTest, HealthyNeverTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 5.f, kDt, 0.3f, [](float) { return kBase; });
	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_LT(std::fabs(d.residualFiltered(0)), 0.1f);
}

// Commanded high but current collapses to ~0 -> trips after the trigger time.
TEST(MotorFailureDetectorTest, OpenFailureTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });
	EXPECT_EQ(d.failure(0), Failure::Undercurrent);
	EXPECT_LT(d.residualFiltered(0), -5.f);
}

// A tighter undercurrent band catches a current collapse that the (wider) overcurrent band alone
// misses -- the point of splitting: the fault side is not held hostage by overcurrent transients.
TEST(MotorFailureDetectorTest, UndercurrentBandCatchesWhatSymmetricMisses)
{
	MotorFailureDetector::Config cfg = makeConfig();
	cfg.overcurrent_threshold_a = 8.f;                       // wider than the fault the motor can produce
	cfg.undercurrent_threshold_a = 8.f;                      // symmetric: one band for both directions
	const auto half_current = [](float) { return kBase * 0.5f; };   // residual ~ -3.15 A

	MotorFailureDetector symmetric;
	symmetric.configure(cfg);
	hrt_abstime now = 0;
	run(symmetric, now, 3.f, kDt, 0.3f, half_current);
	ASSERT_EQ(symmetric.failure(0), Failure::None);                     // one wide band cannot see it

	cfg.undercurrent_threshold_a = 2.f;
	cfg.undercurrent_trigger_time_s = 0.5f;
	MotorFailureDetector asymmetric;
	asymmetric.configure(cfg);
	now = 0;
	run(asymmetric, now, 3.f, kDt, 0.3f, half_current);
	EXPECT_EQ(asymmetric.failure(0), Failure::Undercurrent);
}

// The bands are independent: an overcurrent excursion below the (wide) overcurrent band must not
// trip just because the tight undercurrent band would have caught the same magnitude.
TEST(MotorFailureDetectorTest, UndercurrentBandDoesNotArmTheOvercurrentSide)
{
	MotorFailureDetector::Config cfg = makeConfig();
	cfg.overcurrent_threshold_a = 8.f;
	cfg.undercurrent_threshold_a = 2.f;
	cfg.undercurrent_trigger_time_s = 0.5f;

	MotorFailureDetector d;
	d.configure(cfg);
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return kBase + 4.f; });   // +4 A: over the under band, under the over band
	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_GT(d.residualFiltered(0), 2.f);
}

// Each side keeps its own trigger time: a collapse latches on the undercurrent time, not the
// overcurrent one.
TEST(MotorFailureDetectorTest, PerSideTriggerTime)
{
	MotorFailureDetector::Config cfg = makeConfig();
	cfg.overcurrent_trigger_time_s = 5.f;                    // far longer than the run below
	cfg.undercurrent_threshold_a = 2.f;
	cfg.undercurrent_trigger_time_s = 0.2f;

	MotorFailureDetector d;
	d.configure(cfg);
	hrt_abstime now = 0;
	run(d, now, 1.f, kDt, 0.3f, [](float) { return 0.f; });
	EXPECT_EQ(d.failure(0), Failure::Undercurrent);   // latched on 0.2 s, not 5 s
}

// A fault that crosses the threshold but recovers before the trigger time must not latch
// (exercises the debounce-reject path: the residual goes over-threshold, then clears).
TEST(MotorFailureDetectorTest, ShortSpikeDoesNotLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 0.4f, kDt, 0.3f, [](float) { return 0.f; });   // residual crosses threshold ~0.1 s < 0.5 s trigger time
	ASSERT_GT(std::fabs(d.residualFiltered(0)), makeConfig().overcurrent_threshold_a); // genuinely over-threshold...
	run(d, now, 3.f, kDt, 0.3f, [](float) { return kBase; });  // ...then recovers before latching
	EXPECT_EQ(d.failure(0), Failure::None);
}

// A winding short / mechanical jam draws far MORE current than commanded -> large
// positive residual. The |.| threshold catches the over-current side, not just open circuits.
TEST(MotorFailureDetectorTest, OvercurrentTrips)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 2.5f * kBase; }); // ~2.5x expected
	EXPECT_EQ(d.failure(0), Failure::Overcurrent);
	EXPECT_GT(d.residualFiltered(0), 5.f);                   // positive residual trips
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
	EXPECT_EQ(d.failure(0), Failure::Undercurrent);
	EXPECT_LT(d.residualFiltered(0), -5.f);
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
	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_GT(std::fabs(d.residualFiltered(0)), 0.f);                       // residual present...
	EXPECT_LT(std::fabs(d.residualFiltered(0)), makeConfig().overcurrent_threshold_a);  // ...but below trip
}

// A descending vehicle windmills its props and the ESCs regenerate, reporting negative current --
// more negative than any fault can be, since a dead motor draws 0 A. The residual floor keeps that
// from latching where the band is out of reach, while a stop, whose residual sits exactly at the
// floor, still latches wherever the band is reachable at all.
TEST(MotorFailureDetectorTest, RegenerationBoundedButStopStillTrips)
{
	MotorFailureDetector::Config cfg = makeConfig();
	cfg.undercurrent_threshold_a = 4.f;                      // above I_exp at u = 0.15 (3.15 A)
	cfg.undercurrent_trigger_time_s = 0.5f;

	MotorFailureDetector regen;
	regen.configure(cfg);
	hrt_abstime now = 0;
	run(regen, now, 3.f, kDt, 0.15f, [](float) { return -5.f; });   // current flowing back into the pack
	EXPECT_EQ(regen.failure(0), Failure::None);
	// Floored at -I_exp (-3.15 A), not at the raw -8.15 A that would have crossed the band.
	EXPECT_GE(regen.residualFiltered(0), -cfg.current_slope_a * 0.15f - 0.01f);

	MotorFailureDetector stopped;
	stopped.configure(cfg);
	now = 0;
	run(stopped, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });   // I_exp 6.3 A, band reachable
	EXPECT_EQ(stopped.failure(0), Failure::Undercurrent);
}

// A data gap mid-build must RESET the trigger-time debounce, not just avoid latching in one step.
TEST(MotorFailureDetectorTest, GapResetsTriggerTimer)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                                  // trigger time 0.5 s
	hrt_abstime now = 0;
	run(d, now, 0.45f, kDt, 0.3f, [](float) { return 0.f; });   // fault building, < 0.5 s, not latched
	ASSERT_EQ(d.failure(0), Failure::None);

	now += us(0.5f);                                            // > kMaxGap -> gap resets filters + debounce
	d.update(0, now, 0.3f, 0.f);

	run(d, now, 0.45f, kDt, 0.3f, [](float) { return 0.f; });   // rebuild from zero, < 0.5 s -> must NOT latch
	EXPECT_EQ(d.failure(0), Failure::None);
}

// kMaxGap must clear the ~10 Hz feed interval: at kMaxGap=0.1 the feed (dt~=0.1 s) hit the gap-reset
// every tick, so a sustained fault never latched (replay: 0/100 phases). An outage longer than kMaxGap
// (the ESC-offline timeout) still resets.
TEST(MotorFailureDetectorTest, TenHzFeedLatchesButRealGapStillResets)
{
	static_assert(MotorFailureDetector::kMaxGap > 0.1f, "kMaxGap must exceed the ~10 Hz feed interval");

	EXPECT_TRUE(latchesAtCadence(0.105f)); // 10 Hz feed
	EXPECT_TRUE(latchesAtCadence(0.35f)); // slow but online ESC
	EXPECT_FALSE(latchesAtCadence(0.5f)); // outage
}

// Exclusion gate: a NaN command (motor off, or reversible) is never evaluated.
TEST(MotorFailureDetectorTest, ExcludedGate)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 5.f, kDt, NAN, [](float) { return 150.f; });
	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_EQ(d.residualFiltered(0), 0.f); // excluded: reset every step
}

// A threshold <= 0 disables that band (a default-constructed Config disables both): never latches.
TEST(MotorFailureDetectorTest, DisabledBandNeverLatches)
{
	MotorFailureDetector unconfigured;                    // zero Config
	hrt_abstime now0 = 0;
	run(unconfigured, now0, 5.f, kDt, 0.5f, [](float) { return 123.f; });
	EXPECT_EQ(unconfigured.failure(0), Failure::None);

	MotorFailureDetector::Config cfg = makeConfig();
	cfg.overcurrent_threshold_a = 0.f;
	MotorFailureDetector over_off;
	over_off.configure(cfg);
	hrt_abstime now = 0;
	run(over_off, now, 5.f, kDt, 0.3f, [](float) { return 2.5f * kBase; }); // overcurrent
	EXPECT_EQ(over_off.failure(0), Failure::None);

	cfg = makeConfig();
	cfg.undercurrent_threshold_a = 0.f;
	MotorFailureDetector under_off;
	under_off.configure(cfg);
	now = 0;
	run(under_off, now, 5.f, kDt, 0.3f, [](float) { return 0.f; }); // undercurrent
	EXPECT_EQ(under_off.failure(0), Failure::None);
}

// A non-finite current cannot be evaluated: like a NaN command it resets filter + debounce.
TEST(MotorFailureDetectorTest, NonFiniteCurrentNotEvaluated)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                                // trigger time 0.5 s
	hrt_abstime now = 0;
	run(d, now, 0.45f, kDt, 0.3f, [](float) { return 0.f; });  // fault building, not yet latched
	ASSERT_EQ(d.failure(0), Failure::None);
	run(d, now, 1.0f, kDt, 0.3f, [](float) { return NAN; }); // like a NaN command: reset, no latch
	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_EQ(d.residualFiltered(0), 0.f);
}

// A single step longer than kMaxGap is a data gap: it must reset, not latch in one jump.
TEST(MotorFailureDetectorTest, LargeDtGapNoLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());                            // trigger time 0.5 s
	hrt_abstime now = 0;
	d.update(0, now, 0.3f, 0.f); // seed the clock
	now += us(10.f);                                      // 10 s >> kMaxGap, single step -> gap, no latch
	d.update(0, now, 0.3f, 0.f);
	EXPECT_EQ(d.failure(0), Failure::None);
}

// A residual built up before exclusion must not survive it and trip on re-entry.
TEST(MotorFailureDetectorTest, ExclusionReentryNoStaleTrip)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 0.3f, kDt, 0.3f, [](float) { return 0.f; });   // big residual (< trigger time)
	run(d, now, 1.f, kDt, NAN, [](float) { return 0.f; });     // excluded (motor commanded off)
	ASSERT_EQ(d.residualFiltered(0), 0.f); // excluded: reset every step
	run(d, now, 3.f, kDt, 0.3f, [](float) { return kBase; });  // healthy again
	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_LT(std::fabs(d.residualFiltered(0)), 0.5f); // reseeded, not stale
}

// The latch outlives exclusion. Handling a failure makes that motor unevaluable -- dropped from
// allocation or driven reversible, both a NAN command -- and that must not retract its own trigger.
TEST(MotorFailureDetectorTest, LatchSurvivesExclusion)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; }); // latch
	ASSERT_EQ(d.failure(0), Failure::Undercurrent);
	run(d, now, 1.f, kDt, NAN, [](float) { return kBase; });
	EXPECT_EQ(d.residualFiltered(0), 0.f); // excluded: reset every step
	EXPECT_EQ(d.failure(0), Failure::Undercurrent);
}

// reset() clears the sticky failure latch (re-arm).
TEST(MotorFailureDetectorTest, ResetClearsLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });
	ASSERT_EQ(d.failure(0), Failure::Undercurrent);
	d.reset();
	EXPECT_EQ(d.failure(0), Failure::None);
}

// configure() (parameter change, possibly in flight) keeps the latch; only reset() clears it.
TEST(MotorFailureDetectorTest, ConfigureKeepsLatch)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });
	ASSERT_EQ(d.failure(0), Failure::Undercurrent);
	d.configure(makeConfig());
	EXPECT_EQ(d.failure(0), Failure::Undercurrent);
}

// Each motor keeps its own clock: one that is not updated for > kMaxGap restarts clean, even while
// the others keep being updated (a caller skips motors it has no data for).
TEST(MotorFailureDetectorTest, SkippedMotorRestartsClean)
{
	MotorFailureDetector d;
	d.configure(makeConfig()); // trigger time 0.5 s
	hrt_abstime now = 0;

	for (float t = 0.f; t < 0.45f; t += kDt) { // fault building on motor 0, not latched
		now += us(kDt);
		d.update(0, now, 0.3f, 0.f);
		d.update(1, now, 0.3f, kBase);
	}

	for (float t = 0.f; t < 0.5f; t += kDt) { // motor 0 skipped > kMaxGap, motor 1 continues
		now += us(kDt);
		d.update(1, now, 0.3f, kBase);
	}

	for (float t = 0.f; t < 0.45f; t += kDt) { // motor 0 resumes: rebuild from zero, < 0.5 s
		now += us(kDt);
		d.update(0, now, 0.3f, 0.f);
	}

	EXPECT_EQ(d.failure(0), Failure::None);
}

// A repeated sample timestamp is the same ESC sample again (esc_status republished for another ESC):
// it is ignored, whatever command and current come with it.
TEST(MotorFailureDetectorTest, RepeatedSampleIgnored)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 0.2f, kDt, 0.3f, [](float) { return 0.f; }); // fault building, residual far from 0
	const float residual_before = d.residualFiltered(0);
	ASSERT_LT(residual_before, -1.f);

	for (int k = 0; k < 1000; ++k) { d.update(0, now, NAN, 0.f); } // same timestamp, would otherwise reset

	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_EQ(d.residualFiltered(0), residual_before);
}

// Motors latch independently.
TEST(MotorFailureDetectorTest, MultiMotorIndependent)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	const float current[3] = {kBase, 0.f, 0.f}; // 0 healthy, 1 & 2 open
	hrt_abstime now = 0;

	for (float t = 0.f; t < 3.f; t += kDt) {
		now += us(kDt);

		for (int i = 0; i < 3; ++i) { d.update(i, now, 0.3f, current[i]); }
	}

	EXPECT_EQ(d.failure(0), Failure::None);
	EXPECT_EQ(d.failure(1), Failure::Undercurrent);
	EXPECT_EQ(d.failure(2), Failure::Undercurrent);
}

// The failure kind is the first band that tripped: a later excursion to the other side does not
// overwrite it, so the report stays consistent for the rest of the flight.
TEST(MotorFailureDetectorTest, FailureKindKeepsFirstTrip)
{
	MotorFailureDetector d;
	d.configure(makeConfig());
	hrt_abstime now = 0;
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 0.f; });           // undercurrent latches
	ASSERT_EQ(d.failure(0), Failure::Undercurrent);
	run(d, now, 3.f, kDt, 0.3f, [](float) { return 2.5f * kBase; });  // then sustained overcurrent
	ASSERT_GT(d.residualFiltered(0), 5.f);
	EXPECT_EQ(d.failure(0), Failure::Undercurrent);
}
