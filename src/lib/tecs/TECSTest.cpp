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
#include <cmath>

#include "TECS.hpp"

using namespace math;

namespace
{

// Representative fixed-wing parameter set, enough to exercise the pitch control loop.
TECSControl::Param makeParam()
{
	TECSControl::Param param{};
	param.max_sink_rate = 5.f;
	param.min_sink_rate = 2.f;
	param.max_climb_rate = 5.f;
	param.vert_accel_limit = 10.f;
	param.equivalent_airspeed_trim = 15.f;
	param.tas_trim = 15.f;
	param.tas_min = 10.f;
	param.tas_stall = 7.f;
	param.tas_max = 30.f;
	param.pitch_max = radians(15.f);
	param.pitch_min = radians(-15.f);
	param.throttle_trim = 0.5f;
	param.throttle_max = 1.f;
	param.throttle_min = 0.f;
	param.altitude_error_gain = 0.2f;
	param.altitude_setpoint_gain_ff = 0.f;
	param.airspeed_error_gain = 0.1f;
	param.ste_rate_time_const = 0.1f;
	param.seb_rate_ff = 1.f;
	param.pitch_speed_weight = 1.f;
	param.integrator_gain_pitch = 0.4f;
	param.pitch_damping_gain = 0.1f;
	param.integrator_gain_throttle = 0.3f;
	param.throttle_damping_gain = 0.1f;
	param.throttle_slewrate = 0.f;
	param.load_factor_correction = 0.f;
	param.load_factor = 1.f;
	param.fast_descend = 0.f;
	return param;
}

TECSControl::Flag makeFlag()
{
	TECSControl::Flag flag{};
	flag.airspeed_enabled = true;
	flag.detect_underspeed_enabled = true;
	return flag;
}

TECSControl::Input makeInput()
{
	TECSControl::Input input{};
	input.altitude = 100.f;
	input.altitude_rate = 0.f;
	input.tas = 15.f;
	input.tas_rate = 0.f;
	return input;
}

TECSControl::Setpoint makeSetpoint()
{
	TECSControl::Setpoint setpoint{};
	setpoint.altitude_reference.alt = 100.f;
	setpoint.altitude_reference.alt_rate = 0.f;
	setpoint.altitude_rate_setpoint_direct = NAN;
	setpoint.tas_setpoint = 15.f;
	return setpoint;
}

} // namespace

// A single non-finite specific-energy-balance rate setpoint (here injected through the
// true airspeed setpoint, mirroring what happens on the energy/speed side when exiting
// offboard velocity mode) must not permanently corrupt the pitch integrator. Regression
// test for the in-flight NaN lockup reported in #25906.
TEST(TECSControlTest, PitchIntegratorRejectsNonFiniteInput)
{
	TECSControl control;
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	control.initialize(makeSetpoint(), input, param, flag);

	// Run a few healthy cycles so the integrator builds up some steady-state memory.
	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 120.f; // climb demand drives the integrator off zero
	const float dt = 0.02f;

	for (int i = 0; i < 50; i++) {
		control.update(dt, setpoint, input, param, flag);
	}

	const float integrator_before = control.getDebugOutput().pitch_integrator;
	ASSERT_TRUE(PX4_ISFINITE(integrator_before));

	// Inject a single non-finite setpoint, the kind of one-frame glitch seen on mode exit.
	TECSControl::Setpoint bad_setpoint = setpoint;
	bad_setpoint.tas_setpoint = NAN;
	control.update(dt, bad_setpoint, input, param, flag);

	// The bad frame must not propagate into the integrator or the pitch demand.
	EXPECT_TRUE(PX4_ISFINITE(control.getDebugOutput().pitch_integrator));
	EXPECT_TRUE(PX4_ISFINITE(control.getPitchSetpoint()));

	// Resume healthy updates, the controller must still produce finite output.
	for (int i = 0; i < 10; i++) {
		control.update(dt, setpoint, input, param, flag);
		EXPECT_TRUE(PX4_ISFINITE(control.getDebugOutput().pitch_integrator));
		EXPECT_TRUE(PX4_ISFINITE(control.getPitchSetpoint()));
	}
}

// Directly corrupt the integrator state and confirm the safety reset in
// _calcPitchControlUpdate cleans it. The public update() path zeroes bad
// integrator *input* before it can reach the state, so this branch is only
// reachable if the state itself is already non-finite (e.g. corrupted memory
// or an unguarded upstream write). The FRIEND_TEST seam lets us reach it.
TEST(TECSControlTest, PitchIntegratorResetsCorruptedState)
{
	TECSControl control;
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	control.initialize(makeSetpoint(), input, param, flag);

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 120.f;
	const float dt = 0.02f;

	// Pre-corrupt the integrator state, bypassing the input guard entirely.
	control._pitch_integ_state = NAN;

	// A single healthy update must detect and clear the corrupted state.
	control.update(dt, setpoint, input, param, flag);

	EXPECT_TRUE(PX4_ISFINITE(control._pitch_integ_state));
	EXPECT_TRUE(PX4_ISFINITE(control.getDebugOutput().pitch_integrator));
	EXPECT_TRUE(PX4_ISFINITE(control.getPitchSetpoint()));

	// And the controller keeps producing finite output afterwards.
	for (int i = 0; i < 10; i++) {
		control.update(dt, setpoint, input, param, flag);
		EXPECT_TRUE(PX4_ISFINITE(control._pitch_integ_state));
		EXPECT_TRUE(PX4_ISFINITE(control.getPitchSetpoint()));
	}
}

// Repeated non-finite input across many consecutive frames must never brick the controller:
// every cycle has to keep both the integrator state and the pitch setpoint finite.
TEST(TECSControlTest, PitchIntegratorSurvivesSustainedNonFiniteInput)
{
	TECSControl control;
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	control.initialize(makeSetpoint(), input, param, flag);

	TECSControl::Setpoint bad_setpoint = makeSetpoint();
	bad_setpoint.tas_setpoint = NAN;
	const float dt = 0.02f;

	for (int i = 0; i < 100; i++) {
		control.update(dt, bad_setpoint, input, param, flag);
		EXPECT_TRUE(PX4_ISFINITE(control.getDebugOutput().pitch_integrator));
		EXPECT_TRUE(PX4_ISFINITE(control.getPitchSetpoint()));
	}
}

namespace
{

// Run one update at the given true airspeed and report the resulting underspeed ratio.
float underspeedRatioAt(float tas, TECSControl::Param param, const TECSControl::Flag &flag = makeFlag())
{
	TECSControl control;
	TECSControl::Input input = makeInput();
	input.tas = tas;

	control.initialize(makeSetpoint(), input, param, flag);
	control.update(0.02f, makeSetpoint(), input, param, flag);

	return control.getRatioUndersped();
}

} // namespace

// Underspeed mitigation is referenced to the stall airspeed, not to a percentage of the trim airspeed: it stays
// inactive at and above the minimum airspeed demand and saturates part way down the stall margin, so it is fully
// ramped in with stall margin still in hand.
TEST(TECSControlTest, UnderspeedRatioIsReferencedToStallAirspeed)
{
	TECSControl::Param param = makeParam(); // tas_min = 10, tas_stall = 7
	// Full mitigation half way from the stall airspeed to the minimum airspeed demand.
	const float tas_fully_undersped = 0.5f * (param.tas_stall + param.tas_min); // 8.5

	// At and above the minimum airspeed demand there is no mitigation.
	EXPECT_FLOAT_EQ(underspeedRatioAt(param.tas_min + 5.f, param), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(param.tas_min, param), 0.f);

	// Saturated with stall margin still in hand, and stays saturated below that.
	EXPECT_FLOAT_EQ(underspeedRatioAt(tas_fully_undersped, param), 1.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(param.tas_stall, param), 1.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(0.f, param), 1.f);

	// Linear in between.
	const float midpoint = 0.5f * (param.tas_min + tas_fully_undersped);
	EXPECT_NEAR(underspeedRatioAt(midpoint, param), 0.5f, 1e-4f);
}

// The ramp bounds follow the minimum airspeed demand, which the caller compensates for load factor and flap
// setting. A higher demand (e.g. in a bank) must move the onset up with it.
TEST(TECSControlTest, UnderspeedRatioFollowsMinimumAirspeedDemand)
{
	TECSControl::Param param = makeParam();

	TECSControl::Param banked = param;
	banked.tas_min *= 1.2f; // load factor compensated minimum airspeed demand
	banked.tas_stall *= 1.2f;

	EXPECT_FLOAT_EQ(underspeedRatioAt(param.tas_min, param), 0.f);
	EXPECT_GT(underspeedRatioAt(param.tas_min, banked), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(banked.tas_min, banked), 0.f);
}

// The ramp spans a fixed fraction of the configured stall margin, so a tighter margin gives a correspondingly
// narrower ramp. What must hold regardless of the margin is that there is no mitigation at the minimum airspeed
// demand: mitigation ramping in during normal cruise would latch throttle at maximum.
TEST(TECSControlTest, UnderspeedRatioScalesWithStallMargin)
{
	TECSControl::Param tight = makeParam();
	tight.tas_min = 10.f;
	tight.tas_stall = 9.f; // 1 m/s of stall margin -> full mitigation at 9.5

	EXPECT_FLOAT_EQ(underspeedRatioAt(tight.tas_min, tight), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(9.5f, tight), 1.f);
	EXPECT_NEAR(underspeedRatioAt(9.75f, tight), 0.5f, 1e-3f);

	// The wide margin of the nominal parameter set puts the same half-way point much further down.
	const TECSControl::Param wide = makeParam(); // tas_min = 10, tas_stall = 7
	EXPECT_LT(underspeedRatioAt(9.5f, wide), underspeedRatioAt(9.5f, tight));
}

// A stall airspeed left far too low (e.g. never configured while the minimum airspeed demand was raised) must not
// be able to delay mitigation. The airspeed tracking error term does not depend on the stall airspeed, so it takes
// over and saturates mitigation one tracking error below the minimum airspeed demand.
TEST(TECSControlTest, UnderspeedRatioFallsBackToTrackingErrorForTooLowStallAirspeed)
{
	TECSControl::Param param = makeParam();
	param.tas_min = 20.f;
	param.tas_stall = 7.f;  // left at the default while the minimum airspeed demand was raised
	param.tas_trim = 15.f;

	// The stall term alone would only saturate at 13.5 m/s. The tracking error term saturates at
	// 20 - 0.15 * 15 = 17.75 m/s, and being the higher of the two it wins.
	EXPECT_FLOAT_EQ(underspeedRatioAt(param.tas_min, param), 0.f);
	EXPECT_NEAR(underspeedRatioAt(18.875f, param), 0.5f, 1e-4f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(17.75f, param), 1.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(13.5f, param), 1.f);
}

// Conversely, on an airframe with a properly configured stall airspeed the stall term is the higher of the two and
// governs, so a large trim airspeed cannot pull the saturation point down.
TEST(TECSControlTest, UnderspeedRatioUsesStallTermWhenItSaturatesEarlier)
{
	TECSControl::Param param = makeParam();
	param.tas_min = 20.f;
	param.tas_stall = 16.f; // 25% stall margin -> stall term saturates at 18 m/s
	param.tas_trim = 30.f;  // tracking error term would only saturate at 20 - 4.5 = 15.5 m/s

	EXPECT_FLOAT_EQ(underspeedRatioAt(param.tas_min, param), 0.f);
	EXPECT_NEAR(underspeedRatioAt(19.f, param), 0.5f, 1e-4f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(18.f, param), 1.f);
}

// A degenerate configuration (no stall margin, or a stall airspeed above the minimum airspeed demand, which the
// performance model sanity checks reject but TECS must still survive) collapses the ramp to a step at the minimum
// airspeed demand. It must never place the ramp above the airspeed being flown, which would pin the ratio to 1.
TEST(TECSControlTest, UnderspeedRatioHandlesDegenerateStallMargin)
{
	TECSControl::Param zero_margin = makeParam();
	zero_margin.tas_min = 10.f;
	zero_margin.tas_stall = 10.f;

	EXPECT_FLOAT_EQ(underspeedRatioAt(zero_margin.tas_min + 1.f, zero_margin), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(zero_margin.tas_min, zero_margin), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(9.9f, zero_margin), 1.f); // ramp width floor is 0.1 m/s

	TECSControl::Param inverted = makeParam();
	inverted.tas_min = 10.f;
	inverted.tas_stall = 12.f; // stall airspeed above the minimum airspeed demand

	EXPECT_FLOAT_EQ(underspeedRatioAt(inverted.tas_min + 1.f, inverted), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(inverted.tas_min, inverted), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatioAt(9.9f, inverted), 1.f);
}

// Detection must be inert whenever it is disabled or no airspeed is available.
TEST(TECSControlTest, UnderspeedRatioIsZeroWhenDisabled)
{
	const TECSControl::Param param = makeParam();

	TECSControl::Flag disabled = makeFlag();
	disabled.detect_underspeed_enabled = false;
	EXPECT_FLOAT_EQ(underspeedRatioAt(0.f, param, disabled), 0.f);

	TECSControl::Flag no_airspeed = makeFlag();
	no_airspeed.airspeed_enabled = false;
	EXPECT_FLOAT_EQ(underspeedRatioAt(0.f, param, no_airspeed), 0.f);
}
