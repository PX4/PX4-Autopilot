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
	param.tas_min = 10.f;
	param.tas_max = 30.f;
	param.pitch_max = radians(15.f);
	param.pitch_min = radians(-15.f);
	param.throttle_trim = 0.5f;
	param.throttle_max = 1.f;
	param.throttle_min = 0.f;
	param.altitude_error_gain = 0.2f;
	param.altitude_setpoint_gain_ff = 0.f;
	param.tas_error_percentage = 0.15f;
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

// --- Pitch/throttle loop response time offset (FW_T_PIT_THR_LAG) ------------------------------

// A positive offset (pitch loop faster) must lag only the pitch setpoint towards a step in the
// raw setpoint, while the throttle setpoint passes through unfiltered.
TEST(TECSSetpointFilterTest, PositiveOffsetLagsPitchSetpointOnly)
{
	TECS tecs;
	tecs.set_pitch_thr_response_offset(1.f); // pitch loop 1s faster
	tecs.initialize(100.f, 0.f, 15.f, 1.f);

	const float pitch_initial = tecs.get_pitch_setpoint();
	const float dt = 0.02f;
	const float pitch_step = pitch_initial + radians(10.f);
	const float throttle_step = 0.8f;

	tecs._updateSetpointFilters(dt, pitch_step, throttle_step);

	// After one sample the filtered pitch setpoint must have moved towards the step but not
	// reached it, the throttle setpoint must have passed through unchanged.
	EXPECT_GT(tecs.get_pitch_setpoint(), pitch_initial);
	EXPECT_LT(tecs.get_pitch_setpoint(), pitch_step);
	EXPECT_FLOAT_EQ(tecs.get_throttle_setpoint(), throttle_step);

	// The filter must converge onto a constant setpoint.
	for (int i = 0; i < 1000; i++) {
		tecs._updateSetpointFilters(dt, pitch_step, throttle_step);
	}

	EXPECT_NEAR(tecs.get_pitch_setpoint(), pitch_step, 1e-3f);
}

// A negative offset (throttle loop faster) must lag only the throttle setpoint.
TEST(TECSSetpointFilterTest, NegativeOffsetLagsThrottleSetpointOnly)
{
	TECS tecs;
	tecs.set_pitch_thr_response_offset(-1.f); // throttle loop 1s faster
	tecs.initialize(100.f, 0.f, 15.f, 1.f);

	const float throttle_initial = tecs.get_throttle_setpoint();
	const float dt = 0.02f;
	const float pitch_step = radians(10.f);
	const float throttle_step = throttle_initial + 0.3f;

	tecs._updateSetpointFilters(dt, pitch_step, throttle_step);

	EXPECT_GT(tecs.get_throttle_setpoint(), throttle_initial);
	EXPECT_LT(tecs.get_throttle_setpoint(), throttle_step);
	EXPECT_FLOAT_EQ(tecs.get_pitch_setpoint(), pitch_step);
}

// An offset magnitude at or below the sample time (and thus also exactly zero) is meaningless
// for a first order lag and must result in exact pass-through on both channels.
TEST(TECSSetpointFilterTest, OffsetBelowSampleTimeIsBypassed)
{
	const float dt = 0.02f;
	const float pitch_step = radians(10.f);
	const float throttle_step = 0.8f;

	for (const float offset : {0.f, 0.5f * FLT_EPSILON, 0.01f, dt, -0.01f, NAN}) {
		TECS tecs;
		tecs.set_pitch_thr_response_offset(offset);
		tecs.initialize(100.f, 0.f, 15.f, 1.f);

		tecs._updateSetpointFilters(dt, pitch_step, throttle_step);

		EXPECT_FLOAT_EQ(tecs.get_pitch_setpoint(), pitch_step) << "offset: " << offset;
		EXPECT_FLOAT_EQ(tecs.get_throttle_setpoint(), throttle_step) << "offset: " << offset;
	}
}

// Enabling the filter mid-operation must be bumpless: the inactive filter state is kept primed
// on the pass-through value, so the first filtered sample starts from the last output.
TEST(TECSSetpointFilterTest, EnablingFilterInFlightIsBumpless)
{
	TECS tecs;
	tecs.set_pitch_thr_response_offset(0.f); // disabled
	tecs.initialize(100.f, 0.f, 15.f, 1.f);

	const float dt = 0.02f;
	const float pitch_setpoint = radians(5.f);
	const float throttle_setpoint = 0.6f;

	// Run disabled for a while on a steady setpoint.
	for (int i = 0; i < 10; i++) {
		tecs._updateSetpointFilters(dt, pitch_setpoint, throttle_setpoint);
	}

	// Enable the filter in flight with an unchanged setpoint: the output must not jump.
	tecs.set_pitch_thr_response_offset(1.f);
	tecs._updateSetpointFilters(dt, pitch_setpoint, throttle_setpoint);

	EXPECT_FLOAT_EQ(tecs.get_pitch_setpoint(), pitch_setpoint);
	EXPECT_FLOAT_EQ(tecs.get_throttle_setpoint(), throttle_setpoint);
}
