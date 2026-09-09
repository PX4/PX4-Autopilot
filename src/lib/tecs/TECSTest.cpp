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
#include <unistd.h>

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

// Run the detector for one cycle and report the resulting underspeed ratio.
float underspeedRatio(float tas, float tas_rate, TECSControl::Param param, const TECSControl::Flag &flag)
{
	TECSControl control;
	TECSControl::Input input = makeInput();
	input.tas = tas;
	input.tas_rate = tas_rate;

	control.initialize(makeSetpoint(), input, param, flag);
	control.update(0.02f, makeSetpoint(), input, param, flag);

	return control.getRatioUndersped();
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


// The underspeed ramp is anchored on tas_min itself: at or above the minimum airspeed
// there is no mitigation, and it ramps in linearly over a band 10% of tas_min wide.
TEST(TECSUnderspeedTest, NoMitigationAtOrAboveMinimumAirspeed)
{
	TECSControl::Param param = makeParam(); // tas_min = 10, trim = 15
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(20.f, 0.f, param, flag), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatio(12.f, 0.f, param, flag), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatio(10.f, 0.f, param, flag), 0.f); // exactly at tas_min
}

TEST(TECSUnderspeedTest, RampsLinearlyOverBandBelowMinimumAirspeed)
{
	TECSControl::Param param = makeParam(); // band is [9, 10] m/s
	const TECSControl::Flag flag = makeFlag();

	EXPECT_NEAR(underspeedRatio(9.75f, 0.f, param, flag), 0.25f, 1e-5f);
	EXPECT_NEAR(underspeedRatio(9.5f, 0.f, param, flag), 0.5f, 1e-5f);
	EXPECT_NEAR(underspeedRatio(9.25f, 0.f, param, flag), 0.75f, 1e-5f);
}

TEST(TECSUnderspeedTest, SaturatesAtFullMitigationBelowBand)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(9.f, 0.f, param, flag), 1.f); // bottom of the band
	EXPECT_FLOAT_EQ(underspeedRatio(5.f, 0.f, param, flag), 1.f);
	EXPECT_FLOAT_EQ(underspeedRatio(0.f, 0.f, param, flag), 1.f);
}

// The ramp width scales with tas_min rather than being a fixed airspeed offset.
TEST(TECSUnderspeedTest, RampWidthScalesWithMinimumAirspeed)
{
	TECSControl::Param param = makeParam();
	param.tas_min = 20.f; // band is [18, 20] m/s
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(20.f, 0.f, param, flag), 0.f);
	EXPECT_NEAR(underspeedRatio(19.f, 0.f, param, flag), 0.5f, 1e-5f);
	EXPECT_FLOAT_EQ(underspeedRatio(18.f, 0.f, param, flag), 1.f);
}

// Deceleration is extrapolated one second ahead, so mitigation starts while the
// measured airspeed is still above tas_min.
TEST(TECSUnderspeedTest, LookaheadTriggersOnDeceleration)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	// Same airspeed, decelerating: 10.5 - 1.0 * 1 s = 9.5 -> halfway into the band.
	EXPECT_NEAR(underspeedRatio(10.5f, -1.f, param, flag), 0.5f, 1e-5f);

	// A hard deceleration saturates the ratio even well above tas_min.
	EXPECT_FLOAT_EQ(underspeedRatio(11.f, -2.f, param, flag), 1.f);
}

// Acceleration must not be credited: an accelerating aircraft that is currently below
// tas_min still gets the full mitigation its measured airspeed calls for.
TEST(TECSUnderspeedTest, LookaheadIgnoresAcceleration)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	EXPECT_NEAR(underspeedRatio(9.5f, 5.f, param, flag), 0.5f, 1e-5f);
	EXPECT_FLOAT_EQ(underspeedRatio(9.5f, 5.f, param, flag), underspeedRatio(9.5f, 0.f, param, flag));
}

// The lookahead can only predict down to zero airspeed, never below it.
TEST(TECSUnderspeedTest, LookaheadClampsPredictedAirspeedAtZero)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	const float ratio = underspeedRatio(5.f, -50.f, param, flag);
	EXPECT_TRUE(PX4_ISFINITE(ratio));
	EXPECT_FLOAT_EQ(ratio, 1.f);
}

TEST(TECSUnderspeedTest, NoMitigationWhenDetectionDisabled)
{
	TECSControl::Param param = makeParam();

	TECSControl::Flag flag = makeFlag();
	flag.detect_underspeed_enabled = false;
	EXPECT_FLOAT_EQ(underspeedRatio(5.f, -2.f, param, flag), 0.f);

	flag = makeFlag();
	flag.airspeed_enabled = false;
	EXPECT_FLOAT_EQ(underspeedRatio(5.f, -2.f, param, flag), 0.f);
}

namespace
{

// Fixed-wing TECS configured the way the fw controller does, driven at ~50 Hz.
void configureTECS(TECS &tecs)
{
	tecs.enable_airspeed(true);
	tecs.set_detect_underspeed_enabled(true);
	tecs.set_equivalent_airspeed_min(10.f);
	tecs.set_equivalent_airspeed_max(30.f);
	tecs.set_equivalent_airspeed_trim(15.f);
	tecs.set_max_climb_rate(5.f);
	tecs.set_max_sink_rate(5.f);
	tecs.set_min_sink_rate(2.f);
	tecs.set_vertical_accel_limit(10.f);
	tecs.set_altitude_error_time_constant(5.f);
	tecs.set_airspeed_error_time_constant(2.f);
	tecs.set_speed_weight(1.f);
}

void runTECS(TECS &tecs, float altitude, float hgt_setpoint, float equivalent_airspeed, int cycles)
{
	for (int i = 0; i < cycles; i++) {
		tecs.update(0.f /* pitch */, altitude, hgt_setpoint, 15.f /* EAS setpoint */, equivalent_airspeed,
			    1.f /* eas_to_tas */, 0.f /* throttle_min */, 1.f /* throttle_max */, 0.5f /* throttle_trim */,
			    radians(-15.f), radians(15.f), 5.f /* target climbrate */, 5.f /* target sinkrate */,
			    0.f /* speed_deriv_forward */, 0.f /* hgt_rate */);
		usleep(20000); // 20 ms, keeps dt inside [DT_MIN, DT_MAX]
	}
}

} // namespace

// While the aircraft is undersped the altitude reference is pinned to the current
// altitude, so no altitude error accumulates that would command an aggressive pitch-up
// (and a possible secondary stall) once the speed recovers.
TEST(TECSUnderspeedTest, AltitudeReferenceIsHeldAtCurrentAltitudeWhileUndersped)
{
	TECS tecs;
	configureTECS(tecs);

	const float altitude = 100.f;
	const float hgt_setpoint = 200.f; // large standing climb demand

	runTECS(tecs, altitude, hgt_setpoint, 8.f /* well below the 10 m/s minimum */, 25);

	ASSERT_GT(tecs.get_underspeed_ratio(), 0.5f);
	EXPECT_NEAR(tecs.getStatus().altitude_reference, altitude, 1e-3f);
	EXPECT_NEAR(tecs.getStatus().height_rate_reference, 0.f, 1e-3f);
}

// Sanity counterpart: at a healthy airspeed the same climb demand does move the reference.
TEST(TECSUnderspeedTest, AltitudeReferenceTracksSetpointWhenNotUndersped)
{
	TECS tecs;
	configureTECS(tecs);

	const float altitude = 100.f;
	const float hgt_setpoint = 200.f;

	runTECS(tecs, altitude, hgt_setpoint, 20.f, 25);

	ASSERT_FLOAT_EQ(tecs.get_underspeed_ratio(), 0.f);
	EXPECT_GT(tecs.getStatus().altitude_reference, altitude);
}
