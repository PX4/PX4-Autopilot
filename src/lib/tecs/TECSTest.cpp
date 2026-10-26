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
	param.tas_stall = 7.f;
	param.tas_max = 30.f;
	param.tas_trim = 15.f;
	param.pitch_max = radians(15.f);
	param.pitch_min = radians(-15.f);
	param.level_flight_pitch = 0.f;
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
float underspeedRatio(float tas, TECSControl::Param param, const TECSControl::Flag &flag)
{
	TECSControl control;
	TECSControl::Input input = makeInput();
	input.tas = tas;

	control.initialize(makeSetpoint(), input, param, flag);
	control.update(0.02f, makeSetpoint(), input, param, flag);

	return control.getUnderspeedRatio();
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

// Below the minimum airspeed, the conversion from energy balance rate to pitch must use the minimum airspeed,
// as the pitch integrator does, so the pitch command stays bounded as airspeed drops instead of growing with
// the inverse of the airspeed.
TEST(TECSControlTest, PitchOutputUsesMinimumAirspeedBelowIt)
{
	TECSControl::Param param = makeParam();
	param.pitch_speed_weight = 0.f; // pitch controls height only, so airspeed enters solely through the conversion
	param.pitch_max = 1.5f; // keep both commands clear of the pitch limits
	param.pitch_min = -1.5f;

	TECSControl::Flag flag = makeFlag();
	flag.detect_underspeed_enabled = false; // underspeed handling would shift the weights to speed

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 102.f; // small climb demand

	TECSControl::Input below_min = makeInput();
	below_min.tas = 0.2f * param.tas_min;
	TECSControl::Input at_min = makeInput();
	at_min.tas = param.tas_min;

	TECSControl control_below_min;
	control_below_min.initialize(setpoint, below_min, param, flag);
	TECSControl control_at_min;
	control_at_min.initialize(setpoint, at_min, param, flag);

	EXPECT_GT(control_at_min.getPitchSetpoint(), 0.f);
	EXPECT_FLOAT_EQ(control_below_min.getPitchSetpoint(), control_at_min.getPitchSetpoint());
}

// The reported altitude time constant must be the one the controller uses, including above 100 s.
TEST(TECSTest, ReportsConfiguredAltitudeTimeConstant)
{
	TECS tecs;

	tecs.set_altitude_error_time_constant(200.f);
	EXPECT_FLOAT_EQ(tecs.get_altitude_error_time_constant(), 200.f);

	tecs.set_altitude_error_time_constant(0.f);
	EXPECT_FLOAT_EQ(tecs.get_altitude_error_time_constant(), 0.1f);
}

// A reset with the airspeed sensor disabled and an invalid reading must seed the controller with the filtered
// (trim) airspeed, not the raw reading, so neither the reset nor the updates after it produce non-finite commands.
TEST(TECSTest, ResetWithoutValidAirspeedKeepsCommandsFinite)
{
	TECS tecs;
	tecs.enable_airspeed(false);
	tecs.set_integrator_gain_pitch(0.1f);
	tecs.set_integrator_gain_throttle(0.1f);

	const TECS::Input input{
		.altitude = 100.f,
		.altitude_rate = 0.f,
		.altitude_setpoint = 110.f,
		.altitude_rate_setpoint = NAN,
		.equivalent_airspeed = NAN,
		.speed_deriv_forward = 0.f,
		.equivalent_airspeed_setpoint = 15.f,
		.eas_to_tas = 1.f,
		.throttle_min = 0.f,
		.throttle_max = 1.f,
		.throttle_trim = 0.5f,
		.pitch_min = -0.3f,
		.pitch_max = 0.3f,
		.target_climbrate = 3.f,
		.target_sinkrate = 3.f,
	};

	// The first update resets the controller, the following ones run it with a valid time step.
	for (int i = 0; i < 20; i++) {
		usleep(20000);
		tecs.update(input);
		EXPECT_TRUE(PX4_ISFINITE(tecs.get_pitch_setpoint())) << "update " << i;
		EXPECT_TRUE(PX4_ISFINITE(tecs.get_throttle_setpoint())) << "update " << i;
	}
}

namespace
{

constexpr float kDt = 0.02f;

TECSAltitudeReferenceModel::Param makeReferenceParam()
{
	TECSAltitudeReferenceModel::Param param{};
	param.target_climbrate = 3.f;
	param.target_sinkrate = 3.f;
	param.vert_accel_limit = 5.f;
	param.max_climb_rate = 5.f;
	param.max_sink_rate = 5.f;
	return param;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Airspeed filter
// ---------------------------------------------------------------------------------------------------------------------

TEST(TECSAirspeedFilterTest, ConvergesToMeasurement)
{
	TECSAirspeedFilter filter;
	filter.initialize(10.f, 15.f, true);

	for (int i = 0; i < 500; i++) {
		filter.update(kDt, {.equivalent_airspeed = 20.f, .equivalent_airspeed_rate = 0.f}, {.equivalent_airspeed_trim = 15.f},
			      true);
	}

	EXPECT_NEAR(filter.getState().speed, 20.f, 0.01f);
	EXPECT_NEAR(filter.getState().speed_rate, 0.f, 0.01f);
}

// Without a sensor, the filter must settle on trim airspeed: the airspeed-less controller relies on it for the
// conversion between climb angle and energy balance rate.
TEST(TECSAirspeedFilterTest, FallsBackToTrimWithoutSensor)
{
	TECSAirspeedFilter filter;
	filter.initialize(20.f, 15.f, true);

	for (int i = 0; i < 500; i++) {
		filter.update(kDt, {.equivalent_airspeed = 20.f, .equivalent_airspeed_rate = 0.f}, {.equivalent_airspeed_trim = 15.f},
			      false);
	}

	EXPECT_NEAR(filter.getState().speed, 15.f, 0.01f);
}

TEST(TECSAirspeedFilterTest, InitializesToTrimWithoutValidMeasurement)
{
	TECSAirspeedFilter filter;

	filter.initialize(NAN, 15.f, true);
	EXPECT_FLOAT_EQ(filter.getState().speed, 15.f);

	filter.initialize(20.f, 15.f, false);
	EXPECT_FLOAT_EQ(filter.getState().speed, 15.f);
	EXPECT_FLOAT_EQ(filter.getState().speed_rate, 0.f);
}

TEST(TECSAirspeedFilterTest, NeverEstimatesNegativeAirspeed)
{
	TECSAirspeedFilter filter;
	filter.initialize(1.f, 15.f, true);

	for (int i = 0; i < 200; i++) {
		filter.update(kDt, {.equivalent_airspeed = 0.f, .equivalent_airspeed_rate = -50.f}, {.equivalent_airspeed_trim = 15.f},
			      true);
		ASSERT_GE(filter.getState().speed, 0.f) << "update " << i;
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Altitude reference model
// ---------------------------------------------------------------------------------------------------------------------

// The reference climbs no faster than the target climb rate and reaches the altitude setpoint.
TEST(TECSAltitudeReferenceModelTest, ClimbsAtTargetRateToSetpoint)
{
	const TECSAltitudeReferenceModel::Param param = makeReferenceParam();
	TECSAltitudeReferenceModel model;
	model.initialize({.alt = 100.f, .alt_rate = 0.f});

	float max_rate = 0.f;

	for (int i = 0; i < 3000; i++) {
		model.update(kDt, {.alt = 150.f, .alt_rate = NAN}, 100.f, 0.f, param);
		max_rate = fmaxf(max_rate, model.getAltitudeReference().alt_rate);
	}

	EXPECT_LE(max_rate, param.target_climbrate + 1e-3f);
	EXPECT_NEAR(max_rate, param.target_climbrate, 0.05f);
	EXPECT_NEAR(model.getAltitudeReference().alt, 150.f, 0.01f);
	EXPECT_NEAR(model.getAltitudeReference().alt_rate, 0.f, 0.01f);
	EXPECT_FALSE(PX4_ISFINITE(model.getHeightRateSetpointDirect()));
}

// A height rate setpoint is passed on as a direct, ramped height rate setpoint instead of an altitude reference.
TEST(TECSAltitudeReferenceModelTest, FollowsHeightRateSetpoint)
{
	const TECSAltitudeReferenceModel::Param param = makeReferenceParam();
	TECSAltitudeReferenceModel model;
	model.initialize({.alt = 100.f, .alt_rate = 0.f});

	for (int i = 0; i < 500; i++) {
		model.update(kDt, {.alt = NAN, .alt_rate = 2.f}, 100.f, 0.f, param);
	}

	EXPECT_NEAR(model.getHeightRateSetpointDirect(), 2.f, 0.01f);
}

// Without any setpoint, the reference holds the current altitude.
TEST(TECSAltitudeReferenceModelTest, HoldsCurrentAltitudeWithoutSetpoint)
{
	const TECSAltitudeReferenceModel::Param param = makeReferenceParam();
	TECSAltitudeReferenceModel model;
	model.initialize({.alt = 100.f, .alt_rate = 0.f});

	for (int i = 0; i < 500; i++) {
		model.update(kDt, {.alt = NAN, .alt_rate = NAN}, 120.f, 0.f, param);
	}

	EXPECT_NEAR(model.getAltitudeReference().alt, 120.f, 0.01f);
}

// ---------------------------------------------------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------------------------------------------------

// Initialization always runs the altitude loop: a direct height rate setpoint must not change the initial commands.
TEST(TECSControlTest, InitializeIgnoresDirectHeightRateSetpoint)
{
	const TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl::Setpoint without_direct = makeSetpoint();
	without_direct.altitude_reference.alt = 110.f;
	TECSControl::Setpoint with_direct = without_direct;
	with_direct.altitude_rate_setpoint_direct = 4.f;

	TECSControl control_without;
	control_without.initialize(without_direct, input, param, flag);
	TECSControl control_with;
	control_with.initialize(with_direct, input, param, flag);

	EXPECT_FLOAT_EQ(control_with.getPitchSetpoint(), control_without.getPitchSetpoint());
	EXPECT_FLOAT_EQ(control_with.getThrottleSetpoint(), control_without.getThrottleSetpoint());
	EXPECT_FLOAT_EQ(control_with.getDebugOutput().altitude_rate_control,
			control_without.getDebugOutput().altitude_rate_control);
}

// During updates, a finite direct height rate setpoint replaces the altitude loop.
TEST(TECSControlTest, UpdateUsesDirectHeightRateSetpoint)
{
	const TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_rate_setpoint_direct = 2.f;
	control.update(kDt, setpoint, input, param, flag);

	EXPECT_FLOAT_EQ(control.getDebugOutput().altitude_rate_control, 2.f);
}

// Once the pitch command saturates, the pitch integrator must not wind up further.
TEST(TECSControlTest, PitchIntegratorDoesNotWindUpAtPitchLimit)
{
	TECSControl::Param param = makeParam();
	param.pitch_max = radians(3.f);
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 150.f; // climb demand the pitch limit cannot satisfy

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);

	for (int i = 0; i < 100; i++) {
		control.update(kDt, setpoint, input, param, flag);
	}

	ASSERT_FLOAT_EQ(control.getPitchSetpoint(), param.pitch_max);
	const float integrator_saturated = control.getDebugOutput().pitch_integrator;

	for (int i = 0; i < 500; i++) {
		control.update(kDt, setpoint, input, param, flag);
		EXPECT_LE(control.getDebugOutput().pitch_integrator, integrator_saturated) << "update " << i;
	}
}

// With equal throttle limits the throttle sits at both limits at once, so the throttle integrator must not move in
// either direction, whether the energy demand asks for more throttle (climb) or less (descent).
TEST(TECSControlTest, ThrottleIntegratorHoldsWithEqualThrottleLimits)
{
	TECSControl::Param param = makeParam();
	param.throttle_min = 0.4f;
	param.throttle_max = 0.4f;
	param.throttle_trim = 0.4f;
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	for (const float altitude_setpoint : {150.f, 50.f}) {
		TECSControl::Setpoint setpoint = makeSetpoint();
		setpoint.altitude_reference.alt = altitude_setpoint;

		TECSControl control;
		control.initialize(makeSetpoint(), input, param, flag);

		for (int i = 0; i < 200; i++) {
			control.update(kDt, setpoint, input, param, flag);
			EXPECT_FLOAT_EQ(control.getThrottleSetpoint(), 0.4f);
			EXPECT_FLOAT_EQ(control.getDebugOutput().throttle_integrator, 0.f)
					<< "altitude setpoint " << altitude_setpoint << ", update " << i;
		}
	}
}

TEST(TECSControlTest, ThrottleRespectsSlewRate)
{
	TECSControl::Param param = makeParam();
	param.throttle_slewrate = 0.5f;
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 150.f; // step in demand

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);
	const float max_step = kDt * (param.throttle_max - param.throttle_min) * param.throttle_slewrate;

	float previous = control.getThrottleSetpoint();

	for (int i = 0; i < 100; i++) {
		control.update(kDt, setpoint, input, param, flag);
		EXPECT_LE(fabsf(control.getThrottleSetpoint() - previous), max_step + 1e-6f) << "update " << i;
		previous = control.getThrottleSetpoint();
	}

	EXPECT_GT(previous, param.throttle_trim); // the throttle did move towards the climb demand
}

// The pitch command changes no faster than the vertical acceleration limit allows at the current airspeed.
TEST(TECSControlTest, PitchRespectsVerticalAccelerationLimit)
{
	TECSControl::Param param = makeParam();
	param.vert_accel_limit = 2.f;
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 150.f;

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);
	const float max_step = kDt * param.vert_accel_limit / input.tas;

	float previous = control.getPitchSetpoint();

	for (int i = 0; i < 100; i++) {
		control.update(kDt, setpoint, input, param, flag);
		EXPECT_LE(fabsf(control.getPitchSetpoint() - previous), max_step + 1e-6f) << "update " << i;
		previous = control.getPitchSetpoint();
	}

	EXPECT_GT(previous, 0.f);
}

// Fully engaged fast descend controls airspeed through pitch and gives minimum throttle.
TEST(TECSControlTest, FastDescendCommandsMinimumThrottle)
{
	TECSControl::Param param = makeParam();
	param.throttle_min = 0.1f;
	param.fast_descend = 1.f;
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);

	for (int i = 0; i < 50; i++) {
		control.update(kDt, makeSetpoint(), input, param, flag);
	}

	EXPECT_FLOAT_EQ(control.getThrottleSetpoint(), param.throttle_min);
}

// Far below the minimum airspeed the controller is fully undersped and commands maximum throttle.
TEST(TECSControlTest, UnderspeedCommandsMaximumThrottle)
{
	const TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	TECSControl::Input input = makeInput();
	input.tas = 0.3f * param.tas_min;

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);

	for (int i = 0; i < 50; i++) {
		control.update(kDt, makeSetpoint(), input, param, flag);
	}

	EXPECT_FLOAT_EQ(control.getUnderspeedRatio(), 1.f);
	EXPECT_FLOAT_EQ(control.getThrottleSetpoint(), param.throttle_max);
}

// Without airspeed, there is no underspeed detection and airspeed errors do not reach pitch or throttle.
TEST(TECSControlTest, AirspeedlessIgnoresAirspeed)
{
	const TECSControl::Param param = makeParam();
	TECSControl::Flag flag = makeFlag();
	flag.airspeed_enabled = false;

	TECSControl::Input slow = makeInput();
	slow.tas = 0.3f * param.tas_min;
	TECSControl::Setpoint fast_setpoint = makeSetpoint();
	fast_setpoint.tas_setpoint = 25.f;

	TECSControl control_slow;
	control_slow.initialize(makeSetpoint(), slow, param, flag);
	TECSControl control_reference;
	control_reference.initialize(makeSetpoint(), slow, param, flag);

	for (int i = 0; i < 50; i++) {
		control_slow.update(kDt, fast_setpoint, slow, param, flag);
		control_reference.update(kDt, makeSetpoint(), slow, param, flag);
	}

	EXPECT_FLOAT_EQ(control_slow.getUnderspeedRatio(), 0.f);
	EXPECT_FLOAT_EQ(control_slow.getDebugOutput().true_airspeed_derivative_control, 0.f);
	EXPECT_FLOAT_EQ(control_slow.getPitchSetpoint(), control_reference.getPitchSetpoint());
	EXPECT_FLOAT_EQ(control_slow.getThrottleSetpoint(), control_reference.getThrottleSetpoint());
}

// Below the minimum airspeed, the pitch integrator must integrate exactly as fast as at the minimum airspeed: it uses
// the same floored airspeed as the pitch output.
TEST(TECSControlTest, PitchIntegratorUsesMinimumAirspeedBelowIt)
{
	TECSControl::Param param = makeParam();
	param.pitch_speed_weight = 0.f; // pitch controls height only
	param.pitch_max = 1.5f;
	param.pitch_min = -1.5f;

	TECSControl::Flag flag = makeFlag();
	flag.detect_underspeed_enabled = false;

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 102.f;

	TECSControl::Input below_min = makeInput();
	below_min.tas = 0.2f * param.tas_min;
	TECSControl::Input at_min = makeInput();
	at_min.tas = param.tas_min;

	TECSControl control_below_min;
	control_below_min.initialize(setpoint, below_min, param, flag);
	control_below_min.update(kDt, setpoint, below_min, param, flag);
	TECSControl control_at_min;
	control_at_min.initialize(setpoint, at_min, param, flag);
	control_at_min.update(kDt, setpoint, at_min, param, flag);

	EXPECT_GT(control_at_min.getDebugOutput().pitch_integrator, 0.f);
	EXPECT_FLOAT_EQ(control_below_min.getDebugOutput().pitch_integrator,
			control_at_min.getDebugOutput().pitch_integrator);
}

// A non-finite airspeed must not produce a non-finite pitch command.
TEST(TECSControlTest, NonFiniteAirspeedKeepsPitchFinite)
{
	const TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	TECSControl control;
	control.initialize(makeSetpoint(), makeInput(), param, flag);

	TECSControl::Input input = makeInput();
	input.tas = NAN;

	for (int i = 0; i < 50; i++) {
		control.update(kDt, makeSetpoint(), input, param, flag);
		EXPECT_TRUE(PX4_ISFINITE(control.getPitchSetpoint())) << "update " << i;
	}
}

// Banking raises induced drag: the load factor correction must add throttle compared to level flight.
TEST(TECSControlTest, LoadFactorCorrectionAddsThrottle)
{
	TECSControl::Param level = makeParam();
	level.load_factor_correction = 15.f;
	level.integrator_gain_throttle = 0.f;
	TECSControl::Param banked = level;
	banked.load_factor = 1.f / cosf(radians(30.f));
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl control_level;
	control_level.initialize(makeSetpoint(), input, level, flag);
	TECSControl control_banked;
	control_banked.initialize(makeSetpoint(), input, banked, flag);

	EXPECT_GT(control_banked.getThrottleSetpoint(), control_level.getThrottleSetpoint());
}

// The level-flight pitch is fed forward: it shifts the pitch command by exactly its value while unconstrained.
TEST(TECSControlTest, LevelFlightPitchIsFedForward)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.altitude_reference.alt = 102.f; // small climb demand, well inside the pitch limits

	TECSControl control_without;
	control_without.initialize(setpoint, input, param, flag);

	param.level_flight_pitch = radians(4.f);
	TECSControl control_with;
	control_with.initialize(setpoint, input, param, flag);

	EXPECT_NEAR(control_with.getPitchSetpoint() - control_without.getPitchSetpoint(), radians(4.f), 1e-6f);
}

// Pitch limits are absolute: a level-flight pitch above the maximum pitch must not lift the command past it.
TEST(TECSControlTest, PitchLimitsAreAbsolute)
{
	TECSControl::Param param = makeParam();
	param.level_flight_pitch = radians(10.f);
	param.pitch_max = radians(5.f);
	const TECSControl::Flag flag = makeFlag();
	const TECSControl::Input input = makeInput();

	TECSControl control;
	control.initialize(makeSetpoint(), input, param, flag);
	EXPECT_FLOAT_EQ(control.getPitchSetpoint(), param.pitch_max);

	for (int i = 0; i < 100; i++) {
		control.update(kDt, makeSetpoint(), input, param, flag);
		EXPECT_LE(control.getPitchSetpoint(), param.pitch_max) << "update " << i;
	}

	EXPECT_FLOAT_EQ(control.getPitchSetpoint(), param.pitch_max);
}

// A non-finite demand falls back to the integrator on top of the level-flight pitch, not to the bare integrator.
TEST(TECSControlTest, NonFiniteDemandFallsBackToLevelFlightPitch)
{
	TECSControl::Param param = makeParam();
	param.level_flight_pitch = radians(4.f);
	const TECSControl::Flag flag = makeFlag();

	TECSControl::Setpoint bad_setpoint = makeSetpoint();
	bad_setpoint.tas_setpoint = NAN;

	TECSControl control;
	control.initialize(bad_setpoint, makeInput(), param, flag); // integrator reset to zero

	EXPECT_FLOAT_EQ(control.getPitchSetpoint(), param.level_flight_pitch);
}

// The underspeed ramp is anchored on tas_min itself: at or above the minimum airspeed
// there is no mitigation, and it ramps in linearly over a band 10% of tas_min wide.
TEST(TECSUnderspeedTest, NoMitigationAtOrAboveMinimumAirspeed)
{
	TECSControl::Param param = makeParam(); // tas_min = 10, trim = 15
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(20.f, param, flag), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatio(12.f, param, flag), 0.f);
	EXPECT_FLOAT_EQ(underspeedRatio(10.f, param, flag), 0.f); // exactly at tas_min
}

TEST(TECSUnderspeedTest, RampsLinearlyOverBandBelowMinimumAirspeed)
{
	TECSControl::Param param = makeParam(); // band is [9, 10] m/s
	const TECSControl::Flag flag = makeFlag();

	EXPECT_NEAR(underspeedRatio(9.75f, param, flag), 0.25f, 1e-5f);
	EXPECT_NEAR(underspeedRatio(9.5f, param, flag), 0.5f, 1e-5f);
	EXPECT_NEAR(underspeedRatio(9.25f, param, flag), 0.75f, 1e-5f);
}

TEST(TECSUnderspeedTest, SaturatesAtFullMitigationBelowBand)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(9.f, param, flag), 1.f); // bottom of the band
	EXPECT_FLOAT_EQ(underspeedRatio(5.f, param, flag), 1.f);
	EXPECT_FLOAT_EQ(underspeedRatio(0.f, param, flag), 1.f);
}

// The ramp width scales with tas_min rather than being a fixed airspeed offset.
TEST(TECSUnderspeedTest, RampWidthScalesWithMinimumAirspeed)
{
	TECSControl::Param param = makeParam();
	param.tas_min = 20.f; // band is [18, 20] m/s
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(20.f, param, flag), 0.f);
	EXPECT_NEAR(underspeedRatio(19.f, param, flag), 0.5f, 1e-5f);
	EXPECT_FLOAT_EQ(underspeedRatio(18.f, param, flag), 1.f);
}

// With the stall airspeed close to tas_min the band ends halfway between the two.
TEST(TECSUnderspeedTest, RampEndsHalfwayToStallAirspeed)
{
	TECSControl::Param param = makeParam();
	param.tas_stall = 9.f; // band is [9.5, 10] m/s
	const TECSControl::Flag flag = makeFlag();

	EXPECT_FLOAT_EQ(underspeedRatio(10.f, param, flag), 0.f);
	EXPECT_NEAR(underspeedRatio(9.75f, param, flag), 0.5f, 1e-5f);
	EXPECT_FLOAT_EQ(underspeedRatio(9.5f, param, flag), 1.f);
}

// A stall airspeed at or above tas_min collapses the band to a step at tas_min.
TEST(TECSUnderspeedTest, StallAirspeedAtOrAboveMinimumAirspeed)
{
	TECSControl::Param param = makeParam();
	const TECSControl::Flag flag = makeFlag();

	for (const float tas_stall : {10.f, 12.f}) {
		param.tas_stall = tas_stall;
		EXPECT_FLOAT_EQ(underspeedRatio(10.f, param, flag), 0.f);
		EXPECT_FLOAT_EQ(underspeedRatio(9.99f, param, flag), 1.f);
	}
}

TEST(TECSUnderspeedTest, NoMitigationWhenDetectionDisabled)
{
	TECSControl::Param param = makeParam();

	TECSControl::Flag flag = makeFlag();
	flag.detect_underspeed_enabled = false;
	EXPECT_FLOAT_EQ(underspeedRatio(5.f, param, flag), 0.f);

	flag = makeFlag();
	flag.airspeed_enabled = false;
	EXPECT_FLOAT_EQ(underspeedRatio(5.f, param, flag), 0.f);
}

namespace
{

// Pitch setpoint after holding altitude for a while with the given airspeed, airspeed setpoint and airspeed rate.
float pitchSetpointAtSpeed(float tas, float tas_setpoint, float pitch_speed_weight, float tas_rate = 0.f)
{
	TECSControl control;
	TECSControl::Param param = makeParam();
	param.pitch_speed_weight = pitch_speed_weight;
	const TECSControl::Flag flag = makeFlag();

	TECSControl::Input input = makeInput();
	input.tas = tas;
	input.tas_rate = tas_rate;

	TECSControl::Setpoint setpoint = makeSetpoint();
	setpoint.tas_setpoint = tas_setpoint;

	control.initialize(setpoint, input, param, flag);

	for (int i = 0; i < 100; i++) {
		control.update(0.02f, setpoint, input, param, flag);
	}

	return control.getPitchSetpoint();
}

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

void runTECS(TECS &tecs, float altitude, float hgt_setpoint, float equivalent_airspeed, int cycles,
	     float eas_to_tas = 1.f)
{
	for (int i = 0; i < cycles; i++) {
		tecs.update({
			.altitude = altitude,
			.altitude_rate = 0.f,
			.altitude_setpoint = hgt_setpoint,
			.altitude_rate_setpoint = NAN,
			.equivalent_airspeed = equivalent_airspeed,
			.speed_deriv_forward = 0.f,
			.equivalent_airspeed_setpoint = 15.f,
			.eas_to_tas = eas_to_tas,
			.throttle_min = 0.f,
			.throttle_max = 1.f,
			.throttle_trim = 0.5f,
			.pitch_min = radians(-15.f),
			.pitch_max = radians(15.f),
			.target_climbrate = 5.f,
			.target_sinkrate = 5.f,
		});
		usleep(20000); // 20 ms, keeps dt inside [DT_MIN, DT_MAX]
	}
}

} // namespace

// Above trim airspeed, accelerating towards a higher airspeed setpoint is left to the throttle:
// the pitch loop must hold altitude instead of pitching down.
TEST(TECSControlTest, NoPitchDownToAccelerateAboveTrim)
{
	// WHEN flying above trim airspeed (15 m/s) with a higher airspeed setpoint, at nominal and altitude priority
	const float pitch_nominal_weight = pitchSetpointAtSpeed(20.f, 30.f, 1.f);
	const float pitch_altitude_priority = pitchSetpointAtSpeed(20.f, 30.f, 0.f);

	// THEN the pitch loop holds altitude and does not pitch down
	EXPECT_NEAR(pitch_nominal_weight, 0.f, 1e-4f);
	EXPECT_NEAR(pitch_altitude_priority, 0.f, 1e-4f);
}

// Below trim airspeed, pitch still trades altitude for speed, but only up to trim.
TEST(TECSControlTest, PitchDownToAccelerateBelowTrim)
{
	// WHEN flying below trim airspeed with the setpoint at trim, and with the setpoint above trim
	const float pitch_setpoint_at_trim = pitchSetpointAtSpeed(12.f, 15.f, 1.f);
	const float pitch_setpoint_above_trim = pitchSetpointAtSpeed(12.f, 30.f, 1.f);

	// THEN the pitch loop pitches down, but only as much as needed to reach trim airspeed
	EXPECT_LT(pitch_setpoint_at_trim, 0.f);
	EXPECT_NEAR(pitch_setpoint_above_trim, pitch_setpoint_at_trim, 1e-4f);
}

// With full speed priority (e.g. gliders), pitch is the only way to track the airspeed setpoint.
TEST(TECSControlTest, PitchDownToAccelerateAboveTrimWithSpeedPriority)
{
	// WHEN flying above trim airspeed with a higher airspeed setpoint and full speed priority
	const float pitch = pitchSetpointAtSpeed(20.f, 30.f, 2.f);

	// THEN the pitch loop pitches down to accelerate
	EXPECT_LT(pitch, 0.f);
}

// Between trim airspeed and the setpoint the throttle alone accelerates the vehicle: the pitch loop must not react to
// that acceleration, otherwise it pitches up and turns the throttle's energy into height instead of speed.
TEST(TECSControlTest, NoPitchUpWhileThrottleAcceleratesAboveTrim)
{
	// WHEN accelerating between trim airspeed (15 m/s) and the airspeed setpoint, at nominal and altitude priority
	const float pitch_accelerating = pitchSetpointAtSpeed(18.f, 25.f, 1.f, 1.f);
	const float pitch_steady = pitchSetpointAtSpeed(18.f, 25.f, 1.f, 0.f);

	// THEN the pitch setpoint is the same as without acceleration
	EXPECT_NEAR(pitch_accelerating, pitch_steady, 1e-5f);
}

// Above the airspeed setpoint pitch takes back speed control: an acceleration there is countered with pitch up.
TEST(TECSControlTest, PitchUpOnAccelerationAboveSetpoint)
{
	// WHEN accelerating above the airspeed setpoint
	const float pitch_accelerating = pitchSetpointAtSpeed(22.f, 20.f, 1.f, 1.f);
	const float pitch_steady = pitchSetpointAtSpeed(22.f, 20.f, 1.f, 0.f);

	// THEN the pitch loop pitches up more than without acceleration
	EXPECT_GT(pitch_accelerating, pitch_steady + 1e-3f);
}

// An overspeed above the airspeed setpoint is still corrected with pitch.
TEST(TECSControlTest, PitchUpOnOverspeed)
{
	// WHEN flying faster than the airspeed setpoint
	const float pitch = pitchSetpointAtSpeed(25.f, 20.f, 1.f);

	// THEN the pitch loop pitches up to slow down
	EXPECT_GT(pitch, 0.f);
}

// Between trim airspeed and the setpoint, a speed weight between 1 and 2 must fade the pitch loop's kinetic setpoint
// and estimate equally: accelerating exactly as commanded while holding altitude leaves no energy balance error.
TEST(TECSControlTest, KineticSetpointAndEstimateFadeEquallyAboveTrim)
{
	TECSControl::Param param = makeParam();
	param.pitch_speed_weight = 1.5f;
	TECSControl::Flag flag = makeFlag();
	flag.detect_underspeed_enabled = false;

	TECSControl::Setpoint setpoint = makeSetpoint(); // altitude reference at the current altitude
	setpoint.tas_setpoint = 18.f;

	TECSControl::Input input = makeInput();
	input.tas = 17.f; // between trim (15 m/s) and the setpoint
	input.tas_rate = (setpoint.tas_setpoint - input.tas) * param.airspeed_error_gain; // the commanded acceleration

	TECSControl control;
	control.initialize(setpoint, input, param, flag);

	const TECSControl::DebugOutput &debug = control.getDebugOutput();
	EXPECT_NEAR(debug.energy_balance_rate_sp, debug.energy_balance_rate_estimate, 1e-4f);
}

// The stall airspeed is converted to TAS the same way as the minimum airspeed.
TEST(TECSUnderspeedTest, StallAirspeedIsConvertedToTrueAirspeed)
{
	for (const float eas_to_tas : {1.f, 1.5f}) {
		TECS tecs;
		configureTECS(tecs);
		tecs.set_equivalent_airspeed_stall(9.f); // band is [9.5, 10] m/s EAS

		runTECS(tecs, 100.f, 100.f, 9.75f, 3, eas_to_tas);

		EXPECT_NEAR(tecs.get_underspeed_ratio(), 0.5f, 1e-5f);
	}
}

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
