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

/**
 * Closed-loop unit tests for TECSControl.
 *
 * TECSControl flies a point-mass plant built from the energy model TECS assumes internally, with perfect state
 * knowledge and no noise:
 *
 *   STE rate = piecewise linear in throttle, with the above/below-trim slopes of the TECS throttle model,
 *              independent of airspeed
 *   h'       = V gamma, gamma = pitch (zero angle of attack, zero wind)
 *   V'       = (STE rate - g h') / V
 *
 * The altitude reference model is bypassed: altitude setpoints are steps small enough to stay clear of all limits.
 */

#include <gtest/gtest.h>
#include "TECS.hpp"
#include <lib/geo/geo.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/param.h>

#include <cmath>
#include <string>

namespace
{

static constexpr float ALT_INIT = 300.f; // [m]
static constexpr float DT = 0.02f; // [s]

constexpr float paramDefault(px4::params p)
{
	return px4::parameters[static_cast<size_t>(p)].val.f;
}

// Firmware parameter defaults, mapped the way FwLateralLongitudinalControl feeds them into TECS, without the pitch
// rate limit
TECSControl::Param defaultParams()
{
	return {
		.max_sink_rate             = paramDefault(px4::params::FW_T_SINK_MAX),
		.min_sink_rate             = paramDefault(px4::params::FW_T_SINK_MIN),
		.max_climb_rate            = paramDefault(px4::params::FW_T_CLMB_MAX),
		.vert_accel_limit          = 1000.f, // no pitch rate limit
		.equivalent_airspeed_trim  = paramDefault(px4::params::FW_AIRSPD_TRIM),
		.tas_min                   = paramDefault(px4::params::FW_AIRSPD_MIN),
		.tas_stall                 = paramDefault(px4::params::FW_AIRSPD_STALL),
		.tas_max                   = paramDefault(px4::params::FW_AIRSPD_MAX),
		.tas_trim                  = paramDefault(px4::params::FW_AIRSPD_TRIM), // EAS = TAS in this plant
		.pitch_max                 = math::radians(paramDefault(px4::params::FW_P_LIM_MAX)),
		.pitch_min                 = math::radians(paramDefault(px4::params::FW_P_LIM_MIN)),
		.level_flight_pitch        = 0.f,
		.throttle_trim             = paramDefault(px4::params::FW_THR_TRIM),
		.throttle_max              = paramDefault(px4::params::FW_THR_MAX),
		.throttle_min              = paramDefault(px4::params::FW_THR_MIN),
		.altitude_error_gain       = 1.f / paramDefault(px4::params::FW_T_ALT_TC),
		.altitude_setpoint_gain_ff = paramDefault(px4::params::FW_T_HRATE_FF),
		.airspeed_error_gain       = 1.f / paramDefault(px4::params::FW_T_TAS_TC),
		.ste_rate_time_const       = paramDefault(px4::params::FW_T_STE_R_TC),
		.seb_rate_ff               = paramDefault(px4::params::FW_T_SEB_R_FF),
		.pitch_speed_weight        = paramDefault(px4::params::FW_T_SPDWEIGHT),
		.integrator_gain_pitch     = paramDefault(px4::params::FW_T_I_GAIN_PIT),
		.pitch_damping_gain        = paramDefault(px4::params::FW_T_PTCH_DAMP),
		.integrator_gain_throttle  = paramDefault(px4::params::FW_T_THR_INTEG),
		.throttle_damping_gain     = paramDefault(px4::params::FW_T_THR_DAMPING),
		.throttle_slewrate         = paramDefault(px4::params::FW_THR_SLEW_MAX),
		.load_factor_correction    = paramDefault(px4::params::FW_T_RLL2THR),
		.load_factor               = 1.f,
		.fast_descend              = 0.f,
	};
}

// Deviations of the plant from the TECS model. The default plant is the TECS model.
struct PlantModel {
	bool throttle_failed{false}; // no thrust: the plant sinks at the minimum sink rate
	float pitch_offset{0.f}; // flight path angle at zero pitch is -pitch_offset [rad]
	float ste_rate_offset{0.f}; // [m²/s³]
};

// Total energy rate of the plant: the TECS throttle model, with the deviations above
float steRatePlant(const TECSControl::Param &p, const PlantModel &plant, float throttle)
{
	if (plant.throttle_failed) {
		return -p.min_sink_rate * CONSTANTS_ONE_G;
	}

	const float ste_rate = (throttle >= p.throttle_trim)
			       ? (throttle - p.throttle_trim) * p.max_climb_rate * CONSTANTS_ONE_G
			       / (p.throttle_max - p.throttle_trim)
			       : (throttle - p.throttle_trim) * p.min_sink_rate * CONSTANTS_ONE_G / (p.throttle_trim - p.throttle_min);
	return ste_rate + plant.ste_rate_offset;
}

}

class TECSControlClosedLoopTest : public ::testing::Test
{
protected:
	TECSControl::Param _params{defaultParams()};
	TECSControl::Flag _flag{.airspeed_enabled = true, .detect_underspeed_enabled = true};
	TECSControl _tecs{};
	PlantModel _plant{};

	float _t{0.f};
	float _V{_params.tas_trim};
	float _h{ALT_INIT};
	float _V_sp{_params.tas_trim};
	float _alt_sp{ALT_INIT};
	float _altitude_rate_sp_direct{NAN};
	float _pitch{0.f};
	float _throttle{_params.throttle_trim};
	bool _initialised{false};

	float altitudeRate() const { return _V * (_pitch - _plant.pitch_offset); }

	float airspeedRate() const
	{
		const float ske_rate = steRatePlant(_params, _plant, _throttle) - CONSTANTS_ONE_G * altitudeRate();
		return (_V > 0.1f) ? ske_rate / _V : 0.f;
	}

	TECSControl::Setpoint makeSetpoint() const
	{
		return {
			.altitude_reference = {.alt = _alt_sp, .alt_rate = 0.f},
			.altitude_rate_setpoint_direct = _altitude_rate_sp_direct,
			.tas_setpoint = _V_sp,
		};
	}

	TECSControl::Input makeInput() const
	{
		// Without an airspeed sensor the TECS airspeed filter falls back to trim airspeed and a zero rate
		const bool airspeed = _flag.airspeed_enabled;
		return {
			.altitude = _h,
			.altitude_rate = altitudeRate(),
			.tas = airspeed ? _V : _params.tas_trim,
			.tas_rate = airspeed ? airspeedRate() : 0.f,
		};
	}

	void step()
	{
		if (!_initialised) {
			_tecs.initialize(makeSetpoint(), makeInput(), _params, _flag);
			_initialised = true;
		}

		_tecs.update(DT, makeSetpoint(), makeInput(), _params, _flag);
		_pitch = _tecs.getPitchSetpoint();
		_throttle = _tecs.getThrottleSetpoint();

		const float V_dot = airspeedRate();
		_h += DT * altitudeRate();
		_V += DT * V_dot;
		_t += DT;
	}

	void run(float duration)
	{
		const int steps = static_cast<int>(duration / DT + 0.5f);

		for (int i = 0; i < steps; i++) {
			step();
		}
	}

	struct MaxErrors {
		float altitude{0.f};
		float airspeed{0.f};
	};

	// Run and return the largest altitude and airspeed errors
	MaxErrors runTrackingMaxErrors(float duration)
	{
		MaxErrors max_errors{};
		const int steps = static_cast<int>(duration / DT + 0.5f);

		for (int i = 0; i < steps; i++) {
			step();
			max_errors.altitude = math::max(max_errors.altitude, fabsf(_alt_sp - _h));
			max_errors.airspeed = math::max(max_errors.airspeed, fabsf(_V_sp - _V));
		}

		return max_errors;
	}
};

// --- Speed weight ---------------------------------------------------------------------------------------
//
// With the pitch feedback gains zero and the plant matching the TECS model, the pitch flies the potential
// energy rate (2 - w) SPE rate sp - w SKE rate sp and the throttle the total energy rate SPE rate sp + SKE rate sp,
// with SPE rate sp = g e_h / tau_h and SKE rate sp = V e_V / tau_v from the altitude and airspeed errors
// e_h = h_sp - h, e_V = V_sp - V. The kinetic energy rate is the remainder (w - 1) SPE rate sp + (1 + w) SKE rate sp.
// Linearised around the airspeed setpoint V0, the errors x = (e_h, e_V) follow x' = A x with
//
//   A = -[ (2 - w) / tau_h,          -w V0 / (g tau_v) ]
//        [ (w - 1) g / (V0 tau_h),   (1 + w) / tau_v   ]
//
// The airspeed alone is a first-order response with time constant tau_v / 2 at w = 1, and coupled to the altitude
// otherwise. The steps are small and below trim airspeed, away from all limits and the pitch airspeed cap.

struct SpeedWeightCase {
	float speed_weight;
	float altitude_time_const;
	float airspeed_time_const;
};

class TECSControlClosedLoopSpeedWeightTest : public TECSControlClosedLoopTest,
	public ::testing::WithParamInterface<SpeedWeightCase>
{
protected:
	static constexpr float AIRSPEED_TOLERANCE = 0.03f; // [m/s] linearisation and discretisation error
	static constexpr float ALTITUDE_TOLERANCE = 0.1f; // [m]

	void SetUp() override
	{
		_params.pitch_speed_weight = GetParam().speed_weight;
		_params.altitude_error_gain = 1.f / GetParam().altitude_time_const;
		_params.airspeed_error_gain = 1.f / GetParam().airspeed_time_const;
		_params.pitch_damping_gain = 0.f;
		_params.integrator_gain_pitch = 0.f;
	}

	// Discrete transition matrix expm(A DT), by its Taylor series (A DT is small)
	matrix::SquareMatrix<float, 2> transitionMatrix(float V0) const
	{
		const float w = _params.pitch_speed_weight;
		const float g = CONSTANTS_ONE_G;
		const float k_h = _params.altitude_error_gain;
		const float k_V = _params.airspeed_error_gain;

		matrix::SquareMatrix<float, 2> A;
		A(0, 0) = -(2.f - w) * k_h;
		A(0, 1) = w * V0 / g * k_V;
		A(1, 0) = -(w - 1.f) * g / V0 * k_h;
		A(1, 1) = -(1.f + w) * k_V;

		matrix::SquareMatrix<float, 2> transition = matrix::eye<float, 2>();
		matrix::SquareMatrix<float, 2> term = matrix::eye<float, 2>();

		for (int k = 1; k < 8; k++) {
			term = term * A * (DT / k);
			transition += term;
		}

		return transition;
	}

	// Change the setpoints from steady flight at trim and compare the errors with the linear model
	void checkStep(float airspeed_step, float altitude_step)
	{
		_V_sp = _V = _params.tas_trim - 1.f;
		run(10.f);

		_V_sp += airspeed_step;
		_alt_sp += altitude_step;

		const matrix::SquareMatrix<float, 2> transition = transitionMatrix(_V_sp);
		matrix::Vector2f x{_alt_sp - _h, _V_sp - _V};

		for (int i = 0; i < static_cast<int>(60.f / DT); i++) {
			step();
			x = transition * x;
			ASSERT_NEAR(_alt_sp - _h, x(0), ALTITUDE_TOLERANCE) << "at t=" << _t;
			ASSERT_NEAR(_V_sp - _V, x(1), AIRSPEED_TOLERANCE) << "at t=" << _t;
		}

		EXPECT_NEAR(_h, _alt_sp, ALTITUDE_TOLERANCE);
		EXPECT_NEAR(_V, _V_sp, AIRSPEED_TOLERANCE);
	}
};

TEST_P(TECSControlClosedLoopSpeedWeightTest, AirspeedStepUp) { checkStep(1.f, 0.f); }
TEST_P(TECSControlClosedLoopSpeedWeightTest, AirspeedStepDown) { checkStep(-1.f, 0.f); }
TEST_P(TECSControlClosedLoopSpeedWeightTest, AltitudeStepUp) { checkStep(0.f, 3.f); }
TEST_P(TECSControlClosedLoopSpeedWeightTest, AltitudeStepDown) { checkStep(0.f, -3.f); }
TEST_P(TECSControlClosedLoopSpeedWeightTest, CombinedStep) { checkStep(1.f, -3.f); }

INSTANTIATE_TEST_SUITE_P(SpeedWeight, TECSControlClosedLoopSpeedWeightTest, ::testing::Values(
				 SpeedWeightCase{0.f, 5.f, 5.f},
				 SpeedWeightCase{0.5f, 5.f, 5.f},
				 SpeedWeightCase{1.f, 5.f, 5.f},
				 SpeedWeightCase{1.5f, 5.f, 5.f},
				 SpeedWeightCase{2.f, 5.f, 5.f},
				 SpeedWeightCase{1.f, 3.f, 6.f},
				 SpeedWeightCase{1.5f, 3.f, 6.f}),
			 [](const ::testing::TestParamInfo<SpeedWeightCase> &param_info)
{
	auto name = [](float value) { return std::to_string(static_cast<int>(value * 10.f + 0.5f)); };
	return "Weight" + name(param_info.param.speed_weight) + "_AltTc" + name(param_info.param.altitude_time_const)
	       + "_TasTc" + name(param_info.param.airspeed_time_const);
});

// At w = 1 the airspeed is a first-order response with time constant tau_v / 2, independent of the altitude
TEST_F(TECSControlClosedLoopTest, AirspeedStepFirstOrderAtUnitSpeedWeight)
{
	_params.pitch_speed_weight = 1.f;
	_params.pitch_damping_gain = 0.f;
	_params.integrator_gain_pitch = 0.f;
	_V_sp = _V = _params.tas_trim - 1.f;
	run(10.f);

	_V_sp += 1.f;
	const float t_step = _t;

	for (int i = 0; i < 10; i++) {
		run(1.f);
		const float expected_error = expf(-2.f * (_t - t_step) * _params.airspeed_error_gain);
		EXPECT_NEAR(_V_sp - _V, expected_error, 0.01f) << "at t=" << _t;
	}
}

// --- Airspeedless ---------------------------------------------------------------------------------------
//
// Without airspeed feedback the airspeed is open loop: it moves where the pitch climbs faster or slower than the
// throttle funds.

class TECSControlClosedLoopAirspeedlessTest : public TECSControlClosedLoopTest
{
protected:
	void SetUp() override
	{
		_flag.airspeed_enabled = false;
		_flag.detect_underspeed_enabled = false;
	}

	void checkAltitudeStep(float altitude_step)
	{
		run(10.f);
		_alt_sp += altitude_step;

		float max_airspeed_error = 0.f;

		for (int i = 0; i < static_cast<int>(60.f / DT); i++) {
			step();
			max_airspeed_error = math::max(max_airspeed_error, fabsf(_V - _V_sp));
		}

		EXPECT_LT(max_airspeed_error, 0.05f);
		EXPECT_NEAR(_h, _alt_sp, 0.1f);
	}
};

TEST_F(TECSControlClosedLoopAirspeedlessTest, AltitudeStepUp) { checkAltitudeStep(5.f); }
TEST_F(TECSControlClosedLoopAirspeedlessTest, AltitudeStepDown) { checkAltitudeStep(-5.f); }

// --- Underspeed and propulsion failure ------------------------------------------------------------------

class TECSControlClosedLoopUnderspeedTest : public TECSControlClosedLoopTest
{
protected:
	void SetUp() override
	{
		_V = 7.5f; // deep in underspeed
	}
};

// The vehicle recovers to its airspeed setpoint without losing more airspeed and with little altitude loss
TEST_F(TECSControlClosedLoopUnderspeedTest, RecoversFromUnderspeed)
{
	float V_min = _V;
	float max_altitude_error = 0.f;

	for (int i = 0; i < static_cast<int>(60.f / DT); i++) {
		step();
		V_min = math::min(V_min, _V);
		max_altitude_error = math::max(max_altitude_error, fabsf(_h - _alt_sp));

		if (i == static_cast<int>(3.f / DT)) {
			EXPECT_GT(_V, _params.tas_min) << "not out of underspeed after 3 s";
		}
	}

	EXPECT_GE(V_min, 7.5f - 0.2f);
	EXPECT_LT(max_altitude_error, 2.f);
	EXPECT_NEAR(_V, _V_sp, 0.1f);
	EXPECT_NEAR(_h, _alt_sp, 0.5f);
}

// Underspeed commands maximum throttle, which gives no thrust: the pitch glides out of full underspeed, never
// climbs while undersped and loses no airspeed
TEST_F(TECSControlClosedLoopUnderspeedTest, GlidesOutOfFullUnderspeedWithPropulsionFailure)
{
	_plant.throttle_failed = true;

	float V_min = _V;
	float max_climb_rate_undersped = -INFINITY;
	float max_underspeed_ratio_settled = 0.f;

	for (int i = 0; i < static_cast<int>(60.f / DT); i++) {
		step();
		V_min = math::min(V_min, _V);

		if (_tecs.getUnderspeedRatio() > 0.f) {
			max_climb_rate_undersped = math::max(max_climb_rate_undersped, altitudeRate());
		}

		if (_t > 5.f) {
			max_underspeed_ratio_settled = math::max(max_underspeed_ratio_settled, _tecs.getUnderspeedRatio());
		}
	}

	EXPECT_GE(V_min, 7.5f - 0.2f);
	EXPECT_LE(max_climb_rate_undersped, 0.f) << "climbs while undersped without thrust";
	EXPECT_LT(max_underspeed_ratio_settled, 1.f) << "not out of full underspeed after 5 s";
}

// --- Airspeed setpoint above trim -----------------------------------------------------------------------
//
// Pitch only trades altitude for airspeed up to trim airspeed, accelerating beyond trim is left to the throttle.

TEST_F(TECSControlClosedLoopTest, AirspeedStepAboveTrimHoldsAltitude)
{
	run(10.f);
	_V_sp = _params.tas_trim + 5.f;
	const MaxErrors max_errors = runTrackingMaxErrors(60.f);

	EXPECT_LT(max_errors.altitude, 0.05f);
	EXPECT_NEAR(_V, _V_sp, 0.05f);
}

// At speed weight 2 the pitch tracks the full airspeed setpoint, also above trim. A glider has no throttle range and
// no thrust, and its altitude setpoint follows it down to stay clear of the altitude rate limits.
TEST_F(TECSControlClosedLoopTest, GliderTracksAirspeedStepAboveTrim)
{
	_params.pitch_speed_weight = 2.f;
	_params.throttle_max = _params.throttle_min;
	_plant.throttle_failed = true;

	for (int i = 0; i < static_cast<int>(60.f / DT); i++) {
		if (i == static_cast<int>(30.f / DT)) {
			_V_sp = _params.tas_trim + 3.f;
		}

		_alt_sp = _h;
		step();
	}

	EXPECT_NEAR(_V, _V_sp, 0.1f);
}

// --- Fast descend ---------------------------------------------------------------------------------------
//
// Fast descend is engaged by TECS, outside TECSControl: here it is set directly.

TEST_F(TECSControlClosedLoopTest, FastDescendFliesMaximumAirspeedAtMinimumThrottle)
{
	run(10.f);
	_params.fast_descend = 1.f;
	_V_sp = _params.tas_max;
	_alt_sp = ALT_INIT - 1000.f;
	run(60.f);

	EXPECT_NEAR(_throttle, _params.throttle_min, 1e-3f);
	EXPECT_NEAR(_V, _V_sp, 0.1f);
	EXPECT_LT(altitudeRate(), 0.f);
}

// Leaving fast descend ramps the ratio down, and with it the airspeed setpoint from maximum to trim airspeed
TEST_F(TECSControlClosedLoopTest, FastDescendExit)
{
	_params.fast_descend = 1.f;
	_V_sp = _params.tas_max;
	_alt_sp = ALT_INIT - 1000.f;
	run(60.f);

	_alt_sp = _h;
	const float ramp_duration = 5.f;
	float max_altitude_error = 0.f;

	for (int i = 0; i < static_cast<int>((ramp_duration + 60.f) / DT); i++) {
		_params.fast_descend = math::max(1.f - i * DT / ramp_duration, 0.f);
		_V_sp = math::lerp(_params.tas_trim, _params.tas_max, _params.fast_descend);
		step();
		max_altitude_error = math::max(max_altitude_error, fabsf(_alt_sp - _h));
	}

	EXPECT_LT(max_altitude_error, 2.5f);
	EXPECT_NEAR(_h, _alt_sp, 0.2f);
	EXPECT_NEAR(_V, _V_sp, 0.1f);
}

// --- Landing flare --------------------------------------------------------------------------------------
//
// The flare ramps the maximum throttle down to idle, the minimum throttle, at the nominal speed weight. The pitch
// keeps flying the height rate setpoint and bleeds airspeed, as with any throttle shortfall at speed weight 1.

TEST_F(TECSControlClosedLoopTest, FlareAtIdleThrottleTracksHeightRate)
{
	run(10.f);
	_params.throttle_max = _params.throttle_min;
	_V_sp = _params.tas_trim - 2.f;
	_altitude_rate_sp_direct = -0.5f;
	run(1.f);

	EXPECT_NEAR(altitudeRate(), _altitude_rate_sp_direct, 0.2f);
	EXPECT_LT(_V, _params.tas_trim - 0.5f);
}

// --- Direct altitude rate setpoint ----------------------------------------------------------------------

TEST_F(TECSControlClosedLoopTest, DirectAltitudeRateSetpoint)
{
	run(10.f);
	_altitude_rate_sp_direct = 1.f;
	run(10.f);

	EXPECT_NEAR(altitudeRate(), _altitude_rate_sp_direct, 0.01f);
	EXPECT_NEAR(_V, _V_sp, 0.05f);
}

// --- Integrators ----------------------------------------------------------------------------------------

// An angle of attack that the level-flight pitch does not account for is removed by the pitch integrator
TEST_F(TECSControlClosedLoopTest, PitchOffsetIsIntegratedOut)
{
	_plant.pitch_offset = math::radians(2.f);
	run(120.f);

	EXPECT_NEAR(_h, _alt_sp, 0.1f);
	EXPECT_NEAR(_V, _V_sp, 0.05f);
}

// A wrong trim throttle is removed by the throttle integrator
TEST_F(TECSControlClosedLoopTest, ThrottleTrimOffsetIsIntegratedOut)
{
	_plant.ste_rate_offset = -3.f; // more drag than modelled
	run(120.f);

	EXPECT_NEAR(_h, _alt_sp, 0.1f);
	EXPECT_NEAR(_V, _V_sp, 0.05f);
}
