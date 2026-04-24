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
 * Closed-loop unit tests for TECSControl.
 *
 * A minimal aircraft model is closed-loop coupled to the TECS controller with
 * perfect state knowledge and no noise.  The model is deliberately designed to
 * be energy-consistent with TECS's internal linear throttle-to-STE assumption so
 * that integrators converge to zero at trim and steady-state airspeed errors
 * can be driven to numerical noise levels.
 *
 * Physical model
 * --------------
 *
 *   States:  V (TAS), h (altitude)
 *   Inputs:  throttle, gamma (flight path angle) = pitch angle (zero AoA, zero wind)
 *
 *   d(STE)/dt = piecewise linear in throttle, matching TECS's above/below-trim slopes
 *   h'        = V * gamma
 *   V'        = (STE_rate - g * h') / V     [remainder goes to kinetic energy]
 *
 * At trim (throttle = throttle_trim, gamma = 0) both h' and V' are exactly zero,
 * so the TECS feedforward is exact and integrators start (and stay) at zero.
 */

#include <gtest/gtest.h>
#include "TECS.hpp"
#include <lib/geo/geo.h>

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

using math::constrain;

namespace
{

// --- Vehicle constants --------------------------------------------------------
// Representative small fixed-wing UAV, consistent with the flight data used
// during the TECS analysis that motivated this test suite.

static constexpr float G                = CONSTANTS_ONE_G;
static constexpr float V_TRIM           = 16.f;   ///< TAS at trim [m/s]
static constexpr float THROTTLE_TRIM    = 0.5f;
static constexpr float THROTTLE_MAX     = 1.0f;
static constexpr float THROTTLE_MIN     = 0.05f;
static constexpr float MAX_CLIMB_RATE   = 5.f;    ///< [m/s]
static constexpr float MAX_SINK_RATE    = 5.f;    ///< [m/s]
static constexpr float MIN_SINK_RATE    = 2.f;    ///< [m/s]
static constexpr float VERT_ACCEL_LIMIT = 5.f;    ///< must match param - drives pitch rate limit
static constexpr float ALT_INIT         = 300.f;  ///< initial altitude [m]
static constexpr float STE_RATE_MAX     = MAX_CLIMB_RATE * G;
static constexpr float STE_RATE_MIN_ABS = MIN_SINK_RATE  * G;

// --- Testing constants --------------------------------------------------------

static constexpr float MAX_V_TRACKING_ERR = 0.1; ///< [m/s]

// --- Aircraft model -----------------------------------------------------------
// pitch_cmd drives altitude rate instantly; all pitch-rate limiting comes from
// TECS's vert_accel_limit. STE uses the same piecewise-linear throttle-to-energy
// map as TECS internally (_calcThrottleControlOutput), so the feedforward is
// exact and integrators stay at zero.

struct AircraftState {
	float V{V_TRIM};
	float h{ALT_INIT};
};

AircraftState aircraft_step(float dt, float throttle, float pitch_cmd, AircraftState s)
{
	// Mirrors the throttle_predicted formula in _calcThrottleControlOutput.
	const float ste_rate = (throttle >= THROTTLE_TRIM)
			       ? (throttle - THROTTLE_TRIM) * STE_RATE_MAX     / (THROTTLE_MAX  - THROTTLE_TRIM)
			       : (throttle - THROTTLE_TRIM) * STE_RATE_MIN_ABS / (THROTTLE_TRIM - THROTTLE_MIN);

	const float h_dot    = s.V * pitch_cmd; // Only correct in a linearised sense (close to level)
	const float spe_rate = G * h_dot;
	const float ske_rate = ste_rate - spe_rate;
	const float V_dot    = (s.V > 0.1f) ? ske_rate / s.V : 0.f; // If V < 0.1 we have long stalled...
	return {s.V + dt * V_dot, s.h + dt * h_dot};
}


} // anonymous namespace

// --- Test fixture -------------------------------------------------------------

class TECSClosedLoopTest : public ::testing::Test
{
protected:
	static constexpr float DT = 0.02f;  ///< simulation timestep, 50 Hz [s]

	AircraftState      _state{};
	TECSControl        _tecs{};
	TECSControl::Param _params{};
	TECSControl::Flag  _flag{.airspeed_enabled = true, .detect_underspeed_enabled = true};

	float _V_sp         {V_TRIM};
	float _alt_sp       {ALT_INIT};
	float _alt_ref      {ALT_INIT};   ///< smoothed altitude reference fed to TECS
	float _alt_ref_rate {0.f};
	float _V_prev       {V_TRIM};     ///< for TAS-rate finite-difference
	float _pitch_prev   {0.f};        ///< pitch command from previous step, for altitude_rate input

	struct SimulationStats {
		float max_airspeed_error{0.f};
		float max_altitude_error{0.f};
		float max_throttle_integ_norm{0.f};
		float max_pitch_integ_norm{0.f};
		float max_energy_balance_rate_error{0.f};
		float max_total_energy_rate_error{0.f};
	};

	SimulationStats _sim_stats{};

	void resetSimStats() { _sim_stats = {}; }

	void updateSimStats(const AircraftState &state, const TECSControl::DebugOutput &tecs_debug)
	{
		auto track_max_norm = [](float & field, float val) { field = std::max(field, std::fabs(val)); };

		track_max_norm(_sim_stats.max_airspeed_error,            state.V - _V_sp);
		track_max_norm(_sim_stats.max_altitude_error,            state.h - _alt_sp);
		track_max_norm(_sim_stats.max_energy_balance_rate_error, tecs_debug.energy_balance_rate_sp - tecs_debug.energy_balance_rate_estimate);
		track_max_norm(_sim_stats.max_total_energy_rate_error,   tecs_debug.total_energy_rate_sp   - tecs_debug.total_energy_rate_estimate);

		track_max_norm(_sim_stats.max_throttle_integ_norm,       tecs_debug.throttle_integrator);
		track_max_norm(_sim_stats.max_pitch_integ_norm,          tecs_debug.pitch_integrator);

	}

	void initAircraftState()
	{
		_state        = {};
		_V_prev       = V_TRIM;
		_pitch_prev   = 0.f;
		_alt_ref      = ALT_INIT;
		_alt_ref_rate = 0.f;
	}


	void initParams()
	{
		// TODO get the defaults here if possible?
		_params = {
			.max_sink_rate             = MAX_SINK_RATE,
			.min_sink_rate             = MIN_SINK_RATE,
			.max_climb_rate            = MAX_CLIMB_RATE,
			.vert_accel_limit          = VERT_ACCEL_LIMIT,
			.equivalent_airspeed_trim  = V_TRIM,
			.tas_min                   = 10.f,
			.tas_max                   = 25.f,
			.pitch_max                 = 0.5f,
			.pitch_min                 = -0.5f,
			.throttle_trim             = THROTTLE_TRIM,
			.throttle_max              = THROTTLE_MAX,
			.throttle_min              = THROTTLE_MIN,
			.altitude_error_gain       = 0.2f,
			.altitude_setpoint_gain_ff = 1.f,
			.tas_error_percentage      = 0.15f,
			.airspeed_error_gain       = 0.3f,
			.ste_rate_time_const       = 0.1f,
			.seb_rate_ff               = 1.f,
			.pitch_speed_weight        = 1.f,
			.integrator_gain_pitch     = 0.1f,
			.pitch_damping_gain        = 0.1f,
			.integrator_gain_throttle  = 0.1f,
			.throttle_damping_gain     = 0.1f,
			.throttle_slewrate         = 0.f,
			.load_factor_correction    = 0.f,
			.load_factor               = 1.f,
			.fast_descend              = 0.f,
		};
	}

	// Altitude reference tracks _alt_sp with a first-order approach capped at
	// max climb/sink rate, similar to what TECSAltitudeReferenceModel produces.
	void updateAltRef()
	{
		const float err = _alt_sp - _alt_ref;

		if (fabsf(err) < 1e-4f) {
			_alt_ref      = _alt_sp;
			_alt_ref_rate = 0.f;

		} else {
			_alt_ref_rate = constrain(err * 0.5f, -MAX_SINK_RATE, MAX_CLIMB_RATE);
			_alt_ref     += _alt_ref_rate * DT;
		}
	}

	TECSControl::Setpoint makeSetpoint() const
	{
		return {
			.altitude_reference            = {_alt_ref, _alt_ref_rate},
			.altitude_rate_setpoint_direct = NAN,   // use altitude control loop
			.tas_setpoint                  = _V_sp,
		};
	}

	TECSControl::Input makeInput() const
	{
		return {
			.altitude      = _state.h,
			.altitude_rate = _state.V * _pitch_prev,
			.tas           = _state.V,
			.tas_rate      = (_state.V - _V_prev) / DT,
		};
	}

	// Initialise TECS from current aircraft state with zeroed integrators.
	void initTecs()
	{
		_tecs.initialize(makeSetpoint(), makeInput(), _params, _flag);
	}

	void setAirspeedSetpoint(float v_sp)
	{
		_V_sp = v_sp;
	}

	void setAltitudeSetpoint(float alt_sp)
	{
		_alt_sp = alt_sp;
	}

	// Advance simulation for `duration` seconds, updating _sim_stats.
	// If TECS_TEST_LOG_DIR is set in the environment, writes a CSV to
	// $TECS_TEST_LOG_DIR/<test_name>.csv with one row per timestep.
	void run(float duration)
	{
		const int steps = static_cast<int>(std::lround(duration / DT));

		FILE *csv = nullptr;
		const char *log_dir = std::getenv("TECS_TEST_LOG_DIR");

		if (log_dir) {
			const auto *test_info = ::testing::UnitTest::GetInstance()->current_test_info();
			std::string path = std::string(log_dir) + "/" + test_info->name() + ".csv";
			csv = std::fopen(path.c_str(), "w");

			if (csv) {
				std::fprintf(csv,
					     "t,V,h,V_sp,alt_sp,pitch_cmd,throttle,"
					     "pitch_integ,throttle_integ,"
					     "ste_rate_sp,ste_rate_est,seb_rate_sp,seb_rate_est\n");
			}
		}

		for (int i = 0; i < steps; ++i) {

			updateAltRef();
			_tecs.update(DT, makeSetpoint(), makeInput(), _params, _flag);

			_V_prev     = _state.V;
			_pitch_prev = _tecs.getPitchSetpoint();
			_state      = aircraft_step(DT, _tecs.getThrottleSetpoint(), _pitch_prev, _state);
			updateSimStats(_state, _tecs.getDebugOutput());

			if (csv) {
				const auto &dbg = _tecs.getDebugOutput();
				std::fprintf(csv, "%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
					     (i + 1) * (double) DT,
					     (double) _state.V, (double) _state.h,
					     (double) _V_sp, (double) _alt_sp,
					     (double) _pitch_prev, (double) _tecs.getThrottleSetpoint(),
					     (double) dbg.pitch_integrator, (double) dbg.throttle_integrator,
					     (double) dbg.total_energy_rate_sp, (double) dbg.total_energy_rate_estimate,
					     (double) dbg.energy_balance_rate_sp, (double) dbg.energy_balance_rate_estimate);
			}
		}

		if (csv) { std::fclose(csv); }
	}
};

// --- Steady-state cruise ------------------------------------------------------
// Starting at perfect trim the feedforward is exact, integrators remain zero,
// and airspeed must not drift at all.  This exercises that the model and TECS
// are self-consistent at the operating point.

TEST_F(TECSClosedLoopTest, SteadyStateCruiseDefaultTuning)
{
	initAircraftState();
	initParams();
	initTecs();

	run(60.f);
	EXPECT_LT(_sim_stats.max_airspeed_error, 1e-5f);

	EXPECT_NEAR(_state.V, _V_sp,   1e-5f);
	EXPECT_NEAR(_state.h, _alt_sp, 1e-5f);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseHighSpeedWeight)
{
	initAircraftState();

	initParams();
	_params.pitch_speed_weight = 1.8f;

	initTecs();

	run(60.f);
	EXPECT_LT(_sim_stats.max_airspeed_error, 1e-5f);

	EXPECT_NEAR(_state.V, _V_sp,   1e-5f);
	EXPECT_NEAR(_state.h, _alt_sp, 1e-5f);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseLowSpeedWeight)
{
	initAircraftState();

	initParams();
	_params.pitch_speed_weight = 0.2f;

	initTecs();

	run(60.f);
	EXPECT_LT(_sim_stats.max_airspeed_error, 1e-5f);

	EXPECT_NEAR(_state.V, _V_sp,   1e-5f);
	EXPECT_NEAR(_state.h, _alt_sp, 1e-5f);
}

// --- Altitude steps, default tuning (FW_T_SPDWEIGHT = 1.0) -------------------
// Both speed and altitude are equally weighted; the weighted and unweighted
// feedforward are identical (spe_weight = ske_weight = 1), so these tests pass
// with either the old or new throttle implementation.

TEST_F(TECSClosedLoopTest, AltitudeStepUpDefaultTuning)
{
	initAircraftState();
	initParams();
	initTecs();

	run(60.0f);
	setAltitudeSetpoint(ALT_INIT + 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownDefaultTuning)
{
	initAircraftState();
	initParams();
	initTecs();

	run(60.0f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Altitude step with high speed weight (FW_T_SPDWEIGHT = 1.8) -------------
//
// With FW_T_SPDWEIGHT = 1.8 the pitch loop uses spe_weight = 0.2 and
// ske_weight = 1.8, so only ~20% of the demanded STE goes into altitude gain;
// the pitch demand is gentle.
//
// Without the weighted feedforward fix the throttle feedforward uses the full
// unweighted STE (~STE_rate_max ~49 W/kg during a 50 m step), commanding
// near-maximum throttle.  Pitch barely climbs, so most of the energy surplus
// goes into airspeed -- a ~2-3 m/s transient spike.
//
// With the fix, ste_rate_ff = spe_weight * spe_rate + ske_weight * ske_rate.
// For a pure altitude demand this equals 0.2 * 49 ~= 9.8 W/kg, throttle rises
// to only ~0.6, and the peak airspeed deviation stays below 0.5 m/s.
//
// The EXPECT_LT threshold (1.0 m/s) sits between the two regimes:
// fails without the fix (~2-3 m/s), passes with it (~0.3 m/s).

TEST_F(TECSClosedLoopTest, AltitudeStepUpHighSpeedWeight)
{
	initAircraftState();

	initParams();
	_params.pitch_speed_weight = 1.8f;

	initTecs();

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownHighSpeedWeight)
{
	initAircraftState();

	initParams();
	_params.pitch_speed_weight = 1.8f;

	initTecs();

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpLowSpeedWeight)
{
	initAircraftState();

	initParams();
	_params.pitch_speed_weight = 0.2f;

	initTecs();

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownLowSpeedWeight)
{
	initAircraftState();

	initParams();
	_params.pitch_speed_weight = 0.2f;

	initTecs();

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}



TEST_F(TECSClosedLoopTest, AltitudeStepUpLowAccel)
{
	initAircraftState();

	initParams();
	// 10x smaller vertical accel limit than default
	_params.vert_accel_limit = 0.5f;

	initTecs();

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownLowAccel)
{
	initAircraftState();

	initParams();
	// 10x smaller vertical accel limit than default
	_params.vert_accel_limit = 0.5f;

	initTecs();

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}


// --- Airspeed setpoint change -------------------------------------------------
// Step the airspeed setpoint while holding altitude.  The throttle loop must
// accelerate or decelerate the aircraft and then hold the new speed.

TEST_F(TECSClosedLoopTest, AirspeedStepUpDefaultTuning)
{
	initAircraftState();

	initParams();
	initTecs();

	setAirspeedSetpoint(V_TRIM + 2.f);
	run(60.f);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AirspeedStepDownDefaultTuning)
{
	initAircraftState();

	initParams();
	initTecs();

	setAirspeedSetpoint(V_TRIM - 2.f);
	run(60.f);

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, FastDescend)
{
	// The decision whether or not fast descend runs happens in the outer
	// TECS class, not TECSControl which we test here - so we can only test
	// whether when setting fast descend it runs correctly, NOT whether the
	// outer logic enables it correctly. But for simplicity we isolate the
	// inner TECSControl class in this test.

	initAircraftState();

	initParams();
	_params.fast_descend = 1.0f;

	setAirspeedSetpoint(_params.tas_max);
	initTecs();

	// Irrelevant, _params.fast_descend determines whether or not we do fast descend
	setAltitudeSetpoint(ALT_INIT - 200.0f);

	// Run until we are fully in fast descend
	run(60.f);

	// Throttle is forced to minimum during fast descend
	EXPECT_NEAR(_tecs.getThrottleSetpoint(), _params.throttle_min, 1e-3f);
	// Pitch loop drives airspeed toward tas_max
	EXPECT_NEAR(_state.V, _params.tas_max, MAX_V_TRACKING_ERR);
}

TEST_F(TECSClosedLoopTest, AirspeedDip)
{
	// This captures performance of airspeed recovery _before_ the TECS
	// fixes, to ensure no regressions.

	// With default tuning.
	initAircraftState();
	initParams();
	initTecs();

	// We inject an airspeed disturbance by modifying the state (e.g. sudden
	// wind change, aircraft pushed back)
	_state.V -= 4.0f;

	// Capture the tracking error pretty precisely over the next 20 sec
	// First second: error halves
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 4.0f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 3.0f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 2.5f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 2.0f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 2.0f);

	// First 10 sec: back to 0.1 m/s error
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 1.0f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.5f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f);

	// Error stays below ever after
	resetSimStats();
	run(100.0f);
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
}


TEST_F(TECSClosedLoopTest, AirspeedBump)
{
	// This captures performance of airspeed recovery _before_ the TECS
	// fixes, to ensure no regressions.

	// With default tuning.
	initAircraftState();
	initParams();
	initTecs();

	// We inject an airspeed disturbance by modifying the state (e.g. sudden
	// wind change, aircraft pushed back)
	_state.V += 4.0f;

	// Capture the tracking error pretty precisely over the next 20 sec
	// First second: error decreases slightly
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 4.0f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 4.0f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 3.5f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 3.5f);
	run(0.2f); EXPECT_NEAR(_state.V, _V_sp, 3.0f);

	// First 10 sec: back to 0.1 m/s error
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 2.0f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 1.0f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.5f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.3f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f);
	run(1.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f);

	// Error stays below ever after
	resetSimStats();
	run(100.0f);
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_TRACKING_ERR);
}
