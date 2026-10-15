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
 * perfect state knowledge and no noise. The model is deliberately chosen to be
 * consistent with the energy model used by TECS internally, so that we can find
 * correctness errors in the controller under zero model mismatch. Any air
 * density / weight corrections that would come from the FW performance model
 * are ignored.
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
 */

#include <gtest/gtest.h>
#include "TECS.hpp"
#include <lib/geo/geo.h>
#include <px4_platform_common/param.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace
{

// --- Vehicle constants --------------------------------------------------------

static constexpr float ALT_INIT = 300.f;  // initial altitude [m]

/// Compile-time default of a firmware parameter.
constexpr float paramDefault(px4::params p)
{
	return px4::parameters[static_cast<size_t>(p)].val.f;
}

/// Firmware parameter defaults, mapped the way FwLateralLongitudinalControl feeds them into TECS.
constexpr TECSControl::Param defaultParams()
{
	return {
		.max_sink_rate             = paramDefault(px4::params::FW_T_SINK_MAX),
		.min_sink_rate             = paramDefault(px4::params::FW_T_SINK_MIN),
		.max_climb_rate            = paramDefault(px4::params::FW_T_CLMB_MAX),
		.vert_accel_limit          = paramDefault(px4::params::FW_T_VERT_ACC),
		.equivalent_airspeed_trim  = paramDefault(px4::params::FW_AIRSPD_TRIM),
		.tas_min                   = paramDefault(px4::params::FW_AIRSPD_MIN),
		.tas_stall                 = paramDefault(px4::params::FW_AIRSPD_STALL),
		.tas_max                   = paramDefault(px4::params::FW_AIRSPD_MAX),
		.tas_trim                  = paramDefault(px4::params::FW_AIRSPD_TRIM),  // EAS = CAS = TAS in this sim
		.pitch_max                 = math::radians(paramDefault(px4::params::FW_P_LIM_MAX)),
		.pitch_min                 = math::radians(paramDefault(px4::params::FW_P_LIM_MIN)),
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

static constexpr float DT = 0.02f;  // Simulation time step [s]

// Maximum numerical errors we expect to show up in tracking errors (both speed
// and altitude) which in theory should be 0. Does not need to equal DT, but if
// the only source of error is euler discretisation, then the error should scale
// linearly with DT (first order method).
static constexpr float MAX_NUMERICAL_ERR_PER_DT = 1.0f;
static constexpr float MAX_NUMERICAL_ERR = MAX_NUMERICAL_ERR_PER_DT * DT;

struct AircraftState {
	float V{0.f};
	float h{ALT_INIT};
};

float ste_rate_modelled(const TECSControl::Param &p, float throttle)
{
	// Mirrors the formula in _calcThrottleControlOutput
	const float ste_rate_max     = p.max_climb_rate * CONSTANTS_ONE_G;
	const float ste_rate_min_abs = p.min_sink_rate  * CONSTANTS_ONE_G;
	return (throttle >= p.throttle_trim)
	       ? (throttle - p.throttle_trim) * ste_rate_max     / (p.throttle_max  - p.throttle_trim)
	       : (throttle - p.throttle_trim) * ste_rate_min_abs / (p.throttle_trim - p.throttle_min);
}

// Only correct in a linearised sense (close to level) but matches TECS
float altitude_rate(float pitch_cmd, AircraftState s, float pitch_offset = 0.f)
{
	return s.V * (pitch_cmd - pitch_offset);
}

// Continuous-time airspeed rate of the plant at the given commands: the energy rate the throttle delivers that
// the climb does not absorb
float airspeed_rate(const TECSControl::Param &p, float throttle, float pitch_cmd, AircraftState s,
		    float pitch_offset = 0.f, bool throttle_failed = false)
{
	const float ste_rate = ste_rate_modelled(p, throttle_failed ? 0.0f : throttle);
	const float ske_rate = ste_rate - CONSTANTS_ONE_G * altitude_rate(pitch_cmd, s, pitch_offset);
	return (s.V > 0.1f) ? ske_rate / s.V : 0.f; // If V < 0.1 we have long stalled...
}

AircraftState aircraft_step(const TECSControl::Param &p, float dt, float throttle, float pitch_cmd, AircraftState s,
			    float pitch_offset = 0.f, bool throttle_failed = false)
{
	return {s.V + dt * airspeed_rate(p, throttle, pitch_cmd, s, pitch_offset, throttle_failed),
		s.h + dt * altitude_rate(pitch_cmd, s, pitch_offset)};
}

}

// --- Test fixture -------------------------------------------------------------

class TECSClosedLoopTest : public ::testing::Test
{
protected:
	TECSClosedLoopTest() { _alt_ref_model.initialize({ALT_INIT, 0.f}); }

	float _t{0.f}; /// Simulation time [s]

	TECSControl::Param _params{defaultParams()}; // first: the plant and the setpoints derive from it
	AircraftState      _state{_params.equivalent_airspeed_trim, ALT_INIT};
	TECSControl        _tecs{};
	TECSControl::Flag  _flag{.airspeed_enabled = true, .detect_underspeed_enabled = true};
	TECSAltitudeReferenceModel _alt_ref_model{};

	float _V_sp         {_params.equivalent_airspeed_trim};
	float _alt_sp       {ALT_INIT};
	float _pitch_prev   {0.f};        // pitch command from previous step, for altitude_rate input
	float _throttle_prev{_params.throttle_trim}; // throttle command from previous step, for tas_rate input

	// Options to re-introduce some basic model mismatch
	float _plant_pitch_offset {0.f};  // pitch-to-flight-path offset of the plant, unknown to TECS
	bool _throttle_failed {false};    // simulate failed throttle (giving min total energy rate)

	bool  _tecs_initialised {false};
	FILE *_output_csv_file {nullptr};

	void TearDown() override
	{
		if (_output_csv_file) {
			std::fclose(_output_csv_file);
			_output_csv_file = nullptr;
		}
	}

	struct SimulationStats {
		float max_airspeed_error{0.f};
		float max_altitude_error{0.f};
		float t_max_airspeed_error{0.f}; // simulation time of the peak (independent of CSV logging)
		float t_max_altitude_error{0.f};
		float max_throttle_integ_norm{0.f};
		float max_pitch_integ_norm{0.f};
	};

	SimulationStats _sim_stats{};

	void resetSimStats() { _sim_stats = {}; }

	// Back to the initial trim state with the current parameters, the controller re-initialised on the next run().
	// The simulation time runs on, so repeated runs of one test land in one log
	void resetState()
	{
		_state = {_params.equivalent_airspeed_trim, ALT_INIT};
		_tecs = TECSControl{};
		_alt_ref_model = TECSAltitudeReferenceModel{};
		_alt_ref_model.initialize({ALT_INIT, 0.f});
		_V_sp = _params.equivalent_airspeed_trim;
		_alt_sp = ALT_INIT;
		_pitch_prev = 0.f;
		_throttle_prev = _params.throttle_trim;
		_tecs_initialised = false;
		resetSimStats();
	}

	void updateSimStats(const AircraftState &state, const TECSControl::DebugOutput &tecs_debug)
	{
		auto track_max_norm = [this](float & field, float val, float *t_field = nullptr) {
			if (std::fabs(val) > field) {
				field = std::fabs(val);

				if (t_field) { *t_field = _t; }
			}
		};

		track_max_norm(_sim_stats.max_airspeed_error,            state.V - _V_sp,  &_sim_stats.t_max_airspeed_error);
		track_max_norm(_sim_stats.max_altitude_error,            state.h - _alt_sp, &_sim_stats.t_max_altitude_error);

		track_max_norm(_sim_stats.max_throttle_integ_norm,       tecs_debug.throttle_integrator);
		track_max_norm(_sim_stats.max_pitch_integ_norm,          tecs_debug.pitch_integrator);
	}

	// Climb rate the plant is flying at the current state under the previous pitch command
	float plantAltitudeRate() const { return altitude_rate(_pitch_prev, _state, _plant_pitch_offset); }

	void updateAltRef()
	{
		const TECSAltitudeReferenceModel::Param param{
			.target_climbrate = _params.max_climb_rate,
			.target_sinkrate  = _params.max_sink_rate,
			.vert_accel_limit = _params.vert_accel_limit,
			.max_climb_rate   = _params.max_climb_rate,
			.max_sink_rate    = _params.max_sink_rate,
		};
		_alt_ref_model.update(DT, {_alt_sp, NAN}, _state.h, plantAltitudeRate(), param);
	}

	TECSControl::Setpoint makeSetpoint() const
	{
		return {
			.altitude_reference            = _alt_ref_model.getAltitudeReference(),
			.altitude_rate_setpoint_direct = NAN,   // use altitude control loop
			.tas_setpoint                  = _V_sp,
		};
	}

	TECSControl::Input makeInput() const
	{
		// Without an airspeed sensor the TECS airspeed filter falls back to the trim airspeed and a zero
		// airspeed rate, so that is what the controller sees; the plant airspeed is then open loop.
		const bool airspeed = _flag.airspeed_enabled;
		return {
			.altitude      = _state.h,
			.altitude_rate = plantAltitudeRate(),
			.tas           = airspeed ? _state.V : _params.equivalent_airspeed_trim,
			.tas_rate      = airspeed ? airspeed_rate(_params, _throttle_prev, _pitch_prev, _state, _plant_pitch_offset, _throttle_failed) : 0.f,
		};
	}

	// Called in first run() call, only use in tests if reset is explicitly needed in the middle of the run.
	void initTecs()
	{
		_tecs.initialize(makeSetpoint(), makeInput(), _params, _flag);
		_tecs_initialised = true;
	}

	void setAirspeedSetpoint(float v_sp)
	{
		_V_sp = v_sp;
	}

	void setAltitudeSetpoint(float alt_sp)
	{
		_alt_sp = alt_sp;
	}

	// Airspeed sensor failed/absent: TECS flies altitude with pitch and funds it with the throttle feedforward,
	// the airspeed is open loop.
	void disableAirspeed()
	{
		_flag.airspeed_enabled = false;
		_flag.detect_underspeed_enabled = false;
	}

	// Advance simulation for `duration` seconds, updating _sim_stats.
	// If TECS_TEST_LOG_DIR is set in the environment, writes a CSV to
	// $TECS_TEST_LOG_DIR/<test_name>.csv with one row per timestep.
	void run(float duration)
	{
		float t_start = _t;

		if (!_tecs_initialised) {
			initTecs();
		}

		if (!_output_csv_file) {

			const char *log_dir = std::getenv("TECS_TEST_LOG_DIR");

			if (log_dir) {
				const auto *test_info = ::testing::UnitTest::GetInstance()->current_test_info();
				std::string path = std::string(log_dir) + "/" + test_info->name() + ".csv";
				_output_csv_file = std::fopen(path.c_str(), "w");

				if (_output_csv_file) {
					std::fprintf(_output_csv_file,
						     "t,V,h,V_sp,alt_sp,pitch_cmd,throttle,"
						     "pitch_integ,throttle_integ,"
						     "ste_rate_sp,ste_rate_est,seb_rate_sp,seb_rate_est\n");
				}
			}
		}

		while (_t < t_start + duration) {

			updateAltRef();
			_tecs.update(DT, makeSetpoint(), makeInput(), _params, _flag);

			_pitch_prev = _tecs.getPitchSetpoint();
			_throttle_prev = _tecs.getThrottleSetpoint();
			_state      = aircraft_step(_params, DT, _throttle_prev, _pitch_prev, _state, _plant_pitch_offset, _throttle_failed);
			updateSimStats(_state, _tecs.getDebugOutput());

			if (_output_csv_file) {
				const auto &dbg = _tecs.getDebugOutput();
				std::fprintf(_output_csv_file, "%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
					     (double) _t,
					     (double) _state.V, (double) _state.h,
					     (double) _V_sp, (double) _alt_sp,
					     (double) _pitch_prev, (double) _tecs.getThrottleSetpoint(),
					     (double) dbg.pitch_integrator, (double) dbg.throttle_integrator,
					     (double) dbg.total_energy_rate_sp, (double) dbg.total_energy_rate_estimate,
					     (double) dbg.energy_balance_rate_sp, (double) dbg.energy_balance_rate_estimate);
			}

			_t += DT;
		}
	}
};

// --- Feedforward energy consistency ----------------------------------------------
// With zero errors and zero integrators the throttle and pitch feedforward invert the plant model exactly:
// the climb absorbs precisely the energy rate the throttle adds, so the plant airspeed rate is zero in closed
// form over the climb rates the throttle can fund and the whole airspeed range. No integration involved.

TEST_F(TECSClosedLoopTest, FeedforwardIsEnergyConsistent)
{
	_params.altitude_setpoint_gain_ff = 1.f; // the altitude loop passes the reference rate through unchanged

	for (float h_dot = -_params.min_sink_rate; h_dot <= _params.max_climb_rate; h_dot += 0.1f) {
		for (float V = _params.tas_min; V <= _params.tas_max; V += 0.1f) {
			const TECSControl::Setpoint setpoint{
				.altitude_reference = {ALT_INIT, h_dot},
				.altitude_rate_setpoint_direct = NAN, .tas_setpoint = V
			};
			const TECSControl::Input input{.altitude = ALT_INIT, .altitude_rate = h_dot, .tas = V, .tas_rate = 0.f};

			_tecs.initialize(setpoint, input, _params, _flag);

			EXPECT_NEAR(airspeed_rate(_params, _tecs.getThrottleSetpoint(), _tecs.getPitchSetpoint(), {V, ALT_INIT}), 0.f,
				    1e-5f) << "h_dot " << h_dot << " V " << V;
		}
	}
}

// --- Steady-state cruise ------------------------------------------------------
// Starting at perfect trim the feedforward is exact, integrators remain zero,
// and airspeed must not drift at all.  This exercises that the model and TECS
// are self-consistent at the operating point.

TEST_F(TECSClosedLoopTest, SteadyStateCruiseDefaultTuning)
{
	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.2f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpDefaultTuning)
{
	run(60.0f);
	setAltitudeSetpoint(ALT_INIT + 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownDefaultTuning)
{
	run(60.0f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Altitude step with high speed weight ------------------------------------
//
// Even with a speed weight != 1, the feedforward should still be consistent
// and not introduce any airspeed errors.

TEST_F(TECSClosedLoopTest, AltitudeStepUpHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.2f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(200.f);

	// For some reason this requires higher tolerance. TODO fix
	// Disappears with _params.integrator_gain_pitch = 0.f;
	EXPECT_LT(_sim_stats.max_airspeed_error, 2.0f * MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.2f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpLowAccel)
{
	_params.vert_accel_limit /= 10.f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT + 100.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownLowAccel)
{
	_params.vert_accel_limit /= 10.f;

	run(10.f);

	setAltitudeSetpoint(ALT_INIT - 100.f);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	EXPECT_NEAR(_state.V, _V_sp,   1e-3f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Plant pitch-to-flight-path offset (model mismatch) -----------------------
//
// The plant flies level at pitch_cmd = 10 deg instead of 0 (e.g. incorrect FW_PSP_OFF).
// The pitch integrator should converge to this offset.

TEST_F(TECSClosedLoopTest, SteadyStateCruisePitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);

	// Let integrator converge
	run(120.f);

	// Reset stats to caputre errors after convergence
	resetSimStats();
	run(60.f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_airspeed_error, 0.1f) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_NEAR(_tecs.getDebugOutput().pitch_integrator, _plant_pitch_offset, math::radians(0.1f));
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpDefaultTuningPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);

	// Let integrator converge
	run(120.0f);

	// Altitude step & time to converge
	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(120.f);

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_airspeed_error, 0.1f) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_NEAR(_tecs.getDebugOutput().pitch_integrator, _plant_pitch_offset, math::radians(0.1f));
}

TEST_F(TECSClosedLoopTest, AltitudeStepUpHighSpeedWeightPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);
	_params.pitch_speed_weight = 1.8f;

	// Let integrator converge
	run(120.0f);

	// Altitude step & time to converge
	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(120.f);

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_airspeed_error, 0.1f) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_NEAR(_tecs.getDebugOutput().pitch_integrator, _plant_pitch_offset, math::radians(0.1f));
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownHighSpeedWeightPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);
	_params.pitch_speed_weight = 1.8f;

	// Let integrator converge
	run(120.0f);

	// Altitude step & time to converge
	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(120.f);

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, 0.1f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_airspeed_error, 0.1f) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_NEAR(_tecs.getDebugOutput().pitch_integrator, _plant_pitch_offset, math::radians(0.1f));
}

// --- Pitch limit below pitch demand ----------------------------------------

TEST_F(TECSClosedLoopTest, AltitudeStepUpPitchLimited)
{
	// Max pitch so low that it becomes the limiting factor for the achievable climb rate
	_params.pitch_max = math::radians(5.f);
	const float flyable_climb_rate = _params.equivalent_airspeed_trim * _params.pitch_max;

	run(10.f);
	setAltitudeSetpoint(ALT_INIT + 50.f);
	run(15.f);

	// Mid climb: pitch at limit with correspondingly lower climb rate
	EXPECT_NEAR(_pitch_prev, _params.pitch_max, math::radians(0.5f));
	EXPECT_NEAR(plantAltitudeRate(), flyable_climb_rate, 0.1f) << "V " << _state.V << " pitch " << _pitch_prev;

	run(185.f);

	// Still do not allow airspeed errors
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

TEST_F(TECSClosedLoopTest, AltitudeStepDownPitchLimited)
{
	// Min pitch so low that it becomes the limiting factor for the achievable sink rate
	_params.pitch_min = math::radians(-5.f);
	const float flyable_sink_rate = _params.equivalent_airspeed_trim * _params.pitch_min;

	run(10.f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	run(15.f);

	// Mid climb: pitch at limit with correspondingly lower sink rate
	EXPECT_NEAR(_pitch_prev, _params.pitch_min, math::radians(0.5f));
	EXPECT_NEAR(plantAltitudeRate(), flyable_sink_rate, 0.1f) << "V " << _state.V << " pitch " << _pitch_prev;

	run(185.f);

	// Still do not allow airspeed errors
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
}

// --- Propulsion failure --------------------------------------------------------
//
//  - Plant delivers minimum throttle regardless of command
//  - TECS must give up altitude and pitch down to not lose too much airspeed
//
// Underspeed detection ramps in below tas_min (#26859). Its altitude reference reset lives in the TECS wrapper, not
// in TECSControl, so it is not exercised here: with the default speed weight the control law alone settles inside
// the underspeed band and does not regain tas_min within the run. The floors pin what it achieves without the reset.

TEST_F(TECSClosedLoopTest, PropulsionFailureCruise)
{
	run(20.f);
	_throttle_failed = true;
	resetSimStats();
	run(120.f);

	static constexpr float V_MIN = 9.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN) << "V floor " << _V_sp - _sim_stats.max_airspeed_error << " at t=" <<
			_sim_stats.t_max_airspeed_error << " s";
}

TEST_F(TECSClosedLoopTest, PropulsionFailureClimb)
{
	run(20.f);
	setAltitudeSetpoint(ALT_INIT + 200.f);
	run(15.f);
	_throttle_failed = true;
	resetSimStats();
	run(120.f);

	// Lower floor than from cruise: the failure hits with the nose up in the climb
	static constexpr float V_MIN = 8.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN) << "V floor " << _V_sp - _sim_stats.max_airspeed_error << " at t=" <<
			_sim_stats.t_max_airspeed_error << " s";
}

TEST_F(TECSClosedLoopTest, PropulsionFailureCruiseHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(20.f);
	_throttle_failed = true;
	resetSimStats();
	run(120.f);

	// Same floor as with the default weight (the feedforward is the same), but the weighted pitch integrator
	// recovers the airspeed above tas_min by the end of the run.
	static constexpr float V_MIN = 9.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN) << "V floor " << _V_sp - _sim_stats.max_airspeed_error << " at t=" <<
			_sim_stats.t_max_airspeed_error << " s";
	EXPECT_GT(_state.V, _params.tas_min);
}

TEST_F(TECSClosedLoopTest, PropulsionFailureClimbHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.8f;

	run(20.f);
	setAltitudeSetpoint(ALT_INIT + 200.f);
	run(15.f);
	_throttle_failed = true;
	resetSimStats();
	run(120.f);

	static constexpr float V_MIN = 9.0f;
	EXPECT_LT(_sim_stats.max_airspeed_error, _V_sp - V_MIN) << "V floor " << _V_sp - _sim_stats.max_airspeed_error << " at t=" <<
			_sim_stats.t_max_airspeed_error << " s";
	EXPECT_GT(_state.V, _params.tas_min);
}

// --- Airspeed setpoint change -------------------------------------------------

TEST_F(TECSClosedLoopTest, AirspeedStepUpDefaultTuning)
{
	run(10.f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim + 1.f);

	// Above trim, only the throttle accelerates and pitch holds altitude (#29009): the airspeed error decays as
	// a first-order response with the airspeed loop time constant FW_T_TAS_TC, up to a small discretisation error
	const float step = _V_sp - _state.V;
	const float t_step = _t;

	for (int i = 0; i < 10; i++) {
		run(5.0f);
		const float expected_error = step * expf(-(_t - t_step) * _params.airspeed_error_gain);
		EXPECT_NEAR(_V_sp - _state.V, expected_error, MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
	}

	// Altitude holds to numerical precision
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
}

TEST_F(TECSClosedLoopTest, AirspeedStepDownDefaultTuning)
{
	run(10.f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim - 1.f);

	// Capture the rough shape of the airspeed step response. This one
	// is not a nice first order response, because the throttle and pitch
	// loop both interact, including the pitch integrator.
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.3f) << "at t=" << _t << " s";
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.2f) << "at t=" << _t << " s";
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f) << "at t=" << _t << " s";
	run(5.0f); EXPECT_NEAR(_state.V, _V_sp, 0.1f) << "at t=" << _t << " s";
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.05f) << "at t=" << _t << " s";
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.02f) << "at t=" << _t << " s";
	run(10.0f); EXPECT_NEAR(_state.V, _V_sp, 0.01f) << "at t=" << _t << " s";

	// This is possible without sacrificing a lot of altitude
	EXPECT_LT(_sim_stats.max_altitude_error, 0.5f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
}

// Without the pitch integrator the airspeed step below trim is a first-order response too, with both loops acting
// on the airspeed error: the throttle sheds the kinetic energy at 1/tau, the pitch feedforward converts the speed
// weighted share w of it into altitude at w/tau, and the pitch damping gain K sees the throttle's share as a
// balance error and gives 2K/(1+2K) of the conversion back. With the energy-rate units of K this yields
//   d(airspeed error)/dt = -(1 + 2K + w)/(1 + 2K) * airspeed error / tau,
// independent of the altitude loop time constant, which only shapes the altitude bump: with unity feedforward the
// altitude error enters the throttle and the pitch with opposite signs and cancels in the kinetic energy rate.
// With the integrator, the throttle's share winds up instead and is unwound through the altitude loop (slow tail).
TEST_F(TECSClosedLoopTest, AirspeedStepDownNoPitchInteg)
{
	ASSERT_FLOAT_EQ(_params.seb_rate_ff, 1.f);
	_params.integrator_gain_pitch = 0.f;

	for (const float w : {1.8f, 1.f, 0.2f}) {
		_params.pitch_speed_weight = w;
		resetState();
		run(10.f);

		setAirspeedSetpoint(_params.equivalent_airspeed_trim - 1.f);

		const float K = _params.pitch_damping_gain;
		const float tau_eff = (1.f + 2.f * K) / (1.f + 2.f * K + w) / _params.airspeed_error_gain;
		const float step = _V_sp - _state.V;
		const float t_step = _t;

		for (int i = 0; i < 10; i++) {
			run(5.0f);
			const float expected_error = step * expf(-(_t - t_step) / tau_eff);
			EXPECT_NEAR(_V_sp - _state.V, expected_error, MAX_NUMERICAL_ERR) << "speed weight " << w << " at t=" << _t << " s";
		}
	}
}

TEST_F(TECSClosedLoopTest, FastDescend)
{
	// The decision whether or not fast descend runs happens in the outer
	// TECS class, not TECSControl which we test here - so we can only test
	// whether fast descend runs correctly when enabling it, NOT whether the
	// outer logic enables it correctly.

	run(10.0f);

	_params.fast_descend = 1.0f;
	setAirspeedSetpoint(_params.tas_max);
	initTecs(); // reset the controller state on the mode switch, as the firmware does

	// Irrelevant, _params.fast_descend determines whether or not we do fast descend
	setAltitudeSetpoint(ALT_INIT - 200.0f);

	// Run until we are fully in fast descend
	run(60.f);

	// Throttle is forced to minimum during fast descend
	EXPECT_NEAR(_tecs.getThrottleSetpoint(), _params.throttle_min, 1e-3f);
	// Pitch loop drives airspeed toward tas_max
	EXPECT_NEAR(_state.V, _params.tas_max, 0.05f);
	run(60.f);
	EXPECT_NEAR(_state.V, _params.tas_max, 0.005f);
}

// --- Airspeed-less mode ---------------------------------------------------------

// Without airspeed feedback, pitch and throttle are both only driven by altitude feedback.
// Without model mismatch or noise the open-loop airspeed tracking should still be pretty good.

static constexpr float MAX_V_AIRSPEEDLESS_ERR = 0.05f; // [m/s] maximum open-loop airspeed error

TEST_F(TECSClosedLoopTest, AirspeedlessSteadyStateCruise)
{
	disableAirspeed();
	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSClosedLoopTest, AirspeedlessAltitudeStepUp)
{
	disableAirspeed();
	run(20.f);
	setAltitudeSetpoint(ALT_INIT + 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_AIRSPEEDLESS_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
}

TEST_F(TECSClosedLoopTest, AirspeedlessAltitudeStepDown)
{
	disableAirspeed();
	run(20.f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_AIRSPEEDLESS_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
}
