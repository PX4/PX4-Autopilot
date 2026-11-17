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
#include <algorithm>
#include <string>
#include <sys/stat.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace
{

// --- Vehicle constants --------------------------------------------------------

static constexpr float ALT_INIT = 300.f;  // initial altitude [m]

// Compile-time default of a firmware parameter.
constexpr float paramDefault(px4::params p)
{
	return px4::parameters[static_cast<size_t>(p)].val.f;
}

// Firmware parameter defaults, mapped the way FwLateralLongitudinalControl feeds them into TECS.
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
// the climb does not absorb. ste_rate_offset is added to what the throttle delivers, unknown to TECS.
float airspeed_rate(const TECSControl::Param &p, float throttle, float pitch_cmd, AircraftState s,
		    float pitch_offset = 0.f, bool throttle_failed = false, float ste_rate_offset = 0.f)
{
	const float ste_rate = ste_rate_modelled(p, throttle_failed ? 0.0f : throttle) + ste_rate_offset;
	const float ske_rate = ste_rate - CONSTANTS_ONE_G * altitude_rate(pitch_cmd, s, pitch_offset);
	return (s.V > 0.1f) ? ske_rate / s.V : 0.f; // If V < 0.1 we have long stalled...
}

AircraftState aircraft_step(const TECSControl::Param &p, float dt, float throttle, float pitch_cmd, AircraftState s,
			    float pitch_offset = 0.f, bool throttle_failed = false, float ste_rate_offset = 0.f)
{
	return {s.V + dt * airspeed_rate(p, throttle, pitch_cmd, s, pitch_offset, throttle_failed, ste_rate_offset),
		s.h + dt * altitude_rate(pitch_cmd, s, pitch_offset)};
}

}

// --- Test fixture -------------------------------------------------------------

class TECSClosedLoopTest : public ::testing::Test
{
protected:
	TECSClosedLoopTest() { _alt_ref_model.initialize({ALT_INIT, 0.f}); }

	float _t{0.f}; // Simulation time [s]

	TECSControl::Param _params{defaultParams()}; // first: the plant and the setpoints derive from it
	AircraftState      _state{_params.equivalent_airspeed_trim, ALT_INIT};
	TECSControl        _tecs{};
	TECSControl::Flag  _flag{.airspeed_enabled = true, .detect_underspeed_enabled = true};
	TECSAltitudeReferenceModel _alt_ref_model{};

	float _target_climbrate{paramDefault(px4::params::FW_T_CLMB_R_SP)};
	float _target_sinkrate {paramDefault(px4::params::FW_T_SINK_R_SP)};

	float _V_sp         {_params.equivalent_airspeed_trim};
	float _alt_sp       {ALT_INIT};
	float _pitch_prev   {0.f};        // pitch command from previous step, for altitude_rate input
	float _throttle_prev{_params.throttle_trim}; // throttle command from previous step, for tas_rate input

	// Options to re-introduce some basic model mismatch
	float _plant_pitch_offset {0.f};  // pitch-to-flight-path offset of the plant, unknown to TECS
	bool _throttle_failed {false};    // simulate failed throttle (giving min total energy rate)
	float _plant_ste_rate_offset {0.f}; // total energy rate the plant adds to the throttle's, unknown to TECS [m^2/s^3]

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
		// Mapped the way FwLateralLongitudinalControl feeds the reference model: the target rates come from
		// FW_T_CLMB_R_SP / FW_T_SINK_R_SP, bounded by the performance limits
		const TECSAltitudeReferenceModel::Param param{
			.target_climbrate = math::min(_target_climbrate, _params.max_climb_rate),
			.target_sinkrate  = math::min(_target_sinkrate, _params.max_sink_rate),
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
			.tas_rate      = airspeed ? airspeed_rate(_params, _throttle_prev, _pitch_prev, _state, _plant_pitch_offset, _throttle_failed,
					_plant_ste_rate_offset) : 0.f,
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

				// Parameterized tests are named "Test/Param": flatten to one file name
				std::string name = test_info->name();
				std::replace(name.begin(), name.end(), '/', '_');

				mkdir(log_dir, 0755); // fails harmlessly if it exists
				std::string path = std::string(log_dir) + "/" + name + ".csv";
				_output_csv_file = std::fopen(path.c_str(), "w");

				if (!_output_csv_file) {
					std::fprintf(stderr, "TECS_TEST_LOG_DIR: cannot write %s\n", path.c_str());
				}

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
			_state      = aircraft_step(_params, DT, _throttle_prev, _pitch_prev, _state, _plant_pitch_offset, _throttle_failed,
						    _plant_ste_rate_offset);
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
// form over the climb rates the throttle can fund and the whole airspeed range.

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
// and airspeed must not drift at all. This exercises that the model and TECS
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
	_params.pitch_speed_weight = 1.5f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSClosedLoopTest, SteadyStateCruiseLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.5f;

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

// --- Altitude step with a speed weight != 1 ----------------------------------
//
// Even with a speed weight != 1, the feedforward should still be consistent
// and not introduce any airspeed errors.

TEST_F(TECSClosedLoopTest, AltitudeStepUpLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.5f;

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
	_params.pitch_speed_weight = 0.5f;

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

// --- Gliding at speed weight 2 ---------------------------------------------------
//
// Speed weight 2 is the glider setting: the pitch flies airspeed only, so it tracks an airspeed setpoint with no
// propulsion at all. A glider has no throttle range (throttle max = min). Its climb angle follows from the total energy
// rate, gamma = (SEB rate + 2 STE rate) / (2 g V), which the pitch must take as the glide (minimum total energy rate),
// not the altitude demand of the setpoint, which no throttle delivers.

TEST_F(TECSClosedLoopTest, GlideSpeedWeightTwoAirspeedStep)
{
	_params.pitch_speed_weight = 2.f;
	_params.throttle_max = _params.throttle_min;
	_throttle_prev = _params.throttle_min;

	run(60.f); // settle into the glide

	setAirspeedSetpoint(_params.equivalent_airspeed_trim + 2.f);
	run(60.f);
	EXPECT_NEAR(_state.V, _V_sp, 0.05f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim - 2.f);
	run(60.f);
	EXPECT_NEAR(_state.V, _V_sp, 0.05f);
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

	// The pitch and throttle decouple airspeed from altitude: the airspeed error decays as a first-order response
	// with the airspeed loop time constant FW_T_TAS_TC, up to a small discretisation error
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

// --- Speed weight with unequal time constants -----------------------------------
//
// With the pitch and throttle consistent, the speed weight only sets which error the pitch feedback weights, not the
// closed loop. The energy rate setpoints are formed in the eigenbasis (SPE, SEB) of the energy error dynamics:
//  - the total energy error decays with FW_T_ALT_TC, the throttle's setpoint is the total energy error over it
//  - an airspeed error decays with FW_T_TAS_TC, clamped to at most FW_T_ALT_TC, where the pitch trades altitude for it
//  - an altitude error with the airspeed at its setpoint decays without exciting an airspeed error
// For FW_T_TAS_TC < FW_T_ALT_TC an airspeed error therefore moves the altitude temporarily (pitch down to accelerate).
// Every weight must match the same weight-free responses, so the speed weight has no effect as long as the throttle
// delivers the total energy rate.

struct WeightAndTimeConstants {
	float speed_weight;
	float altitude_time_const;
	float airspeed_time_const;
};

class TECSClosedLoopWeightTest : public TECSClosedLoopTest, public ::testing::WithParamInterface<WeightAndTimeConstants>
{
protected:
	void SetUp() override
	{
		_params.pitch_speed_weight = GetParam().speed_weight;
		_params.altitude_error_gain = 1.f / GetParam().altitude_time_const;
		_params.airspeed_error_gain = 1.f / GetParam().airspeed_time_const;
	}

	// TECS clamps the airspeed time constant to at most the altitude one
	float airspeedErrorGain() const { return math::max(_params.airspeed_error_gain, _params.altitude_error_gain); }

	// Total energy error, linearised about the airspeed setpoint. For FW_T_TAS_TC = FW_T_ALT_TC it decays exactly
	// exponentially; otherwise TECS forms the kinetic error with the measured airspeed, V (V_sp - V), which leaves a
	// term of second order in the airspeed error.
	float totalEnergyError() const
	{
		return CONSTANTS_ONE_G * (_alt_sp - _state.h) + _V_sp * (_V_sp - _state.V);
	}

	// The total energy error decays as a first-order response with FW_T_ALT_TC. Compared as the altitude equivalent.
	void expectTotalEnergyDecay(float error_initial, float t)
	{
		const float expected_error = error_initial * expf(-t * _params.altitude_error_gain);
		EXPECT_NEAR(totalEnergyError() / CONSTANTS_ONE_G, expected_error / CONSTANTS_ONE_G, MAX_NUMERICAL_ERR)
				<< "at t=" << _t << " s";
	}

	// Below trim the pitch trades altitude for airspeed: the airspeed error decays with the clamped FW_T_TAS_TC and
	// the total energy error with FW_T_ALT_TC.
	void checkAirspeedStep(float step)
	{
		run(10.f);
		setAirspeedSetpoint(_params.equivalent_airspeed_trim + step);
		resetSimStats();

		const float error_initial = _V_sp - _state.V;
		const float energy_error_initial = totalEnergyError();
		const float t_step = _t;

		for (int i = 0; i < 10; i++) {
			run(2.0f);
			const float expected_error = error_initial * expf(-(_t - t_step) * airspeedErrorGain());
			EXPECT_NEAR(_V_sp - _state.V, expected_error, MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
			expectTotalEnergyDecay(energy_error_initial, _t - t_step);
		}
	}

	// Above trim, up to a speed weight of 1, the pitch does not trade altitude for airspeed (trim cap) and the throttle
	// demands only the total energy error over FW_T_ALT_TC: the airspeed error decays with FW_T_ALT_TC and the
	// altitude holds.
	void checkAirspeedStepAboveTrim(float step)
	{
		run(10.f);
		setAirspeedSetpoint(_params.equivalent_airspeed_trim + step);
		resetSimStats();

		const float error_initial = _V_sp - _state.V;
		const float energy_error_initial = totalEnergyError();
		const float t_step = _t;

		for (int i = 0; i < 10; i++) {
			run(2.0f);
			const float expected_error = error_initial * expf(-(_t - t_step) * _params.altitude_error_gain);
			EXPECT_NEAR(_V_sp - _state.V, expected_error, MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
			expectTotalEnergyDecay(energy_error_initial, _t - t_step);
		}

		// Altitude holds to numerical precision
		EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	}

	// Above trim with a weight between 1 and 2 the trim cap is partly faded out: the pitch covers only part of the
	// exchange, and the airspeed error decays between the clamped FW_T_TAS_TC and FW_T_ALT_TC responses.
	void checkAirspeedStepFadedTrimCap(float step)
	{
		run(10.f);
		setAirspeedSetpoint(_params.equivalent_airspeed_trim + step);
		resetSimStats();

		const float error_initial = _V_sp - _state.V;
		const float energy_error_initial = totalEnergyError();
		const float t_step = _t;

		for (int i = 0; i < 10; i++) {
			run(2.0f);
			const float t = _t - t_step;
			const float error_fast = error_initial * expf(-t * airspeedErrorGain());
			const float error_slow = error_initial * expf(-t * _params.altitude_error_gain);
			const float error = _V_sp - _state.V;
			EXPECT_GE(fabsf(error), fabsf(error_fast) - MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
			EXPECT_LE(fabsf(error), fabsf(error_slow) + MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
			expectTotalEnergyDecay(energy_error_initial, t);
		}
	}

	void checkAltitudeStep(float step)
	{
		run(10.f);
		setAltitudeSetpoint(ALT_INIT + step);
		resetSimStats();
		run(200.f);

		// Airspeed holds to numerical precision, the altitude converges
		EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
		EXPECT_NEAR(_state.V, _V_sp, 1e-3f);
		EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
	}

	// Disturb the plant state with the setpoints unchanged, e.g. a gust: the airspeed error decays with the clamped
	// FW_T_TAS_TC and the total energy error with FW_T_ALT_TC, whatever the speed weight.
	void checkDisturbance(float altitude_offset, float airspeed_offset)
	{
		run(10.f);
		_state.h += altitude_offset;
		_state.V += airspeed_offset;
		resetSimStats();

		const float energy_error_initial = totalEnergyError();
		const float t_disturbance = _t;

		for (int i = 0; i < 10; i++) {
			run(2.0f);
			const float t = _t - t_disturbance;
			EXPECT_NEAR(_state.V - _V_sp, airspeed_offset * expf(-t * airspeedErrorGain()), MAX_NUMERICAL_ERR)
					<< "at t=" << _t << " s";
			expectTotalEnergyDecay(energy_error_initial, t);
		}
	}
};

TEST_P(TECSClosedLoopWeightTest, AltitudeDisturbance) { checkDisturbance(2.f, 0.f); }
TEST_P(TECSClosedLoopWeightTest, AirspeedDisturbance) { checkDisturbance(0.f, -1.f); }
TEST_P(TECSClosedLoopWeightTest, CombinedDisturbance) { checkDisturbance(-2.f, 1.f); }
TEST_P(TECSClosedLoopWeightTest, AirspeedStepUp)
{
	// A weight above 1 fades out the trim cap: the pitch trades altitude for airspeed above trim as well
	if (_params.pitch_speed_weight > 1.f) {
		checkAirspeedStepFadedTrimCap(1.f);

	} else {
		checkAirspeedStepAboveTrim(1.f);
	}
}
TEST_P(TECSClosedLoopWeightTest, AirspeedStepDown) { checkAirspeedStep(-1.f); }
TEST_P(TECSClosedLoopWeightTest, AltitudeStepUp) { checkAltitudeStep(50.f); }
TEST_P(TECSClosedLoopWeightTest, AltitudeStepDown) { checkAltitudeStep(-50.f); }

INSTANTIATE_TEST_SUITE_P(SpeedWeightAndTimeConstants, TECSClosedLoopWeightTest, ::testing::Values(
				 WeightAndTimeConstants{0.0f, 5.f, 2.5f},
				 WeightAndTimeConstants{0.0f, 3.f, 6.f},
				 WeightAndTimeConstants{1.0f, 5.f, 2.5f},
				 WeightAndTimeConstants{1.0f, 3.f, 6.f},
				 WeightAndTimeConstants{1.8f, 5.f, 5.f},
				 WeightAndTimeConstants{1.8f, 5.f, 2.5f},
				 WeightAndTimeConstants{1.8f, 3.f, 6.f},
				 WeightAndTimeConstants{0.5f, 5.f, 2.5f},
				 WeightAndTimeConstants{0.5f, 3.f, 6.f}),
			 [](const ::testing::TestParamInfo<WeightAndTimeConstants> &param_info)
{
	auto name = [](float value) { return std::to_string(static_cast<int>(value * 10.f + 0.5f)); };
	return "Weight" + name(param_info.param.speed_weight) + "_AltTc" + name(param_info.param.altitude_time_const)
	       + "_TasTc" + name(param_info.param.airspeed_time_const);
});

// --- Throttle shortfall -------------------------------------------------------------
//
// If the throttle does not deliver the total energy rate setpoint, the speed weight splits the shortfall
// d = STE rate setpoint - STE rate between the altitude and airspeed. The pitch integrator settles where the balance
// rate error 2 (SPE rate setpoint - SPE rate) - w d vanishes, so with constant errors (no climb, no acceleration):
//     g e_h / tau_h = (w / 2) d        altitude rate setpoint
//     V e_V / tau_v = (1 - w / 2) d    kinetic energy rate setpoint
// The two shares add up to the whole shortfall: w = 0 puts it on the airspeed, w = 2 on the altitude.
//
// The plant delivers less total energy rate than TECS models, and the throttle integrator, which would otherwise
// remove the shortfall, is off. The throttle damping then settles with a shortfall left over, like a throttle that
// cannot deliver.

class TECSClosedLoopShortfallTest : public TECSClosedLoopTest, public ::testing::WithParamInterface<float>
{
};

TEST_P(TECSClosedLoopShortfallTest, SpeedWeightSplitsShortfall)
{
	_params.pitch_speed_weight = GetParam();
	_params.integrator_gain_throttle = 0.f;
	_plant_ste_rate_offset = -0.5f * CONSTANTS_ONE_G; // the throttle delivers 0.5 m/s less climb than modelled

	run(100.f); // let the pitch integrator settle

	const auto &dbg = _tecs.getDebugOutput();
	const float shortfall = dbg.total_energy_rate_sp - dbg.total_energy_rate_estimate;
	ASSERT_GT(shortfall, 0.1f); // the throttle is short

	// Steady: no climb, no acceleration. The float plant state stops moving once a step falls below its resolution
	// (at 300 m altitude, about 1 mm/s), so the energy rate and the split below hold to about 1 % of the shortfall.
	const float tolerance = 0.01f * shortfall;
	EXPECT_NEAR(dbg.total_energy_rate_estimate, 0.f, tolerance);

	const float w = _params.pitch_speed_weight;
	const float altitude_error = _alt_sp - _state.h;
	const float airspeed_error = _V_sp - _state.V;
	const float altitude_share = CONSTANTS_ONE_G * altitude_error * _params.altitude_error_gain;
	const float airspeed_share = _state.V * airspeed_error * _params.airspeed_error_gain;

	EXPECT_NEAR(altitude_share, 0.5f * w * shortfall, tolerance);
	EXPECT_NEAR(airspeed_share, (1.f - 0.5f * w) * shortfall, tolerance);
	EXPECT_NEAR(altitude_share + airspeed_share, shortfall, tolerance);
}

INSTANTIATE_TEST_SUITE_P(SpeedWeight, TECSClosedLoopShortfallTest, ::testing::Values(0.f, 0.5f, 1.f, 1.5f, 2.f),
			 [](const ::testing::TestParamInfo<float> &param_info)
{
	return "Weight" + std::to_string(static_cast<int>(param_info.param * 10.f + 0.5f));
});

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

	// Level cruise is held very precisely even with open airspeed loop
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSClosedLoopTest, AirspeedlessAltitudeStepUp)
{
	disableAirspeed();
	run(10.f);
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
	run(10.f);
	setAltitudeSetpoint(ALT_INIT - 50.f);
	resetSimStats();
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_V_AIRSPEEDLESS_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_NEAR(_state.h, _alt_sp, 0.5f);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
}
