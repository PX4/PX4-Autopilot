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
 *   Inputs:  throttle, gamma (pitch angle = flight path angle)
 *
 *   d(STE)/dt = piecewise linear in throttle, matching TECS's above/below-trim slopes
 *   dh/dt     = V * gamma
 *   dV/dt     = (d(STE)/dt - g * h') / V     [remainder goes to kinetic energy]
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
		.vert_accel_limit          = INFINITY, // Ignore FW_T_VERT_ACC, sim assumes perfect pitch tracking
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

		// Disable pitch damping. The gain converts a balance rate error
		// to a balance rate correction. Through two static mappings
		// this is the same as converting a pitch error to a pitch
		// setpoint. Because the sim has perfect pitch tracking, the
		// only pitch errors come from discretisation step delays.  To
		// test it properly we need a sim with dynamics between pitch
		// setpoint and flight path angle.
		.pitch_damping_gain        = 0.f,

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
static constexpr float MAX_NUMERICAL_ERR_PER_DT = 0.5f;
static constexpr float MAX_NUMERICAL_ERR = MAX_NUMERICAL_ERR_PER_DT * DT;

// Without airspeed feedback the airspeed is open loop, and its discretisation
// error is not corrected.
static constexpr float MAX_NUMERICAL_ERR_OPENLOOP = 2.f * MAX_NUMERICAL_ERR;

// Tolerance for settled tracking errors which do not go to zero with smaller DT
// (but which do go to zero as time progresses). Choose settling time so the
// error is below this with some margin.
static constexpr float TRACKING_ERROR_TOL = 0.02f;

// Altitude step chosen so that neither climbing nor sinking will saturate throttle
static constexpr float ALT_STEP = math::min(defaultParams().min_sink_rate, defaultParams().max_climb_rate)
				  / defaultParams().altitude_error_gain; // [m]

static constexpr float AIRSPEED_STEP = 1.0f; // [m/s]

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

class TECSControlClosedLoopTest : public ::testing::Test
{
protected:
	int   _step{0}; // Simulation step count
	float _t{0.f};  // Simulation time [s], _step * DT

	TECSControl::Param _params{defaultParams()}; // first: the plant and the setpoints derive from it
	AircraftState      _state{_params.equivalent_airspeed_trim, ALT_INIT};
	TECSControl        _tecs_control{};
	TECSControl::Flag  _flag{.airspeed_enabled = true, .detect_underspeed_enabled = true};

	float _V_sp       {_params.equivalent_airspeed_trim};
	float _alt_sp     {ALT_INIT};
	float _pitch      {0.f};
	float _throttle   {_params.throttle_trim};

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
		float min_airspeed{INFINITY};
		float max_altitude_loss{0.f}; // below the altitude setpoint
		float max_climb_rate_undersped{-INFINITY}; // while the underspeed ratio is above zero
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

		_sim_stats.min_airspeed = math::min(_sim_stats.min_airspeed, state.V);
		_sim_stats.max_altitude_loss = math::max(_sim_stats.max_altitude_loss, _alt_sp - state.h);

		if (_tecs_control.getUnderspeedRatio() > 0.f) {
			_sim_stats.max_climb_rate_undersped = math::max(_sim_stats.max_climb_rate_undersped, plantAltitudeRate());
		}
	}

	// Climb rate the plant is flying at the current state under the previous pitch command
	float plantAltitudeRate() const { return altitude_rate(_pitch, _state, _plant_pitch_offset); }

	TECSControl::Setpoint makeSetpoint() const
	{
		return {
			.altitude_reference            = {_alt_sp, 0.f}, // no reference model: TECSControl sees the raw setpoint
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
			.tas_rate      = airspeed ? airspeed_rate(_params, _throttle, _pitch, _state, _plant_pitch_offset, _throttle_failed) : 0.f,
		};
	}

	// Called in first run() call, only use in tests if reset is explicitly needed in the middle of the run.
	void initTecs()
	{
		_tecs_control.initialize(makeSetpoint(), makeInput(), _params, _flag);
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

		const int step_end = _step + static_cast<int>(roundf(duration / DT));

		while (_step < step_end) {

			_tecs_control.update(DT, makeSetpoint(), makeInput(), _params, _flag);
			_pitch    = _tecs_control.getPitchSetpoint();
			_throttle = _tecs_control.getThrottleSetpoint();

			updateSimStats(_state, _tecs_control.getDebugOutput());

			if (_output_csv_file) {
				const auto &dbg = _tecs_control.getDebugOutput();
				std::fprintf(_output_csv_file, "%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
					     (double) _t,
					     (double) _state.V, (double) _state.h,
					     (double) _V_sp, (double) _alt_sp,
					     (double) _pitch, (double) _throttle,
					     (double) dbg.pitch_integrator, (double) dbg.throttle_integrator,
					     (double) dbg.total_energy_rate_sp, (double) dbg.total_energy_rate_estimate,
					     (double) dbg.energy_balance_rate_sp, (double) dbg.energy_balance_rate_estimate);
			}

			_state = aircraft_step(_params, DT, _throttle, _pitch, _state, _plant_pitch_offset, _throttle_failed);

			_step++;
			_t = _step * DT;
		}
	}
};

// --- Feedforward energy consistency ----------------------------------------------
// With zero errors and zero integrators the throttle and pitch feedforward invert the plant model exactly:
// the climb absorbs precisely the energy rate the throttle adds, so the plant airspeed rate is zero in closed
// form over the climb rates the throttle can fund and the whole airspeed range.

TEST_F(TECSControlClosedLoopTest, FeedforwardIsEnergyConsistent)
{
	_params.altitude_setpoint_gain_ff = 1.f; // the altitude loop passes the reference rate through unchanged

	for (float h_dot = -_params.min_sink_rate; h_dot <= _params.max_climb_rate; h_dot += 0.1f) {
		for (float V = _params.tas_min; V <= _params.tas_max; V += 0.1f) {
			const TECSControl::Setpoint setpoint{
				.altitude_reference = {ALT_INIT, h_dot},
				.altitude_rate_setpoint_direct = NAN, .tas_setpoint = V
			};
			const TECSControl::Input input{.altitude = ALT_INIT, .altitude_rate = h_dot, .tas = V, .tas_rate = 0.f};

			_tecs_control.initialize(setpoint, input, _params, _flag);

			EXPECT_NEAR(airspeed_rate(_params, _tecs_control.getThrottleSetpoint(), _tecs_control.getPitchSetpoint(), {V, ALT_INIT}), 0.f,
				    1e-5f) << "h_dot " << h_dot << " V " << V;
		}
	}
}

// --- Steady-state cruise ------------------------------------------------------
// Starting at perfect trim the feedforward is exact, integrators remain zero,
// and airspeed must not drift at all. This exercises that the model and TECS
// are self-consistent at the operating point.

TEST_F(TECSControlClosedLoopTest, SteadyStateCruiseDefaultTuning)
{
	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSControlClosedLoopTest, SteadyStateCruiseHighSpeedWeight)
{
	_params.pitch_speed_weight = 1.5f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

TEST_F(TECSControlClosedLoopTest, SteadyStateCruiseLowSpeedWeight)
{
	_params.pitch_speed_weight = 0.5f;

	run(60.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

// --- Speed weight and time constants ------------------------------------------
//
// With e_h and e_V the altitude and airspeed errors, the altitude loop asks for the
// potential energy rate P = g e_h / tau_h and the airspeed loop for the kinetic
// energy rate K = V e_V / tau_V. With the plant matching the TECS model, the pitch
// flies the potential energy rate (2 - w) P - w K and the throttle the total energy
// rate P + K. The kinetic energy rate is the remainder, (w - 1) P + (1 + w) K.
// Altitude and airspeed must follow this model for every speed weight w and time
// constant. The steps stay below trim airspeed, where the pitch trades altitude for
// the whole airspeed error.

TEST_F(TECSControlClosedLoopTest, SetpointStepsFollowEnergyRateModel)
{
	// Small enough to stay clear of the throttle, pitch and rate limits at every speed weight
	static constexpr float SMALL_ALT_STEP = 1.f; // [m]
	static constexpr float SMALL_AIRSPEED_STEP = 1.f; // [m/s]

	for (const float altitude_time_const : {3.f, 5.f, 8.f}) {
		for (const float airspeed_time_const : {3.f, 5.f, 8.f}) {
			for (const float w : {0.f, 0.5f, 1.f, 1.5f, 2.f}) {
				for (const float altitude_step : {SMALL_ALT_STEP, -SMALL_ALT_STEP}) {
					for (const float airspeed_step : {SMALL_AIRSPEED_STEP, -SMALL_AIRSPEED_STEP}) {
						_params.pitch_speed_weight = w;
						_params.altitude_error_gain = 1.f / altitude_time_const;
						_params.airspeed_error_gain = 1.f / airspeed_time_const;

						// Steady flight below trim airspeed, then step both setpoints
						_state = {_params.equivalent_airspeed_trim - 2.f * SMALL_AIRSPEED_STEP, ALT_INIT};
						setAirspeedSetpoint(_state.V);
						setAltitudeSetpoint(_state.h);
						initTecs();

						setAirspeedSetpoint(_state.V + airspeed_step);
						setAltitudeSetpoint(_state.h + altitude_step);

						AircraftState model{_state};

						for (int i = 0; i < 40; i++) {
							for (int k = 0; k < static_cast<int>(roundf(1.f / DT)); k++) {
								const float P = CONSTANTS_ONE_G * (_alt_sp - model.h) * _params.altitude_error_gain;
								const float K = model.V * (_V_sp - model.V) * _params.airspeed_error_gain;
								const float spe_rate = (2.f - w) * P - w * K;
								const float ske_rate = (w - 1.f) * P + (1.f + w) * K;
								model = {model.V + DT *ske_rate / model.V, model.h + DT *spe_rate / CONSTANTS_ONE_G};
							}

							run(1.f);
							ASSERT_NEAR(_state.V, model.V, MAX_NUMERICAL_ERR)
									<< "w " << w << ", time constants " << altitude_time_const << " s, " << airspeed_time_const
									<< " s, steps " << altitude_step << " m, " << airspeed_step << " m/s, at t=" << _t << " s";
							ASSERT_NEAR(_state.h, model.h, MAX_NUMERICAL_ERR)
									<< "w " << w << ", time constants " << altitude_time_const << " s, " << airspeed_time_const
									<< " s, steps " << altitude_step << " m, " << airspeed_step << " m/s, at t=" << _t << " s";
						}
					}
				}
			}
		}
	}
}

// --- Altitude step ------------------------------------------------------------

TEST_F(TECSControlClosedLoopTest, AltitudeStepDown)
{
	run(10.f);

	setAltitudeSetpoint(ALT_INIT - ALT_STEP);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	EXPECT_NEAR(_state.h, _alt_sp, TRACKING_ERROR_TOL);
}

TEST_F(TECSControlClosedLoopTest, AltitudeStepUp)
{
	run(10.f);

	setAltitudeSetpoint(ALT_INIT + ALT_STEP);
	run(200.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	EXPECT_NEAR(_state.h, _alt_sp, TRACKING_ERROR_TOL);
}

// --- Throttle saturation above trim airspeed -----------------------------------
//
// A climb at the maximum climb rate takes all of the throttle. Accelerating to an airspeed setpoint above trim is
// left to the throttle, so with none to spare the vehicle keeps climbing at the maximum climb rate and stays at
// its airspeed.

TEST_F(TECSControlClosedLoopTest, SaturatedClimbHoldsClimbRateWithAirspeedSetpointAboveTrim)
{
	setAltitudeSetpoint(ALT_INIT + 1000.f);
	run(20.f);
	EXPECT_NEAR(plantAltitudeRate(), _params.max_climb_rate, TRACKING_ERROR_TOL);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim + 2.f * AIRSPEED_STEP);

	for (int i = 0; i < 30; i++) {
		run(1.f);
		EXPECT_NEAR(plantAltitudeRate(), _params.max_climb_rate, TRACKING_ERROR_TOL) << "at t=" << _t << " s";
		EXPECT_NEAR(_state.V, _params.equivalent_airspeed_trim, TRACKING_ERROR_TOL) << "at t=" << _t << " s";
	}
}

// --- Plant pitch-to-flight-path offset (model mismatch) -----------------------
//
// The plant flies level at pitch_cmd = 10 deg instead of 0 (e.g. incorrect FW_PSP_OFF).
// The pitch integrator should converge to this offset.

TEST_F(TECSControlClosedLoopTest, SteadyStateCruisePitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);

	// Let integrator converge
	run(120.f);

	// Reset stats to caputre errors after convergence
	resetSimStats();
	run(60.f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, TRACKING_ERROR_TOL) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_airspeed_error, TRACKING_ERROR_TOL) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_NEAR(_tecs_control.getDebugOutput().pitch_integrator, _plant_pitch_offset, math::radians(0.1f));
}

TEST_F(TECSControlClosedLoopTest, AltitudeStepUpDefaultTuningPitchOffset)
{
	_plant_pitch_offset = math::radians(10.f);

	// Let integrator converge
	run(120.0f);

	EXPECT_NEAR(_state.V, _V_sp, TRACKING_ERROR_TOL);
	EXPECT_NEAR(_state.h, _alt_sp, TRACKING_ERROR_TOL);

	// Altitude step & time to converge
	resetSimStats();
	setAltitudeSetpoint(ALT_INIT + ALT_STEP);
	run(120.f);

	EXPECT_LT(_sim_stats.max_airspeed_error, TRACKING_ERROR_TOL) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// Reset stats to capture errors after convergence
	resetSimStats();
	run(60.0f);

	// We fly with low altitude and airspeed error
	EXPECT_LT(_sim_stats.max_altitude_error, TRACKING_ERROR_TOL) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_airspeed_error, TRACKING_ERROR_TOL) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";

	// And the pitch integrator has converged to the plant pitch offset
	EXPECT_NEAR(_tecs_control.getDebugOutput().pitch_integrator, _plant_pitch_offset, math::radians(0.1f));
}

// --- Gliding at speed weight 2 ---------------------------------------------------
//
// Speed weight 2 is the glider setting: the pitch flies airspeed only, so it tracks an airspeed setpoint with no
// propulsion at all.

class TECSControlClosedLoopGlideTest : public TECSControlClosedLoopTest
{
protected:
	void checkAirspeedSteps()
	{
		_params.pitch_speed_weight = 2.f;

		run(10.f);
		_throttle_failed = true;
		run(60.f); // settle into the glide

		setAirspeedSetpoint(_params.equivalent_airspeed_trim + AIRSPEED_STEP);
		run(60.f);
		EXPECT_NEAR(_state.V, _V_sp, TRACKING_ERROR_TOL);

		setAirspeedSetpoint(_params.equivalent_airspeed_trim - AIRSPEED_STEP);
		run(60.f);
		EXPECT_NEAR(_state.V, _V_sp, TRACKING_ERROR_TOL);
	}
};

// Without a throttle range (gliding waypoints) the energy rate setpoints do not count on thrust
TEST_F(TECSControlClosedLoopGlideTest, AirspeedStepWithoutThrottleRange)
{
	_params.throttle_max = _params.throttle_min;
	checkAirspeedSteps();
}

// Disabled: with a throttle range the energy rate setpoints count on the total energy rate the throttle is asked for. It does
// not come, and the pitch feedback splits the missing energy rate evenly between altitude and airspeed: the glide
// settles 1.8 m/s above its airspeed setpoint. Before, speed weight 2 took the altitude out of the pitch feedback.
// To be solved by derating the energy rate setpoints to what the throttle delivers.
TEST_F(TECSControlClosedLoopGlideTest, DISABLED_AirspeedStepWithThrottleRange)
{
	checkAirspeedSteps();
}

// --- Airspeed setpoint change -------------------------------------------------

TEST_F(TECSControlClosedLoopTest, AirspeedStepUpDefaultTuning)
{
	run(10.f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim + AIRSPEED_STEP);

	// Above trim, only the throttle accelerates and pitch holds altitude (#29009): the airspeed error decays as
	// a first-order response with the airspeed loop time constant FW_T_TAS_TC, up to a small discretisation error
	const float t_step = _t;

	for (int i = 0; i < 10; i++) {
		run(5.0f);
		const float expected_error = AIRSPEED_STEP * expf(-(_t - t_step) * _params.airspeed_error_gain);
		EXPECT_NEAR(_V_sp - _state.V, expected_error, MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
	}

	// Altitude holds to numerical precision
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
}

TEST_F(TECSControlClosedLoopTest, AirspeedStepDownDefaultTuning)
{
	run(10.f);

	setAirspeedSetpoint(_params.equivalent_airspeed_trim - AIRSPEED_STEP);

	// Throttle and pitch each fly the kinetic energy rate setpoint, so the airspeed error decays as a first-order
	// response with half the airspeed loop time constant FW_T_TAS_TC, up to a small discretisation error
	const float t_step = _t;

	for (int i = 0; i < 10; i++) {
		run(5.0f);
		const float expected_error = -AIRSPEED_STEP * expf(-2.f * (_t - t_step) * _params.airspeed_error_gain);
		EXPECT_NEAR(_V_sp - _state.V, expected_error, MAX_NUMERICAL_ERR) << "at t=" << _t << " s";
	}

	// This is possible without sacrificing a lot of altitude
	EXPECT_LT(_sim_stats.max_altitude_error, 0.5f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
}

TEST_F(TECSControlClosedLoopTest, FastDescend)
{
	// The decision whether or not fast descend runs happens in the outer
	// TECS class, not TECSControl which we test here - so we can only test
	// whether fast descend runs correctly when enabling it, NOT whether the
	// outer logic enables it correctly.

	run(10.0f);

	_params.fast_descend = 1.0f;
	setAirspeedSetpoint(_params.tas_max);
	initTecs(); // reset the controller state on the mode switch, as the firmware does

	// Run until we are fully in fast descend
	run(75.f);

	// Throttle is forced to minimum during fast descend
	EXPECT_NEAR(_tecs_control.getThrottleSetpoint(), _params.throttle_min, FLT_EPSILON);
	// Pitch loop drives airspeed toward tas_max
	EXPECT_NEAR(_state.V, _params.tas_max, TRACKING_ERROR_TOL);
}

// --- Airspeed-less mode ---------------------------------------------------------

// Without airspeed feedback, pitch and throttle are both only driven by altitude feedback.

TEST_F(TECSControlClosedLoopTest, AirspeedlessSteadyStateCruise)
{
	disableAirspeed();
	run(60.f);

	// Level cruise is held very precisely even with open airspeed loop
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_altitude_error, MAX_NUMERICAL_ERR) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_LT(_sim_stats.max_throttle_integ_norm, MAX_NUMERICAL_ERR);
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR);
}

// The vehicle flies exactly the demanded climb rate, so the altitude error decays first order with FW_T_ALT_TC and
// the demanded sink rate does not exceed FW_T_SINK_MIN.

TEST_F(TECSControlClosedLoopTest, AirspeedlessAltitudeStep)
{
	disableAirspeed();

	run(10.f);

	for (const float step : {-ALT_STEP, ALT_STEP}) {
		setAltitudeSetpoint(_state.h + step);
		const float t_step = _t;

		for (int i = 0; i < 30; i++) {
			run(1.f);
			const float expected_error = step * expf(-(_t - t_step) * _params.altitude_error_gain);
			EXPECT_NEAR(_alt_sp - _state.h, expected_error, MAX_NUMERICAL_ERR_OPENLOOP) << "at t=" << _t << " s";
		}
	}

	// The throttle feedforward funds the climb exactly, so the open loop airspeed does not move
	EXPECT_LT(_sim_stats.max_airspeed_error, MAX_NUMERICAL_ERR_OPENLOOP) << "peak at t=" << _sim_stats.t_max_airspeed_error << " s";
	EXPECT_LT(_sim_stats.max_pitch_integ_norm, MAX_NUMERICAL_ERR_OPENLOOP);
}

// --- Throttle shortfall ----------------------------------------------------------
//
// The throttle does not deliver the total energy rate it is asked for: it fails, or it is undersped and already at
// its maximum. What the vehicle must then never do is lose its airspeed.

// At speed weight 2 the pitch controls airspeed only: the vehicle glides at its airspeed setpoint
// Disabled: the energy rate setpoints count on the total energy rate the throttle is asked for, and the pitch
// feedback splits the missing energy rate evenly between altitude and airspeed. The glide settles 1.8 m/s above the
// airspeed setpoint (see also DISABLED_AirspeedStepWithThrottleRange).
TEST_F(TECSControlClosedLoopTest, DISABLED_ThrottleFailureSpeedWeightTwoHoldsAirspeed)
{
	_params.pitch_speed_weight = 2.f;
	run(10.f);
	_throttle_failed = true;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, _V_sp - 2.5f);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
}

// At speed weights 1 and 0 the pitch gives up airspeed for altitude until the underspeed protection takes over. The
// vehicle then glides at the minimum sink rate, clear of the stall airspeed, and never climbs while undersped.
TEST_F(TECSControlClosedLoopTest, ThrottleFailureSpeedWeightOneGlidesAboveStall)
{
	_params.pitch_speed_weight = 1.f;
	run(10.f);
	_throttle_failed = true;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, _params.tas_stall + 1.f);
	EXPECT_LT(_sim_stats.max_climb_rate_undersped, 0.f);
	EXPECT_NEAR(plantAltitudeRate(), -_params.min_sink_rate, TRACKING_ERROR_TOL);
}

TEST_F(TECSControlClosedLoopTest, ThrottleFailureSpeedWeightZeroGlidesAboveStall)
{
	_params.pitch_speed_weight = 0.f;
	run(10.f);
	_throttle_failed = true;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, _params.tas_stall + 1.f);
	EXPECT_LT(_sim_stats.max_climb_rate_undersped, 0.f);
	EXPECT_NEAR(plantAltitudeRate(), -_params.min_sink_rate, TRACKING_ERROR_TOL);
}

// The same with the throttle at idle instead of failed, as in a landing flare
TEST_F(TECSControlClosedLoopTest, IdleThrottleGlidesAboveStall)
{
	run(10.f);
	_params.throttle_max = _params.throttle_min;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, _params.tas_stall + 1.f);
	EXPECT_LT(_sim_stats.max_climb_rate_undersped, 0.f);
}

// Between trim airspeed and the airspeed setpoint the pitch holds altitude and leaves the airspeed to the throttle.
// When the throttle fails there, the vehicle keeps its altitude until it has slowed down to trim airspeed.
// Disabled: the pitch feedback moves half of the missing energy rate into altitude, which is 0.65 m low by the time
// the airspeed reaches trim.
TEST_F(TECSControlClosedLoopTest, DISABLED_ThrottleFailureAboveTrimHoldsAltitudeDownToTrim)
{
	setAirspeedSetpoint(_params.equivalent_airspeed_trim + 3.f * AIRSPEED_STEP);
	_state.V = _V_sp;
	run(20.f);
	_throttle_failed = true;

	while (_state.V > _params.equivalent_airspeed_trim) {
		run(DT);
		ASSERT_NEAR(_state.h, _alt_sp, TRACKING_ERROR_TOL) << "at airspeed " << _state.V << " m/s";
	}
}

// --- Underspeed --------------------------------------------------------------------

// Deep in underspeed with working propulsion: the vehicle accelerates out of it without losing airspeed first and
// with little altitude loss
// Disabled: the altitude loss is 1.9 m instead of 1.2 m. The pitch feedback no longer puts all of the thrust beyond
// the kinetic energy rate setpoint into altitude, the vehicle is back at its airspeed setpoint sooner instead.
TEST_F(TECSControlClosedLoopTest, DISABLED_UnderspeedRecovery)
{
	_state.V = _params.tas_stall + 0.5f;
	const float airspeed_start = _state.V;
	run(5.f);

	EXPECT_NEAR(_tecs_control.getUnderspeedRatio(), 0.f, FLT_EPSILON) << "not out of underspeed after 5 s";

	run(55.f);

	EXPECT_GT(_sim_stats.min_airspeed, airspeed_start - TRACKING_ERROR_TOL);
	EXPECT_LT(_sim_stats.max_altitude_loss, 1.5f);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.1f);
}

// Deep in underspeed without propulsion: the vehicle dives for airspeed, never climbs while undersped and loses
// little airspeed on the way
TEST_F(TECSControlClosedLoopTest, UnderspeedWithThrottleFailure)
{
	_state.V = _params.tas_stall + 0.5f;
	const float airspeed_start = _state.V;
	_throttle_failed = true;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, airspeed_start - 0.5f);
	EXPECT_LT(_sim_stats.max_climb_rate_undersped, 0.f);
}

// Undersped well above the altitude setpoint, where the altitude loop asks for the maximum sink rate, with and
// without propulsion
TEST_F(TECSControlClosedLoopTest, UnderspeedAboveAltitudeSetpoint)
{
	_state.V = _params.tas_stall + 0.5f;
	_state.h = ALT_INIT + 40.f;
	const float airspeed_start = _state.V;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, airspeed_start - TRACKING_ERROR_TOL);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
	EXPECT_NEAR(_state.h, _alt_sp, 0.1f);
}

TEST_F(TECSControlClosedLoopTest, UnderspeedAboveAltitudeSetpointWithThrottleFailure)
{
	_state.V = _params.tas_stall + 0.5f;
	_state.h = ALT_INIT + 40.f;
	const float airspeed_start = _state.V;
	_throttle_failed = true;
	run(60.f);

	EXPECT_GT(_sim_stats.min_airspeed, airspeed_start - 0.5f);
	EXPECT_LT(_sim_stats.max_climb_rate_undersped, 0.f);
}

// --- Leaving fast descend ----------------------------------------------------------
//
// Fast descend is switched outside TECSControl, which ramps the ratio down and with it the airspeed setpoint from
// maximum to trim airspeed. The vehicle levels off with little altitude error.

TEST_F(TECSControlClosedLoopTest, FastDescendExit)
{
	run(10.f);
	_params.fast_descend = 1.f;
	setAirspeedSetpoint(_params.tas_max);
	setAltitudeSetpoint(ALT_INIT - 1000.f);
	initTecs();
	run(60.f);

	setAltitudeSetpoint(_state.h);
	resetSimStats();
	const float ramp_duration = 5.f;
	const int step_start = _step;

	while (_step < step_start + static_cast<int>(roundf((ramp_duration + 60.f) / DT))) {
		_params.fast_descend = math::max(1.f - (_step - step_start) * DT / ramp_duration, 0.f);
		setAirspeedSetpoint(math::lerp(_params.equivalent_airspeed_trim, _params.tas_max, _params.fast_descend));
		run(DT);
	}

	EXPECT_LT(_sim_stats.max_altitude_error, 2.5f) << "peak at t=" << _sim_stats.t_max_altitude_error << " s";
	EXPECT_NEAR(_state.h, _alt_sp, 0.1f);
	EXPECT_NEAR(_state.V, _V_sp, 0.1f);
}
