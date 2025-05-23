/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file AirspeedValidator.hpp
 * Calculates airspeed from differential pressure and checks if this airspeed is valid.
 */

#pragma once

#include <airspeed/airspeed.h>
#include <lib/wind_estimator/WindEstimator.hpp>
#include <uORB/topics/airspeed_wind.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/FilteredDerivative.hpp>


using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

using namespace time_literals;

struct airspeed_validator_update_data {
	uint64_t timestamp;
	float airspeed_indicated_raw;
	float airspeed_true_raw;
	uint64_t airspeed_timestamp;
	matrix::Vector3f ground_velocity;
	bool gnss_valid;
	float lpos_evh;
	float lpos_evv;
	matrix::Quatf q_att;
	float air_pressure_pa;
	float air_temperature_celsius;
	float accel_z;
	float vel_test_ratio;
	float hdg_test_ratio;
	bool in_fixed_wing_flight;
	float fixed_wing_throttle_filtered;
	float fixed_wing_tecs_throttle_trim;
	uint64_t tecs_timestamp;
};

class AirspeedValidator
{
public:
	AirspeedValidator();
	~AirspeedValidator() = default;

	void update_airspeed_validator(const airspeed_validator_update_data &input_data);

	void reset_airspeed_to_invalid(const uint64_t timestamp);

	float get_IAS() { return _IAS; }
	float get_CAS() { return _CAS; }
	float get_TAS() { return _TAS; }
	bool get_airspeed_valid() { return _airspeed_valid; }
	float get_CAS_scale_validated() {return _CAS_scale_validated;}
	float get_airspeed_derivative() { return _IAS_derivative.getState(); }
	float get_pitch_filtered() { return _pitch_filtered.getState(); }

	airspeed_wind_s get_wind_estimator_states(uint64_t timestamp);

	// setters wind estimator parameters
	void set_wind_estimator_wind_process_noise_spectral_density(float wind_nsd) { _wind_estimator.set_wind_process_noise_spectral_density(wind_nsd); }
	void set_wind_estimator_tas_scale_process_noise_spectral_density(float tas_scale_nsd) { _wind_estimator.set_tas_scale_process_noise_spectral_density(tas_scale_nsd); }
	void set_wind_estimator_tas_scale_init(float tas_scale_init)
	{
		_tas_scale_init = tas_scale_init;
	}

	void set_wind_estimator_tas_noise(float tas_sigma) { _wind_estimator.set_tas_noise(tas_sigma); }
	void set_wind_estimator_beta_noise(float beta_var) { _wind_estimator.set_beta_noise(beta_var); }
	void set_wind_estimator_tas_gate(uint8_t gate_size)
	{
		_wind_estimator.set_tas_gate(gate_size);
	}

	void set_wind_estimator_beta_gate(uint8_t gate_size) { _wind_estimator.set_beta_gate(gate_size); }

	// setters for failure detection tuning parameters
	void set_tas_innov_threshold(float tas_innov_threshold) { _tas_innov_threshold = tas_innov_threshold; }
	void set_tas_innov_integ_threshold(float tas_innov_integ_threshold) { _tas_innov_integ_threshold = tas_innov_integ_threshold; }
	void set_checks_fail_delay(float checks_fail_delay) { _checks_fail_delay = checks_fail_delay; }
	void set_checks_clear_delay(float checks_clear_delay) { _checks_clear_delay = checks_clear_delay; }

	void set_airspeed_stall(float airspeed_stall) { _airspeed_stall = airspeed_stall; }

	void set_tas_scale_apply(int tas_scale_apply) { _tas_scale_apply = tas_scale_apply; }
	void set_CAS_scale_validated(float scale) { _CAS_scale_validated = scale; }
	void set_scale_init(float scale) { _wind_estimator.set_scale_init(scale); }

	void set_enable_data_stuck_check(bool enable) { _data_stuck_check_enabled = enable; }
	void set_enable_innovation_check(bool enable) { _innovation_check_enabled = enable; }
	void set_enable_load_factor_check(bool enable) { _load_factor_check_enabled = enable; }
	void set_enable_first_principle_check(bool enable) { _first_principle_check_enabled = enable; }
	void set_psp_off_param(float psp_off_param) { _param_psp_off = psp_off_param; }
	void set_throttle_max_param(float throttle_max_param) { _param_throttle_max = throttle_max_param; }
	void set_fp_t_window(float t_window) { _aspd_fp_t_window = t_window; }

private:

	WindEstimator _wind_estimator{}; ///< wind estimator instance running in this particular airspeedValidator

	// Check enabling
	bool _data_stuck_check_enabled{false};
	bool _innovation_check_enabled{false};
	bool _load_factor_check_enabled{false};
	bool _first_principle_check_enabled{false};

	// airspeed scale validity check
	static constexpr int SCALE_CHECK_SAMPLES = 12; ///< take samples from 12 segments (every 360/12=30°)

	static constexpr float kHighThrottleDelta =
		0.05f; ///<  throttle delta above trim throttle required to consider throttle high
	static constexpr float kIASDerivateThreshold =
		0.1f; ///< threshold for IAS derivative to detect airspeed failure. Failure is
	// detected if in a high throttle and low pitch situation and the filtered IAS derivative is below this threshold

	// general states
	bool _in_fixed_wing_flight{false}; ///< variable to bypass innovation and load factor checks
	float _IAS{0.0f}; ///< indicated airsped in m/s
	float _CAS{0.0f}; ///< calibrated airspeed in m/s
	float _TAS{0.0f}; ///< true airspeed in m/s
	float _CAS_scale_applied{1.0f}; ///< scale factor from IAS to CAS (currently applied value)
	float _CAS_scale_validated{1.0f}; ///< scale factor from IAS to CAS (currently estimated value)

	// data stuck check
	uint64_t _time_last_unequal_data{0};
	bool _data_stuck_test_failed{false};
	float _IAS_prev{0.f};
	static constexpr uint64_t DATA_STUCK_TIMEOUT{2_s}; ///< timeout after which data stuck check triggers when data is flat

	// states of innovation check
	bool _innovations_check_failed{false};  ///< true when airspeed innovations have failed consistency checks
	float _tas_innov_threshold{1.0}; ///< innovation error threshold for triggering innovation check failure
	float _tas_innov_integ_threshold{-1.0}; ///< integrator innovation error threshold for triggering innovation check failure
	uint64_t	_time_last_aspd_innov_check{0};	///< time airspeed innovation was last checked (uSec)
	float		_aspd_innov_integ_state{0.0f};	///< integral of excess normalised airspeed innovation (sec)
	uint64_t	_time_wind_estimator_initialized{0};		///< time last time wind estimator was initialized (uSec)

	// states of load factor check
	bool _load_factor_check_failed{false}; ///< load_factor check has detected failure
	float _airspeed_stall{8.0f}; ///< stall speed of aircraft used for load factor check
	float	_load_factor_ratio{0.5f};	///< ratio of maximum load factor predicted by stall speed to measured load factor

	// first principle check
	bool _first_principle_check_failed{false}; ///< first principle check has detected failure
	float _aspd_fp_t_window{0.f}; ///< time window for first principle check
	FilteredDerivative<float> _IAS_derivative; ///< indicated airspeed derivative for first principle check
	AlphaFilter<float> _pitch_filtered; ///< filtered pitch for first principle check
	hrt_abstime _time_last_first_principle_check{0}; ///< time airspeed first principle was last checked (uSec)
	hrt_abstime _time_last_first_principle_check_passing{0}; ///< time airspeed first principle was last passing (uSec)
	float _param_psp_off{0.0f}; ///< parameter pitch in level flight [rad]
	float _param_throttle_max{0.0f}; ///< parameter maximum throttle value

	// states of airspeed valid declaration
	bool _airspeed_valid{true}; ///< airspeed valid (pitot or groundspeed-windspeed)
	float _checks_fail_delay{2.f}; ///< delay for airspeed invalid declaration after single check failure (Sec)
	float _checks_clear_delay{-1.f}; ///< delay for airspeed valid declaration after all checks passed again (Sec)
	uint64_t	_time_checks_passed{0};	///< time the checks have last passed (uSec)
	uint64_t	_time_checks_failed{0};	///< time the checks have last not passed (uSec)

	hrt_abstime _begin_current_scale_check{0};

	int _tas_scale_apply{0};
	float _tas_scale_init{1.f};

	matrix::Vector<float, SCALE_CHECK_SAMPLES> _scale_check_TAS {};
	matrix::Vector<float, SCALE_CHECK_SAMPLES> _scale_check_groundspeed {};

	void update_in_fixed_wing_flight(bool in_fixed_wing_flight) { _in_fixed_wing_flight = in_fixed_wing_flight; }

	void update_wind_estimator(const uint64_t timestamp, float airspeed_true_raw, bool gnss_valid,
				   const matrix::Vector3f &vI,
				   float lpos_evh, float lpos_evv, const Quatf &q_att);
	void update_CAS_scale_validated(bool gnss_valid, const matrix::Vector3f &vI, float airspeed_true_raw);
	void update_CAS_scale_applied();
	void update_CAS_TAS(float air_pressure_pa, float air_temperature_celsius);
	void check_airspeed_data_stuck(uint64_t timestamp);
	void check_airspeed_innovation(uint64_t timestamp, float estimator_status_vel_test_ratio,
				       float estimator_status_hdg_test_ratio, const matrix::Vector3f &vI, bool gnss_valid);
	void check_load_factor(float accel_z);
	void check_first_principle(const uint64_t timestamp, const float throttle, const float throttle_trim,
				   const uint64_t tecs_timestamp, const Quatf &att_q);
	void update_airspeed_valid_status(const uint64_t timestamp);
	void reset();
	void reset_CAS_scale_check();

};
