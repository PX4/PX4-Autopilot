/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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
 * @file WindEstimator.hpp
 * A wind and airspeed scale estimator.
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using namespace time_literals;

class WindEstimator
{
public:
	WindEstimator() = default;
	~WindEstimator() = default;

	// no copy, assignment, move, move assignment
	WindEstimator(const WindEstimator &) = delete;
	WindEstimator &operator=(const WindEstimator &) = delete;
	WindEstimator(WindEstimator &&) = delete;
	WindEstimator &operator=(WindEstimator &&) = delete;

	void update(uint64_t time_now);

	void fuse_airspeed(uint64_t time_now, float true_airspeed, const matrix::Vector3f &velI,
			   const matrix::Vector2f &velIvar);
	void fuse_beta(uint64_t time_now, const matrix::Vector3f &velI, const matrix::Quatf &q_att);

	bool is_estimate_valid() { return _initialised; }

	bool check_if_meas_is_rejected(uint64_t time_now, float innov, float innov_var, uint8_t gate_size,
				       uint64_t &time_meas_rejected, bool &reinit_filter);

	matrix::Vector2f get_wind() { return matrix::Vector2f{_state(INDEX_W_N), _state(INDEX_W_E)}; }

	// invert scale (CAS = IAS * scale), protect agains division by 0, constrain to [0.1, 10]
	float get_tas_scale() { return 1.f / math::constrain(_state(INDEX_TAS_SCALE), 0.1f, 10.0f); }
	float get_tas_scale_var() { return _P(2, 2); }
	float get_tas_innov() { return _tas_innov; }
	float get_tas_innov_var() { return _tas_innov_var; }
	float get_beta_innov() { return _beta_innov; }
	float get_beta_innov_var() { return _beta_innov_var; }
	matrix::Vector2f  get_wind_var() { return matrix::Vector2f{_P(0, 0), _P(1, 1)}; }
	bool get_wind_estimator_reset() { return _wind_estimator_reset; }

	void set_wind_p_noise(float wind_sigma) { _wind_p_var = wind_sigma * wind_sigma; }
	void set_tas_scale_p_noise(float tas_scale_sigma) { _tas_scale_p_var = tas_scale_sigma * tas_scale_sigma; }
	void set_tas_noise(float tas_sigma) { _tas_var = tas_sigma * tas_sigma; }
	void set_beta_noise(float beta_var) { _beta_var = beta_var * beta_var; }
	void set_tas_gate(uint8_t gate_size) {_tas_gate = gate_size; }
	void set_beta_gate(uint8_t gate_size) {_beta_gate = gate_size; }
	void set_scale_init(float scale_init) {_scale_init = 1.f / math::constrain(scale_init, 0.1f, 10.f); }

private:
	enum {
		INDEX_W_N = 0,
		INDEX_W_E,
		INDEX_TAS_SCALE
	};	///< enum which can be used to access state.

	matrix::Vector3f _state{0.f, 0.f, 1.f};
	matrix::Matrix3f _P;		///< state covariance matrix

	float _tas_innov{0.0f};	///< true airspeed innovation
	float _tas_innov_var{0.0f};	///< true airspeed innovation variance

	float _beta_innov{0.0f};	///< sideslip innovation
	float _beta_innov_var{0.0f};	///< sideslip innovation variance

	bool _initialised{false};	///< True: filter has been initialised

	float _wind_p_var{0.1f};	///< wind process noise variance
	float _tas_scale_p_var{0.0001f};	///< true airspeed scale process noise variance
	float _tas_var{1.4f};		///< true airspeed measurement noise variance
	float _beta_var{0.5f};	///< sideslip measurement noise variance
	uint8_t _tas_gate{3};	///< airspeed fusion gate size
	uint8_t _beta_gate{1};	///< sideslip fusion gate size

	float _scale_init{1.f};

	uint64_t _time_last_airspeed_fuse = 0;	///< timestamp of last airspeed fusion
	uint64_t _time_last_beta_fuse = 0;	///< timestamp of last sideslip fusion
	uint64_t _time_last_update = 0;		///< timestamp of last covariance prediction
	uint64_t _time_rejected_beta = 0;	///< timestamp of when sideslip measurements have consistently started to be rejected
	uint64_t _time_rejected_tas =
		0;	///< timestamp of when true airspeed measurements have consistently started to be rejected

	bool _wind_estimator_reset = false; ///< wind estimator was reset in this cycle

	// initialise state and state covariance matrix
	bool initialise(const matrix::Vector3f &velI, const matrix::Vector2f &velIvar, const float tas_meas);

	void run_sanity_checks();
};
