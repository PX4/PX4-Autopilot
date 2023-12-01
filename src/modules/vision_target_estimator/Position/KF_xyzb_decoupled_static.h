/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file KF_xyzb_decoupled_static.h
 * @brief Filter to estimate the pose of static targets. State: [r, r_dot, bias]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#include "base_KF_decoupled.h"

#pragma once

namespace vision_target_estimator
{
class KF_xyzb_decoupled_static : public Base_KF_decoupled
{
public:
	/**
	 * Default constructor, state not initialized
	 */
	KF_xyzb_decoupled_static() {};

	/**
	 * Default desctructor
	 */
	~KF_xyzb_decoupled_static() {};

	//Prediction step:
	void predictState(float dt, float acc) override;
	void predictCov(float dt) override;

	// Backwards state prediciton
	void syncState(float dt, float acc) override;

	void setH(const matrix::Vector<float, 5> &h_meas) override
	{
		_meas_matrix_row_vect(State::pos_rel) = h_meas(AugmentedState::pos_rel);
		_meas_matrix_row_vect(State::vel_rel) = -h_meas(AugmentedState::vel_uav);
		_meas_matrix_row_vect(State::bias) = h_meas(AugmentedState::bias);
	};

	void setState(const matrix::Vector<float, 5> &state) override
	{
		_state(State::pos_rel) = state(AugmentedState::pos_rel);
		_state(State::vel_rel) = -state(AugmentedState::vel_uav);
		_state(State::bias) = state(AugmentedState::bias);
	};

	void setStateVar(const matrix::Vector<float, 5> &var) override
	{
		_state_covariance(State::pos_rel, State::pos_rel) = var(AugmentedState::pos_rel);
		_state_covariance(State::vel_rel, State::vel_rel) = var(
					AugmentedState::vel_uav); // Variance of vel_uav is equivalent to the variance of vel_rel because Var(aX) = a^2 Var(X)
		_state_covariance(State::bias, State::bias) = var(AugmentedState::bias);
	};

	matrix::Vector<float, 5> getAugmentedState()
	{
		matrix::Vector<float, 5> augmented_state;
		augmented_state(AugmentedState::pos_rel) = _state(State::pos_rel);
		augmented_state(AugmentedState::vel_uav) = -_state(State::vel_rel);
		augmented_state(AugmentedState::bias) = _state(State::bias);

		return augmented_state;
	};

	matrix::Vector<float, 5> getAugmentedStateVar()
	{
		matrix::Vector<float, 5> augmented_state_var;
		augmented_state_var(AugmentedState::pos_rel) = _state_covariance(State::pos_rel, State::pos_rel);
		augmented_state_var(AugmentedState::vel_uav) = _state_covariance(State::vel_rel, State::vel_rel);
		augmented_state_var(AugmentedState::bias) = _state_covariance(State::bias, State::bias);

		return augmented_state_var;
	};

	float computeInnovCov(float measUnc) override;
	float computeInnov(float meas) override;

	bool update() override;

	void setNISthreshold(float nis_threshold) override { _nis_threshold = nis_threshold; };

	float getTestRatio() override {if (fabsf(_innov_cov) < 1e-6f) {return -1.f;} else {return _innov / _innov_cov * _innov;} };
	void setInputAccVar(float var) override { _input_var = var; };
	void setBiasVar(float var) override { _bias_var = var; };

	// Unused function
	void setTargetAccVar(float var)override {};

private:

	enum State {
		pos_rel = 0,
		vel_rel = 1,
		bias = 2,
	};

	matrix::Vector<float, 3> _state;

	matrix::Vector<float, 3> _sync_state;

	matrix::Vector<float, 3> _meas_matrix_row_vect;

	matrix::Matrix<float, 3, 3> _state_covariance;

	float _bias_var{0.f}; // target/UAV GPS bias variance

	float _input_var{0.f}; // UAV acceleration variance

	float _innov{0.0f}; // residual of last measurement update

	float _innov_cov{0.0f}; // innovation covariance of last measurement update

	float _nis_threshold{0.0f}; // Normalized innovation squared test threshold
};
} // namespace vision_target_estimator
