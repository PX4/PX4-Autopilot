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
	virtual ~KF_xyzb_decoupled_static() {};

	//Prediction step:
	void predictState(float dt, float acc) override;
	void predictCov(float dt) override;

	// Backwards state prediciton
	void syncState(float dt, float acc) override;

	void setH(matrix::Vector<float, 15> h_meas, int direction) override;

	virtual float computeInnovCov(float measUnc) override;
	virtual float computeInnov(float meas) override;

	bool update() override;

	void setNISthreshold(float nis_threshold) override { _nis_threshold = nis_threshold; };

	// Init: x_0
	void setPosition(float pos) override { _state(0) = pos; };
	void setVelocity(float vel) override { _state(1) = vel; };
	void setBias(float bias) override { _state(2) = bias; };

	// Init: P_0
	void setStatePosVar(float pos_unc) override { _state_covariance(0, 0) = pos_unc; };
	void setStateVelVar(float vel_unc) override { _state_covariance(1, 1) = vel_unc; };
	void setStateBiasVar(float bias_unc) override { _state_covariance(2, 2) = bias_unc; };

	// Retreive output of filter
	float getPosition() override { return _state(0); };
	float getVelocity() override { return _state(1); };
	float getBias() override { return _state(2); };

	float getPosVar() override { return _state_covariance(0, 0); };
	float getVelVar() override { return _state_covariance(1, 1); };
	float getBiasVar() override { return _state_covariance(2, 2); };

	float getTestRatio() override {if (fabsf(_innov_cov) < 1e-6f) {return -1.f;} else {return _innov / _innov_cov * _innov;} };

	void setInputAccVar(float var) override { _input_var = var; };
	void setBiasVar(float var) override { _bias_var = var; };

	/* Unused functions:  */
	void setTargetAcc(float acc) override {}
	void setStateAccVar(float acc_unc) override {};
	float getAcceleration() override { return 0.f; };
	float getAccVar() override { return 0.f; };
	void setTargetAccVar(float var) override {};
	float getTargetVelVar() override { return 0.f; };
	float getTargetVel() override { return 0.f; };
	void setStateTargetVelVar(float posVect) override {};
	void setTargetVel(float accVect) override {};

private:
	matrix::Vector<float, 3> _state;

	matrix::Vector<float, 3> _sync_state;

	matrix::Matrix<float, 1, 3> _meas_matrix; // row of measurement matrix

	matrix::Matrix<float, 3, 3> _state_covariance;

	float _bias_var{0.f}; // target/UAV GPS bias variance

	float _input_var{0.f}; // UAV acceleration variance

	float _innov{0.0f}; // residual of last measurement update

	float _innov_cov{0.0f}; // innovation covariance of last measurement update

	float _nis_threshold{0.0f}; // Normalized innovation squared test threshold
};
} // namespace vision_target_estimator
