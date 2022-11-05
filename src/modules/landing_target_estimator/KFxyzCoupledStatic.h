/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file KFxyzCoupledStatic.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * State: [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#include "target_estimator_coupled.h"

#pragma once

namespace landing_target_estimator
{
class KFxyzCoupledStatic : public TargetEstimatorCoupled
{
public:
	/**
	 * Default constructor, state not initialized
	 */
	KFxyzCoupledStatic() {};

	/**
	 * Default desctructor
	 */
	virtual ~KFxyzCoupledStatic() {};

	//Prediction step:
	void predictState(float dt, matrix::Vector<float, 3> acc) override;
	void predictCov(float dt) override;

	// Backwards state prediciton
	void syncState(float dt, matrix::Vector<float, 3> acc) override;

	void setH(matrix::Vector<float, 12> h_meas) override;

	virtual float computeInnovCov(float measUnc) override;
	virtual float computeInnov(float meas) override;

	bool update() override;

	// Init: x_0
	void setPosition(matrix::Vector<float, 3> posVect) override { _state(0, 0) = posVect(0); _state(1, 0) = posVect(1); _state(2, 0) = posVect(2);};
	void setVelocity(matrix::Vector<float, 3> velVect) override  { _state(3, 0) = velVect(0); _state(4, 0) = velVect(1); _state(5, 0) = velVect(2);};
	void setBias(matrix::Vector<float, 3> biasVect) override  { _state(6, 0) = biasVect(0); _state(7, 0) = biasVect(1); _state(8, 0) = biasVect(2);};

	// Init: P_0 (assume diagonal)
	void setStatePosVar(matrix::Vector<float, 3> posVect) override { _covariance(0, 0) = posVect(0); _covariance(1, 1) = posVect(1); _covariance(2, 2) = posVect(2);};
	void setStateVelVar(matrix::Vector<float, 3> velVect) override { _covariance(3, 3) = velVect(0); _covariance(4, 4) = velVect(1); _covariance(5, 5) = velVect(2);};
	void setStateBiasVar(matrix::Vector<float, 3> biasVarVect) override { _covariance(6, 6) = biasVarVect(0); _covariance(7, 7) = biasVarVect(1); _covariance(8, 8) = biasVarVect(2);};

	// Retreive output of filter
	matrix::Vector<float, 3> getPositionVect() override {matrix::Vector3f pos_vect(_state(0, 0), _state(1, 0), _state(2, 0)); return pos_vect;};
	matrix::Vector<float, 3> getVelocityVect() override {matrix::Vector3f vel_vect(_state(3, 0), _state(4, 0), _state(5, 0)); return vel_vect;};
	matrix::Vector<float, 3> getBiasVect() override {matrix::Vector3f bias_vect(_state(6, 0), _state(7, 0), _state(8, 0)); return bias_vect;};

	matrix::Vector<float, 3> getPosVarVect() override {matrix::Vector3f pos_var_vect(_covariance(0, 0), _covariance(1, 1), _covariance(2, 2)); return pos_var_vect;};
	matrix::Vector<float, 3> getVelVarVect() override {matrix::Vector3f vel_var_vect(_covariance(3, 3), _covariance(4, 4), _covariance(5, 5)); return vel_var_vect;};
	matrix::Vector<float, 3> getBiasVarVect() override {matrix::Vector3f bias_var_vect(_covariance(6, 6), _covariance(7, 7), _covariance(8, 8)); return bias_var_vect;};

	void setInputAccVar(matrix::Matrix<float, 3, 3> varVect) override {_input_var = varVect;};
	void setBiasVar(matrix::Matrix<float, 3, 3> varVect) override {_bias_var = varVect;};


	/* Unused functions:  */
	void computeDynamicMatrix(float dt) override {};
	void computeProcessNoise(float dt) override {};
	void setTargetAccVar(matrix::Matrix<float, 3, 3> varVect) override {};
	matrix::Vector<float, 3> getAccVarVect() override {matrix::Vector<float, 3> dummy_vect; return dummy_vect;};
	matrix::Vector<float, 3> getAccelerationVect() override {matrix::Vector<float, 3> dummy_vect; return dummy_vect;};
	void setStateAccVar(matrix::Vector<float, 3> posVect) override {};
	void setTargetAcc(matrix::Vector<float, 3> accVect) override {};


private:
	matrix::Matrix<float, 9, 1> _state; // state

	matrix::Matrix<float, 9, 1> _sync_state; // state

	matrix::Matrix<float, 1, 9> _meas_matrix; // row of measurement matrix

	matrix::Matrix<float, 9, 9> _covariance; // state covariance

	matrix::Matrix<float, 3, 3> _bias_var; // target/UAV GPS bias variance

	matrix::Matrix<float, 3, 3> _input_var; // UAV acceleration variance

	float _innov{0.0f}; // residual of last measurement update

	float _innov_cov{0.0f}; // innovation covariance of last measurement update
};
} // namespace landing_target_estimator
