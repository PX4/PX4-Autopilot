/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file target_estimator_coupled.cpp
 * @brief Interface for target estimators
 */

#pragma once

class TargetEstimatorCoupled
{
public:
	TargetEstimatorCoupled() = default;
	virtual ~TargetEstimatorCoupled() = default;

	//Prediction step:
	virtual void predictState(float dt, matrix::Vector<float, 3> acc) = 0;
	virtual void predictCov(float dt) = 0;

	// Backwards state prediciton
	virtual void syncState(float dt, matrix::Vector<float, 3> acc) = 0;

	virtual void setH(matrix::Vector<float, 12> h_meas) = 0;

	virtual float computeInnovCov(float measUnc) = 0;
	virtual float computeInnov(float meas) = 0;

	virtual bool update() { return true; };

	virtual void computeDynamicMatrix(float dt) = 0;
	virtual void computeProcessNoise(float dt) = 0;

	// Init: x_0
	//TODO: Rename with "Init": setInitPosition

	//For the coupled Kalman filter, we need to work with vectors.
	virtual void setPosition(matrix::Vector<float, 3> posVect) = 0;
	virtual void setVelocity(matrix::Vector<float, 3> velVect) = 0;
	virtual void setTargetAcc(matrix::Vector<float, 3> accVect) = 0;
	virtual void setBias(matrix::Vector<float, 3> biasVarVect) = 0;

	// Init: P_0
	virtual void setStatePosVar(matrix::Vector<float, 3> posVarVect) = 0;
	virtual void setStateVelVar(matrix::Vector<float, 3> velVarVect) = 0;
	virtual void setStateAccVar(matrix::Vector<float, 3> accVarVect) = 0;
	virtual void setStateBiasVar(matrix::Vector<float, 3> biasVarVect) = 0;

	// Retreive output of coupled filter
	virtual matrix::Vector<float, 3> getPositionVect() = 0;
	virtual matrix::Vector<float, 3> getVelocityVect() = 0;
	virtual matrix::Vector<float, 3> getAccelerationVect() = 0;
	virtual matrix::Vector<float, 3> getBiasVect() = 0;

	virtual matrix::Vector<float, 3> getPosVarVect() = 0;
	virtual matrix::Vector<float, 3> getVelVarVect() = 0;
	virtual matrix::Vector<float, 3> getAccVarVect() = 0;
	virtual matrix::Vector<float, 3> getBiasVarVect() = 0;

	virtual void setInputAccVar(matrix::Matrix<float, 3, 3> varVect) = 0;
	virtual void setTargetAccVar(matrix::Matrix<float, 3, 3> varVect) = 0;
	virtual void setBiasVar(matrix::Matrix<float, 3, 3> varVect) = 0;
};
