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
	virtual void predictState(float dt, matrix::Vector<float, 3> acc);
	virtual void predictCov(float dt) {};

	// Backwards state prediciton
	virtual void syncState(float dt, matrix::Vector<float, 3> acc) {};

	virtual void setH(matrix::Vector<float, 12> h_meas);

	virtual float computeInnovCov(float measUnc);
	virtual float computeInnov(float meas);

	virtual bool update() { return true; };

	virtual void computeDynamicMatrix(float dt);
	virtual void computeProcessNoise(float dt);

	// Init: x_0
	//TODO: Rename with "Init": setInitPosition

	//For the coupled Kalman filter, we need to work with vectors.
	virtual void setPosition(matrix::Vector<float, 3> posVect);
	virtual void setVelocity(matrix::Vector<float, 3> velVect);
	virtual void setTargetAcc(matrix::Vector<float, 3> accVect);

	// Init: P_0
	virtual void setStatePosVar(matrix::Vector<float, 3> posVarVect);
	virtual void setStateVelVar(matrix::Vector<float, 3> velVarVect);
	virtual void setStateAccVar(matrix::Vector<float, 3> accVarVect);
	virtual void setStateBiasVar(matrix::Vector<float, 3> biasVarVect);

	// Retreive output of coupled filter
	virtual matrix::Vector<float, 3> getPositionVect();
	virtual matrix::Vector<float, 3> getVelocityVect();
	virtual matrix::Vector<float, 3> getAccelerationVect();

	virtual matrix::Vector<float, 3> getPosVarVect();
	virtual matrix::Vector<float, 3> getVelVarVect();
	virtual matrix::Vector<float, 3> getAccVarVect();

	virtual void setInputAccVar(matrix::Matrix<float, 3, 3> varVect);
	virtual void setTargetAccVar(matrix::Matrix<float, 3, 3> varVect);
	virtual void setBiasVar(matrix::Matrix<float, 3, 3> varVect);
};
