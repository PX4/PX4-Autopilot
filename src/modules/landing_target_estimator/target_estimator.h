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
 * @file target_estimator.cpp
 * @brief Interface for target estimators
 */

#pragma once

class TargetEstimator
{
public:
	TargetEstimator() = default;
	virtual ~TargetEstimator() = default;

	//Prediction step:
	virtual void predictState(float dt, float acc) = 0;
	virtual void predictCov(float dt) {};

	// Backwards state prediciton
	virtual void syncState(float dt, float acc);

	virtual void setH(matrix::Vector<float, 12> h_meas);
	virtual void setH(float h_meas);

	virtual float computeInnovCov(float measUnc);
	virtual float computeInnov(float meas);

	virtual bool update() { return true; };

	virtual void computeDynamicMatrix(float dt);
	virtual void computeProcessNoise(float dt);

	// Init: x_0
	//TODO: Rename with "Init": setInitPosition
	virtual void setPosition(float pos);
	virtual void setVelocity(float vel);
	virtual void setTargetAcc(float acc);

	// Init: P_0
	virtual void setStatePosVar(float var);
	virtual void setStateVelVar(float var);
	virtual void setStateAccVar(float var);
	virtual void setStateBiasVar(float var);

	// Retreive output of filter
	virtual float getPosition() { return 0.f; };
	virtual float getVelocity() { return 0.f; };
	virtual float getAcceleration() { return 0.f; };

	virtual float getPosVar() { return 0.f; };
	virtual float getVelVar() { return 0.f; };
	virtual float getAccVar() { return 0.f; };

	virtual void setInputAccVar(float var);
	virtual void setTargetAccVar(float var);
	virtual void setBiasVar(float var);
};
