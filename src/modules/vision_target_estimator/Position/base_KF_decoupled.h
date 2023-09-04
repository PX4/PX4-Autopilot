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
 * @file Base_KF_decoupled.h
 * @brief Interface for decoupled target estimators
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

class Base_KF_decoupled
{
public:
	Base_KF_decoupled() = default;
	virtual ~Base_KF_decoupled() = default;

	//Prediction step:
	virtual void predictState(float dt, float acc) = 0;
	virtual void predictCov(float dt) = 0;

	// Backwards state prediciton
	virtual void syncState(float dt, float acc) = 0;

	virtual void setH(matrix::Vector<float, 15> h_meas, int direction) = 0;

	virtual float computeInnovCov(float measUnc) = 0;
	virtual float computeInnov(float meas) = 0;

	virtual bool update() { return true; }

	// Init: x_0
	virtual void setPosition(float pos) = 0;
	virtual void setVelocity(float vel) = 0;
	virtual void setTargetAcc(float acc) = 0;
	virtual void setBias(float bias) = 0;
	virtual void setTargetVel(float targetVel) = 0;

	// Init: P_0
	virtual void setStatePosVar(float var) = 0;
	virtual void setStateVelVar(float var) = 0;
	virtual void setStateAccVar(float var) = 0;
	virtual void setStateBiasVar(float var) = 0;
	virtual void setStateTargetVelVar(float var) = 0;

	// Retreive output of filter
	virtual float getPosition() { return 0.f; };
	virtual float getVelocity() { return 0.f; };
	virtual float getBias() { return 0.f; };
	virtual float getAcceleration() { return 0.f; };
	virtual float getTargetVel() { return 0.f; };

	virtual float getPosVar() { return 0.f; };
	virtual float getVelVar() { return 0.f; };
	virtual float getBiasVar() { return 0.f; };
	virtual float getAccVar() { return 0.f; };
	virtual float getTargetVelVar() { return 0.f; };
	virtual float getTestRatio() { return 0.f; };

	virtual void setInputAccVar(float var) = 0;
	virtual void setTargetAccVar(float var) = 0;
	virtual void setBiasVar(float var) = 0;

	enum Directions {
		x = 0,
		y = 1,
		z = 2,
		nb_directions = 3,
	};
};
