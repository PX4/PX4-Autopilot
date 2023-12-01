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

	virtual void setH(const matrix::Vector<float, 15> &h_meas, int direction) = 0;

	virtual float computeInnovCov(float measUnc) = 0;
	virtual float computeInnov(float meas) = 0;

	virtual bool update() = 0;

	// Normalized innovation squared (NIS) threshold. Used to reject measurements.
	virtual void setNISthreshold(float nis_threshold) = 0;

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
	virtual float getPosition() = 0;
	virtual float getVelocity() = 0;
	virtual float getBias() = 0;
	virtual float getAcceleration() = 0;
	virtual float getTargetVel() = 0;

	virtual float getPosVar() = 0;
	virtual float getVelVar() = 0;
	virtual float getBiasVar() = 0;
	virtual float getAccVar() = 0;
	virtual float getTargetVelVar() = 0;
	virtual float getTestRatio() = 0;

	virtual void setInputAccVar(float var) = 0;
	virtual void setTargetAccVar(float var) = 0;
	virtual void setBiasVar(float var) = 0;

	enum Direction {
		x = 0,
		y = 1,
		z = 2,
		nb_directions = 3,
	};

	enum ExtendedState {
		pos_rel_x = 0,
		pos_rel_y = 1,
		pos_rel_z = 2,
		vel_uav_x = 3,
		vel_uav_y = 4,
		vel_uav_z = 5,
		bias_x = 6,
		bias_y = 7,
		bias_z = 8,
		acc_target_x = 9,
		acc_target_y = 10,
		acc_target_z = 11,
		vel_target_x = 12,
		vel_target_y = 13,
		vel_target_z = 14,
		nb_extended_state = 15,
	};

};
