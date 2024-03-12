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
 * @file Base_KF_position.h
 * @brief Interface for decoupled target estimators
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

class Base_KF_decoupled
{
public:
	Base_KF_decoupled() = default;
	virtual ~Base_KF_decoupled() = default;

	enum AugmentedState {
		pos_rel = 0,
		vel_uav = 1,
		bias = 2,
		acc_target = 3,
		vel_target = 4,
		COUNT = 5,
	};

	//Prediction step:
	virtual void predictState(float dt, float acc) = 0;
	virtual void predictCov(float dt) = 0;

	// Backwards state prediciton
	virtual void syncState(float dt, float acc) = 0;

	virtual void setH(const matrix::Vector<float, AugmentedState::COUNT> &h_meas) = 0;

	virtual void setState(const matrix::Vector<float, AugmentedState::COUNT> &augmented_state) = 0;
	virtual void setStateVar(const matrix::Vector<float, AugmentedState::COUNT> &augmented_state_var) = 0;

	virtual matrix::Vector<float, AugmentedState::COUNT> getAugmentedState() = 0;
	virtual matrix::Vector<float, AugmentedState::COUNT> getAugmentedStateVar() = 0;

	virtual float computeInnovCov(float measUnc) = 0;
	virtual float computeInnov(float meas) = 0;

	virtual bool update() = 0;

	// Normalized innovation squared (NIS) threshold. Used to reject measurements.
	virtual void setNISthreshold(float nis_threshold) = 0;

	virtual float getTestRatio() = 0;

	virtual void setInputAccVar(float var) = 0;
	virtual void setTargetAccVar(float var) = 0;
	virtual void setBiasVar(float var) = 0;
};
