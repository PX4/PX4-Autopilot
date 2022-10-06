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
 * @file KalmanFilter.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * Constant velocity model. Prediction according to
 * x_{k+1} = A * x_{k}
 * with A = [1 dt; 0 1]
 *
 * Update with a direct measurement of the first state:
 * H = [1 0]
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#include "target_estimator.h"

#pragma once

namespace landing_target_estimator
{
class KalmanFilter : public TargetEstimator
{
public:
	/**
	 * Default constructor, state not initialized
	 */
	KalmanFilter() {};

	/**
	 * Default desctructor
	 */
	virtual ~KalmanFilter() {};

	void setPosition(float pos) override { _x(0) = pos; }
	void setVelocity(float vel) override { _x(1) = vel; }

	/**
	 * Predict the state with an external acceleration estimate
	 * @param dt            Time delta in seconds since last state change
	 * @param acc           Acceleration estimate
	 * @param acc_unc       Variance of acceleration estimate
	 */
	void predict(float dt, float acc) override;

	/**
	 * Update the state estimate with a measurement
	 * @param meas    state measeasurement
	 * @param measUnc measurement uncertainty
	 * @return update success (measurement not rejected)
	 */
	bool fusePosition(float meas, float measUnc) override;

	float getPosition() override { return _x(0); }
	float getVelocity() override { return _x(1); }

	float getPosVar() override { return _covariance(0, 0); }
	float getVelVar() override { return _covariance(0, 0); }

	/**
	 * Get measurement innovation and covariance of last update call
	 * @param innov Measurement innovation
	 * @param innovCov Measurement innovation covariance
	 */
	void getInnovations(float &innov, float &innovCov);

private:
	matrix::Vector<float, 2> _x; // state

	matrix::Matrix<float, 2, 2> _covariance; // state covariance

	float _residual{0.0f}; // residual of last measurement update

	float _innovCov{0.0f}; // innovation covariance of last measurement update
};
} // namespace landing_target_estimator
