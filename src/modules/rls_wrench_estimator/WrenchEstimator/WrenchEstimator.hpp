/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file WrenchEstimator.hpp
 *
 * PMEN - Identify rotor thrust constant and CoM offset.
 *
 * RLS has the following structure:
 * y = Hx + v
 * where y is an m-element noisy measurement vector
 * x is a constant but unknown parameter n-vector and v is measurement noise
 * H is m x n regressor matrix (depends on number of rotors)
 * RLS estimator can be written as
 * x[k] = x[k-1] + K[k]*(y[k]-H[k]*x[k-1])
 * K[k] is nxm and the estimator gain matrix
 * K[k] = P[k-1]*H[k]'*inv(H[k]*P[k-1]*H[k]'+R[k])
 * P[k] = (I - K[k]H[k])*P[k-1]
 * R is mxm the covariance mxm measurement covariance matrix
 * P is nxn the estimator error covariance
 *
 *  For computing motor thrust constant and reduction in bottom propellers
 *  x is a vector [k_f;alpha]*1e-6
 *  y is a function of acceleration+external forces corrupted by noise
 *  H is a function of motor PWM filtered to account for motor dynamics
 * R[0] should be a parameter based on accelerometer noise
 * x[0] should be a parameter with inital guess
 * P[0] should be a parameter based on confidence of initial guess
 * Possibly, K needs to be set to 0 during interaction and landed state
 *
 * Inputs: acceleration [body frame], pwm values
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace matrix;


class WrenchEstimator
{
public:

	WrenchEstimator() = default;
	~WrenchEstimator() = default;

	void initialize(const float &tau_f,const float &tau_m,const Vector3f &inertia_diag);

	void updateForce(const Vector3f &prediction_error_force, const float &dt, const bool &interaction_flag);
	void updateMoment(const Vector3f &prediction_error_moment, const Vector3f &omega, const float &dt, const bool &interaction_flag);

	Vector3f getExternalForce() const { return _Fe_inertial; }
	Vector3f getExternalMoment() const { return _Me; }

private:

	void _integrateForce(const Vector3f &u);
	void _integrateMoment(const Vector3f &u);

	float _dt{0.004f}; //< Sampling Time

	float _K_force{1.0f};    //< 1/Force estimator time constant
	float _K_moment{1.0f};   //< 1/Moment estimator time constant

	Vector3f _Fe{};
	Vector3f _Fe_inertial{};
	Vector3f _Me{};

	Vector3f _omega{};

	SquareMatrix3f _I{};
};
