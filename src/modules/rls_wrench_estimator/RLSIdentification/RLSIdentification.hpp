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
 * @file RLSIdentification.hpp
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

struct VehicleParameters {
	float mass;
	float tilt_angle;
	int n_rotors;
	float lpf_motor_tau;
	float torque_constant;
	float diameter;
	float top_motors_height;
	float bot_motors_height;
};

class RLSIdentification
{
public:

	RLSIdentification() = default;
	~RLSIdentification() = default;

	void initialize(const float (&x_init)[4], const float (&x_confidence)[4],
			const float (&R_diag)[5], const VehicleParameters &params);

	void updateThrust(const Vector3f &y, const Vector<float, 8> &outputs, const float &dt,
				const bool &interaction_flag);
	void updateOffset(const Quatf &q, const bool &interaction_flag);

	Vector<float, 8> getFilteredOutputs() const { return _w_lpf; }

	Vector3f getPredictionErrorThrust() const { return _prediction_error_thrust; }
	Vector2f getEstimationThrust() const { return _xp_thrust; }
	Vector3f getActuatorForceVector() const { return _force_vector; }

	Vector3f getPredictionErrorOffset() const { return _prediction_error_offset; }
	Vector3f getEstimationOffset() const { return _xp_offset; }
	Vector3f getActuatorMomentVector() const { return _moment_vector; }

private:

	void _updateLpf(const Vector<float, 8> &u);

	//RLS Thrust
	void _createHThrust(Vector<float, 8> &W);
	void _computeKThrust(const bool &interaction_flag);
	void _computePredictionErrorThrust(const Vector3f &y);
	void _computePThrust();

	//RLS offset
	void _createHOffset();
	void _computeKOffset(const bool &interaction_flag);
	void _computePredictionErrorOffset();
	void _computePOffset();
	void _createMomentVector();

	float _tilt_angle{0.f}; //< Tilt angle in degrees
	float _mass{0.8f}; //< Vehicle Mass
	int _n_rotors{4}; //< Number of rotors
	float _dt{0.004f}; //< Sampling Time
	float _lpf_motor_tau{0.1f};  //< Motor Dynamics Time Constant
	float _km{1.041f}; //< Motor Torque Constant
	float _diameter{0.25f};   //< Vehicle Diameter [m]
	float _top_motors_height{35.0f};  //< Top Motors distance from CoM [mm]
	float _bot_motors_height{20.0f};  //< Bottom Motors distance from CoM [mm]

	Vector<float, 8> _w_lpf{};;
	Quatf _q{};  //Quaternion rotation from the FRD body frame to the NED earth frame

	//RLS Thrust
	Vector2f _xp_thrust{};
	Matrix<float, 2, 3> _K_thrust{};
	Matrix<float, 3, 2> _H_thrust{};
	SquareMatrix<float, 2> _Pp_thrust{};
	SquareMatrix<float, 3> _R_thrust{};
	Vector3f _prediction_error_thrust{};

	//RLS offset
	Vector3f _xp_offset{};
	SquareMatrix<float, 3> _K_offset{};
	SquareMatrix<float, 3> _H_offset{};
	SquareMatrix<float, 3> _Pp_offset{};
	SquareMatrix<float, 3> _R_offset{};
	Vector3f _prediction_error_offset{};
	Vector3f _moment_vector{};
	Vector3f _force_vector{};

	static constexpr float _GRAVITY = 9.80665f; // m/s^2
	static constexpr float _kf_multiplier = (1E-6f);
	static constexpr float _km_multiplier = (1E-8f);
};
