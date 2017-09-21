/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "../IEKF.hpp"

void IEKF::correctAccel(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->accelerometer_timestamp_relative;

	if (!_sensorAccel.ready(timestamp, dt)) {
		return;
	}

	//ROS_INFO("correct accel");

	// measurement
	Vector3f y_b(
		msg->accelerometer_m_s2[0],
		msg->accelerometer_m_s2[1],
		msg->accelerometer_m_s2[2]);

	// calculate residual
	//
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f a_bias_b(_x(X::accel_bias_bX), _x(X::accel_bias_bY), _x(X::accel_bias_bZ));
	Vector3f a_b_corrected = y_b - a_bias_b;
	Vector3f a_n_corrected = q_nb.conjugate(a_b_corrected);
	Vector3f y_g_n = q_nb.conjugate(a_b_corrected);
	Vector3f r = y_g_n - _g_n;

	Vector3f gyro_bias_b(_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	Vector3f omega_nb_b(_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	float rotRateSq = omega_nb_b_corrected.dot(omega_nb_b_corrected);

	// weight less if accelerating
	float relNormError = ((a_b_corrected).norm()
			      - _g_n.norm()) / _g_n.norm();

	// define R
	// worst accel dir change is if accel is normal to gravity,
	// assume this and calculate angle covariance based on accel norm error
	Matrix<float, Y_accel::n, Y_accel::n> R;
	float cov = (
			    _accel_nd.get() * _accel_nd.get()
			    + 1e0f * relNormError * relNormError
			    + 4e0f * rotRateSq
		    ) / dt;
	R(Y_accel::accel_bX, Y_accel::accel_bX) = cov;
	R(Y_accel::accel_bY, Y_accel::accel_bY) = cov;
	R(Y_accel::accel_bZ, Y_accel::accel_bZ) = cov;

	// define H
	Matrix<float, Y_accel::n, Xe::n> H;
	Matrix3f tmp = a_n_corrected.hat() * 2;

	for (size_t i = 0; i < 3; i++) {
		H(Y_accel::accel_bX + i, Xe::accel_bias_N + i) = 1;

		for (size_t j = 0; j < 3; j++) {
			H(Y_accel::accel_bX + i, Xe::rot_N + j) = tmp(i, j);
		}
	}

	// kalman correction
	SquareMatrix<float, Y_accel::n> S;
	_sensorAccel.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov::ACCEL_acc_X) = r(0);
	_innov(Innov::ACCEL_acc_Y) = r(1);
	_innov(Innov::ACCEL_acc_Z) = r(2);
	_innovStd(Innov::ACCEL_acc_X) = sqrtf(S(0, 0));
	_innovStd(Innov::ACCEL_acc_Y) = sqrtf(S(1, 1));
	_innovStd(Innov::ACCEL_acc_Z) = sqrtf(S(2, 2));

	if (_sensorAccel.shouldCorrect()) {
		nullPositionCorrection(_dxe);
		// don't allow yaw correction
		_dxe(Xe::rot_D) = 0;
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
