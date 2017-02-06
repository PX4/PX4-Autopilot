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

void IEKF::correctGyro(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;

	if (!_sensorGyro.ready(msg->timestamp, dt)) {
		return;
	}

	//ROS_INFO("correct gyro");

	// calculate residual
	Dcmf C_nb(Quatf(_x(X_q_nb_0), _x(X_q_nb_1),
			_x(X_q_nb_2), _x(X_q_nb_3)));
	Vector3f yh_n = C_nb * Vector3f(
				_x(X_angvel_bX),
				_x(X_angvel_bY),
				_x(X_angvel_bZ));
	Vector3f y_n = C_nb * Vector3f(
			       msg->gyro_rad[0],
			       msg->gyro_rad[1],
			       msg->gyro_rad[2]);

	// calculate residual in navigation frame
	// since it is invariant
	Vector<float, Y_gyro_n> r = y_n - yh_n;

	// define R
	SquareMatrix<float, Y_gyro_n> R;
	float gyro_var = _gyro_nd * _gyro_nd / dt;
	R(Y_gyro_angvel_N, Y_gyro_angvel_N) = gyro_var;
	R(Y_gyro_angvel_E, Y_gyro_angvel_E) = gyro_var;
	R(Y_gyro_angvel_D, Y_gyro_angvel_D) = gyro_var;

	// define H
	Matrix<float, Y_gyro_n, Xe_n> H;
	Matrix3f tmp = yh_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		H(Y_gyro_angvel_N + i, Xe_angvel_N + i) = 1;
		H(Y_gyro_angvel_N + i, Xe_gyro_bias_N + i) = 1;

		for (size_t j = 0; j < 3; j++) {
			H(Y_gyro_angvel_N + i, Xe_rot_N + j) = tmp(i, j);
		}
	};

	// kalman correction
	SquareMatrix<float, Y_gyro_n> S;

	_sensorGyro.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	for (int i = 0; i < 3; i++) {
		_innov(Innov_GYRO_angvel_N + i) = r(i);
		_innovStd(Innov_GYRO_angvel_N + i) = sqrtf(S(i, i));
	}

	if (_sensorGyro.shouldCorrect()) {
		Vector<float, X_n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
