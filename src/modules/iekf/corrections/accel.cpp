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

	// expected measurement
	Dcmf C_nb = Quatf(_x(X_q_nb_0), _x(X_q_nb_1),
			  _x(X_q_nb_2), _x(X_q_nb_3));
	Vector3f a_n = C_nb * transAccelFrameB(); // F/m
	Vector3f a_bias_b(_x(X_accel_bias_bX), _x(X_accel_bias_bY), _x(X_accel_bias_bZ));
	Vector3f yh_n = a_n - C_nb * a_bias_b + _g_n;

	// calculate residual in navigation frame
	// since it is invariant
	Vector3f r = C_nb * y_b - yh_n;

	// define R
	Matrix<float, Y_accel_n, Y_accel_n> R;
	float var = _accel_nd * _accel_nd / dt;
	R(Y_accel_acc_N, Y_accel_acc_N) = var;
	R(Y_accel_acc_E, Y_accel_acc_E) = var;
	R(Y_accel_acc_D, Y_accel_acc_D) = var;

	// define H
	Matrix<float, Y_accel_n, Xe_n> H;
	Matrix3f tmp = yh_n.hat() * 2;

	for (size_t i = 0; i < 3; i++) {
		H(Y_accel_acc_N + i, Xe_accel_bias_N + i) = 1;

		for (size_t j = 0; j < 3; j++) {
			H(Y_accel_acc_N + i, Xe_rot_N + j) = tmp(i, j);
		}
	}

	// kalman correction
	SquareMatrix<float, Y_accel_n> S;
	_sensorAccel.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov_ACCEL_acc_N) = r(0);
	_innov(Innov_ACCEL_acc_E) = r(1);
	_innov(Innov_ACCEL_acc_D) = r(2);
	_innovStd(Innov_ACCEL_acc_N) = sqrtf(S(0, 0));
	_innovStd(Innov_ACCEL_acc_E) = sqrtf(S(1, 1));
	_innovStd(Innov_ACCEL_acc_D) = sqrtf(S(2, 2));

	if (_sensorAccel.shouldCorrect()) {
		nullPositionCorrection(_dxe);
		// don't allow yaw correction
		_dxe(Xe_rot_D) = 0;
		Vector<float, X_n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
