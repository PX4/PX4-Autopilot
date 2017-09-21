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

void IEKF::correctMag(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (!_sensorMag.ready(timestamp, dt)) {
		return;
	}

	//ROS_INFO("correct mag");

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_b = Vector3f(
			       msg->magnetometer_ga[0],
			       msg->magnetometer_ga[1],
			       msg->magnetometer_ga[2]).unit();
	Vector3f y_n = q_nb.conjugate(y_b);
	y_n(2) = 0;
	Vector3f yh_n = Dcmf(Eulerf(
				     0, 0, deg2radf * _mag_decl_deg.get())).T() * Vector3f(1, 0, 0);
	Vector<float, 1> r;
	r(0) = asinf(y_n.cross(yh_n)(2)) / y_n.norm() / yh_n.norm();
	//ROS_INFO("mag r: %10.4f\n", double(r(0)));

	// define R
	SquareMatrix<float, Y_mag::n> R;
	R(Y_mag::hdg, Y_mag::hdg) = _mag_nd.get() * _mag_nd.get() / dt;

	// define H
	Matrix<float, Y_mag::n, Xe::n> H;
	H(Y_mag::hdg, Xe::rot_D) = 1;

	// kalman correction
	SquareMatrix<float, Y_mag::n> S;
	_sensorMag.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov::MAG_mag_hdg) = r(0);
	_innovStd(Innov::MAG_mag_hdg) = sqrtf(S(0, 0));

	if (_sensorMag.shouldCorrect()) {
		// don't allow correction of roll/ pitch
		_dxe(Xe::rot_N) = 0;
		_dxe(Xe::rot_E) = 0;
		nullPositionCorrection(_dxe);
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
