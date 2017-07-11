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

void IEKF::correctAirspeed(const airspeed_s *msg)
{
	// return if no new data
	float dt = 0;

	if (!_sensorAirspeed.ready(msg->timestamp, dt)) {
		return;
	}

	//PX4_INFO("correct airspeed");

	// attitude info
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Dcmf C_nb = q_nb;

	// predicted airspeed
	//Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f wind_n(0, 0, 0);
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float yh = -wind_rel_b(0); // body -x component aligned with pitot tube
	float vel_N = _x(X::vel_N);
	float vel_E = _x(X::vel_E);
	float vel_D = _x(X::vel_D);
	float wind_N = wind_n(0);
	float wind_E = wind_n(1);
	float wind_D = wind_n(2);

	// abort if too large of an angle on pitot tube
	float wind_angle = acosf(-wind_rel_b(0)) / wind_rel_b.norm();
	static const float wind_angle_max = 30.0f * deg2radf;

	if (wind_angle > wind_angle_max) {
		if (wind_rel_b.norm() > 5.0f) {
			// if airspeed magnitude large warn the user, this could be a problem
			PX4_WARN("relative wind angle too large for pitot tube correction");
		}

		return;
	}

	// measured airspeed
	float y = msg->true_airspeed_unfiltered_m_s;

	Vector<float, 1> r;
	r(0) = y - yh;

	// define R
	SquareMatrix<float, Y_airspeed::n> R;
	R(Y_airspeed::airspeed, Y_airspeed::airspeed) = 100.0f / dt;

	// define H
	// Note: this measurement is not invariant due to
	// rotation matrix
	Matrix<float, Y_airspeed::n, Xe::n> H;
	float x0 = 2 * C_nb(1, 0);
	float x1 = vel_D - wind_D;
	float x2 = 2 * C_nb(2, 0);
	float x3 = vel_E - wind_E;
	float x4 = 2 * C_nb(0, 0);
	float x5 = vel_N - wind_N;
	H(Y_airspeed::airspeed, Xe::rot_N) = x0 * x1 - x2 * x3;
	H(Y_airspeed::airspeed, Xe::rot_E) = -x1 * x4 + x2 * x5;
	H(Y_airspeed::airspeed, Xe::rot_D) = -x0 * x5 + x3 * x4;
	H(Y_airspeed::airspeed, Xe::vel_N) = C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::vel_E) = C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::vel_D) = C_nb(2, 0);
	//H(Y_airspeed::airspeed, Xe::wind_N) = -C_nb(0, 0);
	//H(Y_airspeed::airspeed, Xe::wind_E) = -C_nb(1, 0);
	//H(Y_airspeed::airspeed, Xe::wind_D) = -C_nb(2, 0);

	// kalman correction
	SquareMatrix<float, Y_airspeed::n> S;
	_sensorAirspeed.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov::PITOT_airspeed) = r(0);
	_innovStd(Innov::PITOT_airspeed) = sqrtf(S(0, 0));

	if (_sensorAirspeed.shouldCorrect()) {
		nullAttitudeCorrection(_dxe);
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_P);
	}
}
