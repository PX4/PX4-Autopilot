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

void IEKF::correctVision(const vehicle_local_position_s *msg)
{
	float dt = 0;

	if (!_sensorVision.ready(msg->timestamp, dt)) {
		return;
	}

	// init global reference
	if (!_origin.xyInitialized()) {
		PX4_INFO("vision origin init lat: %12.6f deg lon: %12.6f deg",
			 double(fake_lat_deg), double(fake_lon_deg));
		_origin.xyInitialize(fake_lat_deg, fake_lon_deg, msg->timestamp);
	}

	Vector<float, Y_vision::n> y;
	y(Y_vision::pos_N) = msg->x;
	y(Y_vision::pos_E) = msg->y;
	y(Y_vision::pos_D) = msg->z;
	y(Y_vision::vel_N) = msg->vx;
	y(Y_vision::vel_E) = msg->vy;
	y(Y_vision::vel_D) = msg->vz;

	//PX4_INFO("vision vx: %10.4f , iekf vx: %10.4f",
	//double(msg->vel_n_m_s), double(_x(X::vel_N)));

	Vector<float, Y_vision::n> yh;
	yh(Y_vision::pos_N) = _x(X::pos_N);
	yh(Y_vision::pos_E) = _x(X::pos_E);
	yh(Y_vision::pos_D) = -_x(X::asl); // TODO really depends on vision origin, most don't use asl, maybe agl better?
	yh(Y_vision::vel_N) = _x(X::vel_N);
	yh(Y_vision::vel_E) = _x(X::vel_E);
	yh(Y_vision::vel_D) = _x(X::vel_D);

	Vector<float, Y_vision::n> r = y - yh;
	//PX4_INFO("r");
	//r.print();

	// define R
	Matrix<float, Y_vision::n, Y_vision::n> R;

	// variances
	float vision_xy_var = _vision_xy_nd.get() * _vision_xy_nd.get() / dt;
	float vision_z_var = _vision_z_nd.get() * _vision_z_nd.get() / dt;
	float vision_vxy_var = _vision_vxy_nd.get() * _vision_vxy_nd.get() / dt;
	float vision_vz_var = _vision_vz_nd.get() * _vision_vz_nd.get() / dt;

	R(Y_vision::pos_N, Y_vision::pos_N) = vision_xy_var;
	R(Y_vision::pos_E, Y_vision::pos_E) = vision_xy_var;
	R(Y_vision::pos_D, Y_vision::pos_D) = vision_z_var;
	R(Y_vision::vel_N, Y_vision::vel_N) = vision_vxy_var;
	R(Y_vision::vel_E, Y_vision::vel_E) = vision_vxy_var;
	R(Y_vision::vel_D, Y_vision::vel_D) = vision_vz_var;

	// define H
	Matrix<float, Y_vision::n, Xe::n> H;
	H(Y_vision::pos_N, Xe::pos_N) = 1;
	H(Y_vision::pos_E, Xe::pos_E) = 1;
	H(Y_vision::pos_D, Xe::asl) = -1;
	H(Y_vision::vel_N, Xe::vel_N) = 1;
	H(Y_vision::vel_E, Xe::vel_E) = 1;
	H(Y_vision::vel_D, Xe::vel_D) = 1;

	// kalman correction
	SquareMatrix<float, Y_vision::n> S;
	_sensorVision.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov::VISION_vel_N) = r(0);
	_innov(Innov::VISION_vel_E) = r(1);
	_innov(Innov::VISION_vel_D) = r(2);
	_innov(Innov::VISION_pos_N) = r(3);
	_innov(Innov::VISION_pos_E) = r(4);
	_innov(Innov::VISION_pos_D) = r(5);
	_innovStd(Innov::VISION_vel_N) = sqrtf(S(0, 0));
	_innovStd(Innov::VISION_vel_E) = sqrtf(S(1, 1));
	_innovStd(Innov::VISION_vel_D) = sqrtf(S(2, 2));
	_innovStd(Innov::VISION_pos_N) = sqrtf(S(3, 3));
	_innovStd(Innov::VISION_pos_E) = sqrtf(S(4, 4));
	_innovStd(Innov::VISION_pos_D) = sqrtf(S(5, 5));

	if (_sensorVision.shouldCorrect()) {
		PX4_INFO("doing vision correction");
		// don't allow attitude correction
		nullAttitudeCorrection(_dxe);
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);

	} else {
		PX4_INFO("aborting vision correction");
	}
}
