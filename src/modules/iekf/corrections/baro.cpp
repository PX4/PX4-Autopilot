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

void IEKF::correctBaro(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->baro_timestamp_relative;

	if (!_sensorBaro.ready(timestamp, dt)) {
		return;
	}

	//ROS_INFO("correct baro");

	// init origin alt
	if (!_origin.altInitialized()) {
		ROS_INFO("baro origin init alt %12.2f m", double(msg->baro_alt_meter));
		_origin.altInitialize(msg->baro_alt_meter, msg->timestamp);
		_x(X_baro_bias) = 0;
		_baroOffset = 0;
		float alt_m = msg->baro_alt_meter;
		_x(X_asl) = alt_m;
		// initial terrain guess
		_baroOffset = 0;
		_x(X_asl) = alt_m;

		// if we have no terrain data, guess we are on the ground
		if (!getTerrainValid()) {
			_x(X_terrain_asl) = alt_m;
		}
	}

	// calculate residual
	Vector<float, Y_baro_n> y;
	y(Y_baro_asl) = msg->baro_alt_meter;
	Vector<float, Y_baro_n> yh;
	yh(Y_baro_asl)	= _x(X_asl) + _x(X_baro_bias) + _baroOffset;
	Vector<float, Y_baro_n> r = y - yh;

	//ROS_INFO("asl: %10g, bias: %10g, offset: %10g, r: %10g",
	//double(_x(X_asl)), double(_x(X_baro_bias)), double(_baroOffset),
	//double(r(0)));

	// save pressure altitude info
	_baroAsl = y(0);

	// define R
	SquareMatrix<float, Y_baro_n> R;
	R(Y_baro_asl, Y_baro_asl) = _baro_nd * _baro_nd / dt;
	//ROS_INFO("baro dt: %10.4f, variance: %10.4f", double(dt), double(sqrtf(R(0, 0))));

	// define H
	Matrix<float, Y_baro_n, Xe_n> H;
	H(Y_baro_asl, Xe_asl) = 1;
	H(Y_baro_asl, Xe_baro_bias) = 1;

	// kalman correction
	SquareMatrix<float, Y_baro_n> S;
	_sensorBaro.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov_BARO_asl) = r(0);
	_innovStd(Innov_BARO_asl) = sqrtf(S(0, 0));

	if (_sensorBaro.shouldCorrect()) {
		nullAttitudeCorrection(_dxe);
		// don't allow position correction in north/ east
		_dxe(Xe_pos_N) = 0;
		_dxe(Xe_pos_E) = 0;
		_dxe(Xe_vel_N) = 0;
		_dxe(Xe_vel_E) = 0;

		Vector<float, X_n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
