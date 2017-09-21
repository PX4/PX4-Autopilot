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

	//PX4_INFO("correct baro");

	// init origin alt
	if (!_origin.altInitialized()) {
		PX4_INFO("baro origin init alt %12.2f m", double(msg->baro_alt_meter));
		_origin.altInitialize(msg->baro_alt_meter, msg->timestamp);
		_x(X::baro_bias) = 0;
		_baroOffset = 0;
		float alt_m = msg->baro_alt_meter;
		_x(X::asl) = alt_m;
		_baroOffset = 0;

		// if we have no terrain data, guess we are on the ground
		if (!getTerrainValid()) {
			_x(X::terrain_asl) = alt_m;
		}
	}

	// calculate residual
	Vector<float, Y_baro::n> y;
	y(Y_baro::asl) = msg->baro_alt_meter;
	Vector<float, Y_baro::n> yh;
	yh(Y_baro::asl)	= _x(X::asl) + _x(X::baro_bias) + _baroOffset;
	Vector<float, Y_baro::n> r = y - yh;

	//PX4_INFO("asl: %10g, bias: %10g, offset: %10g, r: %10g",
	//double(_x(X::asl)), double(_x(X::baro_bias)), double(_baroOffset),
	//double(r(0)));

	// save pressure altitude info
	_baroAsl = y(0);

	// define R
	SquareMatrix<float, Y_baro::n> R;
	R(Y_baro::asl, Y_baro::asl) = _baro_nd.get() * _baro_nd.get() / dt;
	//PX4_INFO("baro dt: %10.4f, variance: %10.4f", double(dt), double(sqrtf(R(0, 0))));

	// define H
	Matrix<float, Y_baro::n, Xe::n> H;
	H(Y_baro::asl, Xe::asl) = 1;
	H(Y_baro::asl, Xe::baro_bias) = 1;

	// kalman correction
	SquareMatrix<float, Y_baro::n> S;
	_sensorBaro.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov::BARO_asl) = r(0);
	_innovStd(Innov::BARO_asl) = sqrtf(S(0, 0));

	if (_sensorBaro.shouldCorrect()) {
		nullAttitudeCorrection(_dxe);
		// don't allow position correction in north/ east
		_dxe(Xe::pos_N) = 0;
		_dxe(Xe::pos_E) = 0;
		_dxe(Xe::vel_N) = 0;
		_dxe(Xe::vel_E) = 0;

		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
