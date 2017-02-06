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

void IEKF::callbackLand(const vehicle_land_detected_s *msg)
{
	//ROS_INFO("callback land");
	_landed = msg->landed;
	_freefall = msg->freefall;
}

void IEKF::correctLand(uint64_t timestamp)
{
	// return if not landed
	if (!_landed) {
		return;
	}

	// return if too far from ground
	if (getAgl() > 0.2f) {
		return;
	}

	// require agl < 0.3 m
	//

	// return if no new data or too fast
	float dt = 0;

	if (!_sensorLand.ready(timestamp, dt)) {
		return;
	}

	//ROS_INFO("correct land");

	// init global reference
	if (_origin.altInitialized() && !_origin.xyInitialized()) {
		float lat_deg = 47.397742f;
		float lon_deg = 8.545594f;
		ROS_INFO("land origin init lat: %12.6f deg lon: %12.6f deg",
			 double(lat_deg), double(lon_deg));
		_origin.xyInitialize(lat_deg, lon_deg, timestamp);
	}

	// calculate residual
	Vector<float, Y_land_n> y;
	y(Y_land_vel_N) = 0;
	y(Y_land_vel_E) = 0;
	y(Y_land_vel_D) = 0;
	y(Y_land_agl) = 0;

	//ROS_INFO("y");
	//y.print();

	Vector<float, Y_land_n> yh;
	yh(Y_land_vel_N) = _x(X_vel_N);
	yh(Y_land_vel_E) = _x(X_vel_E);
	yh(Y_land_vel_D) = _x(X_vel_D);
	yh(Y_land_agl) = getAgl();

	//ROS_INFO("yh");
	//yh.print();

	//ROS_INFO("terrain %10.4f", double(_x(X_terrain_asl)));
	//ROS_INFO("asl %10.4f", double(_x(X_asl)));

	Vector<float, Y_land_n> r = y - yh;

	// define R
	SquareMatrix<float, Y_land_n> R;
	float var_vxy = _land_vxy_nd * _land_vxy_nd / dt;
	float var_vz = _land_vz_nd * _land_vz_nd / dt;
	float var_agl = _land_agl_nd * _land_agl_nd / dt;

	R(Y_land_vel_N, Y_land_vel_N) = var_vxy;
	R(Y_land_vel_E, Y_land_vel_E) = var_vxy;
	R(Y_land_vel_D, Y_land_vel_D) = var_vz;
	R(Y_land_agl, Y_land_agl) = var_agl;

	// define H
	Matrix<float, Y_land_n, Xe_n> H;
	H(Y_land_vel_N, Xe_vel_N) = 1;
	H(Y_land_vel_E, Xe_vel_E) = 1;
	H(Y_land_vel_D, Xe_vel_D) = 1;
	H(Y_land_agl, Xe_asl) = 1;
	H(Y_land_agl, Xe_terrain_asl) = -1;

	// kalman correction
	SquareMatrix<float, Y_land_n> S;
	_sensorLand.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov_LAND_vel_N) = r(0);
	_innov(Innov_LAND_vel_E) = r(1);
	_innov(Innov_LAND_vel_D) = r(2);
	_innov(Innov_LAND_agl) = r(3);
	_innovStd(Innov_LAND_vel_N) = sqrtf(S(0, 0));
	_innovStd(Innov_LAND_vel_E) = sqrtf(S(1, 1));
	_innovStd(Innov_LAND_vel_D) = sqrtf(S(2, 2));
	_innovStd(Innov_LAND_agl) = sqrtf(S(3, 3));

	if (_sensorLand.shouldCorrect()) {
		// don't allow attitude correction
		nullAttitudeCorrection(_dxe);
		Vector<float, X_n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
