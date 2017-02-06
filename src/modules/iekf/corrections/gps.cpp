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

void IEKF::correctGps(const vehicle_gps_position_s *msg)
{
	// return if no new data
	float dt = 0;

	if (!_sensorGps.ready(msg->timestamp, dt)) {
		return;
	}

	// check for good gps signal
	if (msg->satellites_used < 6 || msg->fix_type < 3) {
		return;
	}

	//ROS_INFO("correct gps");

	double lat_deg = msg->lat * 1e-7;
	double lon_deg = msg->lon * 1e-7;
	float alt_m = msg->alt * 1e-3;

	// init global reference
	if (!_origin.xyInitialized()) {
		ROS_INFO("gps origin init lat: %12.6f deg lon: %12.6f deg",
			 double(lat_deg), double(lon_deg));
		_origin.xyInitialize(lat_deg, lon_deg, msg->timestamp);

		// init origin alt
		ROS_INFO("gps origin init alt %12.2f m", double(alt_m));
		_origin.altInitialize(alt_m, msg->timestamp);
		float deltaAlt = alt_m - _baroAsl;
		_baroOffset -= deltaAlt;
		_x(X_asl) = alt_m;

		// if we have no terrain data, guess we are on the ground
		if (!getTerrainValid()) {
			_x(X_terrain_asl) = alt_m;

		} else {
			_x(X_terrain_asl) += deltaAlt;
		}

	}

	// calculate residual
	float gps_pos_N = 0;
	float gps_pos_E = 0;
	_origin.globalToLocalXY(lat_deg, lon_deg, gps_pos_N, gps_pos_E);

	Vector<float, Y_gps_n> y;
	y(Y_gps_pos_N) = gps_pos_N;
	y(Y_gps_pos_E) = gps_pos_E;
	y(Y_gps_asl) = alt_m;
	y(Y_gps_vel_N) = msg->vel_n_m_s;
	y(Y_gps_vel_E) = msg->vel_e_m_s;
	y(Y_gps_vel_D) = msg->vel_d_m_s;

	//ROS_INFO("gps vx: %10.4f , iekf vx: %10.4f",
	//double(msg->vel_n_m_s), double(_x(X_vel_N)));

	Vector<float, Y_gps_n> yh;
	yh(Y_gps_pos_N) = _x(X_pos_N);
	yh(Y_gps_pos_E) = _x(X_pos_E);
	yh(Y_gps_asl) = _x(X_asl);
	yh(Y_gps_vel_N) = _x(X_vel_N);
	yh(Y_gps_vel_E) = _x(X_vel_E);
	yh(Y_gps_vel_D) = _x(X_vel_D);

	Vector<float, Y_gps_n> r = y - yh;

	// define R
	Matrix<float, Y_gps_n, Y_gps_n> R;

	// variances
	float gps_xy_var = _gps_xy_nd * _gps_xy_nd / dt;
	float gps_z_var = _gps_z_nd * _gps_z_nd / dt;
	float gps_vxy_var = _gps_vxy_nd * _gps_vxy_nd;
	float gps_vz_var = _gps_vz_nd * _gps_vz_nd;

	R(Y_gps_pos_N, Y_gps_pos_N) = gps_xy_var;
	R(Y_gps_pos_E, Y_gps_pos_E) = gps_xy_var;
	R(Y_gps_asl, Y_gps_asl) = gps_z_var;
	R(Y_gps_vel_N, Y_gps_vel_N) = gps_vxy_var;
	R(Y_gps_vel_E, Y_gps_vel_E) = gps_vxy_var;
	R(Y_gps_vel_D, Y_gps_vel_D) = gps_vz_var;

	// define H
	Matrix<float, Y_gps_n, Xe_n> H;
	H(Y_gps_pos_N, Xe_pos_N) = 1;
	H(Y_gps_pos_E, Xe_pos_E) = 1;
	H(Y_gps_asl, Xe_asl) = 1;
	H(Y_gps_vel_N, Xe_vel_N) = 1;
	H(Y_gps_vel_E, Xe_vel_E) = 1;
	H(Y_gps_vel_D, Xe_vel_D) = 1;

	// kalman correction
	SquareMatrix<float, Y_gps_n> S;
	_sensorGps.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov_GPS_vel_N) = r(0);
	_innov(Innov_GPS_vel_E) = r(1);
	_innov(Innov_GPS_vel_D) = r(2);
	_innov(Innov_GPS_pos_N) = r(3);
	_innov(Innov_GPS_pos_E) = r(4);
	_innov(Innov_GPS_asl) = r(5);
	_innovStd(Innov_GPS_vel_N) = sqrtf(S(0, 0));
	_innovStd(Innov_GPS_vel_E) = sqrtf(S(1, 1));
	_innovStd(Innov_GPS_vel_D) = sqrtf(S(2, 2));
	_innovStd(Innov_GPS_pos_N) = sqrtf(S(3, 3));
	_innovStd(Innov_GPS_pos_N) = sqrtf(S(4, 4));
	_innovStd(Innov_GPS_asl) = sqrtf(S(5, 5));

	if (_sensorGps.shouldCorrect()) {
		// don't allow attitude correction
		nullAttitudeCorrection(_dxe);
		Vector<float, X_n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
