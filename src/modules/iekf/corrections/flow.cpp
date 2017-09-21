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

void IEKF::correctFlow(const optical_flow_s *msg)
{
	// requires terrain estimate
	if (!getTerrainValid()) {
		//PX4_INFO("flow abort - terrain invalid");
		return;
	}

	// abort if flow quality low
	if (msg->quality < 100) {
		//PX4_INFO("flow abort - low quality %10d", msg->quality);
		return;
	}

	// return if no new data
	float dt = 0;

	if (!_sensorFlow.ready(msg->timestamp, dt)) {
		//PX4_INFO("flow not ready");;
		return;
	}

	//PX4_INFO("dt flow: %10.5f", double(dt));

	// compute agl
	float agl = getAgl();

	// init global reference
	if (_origin.altInitialized() && !_origin.xyInitialized()) {
		PX4_INFO("flow origin init lat: %12.6f deg lon: %12.6f deg",
			 fake_lat_deg, fake_lon_deg);
		_origin.xyInitialize(fake_lat_deg, fake_lon_deg, msg->timestamp);
	}

	// return if too close to ground
	if (agl < 0.2f) {
		//PX4_INFO("flow abort - too close to ground: %10.4f", double(agl));
		return;
	}

	// state info
	Dcmf C_nb = computeDcmNB();
	Vector3f angVelNB = getAngularVelocityNBFrameB();

	float vel_N = _x(X::vel_N);
	float vel_E = _x(X::vel_E);
	float vel_D = _x(X::vel_D);
	float omega_bx = angVelNB(0);
	float omega_by = angVelNB(1);
	float rotRate = angVelNB.norm();

	// abort if rotRate too high
	if (rotRate > 1.0f) {
		PX4_INFO("rotation rate too large for flow correction");
		return;
	}

	// abort if too large of an angle
	if (C_nb(2, 2) < 1e-1f) {
		PX4_INFO("flow correction aborted, too large of an angle");
		return;
	}

	// expected measurement
	Vector<float, Y_flow::n> yh;
	{
		float x0 = agl;
		float x1 = 1 / x0;
		float x2 = C_nb(2, 2);
		float x3 = vel_N;
		float x4 = vel_E;
		float x5 = vel_D;
		yh(0) = -x1 * (x0 * omega_by + x2 * (x3 * C_nb(0, 0) + x4 * C_nb(1, 0) + x5 * C_nb(2, 0)));
		yh(1) =  x1 * (x0 * omega_bx - x2 * (x3 * C_nb(0, 1) + x4 * C_nb(1, 1) + x5 * C_nb(2, 1)));
	}

	// measurement
	Vector2f  y;
	float flow_dt = msg->integration_timespan / 1.0e6f;
	y(0) = -msg->pixel_flow_y_integral / flow_dt;
	y(1) = msg->pixel_flow_x_integral / flow_dt;

	//PX4_INFO("flow vel X %10.4f vel Y %10.4f", double(agl*y(0)), double(agl*y(1)));
	//PX4_INFO("vel N %10.4f E %10.4f", double(vel_N), double(vel_E));

	// residual
	Vector<float, Y_flow::n> r = y - yh;

	//PX4_INFO("flow: (%10g, %10g)", double(y(0)), double(y(1)));
	//PX4_INFO("float dt: %10.4f", double(dt));

	// define R
	SquareMatrix<float, Y_flow::n> R;
	float flow_var = _flow_nd.get() * _flow_nd.get() / dt;
	R(Y_flow::flowX, Y_flow::flowX) = flow_var;
	R(Y_flow::flowY, Y_flow::flowY) = flow_var;

	// define H
	// Note: this measurement is not invariant due to
	// rotation matrix
	Matrix<float, Y_flow::n, Xe::n> H;
	{
		float x0 = agl;
		float x1 = 1 / x0;
		float x2 = 2 * C_nb(1, 2);
		float x3 = C_nb(0, 0);
		float x4 = vel_N;
		float x5 = x3 * x4;
		float x6 = C_nb(1, 0);
		float x7 = vel_E;
		float x8 = x6 * x7;
		float x9 = C_nb(2, 2);
		float x10 = vel_D;
		float x11 = 2 * x10 * x9;
		float x12 = C_nb(2, 0);
		float x13 = x10 * x12;
		float x14 = 2 * x7 * x9;
		float x15 = 2 * x1;
		float x16 = C_nb(0, 2);
		float x17 = x10 * x9;
		float x18 = x4 * x9;
		float x19 = 2 * x1 * x9;
		float x20 = x1 * x9;
		float x21 = C_nb(0, 1);
		float x22 = C_nb(1, 1);
		float x23 = C_nb(2, 1);
		float x24 = x9 / (x0 * x0);
		float x25 = x24 * (x13 + x5 + x8);
		float x26 = x21 * x4;
		float x27 = x22 * x7;
		float x28 = x10 * x23;
		float x29 = x24 * (x26 + x27 + x28);
		H(Y_flow::flowX, Xe::rot_N) = x1 * (-x11 * x6 + x12 * x14 - x13 * x2 - x2 * x5 - x2 * x8);
		H(Y_flow::flowX, Xe::rot_E) = x15 * (-x12 * x18 + x13 * x16 + x16 * x5 + x16 * x8 + x17 * x3);
		H(Y_flow::flowX, Xe::rot_D) = -x19 * (x3 * x7 - x4 * x6);
		H(Y_flow::flowX, Xe::vel_N) = -x20 * x3;
		H(Y_flow::flowX, Xe::vel_E) = -x20 * x6;
		H(Y_flow::flowX, Xe::vel_D) = -x12 * x20;
		H(Y_flow::flowX, Xe::gyro_bias_N) = x21;
		H(Y_flow::flowX, Xe::gyro_bias_E) = x22;
		H(Y_flow::flowX, Xe::gyro_bias_D) = x23;
		H(Y_flow::flowX, Xe::asl) = x25;
		H(Y_flow::flowX, Xe::terrain_asl) = -x25;
		H(Y_flow::flowY, Xe::rot_N) = x1 * (-x11 * x22 + x14 * x23 - x2 * x26 - x2 * x27 - x2 * x28);
		H(Y_flow::flowY, Xe::rot_E) = x15 * (x16 * x26 + x16 * x27 + x16 * x28 + x17 * x21 - x18 * x23);
		H(Y_flow::flowY, Xe::rot_D) = -x19 * (x21 * x7 - x22 * x4);
		H(Y_flow::flowY, Xe::vel_N) = -x20 * x21;
		H(Y_flow::flowY, Xe::vel_E) = -x20 * x22;
		H(Y_flow::flowY, Xe::vel_D) = -x20 * x23;
		H(Y_flow::flowY, Xe::gyro_bias_N) = -x3;
		H(Y_flow::flowY, Xe::gyro_bias_E) = -x6;
		H(Y_flow::flowY, Xe::gyro_bias_D) = -x12;
		H(Y_flow::flowY, Xe::asl) = x29;
		H(Y_flow::flowY, Xe::terrain_asl) = -x29;
	}

	// kalman correction
	SquareMatrix<float, Y_flow::n> S;
	_sensorFlow.kalmanCorrectCond(_P, H, R, r, _dxe, _dP, S);

	// store innovation
	_innov(Innov::FLOW_flow_X) = r(0);
	_innov(Innov::FLOW_flow_Y) = r(1);
	_innovStd(Innov::FLOW_flow_X) = sqrtf(S(0, 0));
	_innovStd(Innov::FLOW_flow_Y) = sqrtf(S(1, 1));

	if (_sensorFlow.shouldCorrect()) {
		//PX4_INFO("correct flow");
		// don't allow attitude correction
		nullAttitudeCorrection(_dxe);
		nullAltitudeCorrection(_dxe);
		Vector<float, X::n> dx = computeErrorCorrection(_dxe);
		incrementX(dx);
		incrementP(_dP);
	}
}
