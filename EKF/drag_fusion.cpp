/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file drag_fusion.cpp
 * body frame drag fusion methods used for multi-rotor wind estimation.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

void Ekf::fuseDrag()
{
	float SH_ACC[4] = {}; // Variable used to optimise calculations of measurement jacobian
	float H_ACC[24] = {}; // Observation Jacobian
	float SK_ACC[9] = {}; // Variable used to optimise calculations of the Kalman gain vector
	float Kfusion[24] = {}; // Kalman gain vector
	// TODO: resolve variance vs stdDev bug
	const float R_ACC = _params.drag_noise; // observation noise variance in specific force drag (m/sec**2)**2

	const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

	// calculate inverse of ballistic coefficient
	if (_params.bcoef_x < 1.0f || _params.bcoef_y < 1.0f) {
		return;
	}

	const float BC_inv_x = 1.0f / _params.bcoef_x;
	const float BC_inv_y = 1.0f / _params.bcoef_y;

	// get latest estimated orientation
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float vn = _state.vel(0);
	const float ve = _state.vel(1);
	const float vd = _state.vel(2);

	// get latest wind velocity in earth frame
	const float vwn = _state.wind_vel(0);
	const float vwe = _state.wind_vel(1);

	// predicted specific forces
	// calculate relative wind velocity in earth frame and rotte into body frame
	Vector3f rel_wind;
	rel_wind(0) = vn - vwn;
	rel_wind(1) = ve - vwe;
	rel_wind(2) = vd;
	const Dcmf earth_to_body = quat_to_invrotmat(_state.quat_nominal);
	rel_wind = earth_to_body * rel_wind;

	// perform sequential fusion of XY specific forces
	for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
		// calculate observation jacobiam and Kalman gain vectors
		if (axis_index == 0) {
			// Estimate the airspeed from the measured drag force and ballistic coefficient
			const float mea_acc = _drag_sample_delayed.accelXY(axis_index)  - _state.delta_vel_bias(axis_index) / _dt_ekf_avg;
			const float airSpd = sqrtf((2.0f * fabsf(mea_acc)) / (BC_inv_x * rho));

			// Estimate the derivative of specific force wrt airspeed along the X axis
			// Limit lower value to prevent arithmetic exceptions
			const float Kacc = fmaxf(1e-1f, rho * BC_inv_x * airSpd);

			SH_ACC[0] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
			SH_ACC[1] = vn - vwn;
			SH_ACC[2] = ve - vwe;
			SH_ACC[3] = 2.0f*q0*q3 + 2.0f*q1*q2;
			H_ACC[0] = -Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd);
			H_ACC[1] = -Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd);
			H_ACC[2] = Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd);
			H_ACC[3] = -Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd);
			H_ACC[4] = -Kacc*SH_ACC[0];
			H_ACC[5] = -Kacc*SH_ACC[3];
			H_ACC[6] = Kacc*(2.0f*q0*q2 - 2.0f*q1*q3);
			H_ACC[22] = Kacc*SH_ACC[0];
			H_ACC[23] = Kacc*SH_ACC[3];
			_drag_innov_var[0] = (R_ACC + Kacc*SH_ACC[0]*(Kacc*P(4,4)*SH_ACC[0] + Kacc*P(5,4)*SH_ACC[3] - Kacc*P(22,4)*SH_ACC[0] - Kacc*P(23,4)*SH_ACC[3] - Kacc*P(6,4)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,4)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,4)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,4)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,4)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*SH_ACC[3]*(Kacc*P(4,5)*SH_ACC[0] + Kacc*P(5,5)*SH_ACC[3] - Kacc*P(22,5)*SH_ACC[0] - Kacc*P(23,5)*SH_ACC[3] - Kacc*P(6,5)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,5)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,5)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,5)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,5)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*SH_ACC[0]*(Kacc*P(4,22)*SH_ACC[0] + Kacc*P(5,22)*SH_ACC[3] - Kacc*P(22,22)*SH_ACC[0] - Kacc*P(23,22)*SH_ACC[3] - Kacc*P(6,22)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,22)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,22)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,22)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,22)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*SH_ACC[3]*(Kacc*P(4,23)*SH_ACC[0] + Kacc*P(5,23)*SH_ACC[3] - Kacc*P(22,23)*SH_ACC[0] - Kacc*P(23,23)*SH_ACC[3] - Kacc*P(6,23)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,23)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,23)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,23)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,23)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*(2.0f*q0*q2 - 2.0f*q1*q3)*(Kacc*P(4,6)*SH_ACC[0] + Kacc*P(5,6)*SH_ACC[3] - Kacc*P(22,6)*SH_ACC[0] - Kacc*P(23,6)*SH_ACC[3] - Kacc*P(6,6)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,6)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,6)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,6)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,6)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)*(Kacc*P(4,0)*SH_ACC[0] + Kacc*P(5,0)*SH_ACC[3] - Kacc*P(22,0)*SH_ACC[0] - Kacc*P(23,0)*SH_ACC[3] - Kacc*P(6,0)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,0)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,0)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,0)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,0)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd)*(Kacc*P(4,1)*SH_ACC[0] + Kacc*P(5,1)*SH_ACC[3] - Kacc*P(22,1)*SH_ACC[0] - Kacc*P(23,1)*SH_ACC[3] - Kacc*P(6,1)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,1)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,1)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,1)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,1)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd)*(Kacc*P(4,2)*SH_ACC[0] + Kacc*P(5,2)*SH_ACC[3] - Kacc*P(22,2)*SH_ACC[0] - Kacc*P(23,2)*SH_ACC[3] - Kacc*P(6,2)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,2)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,2)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,2)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,2)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)*(Kacc*P(4,3)*SH_ACC[0] + Kacc*P(5,3)*SH_ACC[3] - Kacc*P(22,3)*SH_ACC[0] - Kacc*P(23,3)*SH_ACC[3] - Kacc*P(6,3)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,3)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,3)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,3)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,3)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)));
			if (_drag_innov_var[0] < R_ACC) {
				return;
			}
			SK_ACC[0] = 1.0f/_drag_innov_var[0];
			SK_ACC[1] = 2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd;
			SK_ACC[2] = 2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd;
			SK_ACC[3] = 2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd;
			SK_ACC[4] = 2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd;
			SK_ACC[5] = 2.0f*q0*q2 - 2.0f*q1*q3;
			SK_ACC[6] = SH_ACC[3];
// Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
//			Kfusion[0] = -SK_ACC[0]*(Kacc*P(0,4)*SH_ACC[0] - Kacc*P(0,22)*SH_ACC[0] + Kacc*P(0,0)*SK_ACC[3] - Kacc*P(0,2)*SK_ACC[2] + Kacc*P(0,3)*SK_ACC[1] + Kacc*P(0,1)*SK_ACC[4] + Kacc*P(0,5)*SK_ACC[6] - Kacc*P(0,6)*SK_ACC[5] - Kacc*P(0,23)*SK_ACC[6]);
//			Kfusion[1] = -SK_ACC[0]*(Kacc*P(1,4)*SH_ACC[0] - Kacc*P(1,22)*SH_ACC[0] + Kacc*P(1,0)*SK_ACC[3] - Kacc*P(1,2)*SK_ACC[2] + Kacc*P(1,3)*SK_ACC[1] + Kacc*P(1,1)*SK_ACC[4] + Kacc*P(1,5)*SK_ACC[6] - Kacc*P(1,6)*SK_ACC[5] - Kacc*P(1,23)*SK_ACC[6]);
//			Kfusion[2] = -SK_ACC[0]*(Kacc*P(2,4)*SH_ACC[0] - Kacc*P(2,22)*SH_ACC[0] + Kacc*P(2,0)*SK_ACC[3] - Kacc*P(2,2)*SK_ACC[2] + Kacc*P(2,3)*SK_ACC[1] + Kacc*P(2,1)*SK_ACC[4] + Kacc*P(2,5)*SK_ACC[6] - Kacc*P(2,6)*SK_ACC[5] - Kacc*P(2,23)*SK_ACC[6]);
//			Kfusion[3] = -SK_ACC[0]*(Kacc*P(3,4)*SH_ACC[0] - Kacc*P(3,22)*SH_ACC[0] + Kacc*P(3,0)*SK_ACC[3] - Kacc*P(3,2)*SK_ACC[2] + Kacc*P(3,3)*SK_ACC[1] + Kacc*P(3,1)*SK_ACC[4] + Kacc*P(3,5)*SK_ACC[6] - Kacc*P(3,6)*SK_ACC[5] - Kacc*P(3,23)*SK_ACC[6]);
//			Kfusion[4] = -SK_ACC[0]*(Kacc*P(4,4)*SH_ACC[0] - Kacc*P(4,22)*SH_ACC[0] + Kacc*P(4,0)*SK_ACC[3] - Kacc*P(4,2)*SK_ACC[2] + Kacc*P(4,3)*SK_ACC[1] + Kacc*P(4,1)*SK_ACC[4] + Kacc*P(4,5)*SK_ACC[6] - Kacc*P(4,6)*SK_ACC[5] - Kacc*P(4,23)*SK_ACC[6]);
//			Kfusion[5] = -SK_ACC[0]*(Kacc*P(5,4)*SH_ACC[0] - Kacc*P(5,22)*SH_ACC[0] + Kacc*P(5,0)*SK_ACC[3] - Kacc*P(5,2)*SK_ACC[2] + Kacc*P(5,3)*SK_ACC[1] + Kacc*P(5,1)*SK_ACC[4] + Kacc*P(5,5)*SK_ACC[6] - Kacc*P(5,6)*SK_ACC[5] - Kacc*P(5,23)*SK_ACC[6]);
//			Kfusion[6] = -SK_ACC[0]*(Kacc*P(6,4)*SH_ACC[0] - Kacc*P(6,22)*SH_ACC[0] + Kacc*P(6,0)*SK_ACC[3] - Kacc*P(6,2)*SK_ACC[2] + Kacc*P(6,3)*SK_ACC[1] + Kacc*P(6,1)*SK_ACC[4] + Kacc*P(6,5)*SK_ACC[6] - Kacc*P(6,6)*SK_ACC[5] - Kacc*P(6,23)*SK_ACC[6]);
//			Kfusion[7] = -SK_ACC[0]*(Kacc*P(7,4)*SH_ACC[0] - Kacc*P(7,22)*SH_ACC[0] + Kacc*P(7,0)*SK_ACC[3] - Kacc*P(7,2)*SK_ACC[2] + Kacc*P(7,3)*SK_ACC[1] + Kacc*P(7,1)*SK_ACC[4] + Kacc*P(7,5)*SK_ACC[6] - Kacc*P(7,6)*SK_ACC[5] - Kacc*P(7,23)*SK_ACC[6]);
//			Kfusion[8] = -SK_ACC[0]*(Kacc*P(8,4)*SH_ACC[0] - Kacc*P(8,22)*SH_ACC[0] + Kacc*P(8,0)*SK_ACC[3] - Kacc*P(8,2)*SK_ACC[2] + Kacc*P(8,3)*SK_ACC[1] + Kacc*P(8,1)*SK_ACC[4] + Kacc*P(8,5)*SK_ACC[6] - Kacc*P(8,6)*SK_ACC[5] - Kacc*P(8,23)*SK_ACC[6]);
//			Kfusion[9] = -SK_ACC[0]*(Kacc*P(9,4)*SH_ACC[0] - Kacc*P(9,22)*SH_ACC[0] + Kacc*P(9,0)*SK_ACC[3] - Kacc*P(9,2)*SK_ACC[2] + Kacc*P(9,3)*SK_ACC[1] + Kacc*P(9,1)*SK_ACC[4] + Kacc*P(9,5)*SK_ACC[6] - Kacc*P(9,6)*SK_ACC[5] - Kacc*P(9,23)*SK_ACC[6]);
//			Kfusion[10] = -SK_ACC[0]*(Kacc*P(10,4)*SH_ACC[0] - Kacc*P(10,22)*SH_ACC[0] + Kacc*P(10,0)*SK_ACC[3] - Kacc*P(10,2)*SK_ACC[2] + Kacc*P(10,3)*SK_ACC[1] + Kacc*P(10,1)*SK_ACC[4] + Kacc*P(10,5)*SK_ACC[6] - Kacc*P(10,6)*SK_ACC[5] - Kacc*P(10,23)*SK_ACC[6]);
//			Kfusion[11] = -SK_ACC[0]*(Kacc*P(11,4)*SH_ACC[0] - Kacc*P(11,22)*SH_ACC[0] + Kacc*P(11,0)*SK_ACC[3] - Kacc*P(11,2)*SK_ACC[2] + Kacc*P(11,3)*SK_ACC[1] + Kacc*P(11,1)*SK_ACC[4] + Kacc*P(11,5)*SK_ACC[6] - Kacc*P(11,6)*SK_ACC[5] - Kacc*P(11,23)*SK_ACC[6]);
//			Kfusion[12] = -SK_ACC[0]*(Kacc*P(12,4)*SH_ACC[0] - Kacc*P(12,22)*SH_ACC[0] + Kacc*P(12,0)*SK_ACC[3] - Kacc*P(12,2)*SK_ACC[2] + Kacc*P(12,3)*SK_ACC[1] + Kacc*P(12,1)*SK_ACC[4] + Kacc*P(12,5)*SK_ACC[6] - Kacc*P(12,6)*SK_ACC[5] - Kacc*P(12,23)*SK_ACC[6]);
//			Kfusion[13] = -SK_ACC[0]*(Kacc*P(13,4)*SH_ACC[0] - Kacc*P(13,22)*SH_ACC[0] + Kacc*P(13,0)*SK_ACC[3] - Kacc*P(13,2)*SK_ACC[2] + Kacc*P(13,3)*SK_ACC[1] + Kacc*P(13,1)*SK_ACC[4] + Kacc*P(13,5)*SK_ACC[6] - Kacc*P(13,6)*SK_ACC[5] - Kacc*P(13,23)*SK_ACC[6]);
//			Kfusion[14] = -SK_ACC[0]*(Kacc*P(14,4)*SH_ACC[0] - Kacc*P(14,22)*SH_ACC[0] + Kacc*P(14,0)*SK_ACC[3] - Kacc*P(14,2)*SK_ACC[2] + Kacc*P(14,3)*SK_ACC[1] + Kacc*P(14,1)*SK_ACC[4] + Kacc*P(14,5)*SK_ACC[6] - Kacc*P(14,6)*SK_ACC[5] - Kacc*P(14,23)*SK_ACC[6]);
//			Kfusion[15] = -SK_ACC[0]*(Kacc*P(15,4)*SH_ACC[0] - Kacc*P(15,22)*SH_ACC[0] + Kacc*P(15,0)*SK_ACC[3] - Kacc*P(15,2)*SK_ACC[2] + Kacc*P(15,3)*SK_ACC[1] + Kacc*P(15,1)*SK_ACC[4] + Kacc*P(15,5)*SK_ACC[6] - Kacc*P(15,6)*SK_ACC[5] - Kacc*P(15,23)*SK_ACC[6]);
//			Kfusion[16] = -SK_ACC[0]*(Kacc*P(16,4)*SH_ACC[0] - Kacc*P(16,22)*SH_ACC[0] + Kacc*P(16,0)*SK_ACC[3] - Kacc*P(16,2)*SK_ACC[2] + Kacc*P(16,3)*SK_ACC[1] + Kacc*P(16,1)*SK_ACC[4] + Kacc*P(16,5)*SK_ACC[6] - Kacc*P(16,6)*SK_ACC[5] - Kacc*P(16,23)*SK_ACC[6]);
//			Kfusion[17] = -SK_ACC[0]*(Kacc*P(17,4)*SH_ACC[0] - Kacc*P(17,22)*SH_ACC[0] + Kacc*P(17,0)*SK_ACC[3] - Kacc*P(17,2)*SK_ACC[2] + Kacc*P(17,3)*SK_ACC[1] + Kacc*P(17,1)*SK_ACC[4] + Kacc*P(17,5)*SK_ACC[6] - Kacc*P(17,6)*SK_ACC[5] - Kacc*P(17,23)*SK_ACC[6]);
//			Kfusion[18] = -SK_ACC[0]*(Kacc*P(18,4)*SH_ACC[0] - Kacc*P(18,22)*SH_ACC[0] + Kacc*P(18,0)*SK_ACC[3] - Kacc*P(18,2)*SK_ACC[2] + Kacc*P(18,3)*SK_ACC[1] + Kacc*P(18,1)*SK_ACC[4] + Kacc*P(18,5)*SK_ACC[6] - Kacc*P(18,6)*SK_ACC[5] - Kacc*P(18,23)*SK_ACC[6]);
//			Kfusion[19] = -SK_ACC[0]*(Kacc*P(19,4)*SH_ACC[0] - Kacc*P(19,22)*SH_ACC[0] + Kacc*P(19,0)*SK_ACC[3] - Kacc*P(19,2)*SK_ACC[2] + Kacc*P(19,3)*SK_ACC[1] + Kacc*P(19,1)*SK_ACC[4] + Kacc*P(19,5)*SK_ACC[6] - Kacc*P(19,6)*SK_ACC[5] - Kacc*P(19,23)*SK_ACC[6]);
//			Kfusion[20] = -SK_ACC[0]*(Kacc*P(20,4)*SH_ACC[0] - Kacc*P(20,22)*SH_ACC[0] + Kacc*P(20,0)*SK_ACC[3] - Kacc*P(20,2)*SK_ACC[2] + Kacc*P(20,3)*SK_ACC[1] + Kacc*P(20,1)*SK_ACC[4] + Kacc*P(20,5)*SK_ACC[6] - Kacc*P(20,6)*SK_ACC[5] - Kacc*P(20,23)*SK_ACC[6]);
//			Kfusion[21] = -SK_ACC[0]*(Kacc*P(21,4)*SH_ACC[0] - Kacc*P(21,22)*SH_ACC[0] + Kacc*P(21,0)*SK_ACC[3] - Kacc*P(21,2)*SK_ACC[2] + Kacc*P(21,3)*SK_ACC[1] + Kacc*P(21,1)*SK_ACC[4] + Kacc*P(21,5)*SK_ACC[6] - Kacc*P(21,6)*SK_ACC[5] - Kacc*P(21,23)*SK_ACC[6]);
			Kfusion[22] = -SK_ACC[0]*(Kacc*P(22,4)*SH_ACC[0] - Kacc*P(22,22)*SH_ACC[0] + Kacc*P(22,0)*SK_ACC[3] - Kacc*P(22,2)*SK_ACC[2] + Kacc*P(22,3)*SK_ACC[1] + Kacc*P(22,1)*SK_ACC[4] + Kacc*P(22,5)*SK_ACC[6] - Kacc*P(22,6)*SK_ACC[5] - Kacc*P(22,23)*SK_ACC[6]);
			Kfusion[23] = -SK_ACC[0]*(Kacc*P(23,4)*SH_ACC[0] - Kacc*P(23,22)*SH_ACC[0] + Kacc*P(23,0)*SK_ACC[3] - Kacc*P(23,2)*SK_ACC[2] + Kacc*P(23,3)*SK_ACC[1] + Kacc*P(23,1)*SK_ACC[4] + Kacc*P(23,5)*SK_ACC[6] - Kacc*P(23,6)*SK_ACC[5] - Kacc*P(23,23)*SK_ACC[6]);

			// calculate the predicted acceleration and innovation measured along the X body axis
			float drag_sign;

			if (rel_wind(axis_index) >= 0.0f) {
				drag_sign = 1.0f;

			} else {
				drag_sign = -1.0f;
			}

			const float predAccel = -BC_inv_x * 0.5f * rho * sq(rel_wind(axis_index)) * drag_sign;
			_drag_innov[axis_index] = predAccel - mea_acc;
			_drag_test_ratio[axis_index] = sq(_drag_innov[axis_index]) / (25.0f * _drag_innov_var[axis_index]);

		} else if (axis_index == 1) {
			// Estimate the airspeed from the measured drag force and ballistic coefficient
			const float mea_acc = _drag_sample_delayed.accelXY(axis_index)  - _state.delta_vel_bias(axis_index) / _dt_ekf_avg;
			const float airSpd = sqrtf((2.0f * fabsf(mea_acc)) / (BC_inv_y * rho));

			// Estimate the derivative of specific force wrt airspeed along the X axis
			// Limit lower value to prevent arithmetic exceptions
			const float Kacc = fmaxf(1e-1f, rho * BC_inv_y * airSpd);

			SH_ACC[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
			SH_ACC[1] = vn - vwn;
			SH_ACC[2] = ve - vwe;
			H_ACC[0] = -Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd);
			H_ACC[1] = -Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd);
			H_ACC[2] = -Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd);
			H_ACC[3] = Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd);
			H_ACC[4] = Kacc*(2.0f*q0*q3 - 2.0f*q1*q2);
			H_ACC[5] = -Kacc*SH_ACC[0];
			H_ACC[6] = -Kacc*(2.0f*q0*q1 + 2.0f*q2*q3);
			H_ACC[22] = -2.0f*Kacc*(q0*q3 - q1*q2);
			H_ACC[23] = Kacc*SH_ACC[0];
			_drag_innov_var[1] = (R_ACC + Kacc*SH_ACC[0]*(Kacc*P(5,5)*SH_ACC[0] - Kacc*P(23,5)*SH_ACC[0] - Kacc*P(4,5)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,5)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,5)*(q0*q3 - q1*q2) + Kacc*P(0,5)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,5)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,5)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,5)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) - Kacc*SH_ACC[0]*(Kacc*P(5,23)*SH_ACC[0] - Kacc*P(23,23)*SH_ACC[0] - Kacc*P(4,23)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,23)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,23)*(q0*q3 - q1*q2) + Kacc*P(0,23)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,23)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,23)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,23)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) - Kacc*(2.0f*q0*q3 - 2.0f*q1*q2)*(Kacc*P(5,4)*SH_ACC[0] - Kacc*P(23,4)*SH_ACC[0] - Kacc*P(4,4)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,4)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,4)*(q0*q3 - q1*q2) + Kacc*P(0,4)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,4)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,4)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,4)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q0*q1 + 2.0f*q2*q3)*(Kacc*P(5,6)*SH_ACC[0] - Kacc*P(23,6)*SH_ACC[0] - Kacc*P(4,6)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,6)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,6)*(q0*q3 - q1*q2) + Kacc*P(0,6)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,6)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,6)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,6)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + 2*Kacc*(q0*q3 - q1*q2)*(Kacc*P(5,22)*SH_ACC[0] - Kacc*P(23,22)*SH_ACC[0] - Kacc*P(4,22)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,22)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,22)*(q0*q3 - q1*q2) + Kacc*P(0,22)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,22)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,22)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,22)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)*(Kacc*P(5,0)*SH_ACC[0] - Kacc*P(23,0)*SH_ACC[0] - Kacc*P(4,0)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,0)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,0)*(q0*q3 - q1*q2) + Kacc*P(0,0)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,0)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,0)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,0)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd)*(Kacc*P(5,1)*SH_ACC[0] - Kacc*P(23,1)*SH_ACC[0] - Kacc*P(4,1)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,1)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,1)*(q0*q3 - q1*q2) + Kacc*P(0,1)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,1)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,1)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,1)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd)*(Kacc*P(5,2)*SH_ACC[0] - Kacc*P(23,2)*SH_ACC[0] - Kacc*P(4,2)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,2)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,2)*(q0*q3 - q1*q2) + Kacc*P(0,2)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,2)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,2)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,2)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) - Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)*(Kacc*P(5,3)*SH_ACC[0] - Kacc*P(23,3)*SH_ACC[0] - Kacc*P(4,3)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,3)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,3)*(q0*q3 - q1*q2) + Kacc*P(0,3)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,3)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,3)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,3)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)));
			if (_drag_innov_var[1] < R_ACC) {
				// calculation is badly conditioned
				return;
			}
			SK_ACC[0] = 1.0f/_drag_innov_var[1];
			SK_ACC[1] = 2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd;
			SK_ACC[2] = 2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd;
			SK_ACC[3] = 2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd;
			SK_ACC[4] = 2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd;
			SK_ACC[5] = 2.0f*q0*q3 - 2.0f*q1*q2;
			SK_ACC[6] = q0*q3 - q1*q2;
			SK_ACC[7] = 2.0f*q0*q1 + 2.0f*q2*q3;
			SK_ACC[8] = SH_ACC[0];
// Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
//			Kfusion[0] = -SK_ACC[0]*(Kacc*P(0,0)*SK_ACC[3] + Kacc*P(0,1)*SK_ACC[2] - Kacc*P(0,3)*SK_ACC[1] + Kacc*P(0,2)*SK_ACC[4] - Kacc*P(0,4)*SK_ACC[5] + Kacc*P(0,5)*SK_ACC[8] + Kacc*P(0,6)*SK_ACC[7] + 2*Kacc*P(0,22)*SK_ACC[6] - Kacc*P(0,23)*SK_ACC[8]);
//			Kfusion[1] = -SK_ACC[0]*(Kacc*P(1,0)*SK_ACC[3] + Kacc*P(1,1)*SK_ACC[2] - Kacc*P(1,3)*SK_ACC[1] + Kacc*P(1,2)*SK_ACC[4] - Kacc*P(1,4)*SK_ACC[5] + Kacc*P(1,5)*SK_ACC[8] + Kacc*P(1,6)*SK_ACC[7] + 2*Kacc*P(1,22)*SK_ACC[6] - Kacc*P(1,23)*SK_ACC[8]);
//			Kfusion[2] = -SK_ACC[0]*(Kacc*P(2,0)*SK_ACC[3] + Kacc*P(2,1)*SK_ACC[2] - Kacc*P(2,3)*SK_ACC[1] + Kacc*P(2,2)*SK_ACC[4] - Kacc*P(2,4)*SK_ACC[5] + Kacc*P(2,5)*SK_ACC[8] + Kacc*P(2,6)*SK_ACC[7] + 2*Kacc*P(2,22)*SK_ACC[6] - Kacc*P(2,23)*SK_ACC[8]);
//			Kfusion[3] = -SK_ACC[0]*(Kacc*P(3,0)*SK_ACC[3] + Kacc*P(3,1)*SK_ACC[2] - Kacc*P(3,3)*SK_ACC[1] + Kacc*P(3,2)*SK_ACC[4] - Kacc*P(3,4)*SK_ACC[5] + Kacc*P(3,5)*SK_ACC[8] + Kacc*P(3,6)*SK_ACC[7] + 2*Kacc*P(3,22)*SK_ACC[6] - Kacc*P(3,23)*SK_ACC[8]);
//			Kfusion[4] = -SK_ACC[0]*(Kacc*P(4,0)*SK_ACC[3] + Kacc*P(4,1)*SK_ACC[2] - Kacc*P(4,3)*SK_ACC[1] + Kacc*P(4,2)*SK_ACC[4] - Kacc*P(4,4)*SK_ACC[5] + Kacc*P(4,5)*SK_ACC[8] + Kacc*P(4,6)*SK_ACC[7] + 2*Kacc*P(4,22)*SK_ACC[6] - Kacc*P(4,23)*SK_ACC[8]);
//			Kfusion[5] = -SK_ACC[0]*(Kacc*P(5,0)*SK_ACC[3] + Kacc*P(5,1)*SK_ACC[2] - Kacc*P(5,3)*SK_ACC[1] + Kacc*P(5,2)*SK_ACC[4] - Kacc*P(5,4)*SK_ACC[5] + Kacc*P(5,5)*SK_ACC[8] + Kacc*P(5,6)*SK_ACC[7] + 2*Kacc*P(5,22)*SK_ACC[6] - Kacc*P(5,23)*SK_ACC[8]);
//			Kfusion[6] = -SK_ACC[0]*(Kacc*P(6,0)*SK_ACC[3] + Kacc*P(6,1)*SK_ACC[2] - Kacc*P(6,3)*SK_ACC[1] + Kacc*P(6,2)*SK_ACC[4] - Kacc*P(6,4)*SK_ACC[5] + Kacc*P(6,5)*SK_ACC[8] + Kacc*P(6,6)*SK_ACC[7] + 2*Kacc*P(6,22)*SK_ACC[6] - Kacc*P(6,23)*SK_ACC[8]);
//			Kfusion[7] = -SK_ACC[0]*(Kacc*P(7,0)*SK_ACC[3] + Kacc*P(7,1)*SK_ACC[2] - Kacc*P(7,3)*SK_ACC[1] + Kacc*P(7,2)*SK_ACC[4] - Kacc*P(7,4)*SK_ACC[5] + Kacc*P(7,5)*SK_ACC[8] + Kacc*P(7,6)*SK_ACC[7] + 2*Kacc*P(7,22)*SK_ACC[6] - Kacc*P(7,23)*SK_ACC[8]);
//			Kfusion[8] = -SK_ACC[0]*(Kacc*P(8,0)*SK_ACC[3] + Kacc*P(8,1)*SK_ACC[2] - Kacc*P(8,3)*SK_ACC[1] + Kacc*P(8,2)*SK_ACC[4] - Kacc*P(8,4)*SK_ACC[5] + Kacc*P(8,5)*SK_ACC[8] + Kacc*P(8,6)*SK_ACC[7] + 2*Kacc*P(8,22)*SK_ACC[6] - Kacc*P(8,23)*SK_ACC[8]);
//			Kfusion[9] = -SK_ACC[0]*(Kacc*P(9,0)*SK_ACC[3] + Kacc*P(9,1)*SK_ACC[2] - Kacc*P(9,3)*SK_ACC[1] + Kacc*P(9,2)*SK_ACC[4] - Kacc*P(9,4)*SK_ACC[5] + Kacc*P(9,5)*SK_ACC[8] + Kacc*P(9,6)*SK_ACC[7] + 2*Kacc*P(9,22)*SK_ACC[6] - Kacc*P(9,23)*SK_ACC[8]);
//			Kfusion[10] = -SK_ACC[0]*(Kacc*P(10,0)*SK_ACC[3] + Kacc*P(10,1)*SK_ACC[2] - Kacc*P(10,3)*SK_ACC[1] + Kacc*P(10,2)*SK_ACC[4] - Kacc*P(10,4)*SK_ACC[5] + Kacc*P(10,5)*SK_ACC[8] + Kacc*P(10,6)*SK_ACC[7] + 2*Kacc*P(10,22)*SK_ACC[6] - Kacc*P(10,23)*SK_ACC[8]);
//			Kfusion[11] = -SK_ACC[0]*(Kacc*P(11,0)*SK_ACC[3] + Kacc*P(11,1)*SK_ACC[2] - Kacc*P(11,3)*SK_ACC[1] + Kacc*P(11,2)*SK_ACC[4] - Kacc*P(11,4)*SK_ACC[5] + Kacc*P(11,5)*SK_ACC[8] + Kacc*P(11,6)*SK_ACC[7] + 2*Kacc*P(11,22)*SK_ACC[6] - Kacc*P(11,23)*SK_ACC[8]);
//			Kfusion[12] = -SK_ACC[0]*(Kacc*P(12,0)*SK_ACC[3] + Kacc*P(12,1)*SK_ACC[2] - Kacc*P(12,3)*SK_ACC[1] + Kacc*P(12,2)*SK_ACC[4] - Kacc*P(12,4)*SK_ACC[5] + Kacc*P(12,5)*SK_ACC[8] + Kacc*P(12,6)*SK_ACC[7] + 2*Kacc*P(12,22)*SK_ACC[6] - Kacc*P(12,23)*SK_ACC[8]);
//			Kfusion[13] = -SK_ACC[0]*(Kacc*P(13,0)*SK_ACC[3] + Kacc*P(13,1)*SK_ACC[2] - Kacc*P(13,3)*SK_ACC[1] + Kacc*P(13,2)*SK_ACC[4] - Kacc*P(13,4)*SK_ACC[5] + Kacc*P(13,5)*SK_ACC[8] + Kacc*P(13,6)*SK_ACC[7] + 2*Kacc*P(13,22)*SK_ACC[6] - Kacc*P(13,23)*SK_ACC[8]);
//			Kfusion[14] = -SK_ACC[0]*(Kacc*P(14,0)*SK_ACC[3] + Kacc*P(14,1)*SK_ACC[2] - Kacc*P(14,3)*SK_ACC[1] + Kacc*P(14,2)*SK_ACC[4] - Kacc*P(14,4)*SK_ACC[5] + Kacc*P(14,5)*SK_ACC[8] + Kacc*P(14,6)*SK_ACC[7] + 2*Kacc*P(14,22)*SK_ACC[6] - Kacc*P(14,23)*SK_ACC[8]);
//			Kfusion[15] = -SK_ACC[0]*(Kacc*P(15,0)*SK_ACC[3] + Kacc*P(15,1)*SK_ACC[2] - Kacc*P(15,3)*SK_ACC[1] + Kacc*P(15,2)*SK_ACC[4] - Kacc*P(15,4)*SK_ACC[5] + Kacc*P(15,5)*SK_ACC[8] + Kacc*P(15,6)*SK_ACC[7] + 2*Kacc*P(15,22)*SK_ACC[6] - Kacc*P(15,23)*SK_ACC[8]);
//			Kfusion[16] = -SK_ACC[0]*(Kacc*P(16,0)*SK_ACC[3] + Kacc*P(16,1)*SK_ACC[2] - Kacc*P(16,3)*SK_ACC[1] + Kacc*P(16,2)*SK_ACC[4] - Kacc*P(16,4)*SK_ACC[5] + Kacc*P(16,5)*SK_ACC[8] + Kacc*P(16,6)*SK_ACC[7] + 2*Kacc*P(16,22)*SK_ACC[6] - Kacc*P(16,23)*SK_ACC[8]);
//			Kfusion[17] = -SK_ACC[0]*(Kacc*P(17,0)*SK_ACC[3] + Kacc*P(17,1)*SK_ACC[2] - Kacc*P(17,3)*SK_ACC[1] + Kacc*P(17,2)*SK_ACC[4] - Kacc*P(17,4)*SK_ACC[5] + Kacc*P(17,5)*SK_ACC[8] + Kacc*P(17,6)*SK_ACC[7] + 2*Kacc*P(17,22)*SK_ACC[6] - Kacc*P(17,23)*SK_ACC[8]);
//			Kfusion[18] = -SK_ACC[0]*(Kacc*P(18,0)*SK_ACC[3] + Kacc*P(18,1)*SK_ACC[2] - Kacc*P(18,3)*SK_ACC[1] + Kacc*P(18,2)*SK_ACC[4] - Kacc*P(18,4)*SK_ACC[5] + Kacc*P(18,5)*SK_ACC[8] + Kacc*P(18,6)*SK_ACC[7] + 2*Kacc*P(18,22)*SK_ACC[6] - Kacc*P(18,23)*SK_ACC[8]);
//			Kfusion[19] = -SK_ACC[0]*(Kacc*P(19,0)*SK_ACC[3] + Kacc*P(19,1)*SK_ACC[2] - Kacc*P(19,3)*SK_ACC[1] + Kacc*P(19,2)*SK_ACC[4] - Kacc*P(19,4)*SK_ACC[5] + Kacc*P(19,5)*SK_ACC[8] + Kacc*P(19,6)*SK_ACC[7] + 2*Kacc*P(19,22)*SK_ACC[6] - Kacc*P(19,23)*SK_ACC[8]);
//			Kfusion[20] = -SK_ACC[0]*(Kacc*P(20,0)*SK_ACC[3] + Kacc*P(20,1)*SK_ACC[2] - Kacc*P(20,3)*SK_ACC[1] + Kacc*P(20,2)*SK_ACC[4] - Kacc*P(20,4)*SK_ACC[5] + Kacc*P(20,5)*SK_ACC[8] + Kacc*P(20,6)*SK_ACC[7] + 2*Kacc*P(20,22)*SK_ACC[6] - Kacc*P(20,23)*SK_ACC[8]);
//			Kfusion[21] = -SK_ACC[0]*(Kacc*P(21,0)*SK_ACC[3] + Kacc*P(21,1)*SK_ACC[2] - Kacc*P(21,3)*SK_ACC[1] + Kacc*P(21,2)*SK_ACC[4] - Kacc*P(21,4)*SK_ACC[5] + Kacc*P(21,5)*SK_ACC[8] + Kacc*P(21,6)*SK_ACC[7] + 2*Kacc*P(21,22)*SK_ACC[6] - Kacc*P(21,23)*SK_ACC[8]);
			Kfusion[22] = -SK_ACC[0]*(Kacc*P(22,0)*SK_ACC[3] + Kacc*P(22,1)*SK_ACC[2] - Kacc*P(22,3)*SK_ACC[1] + Kacc*P(22,2)*SK_ACC[4] - Kacc*P(22,4)*SK_ACC[5] + Kacc*P(22,5)*SK_ACC[8] + Kacc*P(22,6)*SK_ACC[7] + 2*Kacc*P(22,22)*SK_ACC[6] - Kacc*P(22,23)*SK_ACC[8]);
			Kfusion[23] = -SK_ACC[0]*(Kacc*P(23,0)*SK_ACC[3] + Kacc*P(23,1)*SK_ACC[2] - Kacc*P(23,3)*SK_ACC[1] + Kacc*P(23,2)*SK_ACC[4] - Kacc*P(23,4)*SK_ACC[5] + Kacc*P(23,5)*SK_ACC[8] + Kacc*P(23,6)*SK_ACC[7] + 2*Kacc*P(23,22)*SK_ACC[6] - Kacc*P(23,23)*SK_ACC[8]);

			// calculate the predicted acceleration and innovation measured along the Y body axis
			float drag_sign;

			if (rel_wind(axis_index) >= 0.0f) {
				drag_sign = 1.0f;

			} else {
				drag_sign = -1.0f;
			}

			const float predAccel = -BC_inv_y * 0.5f * rho * sq(rel_wind(axis_index)) * drag_sign;
			_drag_innov[axis_index] = predAccel - mea_acc;
			_drag_test_ratio[axis_index] = sq(_drag_innov[axis_index]) / (25.0f * _drag_innov_var[axis_index]);

		}

		// if the innovation consistency check fails then don't fuse the sample
		if (_drag_test_ratio[axis_index] <= 1.0f) {
			// apply covariance correction via P_new = (I -K*H)*P
			// first calculate expression for KHP
			// then calculate P - KHP
			matrix::SquareMatrix<float, _k_num_states> KHP;
			float KH[9];

			for (unsigned row = 0; row < _k_num_states; row++) {

				KH[0] = Kfusion[row] * H_ACC[0];
				KH[1] = Kfusion[row] * H_ACC[1];
				KH[2] = Kfusion[row] * H_ACC[2];
				KH[3] = Kfusion[row] * H_ACC[3];
				KH[4] = Kfusion[row] * H_ACC[4];
				KH[5] = Kfusion[row] * H_ACC[5];
				KH[6] = Kfusion[row] * H_ACC[6];
				KH[7] = Kfusion[row] * H_ACC[22];
				KH[8] = Kfusion[row] * H_ACC[23];

				for (unsigned column = 0; column < _k_num_states; column++) {
					float tmp = KH[0] * P(0,column);
					tmp += KH[1] * P(1,column);
					tmp += KH[2] * P(2,column);
					tmp += KH[3] * P(3,column);
					tmp += KH[4] * P(4,column);
					tmp += KH[5] * P(5,column);
					tmp += KH[6] * P(6,column);
					tmp += KH[7] * P(22,column);
					tmp += KH[8] * P(23,column);
					KHP(row,column) = tmp;
				}
			}

			// if the covariance correction will result in a negative variance, then
			// the covariance matrix is unhealthy and must be corrected
			bool healthy = true;

			//_fault_status.flags.bad_sideslip = false;
			for (int i = 0; i < _k_num_states; i++) {
				if (P(i,i) < KHP(i,i)) {
					// zero rows and columns
					P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

					//flag as unhealthy
					healthy = false;

					// update individual measurement health status
					//_fault_status.flags.bad_sideslip = true;

				}
			}

			// only apply covariance and state corrections if healthy
			if (healthy) {
				// apply the covariance corrections
				for (unsigned row = 0; row < _k_num_states; row++) {
					for (unsigned column = 0; column < _k_num_states; column++) {
						P(row,column) = P(row,column) - KHP(row,column);
					}
				}

				// correct the covariance matrix for gross errors
				fixCovarianceErrors(true);

				// apply the state corrections
				fuse(Kfusion, _drag_innov[axis_index]);

			}
		}
	}
}
