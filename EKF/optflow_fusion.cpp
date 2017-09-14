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
 * @file vel_pos_fusion.cpp
 * Function for fusing gps and baro measurements/
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

#include "ekf.h"
#include "mathlib.h"

void Ekf::fuseOptFlow()
{
	float gndclearance = fmaxf(_params.rng_gnd_clearance, 0.1f);
	float optflow_test_ratio[2] = {0};

	// get latest estimated orientation
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	float vn = _state.vel(0);
	float ve = _state.vel(1);
	float vd = _state.vel(2);

	// calculate the optical flow observation variance
	float R_LOS = calcOptFlowMeasVar();

	float H_LOS[2][24] = {}; // Optical flow observation Jacobians
	float Kfusion[24][2] = {}; // Optical flow Kalman gains

	// constrain height above ground to be above minimum height when sitting on ground
	float heightAboveGndEst = math::max((_terrain_vpos - _state.pos(2)), gndclearance);

	// get rotation nmatrix from earth to body
	Dcmf earth_to_body(_state.quat_nominal);
	earth_to_body = earth_to_body.transpose();

	// calculate the sensor position relative to the IMU
	Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor reelative to the imu in body frame
	Vector3f vel_rel_imu_body = cross_product(_flow_sample_delayed.gyroXYZ, pos_offset_body);

	// calculate the velocity of the sensor in the earth frame
	Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	Vector3f vel_body = earth_to_body * vel_rel_earth;

	// calculate range from focal point to centre of image
	float range = heightAboveGndEst / earth_to_body(2, 2); // absolute distance to the frame region in view

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotaton of the scene about that axis.
	Vector2f opt_flow_rate;
	opt_flow_rate(0) = _flow_sample_delayed.flowRadXYcomp(0) / _flow_sample_delayed.dt + _flow_gyro_bias(0);
	opt_flow_rate(1) = _flow_sample_delayed.flowRadXYcomp(1) / _flow_sample_delayed.dt + _flow_gyro_bias(1);

	if (opt_flow_rate.norm() < _params.flow_rate_max) {
		_flow_innov[0] =  vel_body(1) / range - opt_flow_rate(0); // flow around the X axis
		_flow_innov[1] = -vel_body(0) / range - opt_flow_rate(1); // flow around the Y axis

	} else {
		return;
	}


	// Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
	// Calculate Obser ation Jacobians and Kalman gans for each measurement axis
	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {

		if (obs_index == 0) {

			// calculate X axis observation Jacobian
			float t2 = 1.0f / range;
			H_LOS[0][0] = t2*(q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f);
			H_LOS[0][1] = t2*(q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f);
			H_LOS[0][2] = t2*(q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f);
			H_LOS[0][3] = -t2*(q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f);
			H_LOS[0][4] = -t2*(q0*q3*2.0f-q1*q2*2.0f);
			H_LOS[0][5] = t2*(q0*q0-q1*q1+q2*q2-q3*q3);
			H_LOS[0][6] = t2*(q0*q1*2.0f+q2*q3*2.0f);

			// calculate intermediate variables for the X observaton innovatoin variance and Kalman gains
			float t3 = q1*vd*2.0f;
			float t4 = q0*ve*2.0f;
			float t11 = q3*vn*2.0f;
			float t5 = t3+t4-t11;
			float t6 = q0*q3*2.0f;
			float t29 = q1*q2*2.0f;
			float t7 = t6-t29;
			float t8 = q0*q1*2.0f;
			float t9 = q2*q3*2.0f;
			float t10 = t8+t9;
			float t12 = P[0][0]*t2*t5;
			float t13 = q0*vd*2.0f;
			float t14 = q2*vn*2.0f;
			float t28 = q1*ve*2.0f;
			float t15 = t13+t14-t28;
			float t16 = q3*vd*2.0f;
			float t17 = q2*ve*2.0f;
			float t18 = q1*vn*2.0f;
			float t19 = t16+t17+t18;
			float t20 = q3*ve*2.0f;
			float t21 = q0*vn*2.0f;
			float t30 = q2*vd*2.0f;
			float t22 = t20+t21-t30;
			float t23 = q0*q0;
			float t24 = q1*q1;
			float t25 = q2*q2;
			float t26 = q3*q3;
			float t27 = t23-t24+t25-t26;
			float t31 = P[1][1]*t2*t15;
			float t32 = P[6][0]*t2*t10;
			float t33 = P[1][0]*t2*t15;
			float t34 = P[2][0]*t2*t19;
			float t35 = P[5][0]*t2*t27;
			float t79 = P[4][0]*t2*t7;
			float t80 = P[3][0]*t2*t22;
			float t36 = t12+t32+t33+t34+t35-t79-t80;
			float t37 = t2*t5*t36;
			float t38 = P[6][1]*t2*t10;
			float t39 = P[0][1]*t2*t5;
			float t40 = P[2][1]*t2*t19;
			float t41 = P[5][1]*t2*t27;
			float t81 = P[4][1]*t2*t7;
			float t82 = P[3][1]*t2*t22;
			float t42 = t31+t38+t39+t40+t41-t81-t82;
			float t43 = t2*t15*t42;
			float t44 = P[6][2]*t2*t10;
			float t45 = P[0][2]*t2*t5;
			float t46 = P[1][2]*t2*t15;
			float t47 = P[2][2]*t2*t19;
			float t48 = P[5][2]*t2*t27;
			float t83 = P[4][2]*t2*t7;
			float t84 = P[3][2]*t2*t22;
			float t49 = t44+t45+t46+t47+t48-t83-t84;
			float t50 = t2*t19*t49;
			float t51 = P[6][3]*t2*t10;
			float t52 = P[0][3]*t2*t5;
			float t53 = P[1][3]*t2*t15;
			float t54 = P[2][3]*t2*t19;
			float t55 = P[5][3]*t2*t27;
			float t85 = P[4][3]*t2*t7;
			float t86 = P[3][3]*t2*t22;
			float t56 = t51+t52+t53+t54+t55-t85-t86;
			float t57 = P[6][5]*t2*t10;
			float t58 = P[0][5]*t2*t5;
			float t59 = P[1][5]*t2*t15;
			float t60 = P[2][5]*t2*t19;
			float t61 = P[5][5]*t2*t27;
			float t88 = P[4][5]*t2*t7;
			float t89 = P[3][5]*t2*t22;
			float t62 = t57+t58+t59+t60+t61-t88-t89;
			float t63 = t2*t27*t62;
			float t64 = P[6][4]*t2*t10;
			float t65 = P[0][4]*t2*t5;
			float t66 = P[1][4]*t2*t15;
			float t67 = P[2][4]*t2*t19;
			float t68 = P[5][4]*t2*t27;
			float t90 = P[4][4]*t2*t7;
			float t91 = P[3][4]*t2*t22;
			float t69 = t64+t65+t66+t67+t68-t90-t91;
			float t70 = P[6][6]*t2*t10;
			float t71 = P[0][6]*t2*t5;
			float t72 = P[1][6]*t2*t15;
			float t73 = P[2][6]*t2*t19;
			float t74 = P[5][6]*t2*t27;
			float t93 = P[4][6]*t2*t7;
			float t94 = P[3][6]*t2*t22;
			float t75 = t70+t71+t72+t73+t74-t93-t94;
			float t76 = t2*t10*t75;
			float t87 = t2*t22*t56;
			float t92 = t2*t7*t69;
			float t77 = R_LOS+t37+t43+t50+t63+t76-t87-t92;
			float t78;
			// calculate innovation variance for X axis observation and protect against a badly conditioned calculation
			if (t77 >= R_LOS) {
				t78 = 1.0f / t77;
				_flow_innov_var[0] = t77;

			} else {
				// we need to reinitialise the covariance matrix and abort this fusion step
				initialiseCovariance();
				return;
			}

			// calculate Kalman gains for X-axis observation
			Kfusion[0][0] = t78*(t12-P[0][4]*t2*t7+P[0][1]*t2*t15+P[0][6]*t2*t10+P[0][2]*t2*t19-P[0][3]*t2*t22+P[0][5]*t2*t27);
			Kfusion[1][0] = t78*(t31+P[1][0]*t2*t5-P[1][4]*t2*t7+P[1][6]*t2*t10+P[1][2]*t2*t19-P[1][3]*t2*t22+P[1][5]*t2*t27);
			Kfusion[2][0] = t78*(t47+P[2][0]*t2*t5-P[2][4]*t2*t7+P[2][1]*t2*t15+P[2][6]*t2*t10-P[2][3]*t2*t22+P[2][5]*t2*t27);
			Kfusion[3][0] = t78*(-t86+P[3][0]*t2*t5-P[3][4]*t2*t7+P[3][1]*t2*t15+P[3][6]*t2*t10+P[3][2]*t2*t19+P[3][5]*t2*t27);
			Kfusion[4][0] = t78*(-t90+P[4][0]*t2*t5+P[4][1]*t2*t15+P[4][6]*t2*t10+P[4][2]*t2*t19-P[4][3]*t2*t22+P[4][5]*t2*t27);
			Kfusion[5][0] = t78*(t61+P[5][0]*t2*t5-P[5][4]*t2*t7+P[5][1]*t2*t15+P[5][6]*t2*t10+P[5][2]*t2*t19-P[5][3]*t2*t22);
			Kfusion[6][0] = t78*(t70+P[6][0]*t2*t5-P[6][4]*t2*t7+P[6][1]*t2*t15+P[6][2]*t2*t19-P[6][3]*t2*t22+P[6][5]*t2*t27);
			Kfusion[7][0] = t78*(P[7][0]*t2*t5-P[7][4]*t2*t7+P[7][1]*t2*t15+P[7][6]*t2*t10+P[7][2]*t2*t19-P[7][3]*t2*t22+P[7][5]*t2*t27);
			Kfusion[8][0] = t78*(P[8][0]*t2*t5-P[8][4]*t2*t7+P[8][1]*t2*t15+P[8][6]*t2*t10+P[8][2]*t2*t19-P[8][3]*t2*t22+P[8][5]*t2*t27);
			Kfusion[9][0] = t78*(P[9][0]*t2*t5-P[9][4]*t2*t7+P[9][1]*t2*t15+P[9][6]*t2*t10+P[9][2]*t2*t19-P[9][3]*t2*t22+P[9][5]*t2*t27);
			Kfusion[10][0] = t78*(P[10][0]*t2*t5-P[10][4]*t2*t7+P[10][1]*t2*t15+P[10][6]*t2*t10+P[10][2]*t2*t19-P[10][3]*t2*t22+P[10][5]*t2*t27);
			Kfusion[11][0] = t78*(P[11][0]*t2*t5-P[11][4]*t2*t7+P[11][1]*t2*t15+P[11][6]*t2*t10+P[11][2]*t2*t19-P[11][3]*t2*t22+P[11][5]*t2*t27);
			Kfusion[12][0] = t78*(P[12][0]*t2*t5-P[12][4]*t2*t7+P[12][1]*t2*t15+P[12][6]*t2*t10+P[12][2]*t2*t19-P[12][3]*t2*t22+P[12][5]*t2*t27);
			Kfusion[13][0] = t78*(P[13][0]*t2*t5-P[13][4]*t2*t7+P[13][1]*t2*t15+P[13][6]*t2*t10+P[13][2]*t2*t19-P[13][3]*t2*t22+P[13][5]*t2*t27);
			Kfusion[14][0] = t78*(P[14][0]*t2*t5-P[14][4]*t2*t7+P[14][1]*t2*t15+P[14][6]*t2*t10+P[14][2]*t2*t19-P[14][3]*t2*t22+P[14][5]*t2*t27);
			Kfusion[15][0] = t78*(P[15][0]*t2*t5-P[15][4]*t2*t7+P[15][1]*t2*t15+P[15][6]*t2*t10+P[15][2]*t2*t19-P[15][3]*t2*t22+P[15][5]*t2*t27);
			Kfusion[16][0] = t78*(P[16][0]*t2*t5-P[16][4]*t2*t7+P[16][1]*t2*t15+P[16][6]*t2*t10+P[16][2]*t2*t19-P[16][3]*t2*t22+P[16][5]*t2*t27);
			Kfusion[17][0] = t78*(P[17][0]*t2*t5-P[17][4]*t2*t7+P[17][1]*t2*t15+P[17][6]*t2*t10+P[17][2]*t2*t19-P[17][3]*t2*t22+P[17][5]*t2*t27);
			Kfusion[18][0] = t78*(P[18][0]*t2*t5-P[18][4]*t2*t7+P[18][1]*t2*t15+P[18][6]*t2*t10+P[18][2]*t2*t19-P[18][3]*t2*t22+P[18][5]*t2*t27);
			Kfusion[19][0] = t78*(P[19][0]*t2*t5-P[19][4]*t2*t7+P[19][1]*t2*t15+P[19][6]*t2*t10+P[19][2]*t2*t19-P[19][3]*t2*t22+P[19][5]*t2*t27);
			Kfusion[20][0] = t78*(P[20][0]*t2*t5-P[20][4]*t2*t7+P[20][1]*t2*t15+P[20][6]*t2*t10+P[20][2]*t2*t19-P[20][3]*t2*t22+P[20][5]*t2*t27);
			Kfusion[21][0] = t78*(P[21][0]*t2*t5-P[21][4]*t2*t7+P[21][1]*t2*t15+P[21][6]*t2*t10+P[21][2]*t2*t19-P[21][3]*t2*t22+P[21][5]*t2*t27);
			Kfusion[22][0] = t78*(P[22][0]*t2*t5-P[22][4]*t2*t7+P[22][1]*t2*t15+P[22][6]*t2*t10+P[22][2]*t2*t19-P[22][3]*t2*t22+P[22][5]*t2*t27);
			Kfusion[23][0] = t78*(P[23][0]*t2*t5-P[23][4]*t2*t7+P[23][1]*t2*t15+P[23][6]*t2*t10+P[23][2]*t2*t19-P[23][3]*t2*t22+P[23][5]*t2*t27);

			// run innovation consistency checks
			optflow_test_ratio[0] = sq(_flow_innov[0]) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var[0]);

		} else if (obs_index == 1) {

			// calculate Y axis observation Jacobian
			float t2 = 1.0f / range;
			H_LOS[1][0] = -t2*(q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f);
			H_LOS[1][1] = -t2*(q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f);
			H_LOS[1][2] = t2*(q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f);
			H_LOS[1][3] = -t2*(q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f);
			H_LOS[1][4] = -t2*(q0*q0+q1*q1-q2*q2-q3*q3);
			H_LOS[1][5] = -t2*(q0*q3*2.0f+q1*q2*2.0f);
			H_LOS[1][6] = t2*(q0*q2*2.0f-q1*q3*2.0f);

			// calculate intermediate variables for the Y observaton innovatoin variance and Kalman gains
			float t3 = q3*ve*2.0f;
			float t4 = q0*vn*2.0f;
			float t11 = q2*vd*2.0f;
			float t5 = t3+t4-t11;
			float t6 = q0*q3*2.0f;
			float t7 = q1*q2*2.0f;
			float t8 = t6+t7;
			float t9 = q0*q2*2.0f;
			float t28 = q1*q3*2.0f;
			float t10 = t9-t28;
			float t12 = P[0][0]*t2*t5;
			float t13 = q3*vd*2.0f;
			float t14 = q2*ve*2.0f;
			float t15 = q1*vn*2.0f;
			float t16 = t13+t14+t15;
			float t17 = q0*vd*2.0f;
			float t18 = q2*vn*2.0f;
			float t29 = q1*ve*2.0f;
			float t19 = t17+t18-t29;
			float t20 = q1*vd*2.0f;
			float t21 = q0*ve*2.0f;
			float t30 = q3*vn*2.0f;
			float t22 = t20+t21-t30;
			float t23 = q0*q0;
			float t24 = q1*q1;
			float t25 = q2*q2;
			float t26 = q3*q3;
			float t27 = t23+t24-t25-t26;
			float t31 = P[1][1]*t2*t16;
			float t32 = P[5][0]*t2*t8;
			float t33 = P[1][0]*t2*t16;
			float t34 = P[3][0]*t2*t22;
			float t35 = P[4][0]*t2*t27;
			float t80 = P[6][0]*t2*t10;
			float t81 = P[2][0]*t2*t19;
			float t36 = t12+t32+t33+t34+t35-t80-t81;
			float t37 = t2*t5*t36;
			float t38 = P[5][1]*t2*t8;
			float t39 = P[0][1]*t2*t5;
			float t40 = P[3][1]*t2*t22;
			float t41 = P[4][1]*t2*t27;
			float t82 = P[6][1]*t2*t10;
			float t83 = P[2][1]*t2*t19;
			float t42 = t31+t38+t39+t40+t41-t82-t83;
			float t43 = t2*t16*t42;
			float t44 = P[5][2]*t2*t8;
			float t45 = P[0][2]*t2*t5;
			float t46 = P[1][2]*t2*t16;
			float t47 = P[3][2]*t2*t22;
			float t48 = P[4][2]*t2*t27;
			float t79 = P[2][2]*t2*t19;
			float t84 = P[6][2]*t2*t10;
			float t49 = t44+t45+t46+t47+t48-t79-t84;
			float t50 = P[5][3]*t2*t8;
			float t51 = P[0][3]*t2*t5;
			float t52 = P[1][3]*t2*t16;
			float t53 = P[3][3]*t2*t22;
			float t54 = P[4][3]*t2*t27;
			float t86 = P[6][3]*t2*t10;
			float t87 = P[2][3]*t2*t19;
			float t55 = t50+t51+t52+t53+t54-t86-t87;
			float t56 = t2*t22*t55;
			float t57 = P[5][4]*t2*t8;
			float t58 = P[0][4]*t2*t5;
			float t59 = P[1][4]*t2*t16;
			float t60 = P[3][4]*t2*t22;
			float t61 = P[4][4]*t2*t27;
			float t88 = P[6][4]*t2*t10;
			float t89 = P[2][4]*t2*t19;
			float t62 = t57+t58+t59+t60+t61-t88-t89;
			float t63 = t2*t27*t62;
			float t64 = P[5][5]*t2*t8;
			float t65 = P[0][5]*t2*t5;
			float t66 = P[1][5]*t2*t16;
			float t67 = P[3][5]*t2*t22;
			float t68 = P[4][5]*t2*t27;
			float t90 = P[6][5]*t2*t10;
			float t91 = P[2][5]*t2*t19;
			float t69 = t64+t65+t66+t67+t68-t90-t91;
			float t70 = t2*t8*t69;
			float t71 = P[5][6]*t2*t8;
			float t72 = P[0][6]*t2*t5;
			float t73 = P[1][6]*t2*t16;
			float t74 = P[3][6]*t2*t22;
			float t75 = P[4][6]*t2*t27;
			float t92 = P[6][6]*t2*t10;
			float t93 = P[2][6]*t2*t19;
			float t76 = t71+t72+t73+t74+t75-t92-t93;
			float t85 = t2*t19*t49;
			float t94 = t2*t10*t76;
			float t77 = R_LOS+t37+t43+t56+t63+t70-t85-t94;
			float t78;
			// calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
			if (t77 >= R_LOS) {
				t78 = 1.0f / t77;
				_flow_innov_var[1] = t77;

			} else {
				// we need to reinitialise the covariance matrix and abort this fusion step
				initialiseCovariance();
				return;
			}

			// calculate Kalman gains for Y-axis observation
			Kfusion[0][1] = -t78*(t12+P[0][5]*t2*t8-P[0][6]*t2*t10+P[0][1]*t2*t16-P[0][2]*t2*t19+P[0][3]*t2*t22+P[0][4]*t2*t27);
			Kfusion[1][1] = -t78*(t31+P[1][0]*t2*t5+P[1][5]*t2*t8-P[1][6]*t2*t10-P[1][2]*t2*t19+P[1][3]*t2*t22+P[1][4]*t2*t27);
			Kfusion[2][1] = -t78*(-t79+P[2][0]*t2*t5+P[2][5]*t2*t8-P[2][6]*t2*t10+P[2][1]*t2*t16+P[2][3]*t2*t22+P[2][4]*t2*t27);
			Kfusion[3][1] = -t78*(t53+P[3][0]*t2*t5+P[3][5]*t2*t8-P[3][6]*t2*t10+P[3][1]*t2*t16-P[3][2]*t2*t19+P[3][4]*t2*t27);
			Kfusion[4][1] = -t78*(t61+P[4][0]*t2*t5+P[4][5]*t2*t8-P[4][6]*t2*t10+P[4][1]*t2*t16-P[4][2]*t2*t19+P[4][3]*t2*t22);
			Kfusion[5][1] = -t78*(t64+P[5][0]*t2*t5-P[5][6]*t2*t10+P[5][1]*t2*t16-P[5][2]*t2*t19+P[5][3]*t2*t22+P[5][4]*t2*t27);
			Kfusion[6][1] = -t78*(-t92+P[6][0]*t2*t5+P[6][5]*t2*t8+P[6][1]*t2*t16-P[6][2]*t2*t19+P[6][3]*t2*t22+P[6][4]*t2*t27);
			Kfusion[7][1] = -t78*(P[7][0]*t2*t5+P[7][5]*t2*t8-P[7][6]*t2*t10+P[7][1]*t2*t16-P[7][2]*t2*t19+P[7][3]*t2*t22+P[7][4]*t2*t27);
			Kfusion[8][1] = -t78*(P[8][0]*t2*t5+P[8][5]*t2*t8-P[8][6]*t2*t10+P[8][1]*t2*t16-P[8][2]*t2*t19+P[8][3]*t2*t22+P[8][4]*t2*t27);
			Kfusion[9][1] = -t78*(P[9][0]*t2*t5+P[9][5]*t2*t8-P[9][6]*t2*t10+P[9][1]*t2*t16-P[9][2]*t2*t19+P[9][3]*t2*t22+P[9][4]*t2*t27);
			Kfusion[10][1] = -t78*(P[10][0]*t2*t5+P[10][5]*t2*t8-P[10][6]*t2*t10+P[10][1]*t2*t16-P[10][2]*t2*t19+P[10][3]*t2*t22+P[10][4]*t2*t27);
			Kfusion[11][1] = -t78*(P[11][0]*t2*t5+P[11][5]*t2*t8-P[11][6]*t2*t10+P[11][1]*t2*t16-P[11][2]*t2*t19+P[11][3]*t2*t22+P[11][4]*t2*t27);
			Kfusion[12][1] = -t78*(P[12][0]*t2*t5+P[12][5]*t2*t8-P[12][6]*t2*t10+P[12][1]*t2*t16-P[12][2]*t2*t19+P[12][3]*t2*t22+P[12][4]*t2*t27);
			Kfusion[13][1] = -t78*(P[13][0]*t2*t5+P[13][5]*t2*t8-P[13][6]*t2*t10+P[13][1]*t2*t16-P[13][2]*t2*t19+P[13][3]*t2*t22+P[13][4]*t2*t27);
			Kfusion[14][1] = -t78*(P[14][0]*t2*t5+P[14][5]*t2*t8-P[14][6]*t2*t10+P[14][1]*t2*t16-P[14][2]*t2*t19+P[14][3]*t2*t22+P[14][4]*t2*t27);
			Kfusion[15][1] = -t78*(P[15][0]*t2*t5+P[15][5]*t2*t8-P[15][6]*t2*t10+P[15][1]*t2*t16-P[15][2]*t2*t19+P[15][3]*t2*t22+P[15][4]*t2*t27);
			Kfusion[16][1] = -t78*(P[16][0]*t2*t5+P[16][5]*t2*t8-P[16][6]*t2*t10+P[16][1]*t2*t16-P[16][2]*t2*t19+P[16][3]*t2*t22+P[16][4]*t2*t27);
			Kfusion[17][1] = -t78*(P[17][0]*t2*t5+P[17][5]*t2*t8-P[17][6]*t2*t10+P[17][1]*t2*t16-P[17][2]*t2*t19+P[17][3]*t2*t22+P[17][4]*t2*t27);
			Kfusion[18][1] = -t78*(P[18][0]*t2*t5+P[18][5]*t2*t8-P[18][6]*t2*t10+P[18][1]*t2*t16-P[18][2]*t2*t19+P[18][3]*t2*t22+P[18][4]*t2*t27);
			Kfusion[19][1] = -t78*(P[19][0]*t2*t5+P[19][5]*t2*t8-P[19][6]*t2*t10+P[19][1]*t2*t16-P[19][2]*t2*t19+P[19][3]*t2*t22+P[19][4]*t2*t27);
			Kfusion[20][1] = -t78*(P[20][0]*t2*t5+P[20][5]*t2*t8-P[20][6]*t2*t10+P[20][1]*t2*t16-P[20][2]*t2*t19+P[20][3]*t2*t22+P[20][4]*t2*t27);
			Kfusion[21][1] = -t78*(P[21][0]*t2*t5+P[21][5]*t2*t8-P[21][6]*t2*t10+P[21][1]*t2*t16-P[21][2]*t2*t19+P[21][3]*t2*t22+P[21][4]*t2*t27);
			Kfusion[22][1] = -t78*(P[22][0]*t2*t5+P[22][5]*t2*t8-P[22][6]*t2*t10+P[22][1]*t2*t16-P[22][2]*t2*t19+P[22][3]*t2*t22+P[22][4]*t2*t27);
			Kfusion[23][1] = -t78*(P[23][0]*t2*t5+P[23][5]*t2*t8-P[23][6]*t2*t10+P[23][1]*t2*t16-P[23][2]*t2*t19+P[23][3]*t2*t22+P[23][4]*t2*t27);

			// run innovation consistency check
			optflow_test_ratio[1] = sq(_flow_innov[1]) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var[1]);

		}
	}

	// record the innovation test pass/fail
	bool flow_fail = false;
	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {
		if (optflow_test_ratio[obs_index] > 1.0f) {
			flow_fail = true;
			_innov_check_fail_status.value |= (1 << (obs_index + 10));

		} else {
			_innov_check_fail_status.value &= ~(1 << (obs_index + 10));

		}
	}

	// if either axis fails we abort the fusion
	if (flow_fail) {
		return;

	}

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {

		// copy the Kalman gain vector for the axis we are fusing
		float gain[24];

		for (unsigned row = 0; row <= 23; row++) {
			gain[row] = Kfusion[row][obs_index];
		}

		// apply covariance correction via P_new = (I -K*H)*P
		// first calculate expression for KHP
		// then calculate P - KHP
		float KHP[_k_num_states][_k_num_states];
		float KH[7];
		for (unsigned row = 0; row < _k_num_states; row++) {

			KH[0] = gain[row] * H_LOS[obs_index][0];
			KH[1] = gain[row] * H_LOS[obs_index][1];
			KH[2] = gain[row] * H_LOS[obs_index][2];
			KH[3] = gain[row] * H_LOS[obs_index][3];
			KH[4] = gain[row] * H_LOS[obs_index][4];
			KH[5] = gain[row] * H_LOS[obs_index][5];
			KH[6] = gain[row] * H_LOS[obs_index][6];

			for (unsigned column = 0; column < _k_num_states; column++) {
				float tmp = KH[0] * P[0][column];
				tmp += KH[1] * P[1][column];
				tmp += KH[2] * P[2][column];
				tmp += KH[3] * P[3][column];
				tmp += KH[4] * P[4][column];
				tmp += KH[5] * P[5][column];
				tmp += KH[6] * P[6][column];
				KHP[row][column] = tmp;
			}
		}

		// if the covariance correction will result in a negative variance, then
		// the covariance marix is unhealthy and must be corrected
		bool healthy = true;
		_fault_status.flags.bad_optflow_X = false;
		_fault_status.flags.bad_optflow_Y = false;
		for (int i = 0; i < _k_num_states; i++) {
			if (P[i][i] < KHP[i][i]) {
				// zero rows and columns
				zeroRows(P,i,i);
				zeroCols(P,i,i);

				//flag as unhealthy
				healthy = false;

				// update individual measurement health status
				if (obs_index == 0) {
					_fault_status.flags.bad_optflow_X = true;
				} else if (obs_index == 1) {
					_fault_status.flags.bad_optflow_Y = true;
				}
			}
		}

		// only apply covariance and state corrrections if healthy
		if (healthy) {
			// apply the covariance corrections
			for (unsigned row = 0; row < _k_num_states; row++) {
				for (unsigned column = 0; column < _k_num_states; column++) {
					P[row][column] = P[row][column] - KHP[row][column];
				}
			}

			// correct the covariance marix for gross errors
			fixCovarianceErrors();

			// apply the state corrections
			fuse(gain, _flow_innov[obs_index]);

			_time_last_of_fuse = _time_last_imu;
			_gps_check_fail_status.value = 0;
		}
	}
}

void Ekf::get_flow_innov(float flow_innov[2])
{
	memcpy(flow_innov, _flow_innov, sizeof(_flow_innov));
}


void Ekf::get_flow_innov_var(float flow_innov_var[2])
{
	memcpy(flow_innov_var, _flow_innov_var, sizeof(_flow_innov_var));
}

void Ekf::get_drag_innov(float drag_innov[2])
{
	memcpy(drag_innov, _drag_innov, sizeof(_drag_innov));
}


void Ekf::get_drag_innov_var(float drag_innov_var[2])
{
	memcpy(drag_innov_var, _drag_innov_var, sizeof(_drag_innov_var));
}

// calculate optical flow gyro bias errors
void Ekf::calcOptFlowBias()
{
	// reset the accumulators if the time interval is too large
	if (_delta_time_of > 1.0f) {
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
		return;
	}

	// if accumulation time differences are not excessive and accumulation time is adequate
	// compare the optical flow and and navigation rate data and calculate a bias error
	if ((fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.1f) && (_delta_time_of > 0.01f)) {
		// calculate a reference angular rate
		Vector3f reference_body_rate;
		reference_body_rate = _imu_del_ang_of * (1.0f / _delta_time_of);

		// calculate the optical flow sensor measured body rate
		Vector3f of_body_rate;
		of_body_rate = _flow_sample_delayed.gyroXYZ * (1.0f / _flow_sample_delayed.dt);

		// calculate the bias estimate using  a combined LPF and spike filter
		_flow_gyro_bias(0) = 0.99f * _flow_gyro_bias(0) + 0.01f * math::constrain((of_body_rate(0) - reference_body_rate(0)),
				     -0.1f, 0.1f);
		_flow_gyro_bias(1) = 0.99f * _flow_gyro_bias(1) + 0.01f * math::constrain((of_body_rate(1) - reference_body_rate(1)),
				     -0.1f, 0.1f);
		_flow_gyro_bias(2) = 0.99f * _flow_gyro_bias(2) + 0.01f * math::constrain((of_body_rate(2) - reference_body_rate(2)),
				     -0.1f, 0.1f);
	}

	// reset the accumulators
	_imu_del_ang_of.setZero();
	_delta_time_of = 0.0f;
}

// calculate the measurement variance for the optical flow sensor (rad/sec)^2
float Ekf::calcOptFlowMeasVar()
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.0f - (float)_params.flow_qual_min);

	if (weighting >= 1.0f) {
		weighting = math::constrain(((float)_flow_sample_delayed.quality - (float)_params.flow_qual_min) / weighting, 0.0f,
					    1.0f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noie for the best and wort flow quality
	float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.0f - weighting));

	return R_LOS;
}
