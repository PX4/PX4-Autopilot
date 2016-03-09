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

	float H_LOS[2][24] = {}; // Optical flow observation Jacobians
	float Kfusion[24][2] = {}; // Optical flow Kalman gains

	// constrain height above ground to be above minimum height when sitting on ground
	float heightAboveGndEst = math::max((_terrain_vpos - _state.pos(2)), gndclearance);

	// get rotation nmatrix from earth to body
	matrix::Dcm<float> earth_to_body(_state.quat_nominal);
	earth_to_body = earth_to_body.transpose();

	// rotate earth velocities into body frame
	Vector3f vel_body = earth_to_body * _state.vel;

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
			float t3 = q0 * q0;
			float t4 = q1 * q1;
			float t5 = q2 * q2;
			float t6 = q3 * q3;
			float t7 = q0 * q2 * 2.0f;
			float t8 = q1 * q3 * 2.0f;
			float t9 = q0 * q3 * 2.0f;
			float t10 = q1 * q2 * 2.0f;
			float t11 = q0 * q1 * 2.0f;
			H_LOS[0][0] = t2 * (vn * (t7 + t8) + vd * (t3 - t4 - t5 + t6) - ve * (t11 - q2 * q3 * 2.0f));
			H_LOS[0][2] = -t2 * (ve * (t9 + t10) - vd * (t7 - t8) + vn * (t3 + t4 - t5 - t6));
			H_LOS[0][3] = -t2 * (t9 - t10);
			H_LOS[0][4] = t2 * (t3 - t4 + t5 - t6);
			H_LOS[0][5] = t2 * (t11 + q2 * q3 * 2.0f);

			// calculate intermediate variables for the X observaton innovatoin variance and klmna gains
			t2 = 1.0f / range;
			t3 = q0 * q1 * 2.0f;
			t4 = q2 * q3 * 2.0f;
			t5 = q0 * q0;
			t6 = q1 * q1;
			t7 = q2 * q2;
			t8 = q3 * q3;
			t9 = q0 * q2 * 2.0f;
			t10 = q1 * q3 * 2.0f;
			t11 = q0 * q3 * 2.0f;
			float t12 = q1 * q2 * 2.0f;
			float t13 = t11 - t12;
			float t14 = t3 + t4;
			float t15 = t5 - t6 - t7 + t8;
			float t16 = t15 * vd;
			float t17 = t3 - t4;
			float t18 = t9 + t10;
			float t19 = t18 * vn;
			float t28 = t17 * ve;
			float t20 = t16 + t19 - t28;
			float t21 = t5 + t6 - t7 - t8;
			float t22 = t21 * vn;
			float t23 = t9 - t10;
			float t24 = t11 + t12;
			float t25 = t24 * ve;
			float t29 = t23 * vd;
			float t26 = t22 + t25 - t29;
			float t27 = t5 - t6 + t7 - t8;
			float t30 = P[0][0] * t2 * t20;
			float t31 = P[5][3] * t2 * t14;
			float t32 = P[0][3] * t2 * t20;
			float t33 = P[4][3] * t2 * t27;
			float t56 = P[3][3] * t2 * t13;
			float t57 = P[2][3] * t2 * t26;
			float t34 = t31 + t32 + t33 - t56 - t57;
			float t35 = P[5][5] * t2 * t14;
			float t36 = P[0][5] * t2 * t20;
			float t37 = P[4][5] * t2 * t27;
			float t59 = P[3][5] * t2 * t13;
			float t60 = P[2][5] * t2 * t26;
			float t38 = t35 + t36 + t37 - t59 - t60;
			float t39 = t2 * t14 * t38;
			float t40 = P[5][0] * t2 * t14;
			float t41 = P[4][0] * t2 * t27;
			float t61 = P[3][0] * t2 * t13;
			float t62 = P[2][0] * t2 * t26;
			float t42 = t30 + t40 + t41 - t61 - t62;
			float t43 = t2 * t20 * t42;
			float t44 = P[5][2] * t2 * t14;
			float t45 = P[0][2] * t2 * t20;
			float t46 = P[4][2] * t2 * t27;
			float t55 = P[2][2] * t2 * t26;
			float t63 = P[3][2] * t2 * t13;
			float t47 = t44 + t45 + t46 - t55 - t63;
			float t48 = P[5][4] * t2 * t14;
			float t49 = P[0][4] * t2 * t20;
			float t50 = P[4][4] * t2 * t27;
			float t65 = P[3][4] * t2 * t13;
			float t66 = P[2][4] * t2 * t26;
			float t51 = t48 + t49 + t50 - t65 - t66;
			float t52 = t2 * t27 * t51;
			float t58 = t2 * t13 * t34;
			float t64 = t2 * t26 * t47;
			float t53 = R_LOS + t39 + t43 + t52 - t58 - t64;
			float t54;

			// calculate innovation variance for X axis observation and protect against a badly conditioned calculation
			if (t53 >= R_LOS) {
				t54 = 1.0f / t53;
				_flow_innov_var[0] = t53;

			} else {
				// we need to reinitialise the covariance matrix and abort this fusion step
				initialiseCovariance();
				return;
			}

			// calculate Kalman gains for X-axis observation
			Kfusion[0][0] = t54 * (t30 - P[0][3] * t2 * (t11 - q1 * q2 * 2.0f) + P[0][5] * t2 * t14 - P[0][2] * t2 * t26 + P[0][4] *
					       t2 * t27);
			Kfusion[1][0] = t54 * (-P[1][3] * t2 * t13 + P[1][5] * t2 * t14 + P[1][0] * t2 * t20 - P[1][2] * t2 * t26 + P[1][4] * t2
					       * t27);
			Kfusion[2][0] = t54 * (-t55 - P[2][3] * t2 * t13 + P[2][5] * t2 * t14 + P[2][0] * t2 * t20 + P[2][4] * t2 * t27);
			Kfusion[3][0] = t54 * (-t56 + P[3][5] * t2 * t14 + P[3][0] * t2 * t20 - P[3][2] * t2 * t26 + P[3][4] * t2 * t27);
			Kfusion[4][0] = t54 * (t50 - P[4][3] * t2 * t13 + P[4][5] * t2 * t14 + P[4][0] * t2 * t20 - P[4][2] * t2 * t26);
			Kfusion[5][0] = t54 * (t35 - P[5][3] * t2 * t13 + P[5][0] * t2 * t20 - P[5][2] * t2 * t26 + P[5][4] * t2 * t27);
			Kfusion[6][0] = t54 * (-P[6][3] * t2 * t13 + P[6][5] * t2 * t14 + P[6][0] * t2 * t20 - P[6][2] * t2 * t26 + P[6][4] * t2
					       * t27);
			Kfusion[7][0] = t54 * (-P[7][3] * t2 * t13 + P[7][5] * t2 * t14 + P[7][0] * t2 * t20 - P[7][2] * t2 * t26 + P[7][4] * t2
					       * t27);
			Kfusion[8][0] = t54 * (-P[8][3] * t2 * t13 + P[8][5] * t2 * t14 + P[8][0] * t2 * t20 - P[8][2] * t2 * t26 + P[8][4] * t2
					       * t27);
			Kfusion[9][0] = t54 * (-P[9][3] * t2 * t13 + P[9][5] * t2 * t14 + P[9][0] * t2 * t20 - P[9][2] * t2 * t26 + P[9][4] * t2
					       * t27);
			Kfusion[10][0] = t54 * (-P[10][3] * t2 * t13 + P[10][5] * t2 * t14 + P[10][0] * t2 * t20 - P[10][2] * t2 * t26 +
						P[10][4] * t2 * t27);
			Kfusion[11][0] = t54 * (-P[11][3] * t2 * t13 + P[11][5] * t2 * t14 + P[11][0] * t2 * t20 - P[11][2] * t2 * t26 +
						P[11][4] * t2 * t27);
			Kfusion[12][0] = t54 * (-P[12][3] * t2 * t13 + P[12][5] * t2 * t14 + P[12][0] * t2 * t20 - P[12][2] * t2 * t26 +
						P[12][4] * t2 * t27);
			Kfusion[13][0] = t54 * (-P[13][3] * t2 * t13 + P[13][5] * t2 * t14 + P[13][0] * t2 * t20 - P[13][2] * t2 * t26 +
						P[13][4] * t2 * t27);
			Kfusion[14][0] = t54 * (-P[14][3] * t2 * t13 + P[14][5] * t2 * t14 + P[14][0] * t2 * t20 - P[14][2] * t2 * t26 +
						P[14][4] * t2 * t27);
			Kfusion[15][0] = t54 * (-P[15][3] * t2 * t13 + P[15][5] * t2 * t14 + P[15][0] * t2 * t20 - P[15][2] * t2 * t26 +
						P[15][4] * t2 * t27);

			if (_control_status.flags.mag_3D) {
				Kfusion[16][0] = t54 * (-P[16][3] * t2 * t13 + P[16][5] * t2 * t14 + P[16][0] * t2 * t20 - P[16][2] * t2 * t26 +
							P[16][4] * t2 * t27);
				Kfusion[17][0] = t54 * (-P[17][3] * t2 * t13 + P[17][5] * t2 * t14 + P[17][0] * t2 * t20 - P[17][2] * t2 * t26 +
							P[17][4] * t2 * t27);
				Kfusion[18][0] = t54 * (-P[18][3] * t2 * t13 + P[18][5] * t2 * t14 + P[18][0] * t2 * t20 - P[18][2] * t2 * t26 +
							P[18][4] * t2 * t27);
				Kfusion[19][0] = t54 * (-P[19][3] * t2 * t13 + P[19][5] * t2 * t14 + P[19][0] * t2 * t20 - P[19][2] * t2 * t26 +
							P[19][4] * t2 * t27);
				Kfusion[20][0] = t54 * (-P[20][3] * t2 * t13 + P[20][5] * t2 * t14 + P[20][0] * t2 * t20 - P[20][2] * t2 * t26 +
							P[20][4] * t2 * t27);
				Kfusion[21][0] = t54 * (-P[21][3] * t2 * t13 + P[21][5] * t2 * t14 + P[21][0] * t2 * t20 - P[21][2] * t2 * t26 +
							P[21][4] * t2 * t27);
			}

			if (_control_status.flags.wind) {
				Kfusion[22][0] = t54 * (-P[22][3] * t2 * t13 + P[22][5] * t2 * t14 + P[22][0] * t2 * t20 - P[22][2] * t2 * t26 +
							P[22][4] * t2 * t27);
				Kfusion[23][0] = t54 * (-P[23][3] * t2 * t13 + P[23][5] * t2 * t14 + P[23][0] * t2 * t20 - P[23][2] * t2 * t26 +
							P[23][4] * t2 * t27);
			}

			// run innovation consistency checks
			optflow_test_ratio[0] = sq(_flow_innov[0]) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var[0]);

		} else if (obs_index == 1) {

			// calculate Y axis observation Jacobian
			float t2 = 1.0f / range;
			float t3 = q0 * q0;
			float t4 = q1 * q1;
			float t5 = q2 * q2;
			float t6 = q3 * q3;
			float t7 = q0 * q1 * 2.0f;
			float t8 = q0 * q3 * 2.0f;
			float t9 = q0 * q2 * 2.0f;
			float t10 = q1 * q3 * 2.0f;
			H_LOS[1][1] = t2 * (vn * (t9 + t10) + vd * (t3 - t4 - t5 + t6) - ve * (t7 - q2 * q3 * 2.0f));
			H_LOS[1][2] = -t2 * (ve * (t3 - t4 + t5 - t6) + vd * (t7 + q2 * q3 * 2.0f) - vn * (t8 - q1 * q2 * 2.0f));
			H_LOS[1][3] = -t2 * (t3 + t4 - t5 - t6);
			H_LOS[1][4] = -t2 * (t8 + q1 * q2 * 2.0f);
			H_LOS[1][5] = t2 * (t9 - t10);

			// calculate intermediate variables for the X observaton innovatoin variance and klmna gains
			t2 = 1.0f / range;
			t3 = q0 * q2 * 2.0f;
			t4 = q0 * q0;
			t5 = q1 * q1;
			t6 = q2 * q2;
			t7 = q3 * q3;
			t8 = q0 * q1 * 2.0f;
			t9 = q0 * q3 * 2.0f;
			t10 = q1 * q2 * 2.0f;
			float t11 = t9 + t10;
			float t12 = q1 * q3 * 2.0f;
			float t13 = t4 - t5 - t6 + t7;
			float t14 = t13 * vd;
			float t15 = q2 * q3 * 2.0f;
			float t16 = t3 + t12;
			float t17 = t16 * vn;
			float t18 = t4 - t5 + t6 - t7;
			float t19 = t18 * ve;
			float t20 = t8 + t15;
			float t21 = t20 * vd;
			float t22 = t9 - t10;
			float t28 = t22 * vn;
			float t23 = t19 + t21 - t28;
			float t24 = t4 + t5 - t6 - t7;
			float t25 = t3 - t12;
			float t26 = t8 - t15;
			float t29 = t26 * ve;
			float t27 = t14 + t17 - t29;
			float t30 = P[4][4] * t2 * t11;
			float t31 = P[2][4] * t2 * t23;
			float t32 = P[3][4] * t2 * t24;
			float t56 = P[5][4] * t2 * t25;
			float t57 = P[1][4] * t2 * t27;
			float t33 = t30 + t31 + t32 - t56 - t57;
			float t34 = t2 * t11 * t33;
			float t35 = P[4][5] * t2 * t11;
			float t36 = P[2][5] * t2 * t23;
			float t37 = P[3][5] * t2 * t24;
			float t58 = P[5][5] * t2 * t25;
			float t59 = P[1][5] * t2 * t27;
			float t38 = t35 + t36 + t37 - t58 - t59;
			float t39 = P[4][1] * t2 * t11;
			float t40 = P[2][1] * t2 * t23;
			float t41 = P[3][1] * t2 * t24;
			float t55 = P[1][1] * t2 * t27;
			float t61 = P[5][1] * t2 * t25;
			float t42 = t39 + t40 + t41 - t55 - t61;
			float t43 = P[4][2] * t2 * t11;
			float t44 = P[2][2] * t2 * t23;
			float t45 = P[3][2] * t2 * t24;
			float t63 = P[5][2] * t2 * t25;
			float t64 = P[1][2] * t2 * t27;
			float t46 = t43 + t44 + t45 - t63 - t64;
			float t47 = t2 * t23 * t46;
			float t48 = P[4][3] * t2 * t11;
			float t49 = P[2][3] * t2 * t23;
			float t50 = P[3][3] * t2 * t24;
			float t65 = P[5][3] * t2 * t25;
			float t66 = P[1][3] * t2 * t27;
			float t51 = t48 + t49 + t50 - t65 - t66;
			float t52 = t2 * t24 * t51;
			float t60 = t2 * t25 * t38;
			float t62 = t2 * t27 * t42;
			float t53 = R_LOS + t34 + t47 + t52 - t60 - t62;
			float t54;

			// calculate innovation variance for X axis observation and protect against a badly conditioned calculation
			if (t53 >= R_LOS) {
				t54 = 1.0f / t53;
				_flow_innov_var[1] = t53;

			} else {
				// we need to reinitialise the covariance matrix and abort this fusion step
				initialiseCovariance();
				return;
			}

			// calculate Kalman gains for X-axis observation
			Kfusion[0][1] = -t54 * (P[0][4] * t2 * t11 + P[0][2] * t2 * t23 + P[0][3] * t2 * t24 - P[0][1] * t2 * t27 - P[0][5] * t2
						* t25);
			Kfusion[1][1] = -t54 * (-t55 + P[1][4] * t2 * t11 + P[1][2] * t2 * t23 + P[1][3] * t2 * t24 - P[1][5] * t2 * t25);
			Kfusion[2][1] = -t54 * (t44 + P[2][4] * t2 * t11 + P[2][3] * t2 * t24 - P[2][1] * t2 * t27 - P[2][5] * t2 * t25);
			Kfusion[3][1] = -t54 * (t50 + P[3][4] * t2 * t11 + P[3][2] * t2 * t23 - P[3][1] * t2 * t27 - P[3][5] * t2 * t25);
			Kfusion[4][1] = -t54 * (t30 + P[4][2] * t2 * t23 + P[4][3] * t2 * t24 - P[4][1] * t2 * t27 - P[4][5] * t2 * t25);
			Kfusion[5][1] = -t54 * (-t58 + P[5][4] * t2 * t11 + P[5][2] * t2 * t23 + P[5][3] * t2 * t24 - P[5][1] * t2 * t27);
			Kfusion[6][1] = -t54 * (P[6][4] * t2 * t11 + P[6][2] * t2 * t23 + P[6][3] * t2 * t24 - P[6][1] * t2 * t27 - P[6][5] * t2
						* t25);
			Kfusion[7][1] = -t54 * (P[7][4] * t2 * t11 + P[7][2] * t2 * t23 + P[7][3] * t2 * t24 - P[7][1] * t2 * t27 - P[7][5] * t2
						* t25);
			Kfusion[8][1] = -t54 * (P[8][4] * t2 * t11 + P[8][2] * t2 * t23 + P[8][3] * t2 * t24 - P[8][1] * t2 * t27 - P[8][5] * t2
						* t25);
			Kfusion[9][1] = -t54 * (P[9][4] * t2 * t11 + P[9][2] * t2 * t23 + P[9][3] * t2 * t24 - P[9][1] * t2 * t27 - P[9][5] * t2
						* t25);
			Kfusion[10][1] = -t54 * (P[10][4] * t2 * t11 + P[10][2] * t2 * t23 + P[10][3] * t2 * t24 - P[10][1] * t2 * t27 -
						 P[10][5] * t2 * t25);
			Kfusion[11][1] = -t54 * (P[11][4] * t2 * t11 + P[11][2] * t2 * t23 + P[11][3] * t2 * t24 - P[11][1] * t2 * t27 -
						 P[11][5] * t2 * t25);
			Kfusion[12][1] = -t54 * (P[12][4] * t2 * t11 + P[12][2] * t2 * t23 + P[12][3] * t2 * t24 - P[12][1] * t2 * t27 -
						 P[12][5] * t2 * t25);
			Kfusion[13][1] = -t54 * (P[13][4] * t2 * t11 + P[13][2] * t2 * t23 + P[13][3] * t2 * t24 - P[13][1] * t2 * t27 -
						 P[13][5] * t2 * t25);
			Kfusion[14][1] = -t54 * (P[14][4] * t2 * t11 + P[14][2] * t2 * t23 + P[14][3] * t2 * t24 - P[14][1] * t2 * t27 -
						 P[14][5] * t2 * t25);
			Kfusion[15][1] = -t54 * (P[15][4] * t2 * t11 + P[15][2] * t2 * t23 + P[15][3] * t2 * t24 - P[15][1] * t2 * t27 -
						 P[15][5] * t2 * t25);

			if (_control_status.flags.mag_3D) {
				Kfusion[16][1] = -t54 * (P[16][4] * t2 * t11 + P[16][2] * t2 * t23 + P[16][3] * t2 * t24 - P[16][1] * t2 * t27 -
							 P[16][5] * t2 * t25);
				Kfusion[17][1] = -t54 * (P[17][4] * t2 * t11 + P[17][2] * t2 * t23 + P[17][3] * t2 * t24 - P[17][1] * t2 * t27 -
							 P[17][5] * t2 * t25);
				Kfusion[18][1] = -t54 * (P[18][4] * t2 * t11 + P[18][2] * t2 * t23 + P[18][3] * t2 * t24 - P[18][1] * t2 * t27 -
							 P[18][5] * t2 * t25);
				Kfusion[19][1] = -t54 * (P[19][4] * t2 * t11 + P[19][2] * t2 * t23 + P[19][3] * t2 * t24 - P[19][1] * t2 * t27 -
							 P[19][5] * t2 * t25);
				Kfusion[20][1] = -t54 * (P[20][4] * t2 * t11 + P[20][2] * t2 * t23 + P[20][3] * t2 * t24 - P[20][1] * t2 * t27 -
							 P[20][5] * t2 * t25);
				Kfusion[21][1] = -t54 * (P[21][4] * t2 * t11 + P[21][2] * t2 * t23 + P[21][3] * t2 * t24 - P[21][1] * t2 * t27 -
							 P[21][5] * t2 * t25);
			}

			if (_control_status.flags.wind) {
				Kfusion[22][1] = -t54 * (P[22][4] * t2 * t11 + P[22][2] * t2 * t23 + P[22][3] * t2 * t24 - P[22][1] * t2 * t27 -
							 P[22][5] * t2 * t25);
				Kfusion[23][1] = -t54 * (P[23][4] * t2 * t11 + P[23][2] * t2 * t23 + P[23][3] * t2 * t24 - P[23][1] * t2 * t27 -
							 P[23][5] * t2 * t25);
			}

			// run innovation consistency check
			optflow_test_ratio[1] = sq(_flow_innov[1]) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var[1]);

		} else {
			return;
		}
	}

	// if either axis fails, we fail the sensor
	if (optflow_test_ratio[0] > 1.0f || optflow_test_ratio[1] > 1.0f) {
		return;
	}

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {

		// by definition our error state is zero at the time of fusion
		_state.ang_error.setZero();

		// copy the Kalman gain vector for the axis we are fusing
		float gain[24];

		for (unsigned row = 0; row <= 23; row++) {
			gain[row] = Kfusion[row][obs_index];
		}

		// Update the state vector
		fuse(gain, _flow_innov[obs_index]);

		// correct the quaternion using the attitude error state
		Quaternion q_correction;
		q_correction.from_axis_angle(_state.ang_error);
		_state.quat_nominal = q_correction * _state.quat_nominal;
		_state.quat_nominal.normalize();

		// reset attitude error to zero after the correction has been applied
		_state.ang_error.setZero();

		// apply covariance correction via P_new = (I -K*H)*P
		// first calculate expression for KHP
		// then calculate P - KHP
		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column <= 5; column++) {
				KH[row][column] = gain[row] * H_LOS[obs_index][column];
			}
		}

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				float tmp = KH[row][0] * P[0][column];
				tmp += KH[row][1] * P[1][column];
				tmp += KH[row][2] * P[2][column];
				tmp += KH[row][3] * P[3][column];
				tmp += KH[row][4] * P[4][column];
				tmp += KH[row][5] * P[5][column];
				KHP[row][column] = tmp;
			}
		}

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				P[row][column] -= KHP[row][column];
			}
		}

		_time_last_of_fuse = _time_last_imu;
		_gps_check_fail_status.value = 0;
		makeSymmetrical();
		limitCov();
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

// calculate optical flow gyro bias errors
void Ekf::calcOptFlowBias()
{
	// accumulate the bias corrected delta angles from the navigation sensor and lapsed time
	_imu_del_ang_of(0) += _imu_sample_delayed.delta_ang(0);
	_imu_del_ang_of(1) += _imu_sample_delayed.delta_ang(1);
	_delta_time_of += _imu_sample_delayed.delta_ang_dt;

	// reset the accumulators if the time interval is too large
	if (_delta_time_of > 1.0f) {
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
	}

	// if accumulation time differences are not excessive and accumulation time is adequate
	// compare the optical flow and and navigation rate data and calculate a bias error
	if (_fuse_flow) {
		if ((fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.05f) && (_delta_time_of > 0.01f)
		    && (_flow_sample_delayed.dt > 0.01f)) {
			// calculate a reference angular rate
			Vector2f reference_body_rate;
			reference_body_rate(0) = _imu_del_ang_of(0) / _delta_time_of;
			reference_body_rate(1) = _imu_del_ang_of(1) / _delta_time_of;

			// calculate the optical flow sensor measured body rate
			Vector2f of_body_rate;
			of_body_rate(0) = _flow_sample_delayed.gyroXY(0) / _flow_sample_delayed.dt;
			of_body_rate(1) = _flow_sample_delayed.gyroXY(1) / _flow_sample_delayed.dt;

			// calculate the bias estimate using  a combined LPF and spike filter
			_flow_gyro_bias(0) = 0.99f * _flow_gyro_bias(0) + 0.01f * math::constrain((of_body_rate(0) - reference_body_rate(0)),
					     -0.1f, 0.1f);
			_flow_gyro_bias(1) = 0.99f * _flow_gyro_bias(1) + 0.01f * math::constrain((of_body_rate(1) - reference_body_rate(1)),
					     -0.1f, 0.1f);

		}

		// reset the accumulators
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
	}
}
