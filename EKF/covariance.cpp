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
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include <math.h>
#include "mathlib.h"

void Ekf::initialiseCovariance()
{
	for (unsigned i = 0; i < _k_num_states; i++) {
		for (unsigned j = 0; j < _k_num_states; j++) {
			P[i][j] = 0.0f;
		}
	}

	// calculate average prediction time step in sec
	float dt = 0.001f * (float)FILTER_UPDATE_PERRIOD_MS;

	// XXX use initial guess for the diagonal elements for the covariance matrix
	// angle error
	P[0][0] = 0.1f;
	P[1][1] = 0.1f;
	P[2][2] = 0.1f;

	// velocity
	P[3][3] = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P[4][4] = P[3][3];
	P[5][5] = sq(1.5f) * P[3][3];

	// position
	P[6][6] = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P[7][7] = P[6][6];
	P[8][8] = sq(fmaxf(_params.baro_noise, 0.01f));

	// gyro bias
	P[9][9] = sq(0.035f * dt);
	P[10][10] = P[9][9];
	P[11][11] = P[9][9];

	// gyro scale
	P[12][12] = sq(0.001f);
	P[13][13] = P[12][12];
	P[14][14] = P[12][12];

	// accel z bias
	P[15][15] = sq(0.2f * dt);

	// variances for optional states
	// these state variances are set to zero until the states are required, then they must be initialised

	// earth magnetic field
	P[16][16] = 0.0f;
	P[17][17] = 0.0f;
	P[18][18] = 0.0f;

	// body magnetic field
	P[19][19] = 0.0f;
	P[20][20] = 0.0f;
	P[21][21] = 0.0f;

	// wind
	P[22][22] = 0.0f;
	P[23][23] = 0.0f;

}

void Ekf::get_pos_var(Vector3f &pos_var)
{
	pos_var(0) = P[6][6];
	pos_var(1) = P[7][7];
	pos_var(2) = P[8][8];
}

void Ekf::get_vel_var(Vector3f &vel_var)
{
	vel_var(0) = P[3][3];
	vel_var(1) = P[4][4];
	vel_var(2) = P[5][5];
}

void Ekf::predictCovariance()
{
	// assign intermediate state variables
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	float dax = _imu_sample_delayed.delta_ang(0);
	float day = _imu_sample_delayed.delta_ang(1);
	float daz = _imu_sample_delayed.delta_ang(2);

	float dvx = _imu_sample_delayed.delta_vel(0);
	float dvy = _imu_sample_delayed.delta_vel(1);
	float dvz = _imu_sample_delayed.delta_vel(2);

	float dax_b = _state.gyro_bias(0);
	float day_b = _state.gyro_bias(1);
	float daz_b = _state.gyro_bias(2);

	float dax_s = _state.gyro_scale(0);
	float day_s = _state.gyro_scale(1);
	float daz_s = _state.gyro_scale(2);

	float dvz_b = _state.accel_z_bias;

	float dt = _imu_sample_delayed.delta_vel_dt;

	// compute process noise
	float process_noise[_k_num_states] = {};

	float d_ang_bias_sig = dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1e-4f);
	float d_vel_bias_sig = dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1e-2f);
	float d_ang_scale_sig = dt * math::constrain(_params.gyro_scale_p_noise, 0.0f, 1e-2f);
	float mag_I_sig, mag_B_sig;

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	if (_control_status.flags.mag_3D && (P[16][16] + P[17][17] + P[18][8]) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1e-1f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	if (_control_status.flags.mag_3D && (P[19][19] + P[20][20] + P[21][21]) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1e-1f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_sig;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && (P[22][22] + P[22][22]) < 1000.0f) {
		wind_vel_sig = dt * math::constrain(_params.wind_vel_p_noise, 0.0f, 1.0f);

	} else {
		wind_vel_sig = 0.0f;
	}

	for (unsigned i = 0; i < 9; i++) {
		process_noise[i] = 0.0f;
	}

	for (unsigned i = 9; i < 12; i++) {
		process_noise[i] = sq(d_ang_bias_sig);
	}

	for (unsigned i = 12; i < 15; i++) {
		process_noise[i] = sq(d_ang_scale_sig);
	}

	process_noise[15] = sq(d_vel_bias_sig);

	for (unsigned i = 16; i < 19; i++) {
		process_noise[i] = sq(mag_I_sig);
	}

	for (unsigned i = 19; i < 22; i++) {
		process_noise[i] = sq(mag_B_sig);
	}

	for (unsigned i = 22; i < 24; i++) {
		process_noise[i] = sq(wind_vel_sig);
	}


	// assign input noise
	// inputs to the system are 3 delta angles and 3 delta velocities
	float daxNoise, dayNoise, dazNoise;
	float dvxNoise, dvyNoise, dvzNoise;
	float gyro_noise = math::constrain(_params.gyro_noise, 1e-4f, 1e-1f);
	daxNoise = dayNoise = dazNoise = dt * gyro_noise;
	float accel_noise = math::constrain(_params.accel_noise, 1e-2f, 1.0f);
	dvxNoise = dvyNoise = dvzNoise = dt * accel_noise;

	// predict covarinace matrix

	// intermediate calculations
	float SF[25] = {};
	SF[0] = daz_b / 2 + dazNoise / 2 - (daz * daz_s) / 2;
	SF[1] = day_b / 2 + dayNoise / 2 - (day * day_s) / 2;
	SF[2] = dax_b / 2 + daxNoise / 2 - (dax * dax_s) / 2;
	SF[3] = q3 / 2 - (q0 * SF[0]) / 2 + (q1 * SF[1]) / 2 - (q2 * SF[2]) / 2;
	SF[4] = q0 / 2 - (q1 * SF[2]) / 2 - (q2 * SF[1]) / 2 + (q3 * SF[0]) / 2;
	SF[5] = q1 / 2 + (q0 * SF[2]) / 2 - (q2 * SF[0]) / 2 - (q3 * SF[1]) / 2;
	SF[6] = q3 / 2 + (q0 * SF[0]) / 2 - (q1 * SF[1]) / 2 - (q2 * SF[2]) / 2;
	SF[7] = q0 / 2 - (q1 * SF[2]) / 2 + (q2 * SF[1]) / 2 - (q3 * SF[0]) / 2;
	SF[8] = q0 / 2 + (q1 * SF[2]) / 2 - (q2 * SF[1]) / 2 - (q3 * SF[0]) / 2;
	SF[9] = q2 / 2 + (q0 * SF[1]) / 2 + (q1 * SF[0]) / 2 + (q3 * SF[2]) / 2;
	SF[10] = q2 / 2 - (q0 * SF[1]) / 2 - (q1 * SF[0]) / 2 + (q3 * SF[2]) / 2;
	SF[11] = q2 / 2 + (q0 * SF[1]) / 2 - (q1 * SF[0]) / 2 - (q3 * SF[2]) / 2;
	SF[12] = q1 / 2 + (q0 * SF[2]) / 2 + (q2 * SF[0]) / 2 + (q3 * SF[1]) / 2;
	SF[13] = q1 / 2 - (q0 * SF[2]) / 2 + (q2 * SF[0]) / 2 - (q3 * SF[1]) / 2;
	SF[14] = q3 / 2 + (q0 * SF[0]) / 2 + (q1 * SF[1]) / 2 + (q2 * SF[2]) / 2;
	SF[15] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
	SF[16] = dvz_b - dvz + dvzNoise;
	SF[17] = dvx - dvxNoise;
	SF[18] = dvy - dvyNoise;
	SF[19] = sq(q2);
	SF[20] = SF[19] - sq(q0) + sq(q1) - sq(q3);
	SF[21] = SF[19] + sq(q0) - sq(q1) - sq(q3);
	SF[22] = 2 * q0 * q1 - 2 * q2 * q3;
	SF[23] = SF[19] - sq(q0) - sq(q1) + sq(q3);
	SF[24] = 2 * q1 * q2;

	float SG[5] = {};
	SG[0] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
	SG[1] = sq(q3);
	SG[2] = sq(q2);
	SG[3] = sq(q1);
	SG[4] = sq(q0);

	float SQ[10] = {};
	SQ[0] = - sq(dvyNoise)*(2*q0*q1 + 2*q2*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - sq(dvzNoise)*(2*q0*q1 - 2*q2*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - sq(dvxNoise)*(2*q0*q2 - 2*q1*q3)*(2*q0*q3 + 2*q1*q2);
	SQ[1] = sq(dvxNoise)*(2*q0*q2 - 2*q1*q3)*(SG[1] + SG[2] - SG[3] - SG[4]) + sq(dvzNoise)*(2*q0*q2 + 2*q1*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - sq(dvyNoise)*(2*q0*q1 + 2*q2*q3)*(2*q0*q3 - 2*q1*q2);
	SQ[2] = sq(dvyNoise)*(2*q0*q3 - 2*q1*q2)*(SG[1] - SG[2] + SG[3] - SG[4]) - sq(dvxNoise)*(2*q0*q3 + 2*q1*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) - sq(dvzNoise)*(2*q0*q1 - 2*q2*q3)*(2*q0*q2 + 2*q1*q3);
	SQ[3] = sq(SG[0]);
	SQ[4] = sq(dvyNoise);
	SQ[5] = sq(dvzNoise);
	SQ[6] = sq(dvxNoise);
	SQ[7] = 2*q2*q3;
	SQ[8] = 2*q1*q3;
	SQ[9] = 2*q1*q2;

	float SPP[23] = {};
	SPP[0] = SF[17]*(2*q0*q1 + 2*q2*q3) + SF[18]*(2*q0*q2 - 2*q1*q3);
	SPP[1] = SF[18]*(2*q0*q2 + 2*q1*q3) + SF[16]*(SF[24] - 2*q0*q3);
	SPP[2] = 2*q3*SF[8] + 2*q1*SF[11] - 2*q0*SF[14] - 2*q2*SF[13];
	SPP[3] = 2*q1*SF[7] + 2*q2*SF[6] - 2*q0*SF[12] - 2*q3*SF[10];
	SPP[4] = 2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12];
	SPP[5] = 2*q0*SF[8] + 2*q2*SF[11] + 2*q1*SF[13] + 2*q3*SF[14];
	SPP[6] = 2*q0*SF[7] + 2*q3*SF[6] + 2*q2*SF[10] + 2*q1*SF[12];
	SPP[7] = 2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9];
	SPP[8] = 2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9];
	SPP[9] = SF[18]*SF[20] - SF[16]*(2*q0*q1 + 2*q2*q3);
	SPP[10] = SF[17]*SF[20] + SF[16]*(2*q0*q2 - 2*q1*q3);
	SPP[11] = SF[17]*SF[21] - SF[18]*(SF[24] + 2*q0*q3);
	SPP[12] = SF[17]*SF[22] - SF[16]*(SF[24] + 2*q0*q3);
	SPP[13] = 2*q0*SF[4] + 2*q1*SF[5] + 2*q3*SF[3] + 2*q2*SF[9];
	SPP[14] = 2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13];
	SPP[15] = SF[18]*SF[23] + SF[17]*(SF[24] - 2*q0*q3);
	SPP[16] = daz*SF[19] + daz*sq(q0) + daz*sq(q1) + daz*sq(q3);
	SPP[17] = day*SF[19] + day*sq(q0) + day*sq(q1) + day*sq(q3);
	SPP[18] = dax*SF[19] + dax*sq(q0) + dax*sq(q1) + dax*sq(q3);
	SPP[19] = SF[16]*SF[23] - SF[17]*(2*q0*q2 + 2*q1*q3);
	SPP[20] = SF[16]*SF[21] - SF[18]*SF[22];
	SPP[21] = 2*q0*q2 + 2*q1*q3;
	SPP[22] = SF[15];

	// covariance update
	float nextP[24][24] = {};
	nextP[0][0] = SPP[5]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[7] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) - SPP[4]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[7] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[7]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[7] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) + SPP[22]*(P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[2][9]*SPP[7] + P[9][9]*SPP[22] + P[12][9]*SPP[18]) + SPP[18]*(P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[2][12]*SPP[7] + P[9][12]*SPP[22] + P[12][12]*SPP[18]) + sq(daxNoise)*SQ[3];
	nextP[0][1] = SPP[6] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[7] + P[9][1] * SPP[22] + P[12][1] * SPP[18])
		      - SPP[2] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[7] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) -
		      SPP[8] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[7] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) + SPP[22] *
		      (P[0][10] * SPP[5] - P[1][10] * SPP[4] + P[2][10] * SPP[7] + P[9][10] * SPP[22] + P[12][10] * SPP[18]) + SPP[17] *
		      (P[0][13] * SPP[5] - P[1][13] * SPP[4] + P[2][13] * SPP[7] + P[9][13] * SPP[22] + P[12][13] * SPP[18]);
	nextP[1][1] = SPP[6]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[8] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) - SPP[2]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[8] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) - SPP[8]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[8] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[22]*(P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[8] + P[10][10]*SPP[22] + P[13][10]*SPP[17]) + SPP[17]*(P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[8] + P[10][13]*SPP[22] + P[13][13]*SPP[17]) + sq(dayNoise)*SQ[3];
	nextP[0][2] = SPP[14] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[7] + P[9][0] * SPP[22] + P[12][0] *
				 SPP[18]) - SPP[3] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[7] + P[9][1] * SPP[22] + P[12][1] * SPP[18]) +
		      SPP[13] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[7] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) +
		      SPP[22] * (P[0][11] * SPP[5] - P[1][11] * SPP[4] + P[2][11] * SPP[7] + P[9][11] * SPP[22] + P[12][11] * SPP[18]) +
		      SPP[16] * (P[0][14] * SPP[5] - P[1][14] * SPP[4] + P[2][14] * SPP[7] + P[9][14] * SPP[22] + P[12][14] * SPP[18]);
	nextP[1][2] = SPP[14] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[8] + P[10][0] * SPP[22] + P[13][0] *
				 SPP[17]) - SPP[3] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[8] + P[10][1] * SPP[22] + P[13][1] * SPP[17]) +
		      SPP[13] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[8] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) +
		      SPP[22] * (P[1][11] * SPP[6] - P[0][11] * SPP[2] - P[2][11] * SPP[8] + P[10][11] * SPP[22] + P[13][11] * SPP[17]) +
		      SPP[16] * (P[1][14] * SPP[6] - P[0][14] * SPP[2] - P[2][14] * SPP[8] + P[10][14] * SPP[22] + P[13][14] * SPP[17]);
	nextP[2][2] = SPP[14]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) - SPP[3]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[13]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[22]*(P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16]) + SPP[16]*(P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16]) + sq(dazNoise)*SQ[3];
	nextP[0][3] = P[0][3] * SPP[5] - P[1][3] * SPP[4] + P[2][3] * SPP[7] + P[9][3] * SPP[22] + P[12][3] * SPP[18] +
		      SPP[1] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[7] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) + SPP[19] *
		      (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[7] + P[9][1] * SPP[22] + P[12][1] * SPP[18]) + SPP[15] *
		      (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[7] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) - SPP[21] *
		      (P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[7] + P[9][15] * SPP[22] + P[12][15] * SPP[18]);
	nextP[1][3] = P[1][3] * SPP[6] - P[0][3] * SPP[2] - P[2][3] * SPP[8] + P[10][3] * SPP[22] + P[13][3] * SPP[17] +
		      SPP[1] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[8] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) +
		      SPP[19] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[8] + P[10][1] * SPP[22] + P[13][1] * SPP[17]) +
		      SPP[15] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[8] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) -
		      SPP[21] * (P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[8] + P[10][15] * SPP[22] + P[13][15] * SPP[17]);
	nextP[2][3] = P[0][3] * SPP[14] - P[1][3] * SPP[3] + P[2][3] * SPP[13] + P[11][3] * SPP[22] + P[14][3] * SPP[16] +
		      SPP[1] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) +
		      SPP[19] * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]) +
		      SPP[15] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]) -
		      SPP[21] * (P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16]);
	nextP[3][3] = P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21] + SQ[5]*sq(SQ[8] + 2*q0*q2) + SQ[4]*sq(SQ[9] - 2*q0*q3) + SPP[1]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[19]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[15]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) - SPP[21]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) + SQ[6]*sq(SG[1] + SG[2] - SG[3] - SG[4]);
	nextP[0][4] = P[0][4] * SPP[5] - P[1][4] * SPP[4] + P[2][4] * SPP[7] + P[9][4] * SPP[22] + P[12][4] * SPP[18] +
		      SF[22] * (P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[7] + P[9][15] * SPP[22] + P[12][15] * SPP[18]) +
		      SPP[12] * (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[7] + P[9][1] * SPP[22] + P[12][1] * SPP[18]) +
		      SPP[20] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[7] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) +
		      SPP[11] * (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[7] + P[9][2] * SPP[22] + P[12][2] * SPP[18]);
	nextP[1][4] = P[1][4] * SPP[6] - P[0][4] * SPP[2] - P[2][4] * SPP[8] + P[10][4] * SPP[22] + P[13][4] * SPP[17] +
		      SF[22] * (P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[8] + P[10][15] * SPP[22] + P[13][15] * SPP[17]) +
		      SPP[12] * (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[8] + P[10][1] * SPP[22] + P[13][1] * SPP[17]) +
		      SPP[20] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[8] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) +
		      SPP[11] * (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[8] + P[10][2] * SPP[22] + P[13][2] * SPP[17]);
	nextP[2][4] = P[0][4] * SPP[14] - P[1][4] * SPP[3] + P[2][4] * SPP[13] + P[11][4] * SPP[22] + P[14][4] * SPP[16] +
		      SF[22] * (P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16]) +
		      SPP[12] * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]) +
		      SPP[20] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) +
		      SPP[11] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]);
	nextP[3][4] = P[3][4] + SQ[2] + P[0][4] * SPP[1] + P[1][4] * SPP[19] + P[2][4] * SPP[15] - P[15][4] * SPP[21] +
		      SF[22] * (P[3][15] + P[0][15] * SPP[1] + P[1][15] * SPP[19] + P[2][15] * SPP[15] - P[15][15] * SPP[21]) + SPP[12] *
		      (P[3][1] + P[0][1] * SPP[1] + P[1][1] * SPP[19] + P[2][1] * SPP[15] - P[15][1] * SPP[21]) + SPP[20] *
		      (P[3][0] + P[0][0] * SPP[1] + P[1][0] * SPP[19] + P[2][0] * SPP[15] - P[15][0] * SPP[21]) + SPP[11] *
		      (P[3][2] + P[0][2] * SPP[1] + P[1][2] * SPP[19] + P[2][2] * SPP[15] - P[15][2] * SPP[21]);
	nextP[4][4] = P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11] + SQ[5]*sq(SQ[7] - 2*q0*q1) + SQ[6]*sq(SQ[9] + 2*q0*q3) + SF[22]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) + SPP[12]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]) + SPP[20]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[11]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + SQ[4]*sq(SG[1] - SG[2] + SG[3] - SG[4]);
	nextP[0][5] = P[0][5] * SPP[5] - P[1][5] * SPP[4] + P[2][5] * SPP[7] + P[9][5] * SPP[22] + P[12][5] * SPP[18] +
		      SF[20] * (P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[7] + P[9][15] * SPP[22] + P[12][15] * SPP[18]) -
		      SPP[9] * (P[0][0] * SPP[5] - P[1][0] * SPP[4] + P[2][0] * SPP[7] + P[9][0] * SPP[22] + P[12][0] * SPP[18]) + SPP[0] *
		      (P[0][2] * SPP[5] - P[1][2] * SPP[4] + P[2][2] * SPP[7] + P[9][2] * SPP[22] + P[12][2] * SPP[18]) + SPP[10] *
		      (P[0][1] * SPP[5] - P[1][1] * SPP[4] + P[2][1] * SPP[7] + P[9][1] * SPP[22] + P[12][1] * SPP[18]);
	nextP[1][5] = P[1][5] * SPP[6] - P[0][5] * SPP[2] - P[2][5] * SPP[8] + P[10][5] * SPP[22] + P[13][5] * SPP[17] +
		      SF[20] * (P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[8] + P[10][15] * SPP[22] + P[13][15] * SPP[17]) -
		      SPP[9] * (P[1][0] * SPP[6] - P[0][0] * SPP[2] - P[2][0] * SPP[8] + P[10][0] * SPP[22] + P[13][0] * SPP[17]) + SPP[0] *
		      (P[1][2] * SPP[6] - P[0][2] * SPP[2] - P[2][2] * SPP[8] + P[10][2] * SPP[22] + P[13][2] * SPP[17]) + SPP[10] *
		      (P[1][1] * SPP[6] - P[0][1] * SPP[2] - P[2][1] * SPP[8] + P[10][1] * SPP[22] + P[13][1] * SPP[17]);
	nextP[2][5] = P[0][5] * SPP[14] - P[1][5] * SPP[3] + P[2][5] * SPP[13] + P[11][5] * SPP[22] + P[14][5] * SPP[16] +
		      SF[20] * (P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16]) -
		      SPP[9] * (P[0][0] * SPP[14] - P[1][0] * SPP[3] + P[2][0] * SPP[13] + P[11][0] * SPP[22] + P[14][0] * SPP[16]) +
		      SPP[0] * (P[0][2] * SPP[14] - P[1][2] * SPP[3] + P[2][2] * SPP[13] + P[11][2] * SPP[22] + P[14][2] * SPP[16]) +
		      SPP[10] * (P[0][1] * SPP[14] - P[1][1] * SPP[3] + P[2][1] * SPP[13] + P[11][1] * SPP[22] + P[14][1] * SPP[16]);
	nextP[3][5] = P[3][5] + SQ[1] + P[0][5] * SPP[1] + P[1][5] * SPP[19] + P[2][5] * SPP[15] - P[15][5] * SPP[21] +
		      SF[20] * (P[3][15] + P[0][15] * SPP[1] + P[1][15] * SPP[19] + P[2][15] * SPP[15] - P[15][15] * SPP[21]) - SPP[9] *
		      (P[3][0] + P[0][0] * SPP[1] + P[1][0] * SPP[19] + P[2][0] * SPP[15] - P[15][0] * SPP[21]) + SPP[0] *
		      (P[3][2] + P[0][2] * SPP[1] + P[1][2] * SPP[19] + P[2][2] * SPP[15] - P[15][2] * SPP[21]) + SPP[10] *
		      (P[3][1] + P[0][1] * SPP[1] + P[1][1] * SPP[19] + P[2][1] * SPP[15] - P[15][1] * SPP[21]);
	nextP[4][5] = P[4][5] + SQ[0] + P[15][5] * SF[22] + P[0][5] * SPP[20] + P[1][5] * SPP[12] + P[2][5] * SPP[11] +
		      SF[20] * (P[4][15] + P[15][15] * SF[22] + P[0][15] * SPP[20] + P[1][15] * SPP[12] + P[2][15] * SPP[11]) - SPP[9] *
		      (P[4][0] + P[15][0] * SF[22] + P[0][0] * SPP[20] + P[1][0] * SPP[12] + P[2][0] * SPP[11]) + SPP[0] *
		      (P[4][2] + P[15][2] * SF[22] + P[0][2] * SPP[20] + P[1][2] * SPP[12] + P[2][2] * SPP[11]) + SPP[10] *
		      (P[4][1] + P[15][1] * SF[22] + P[0][1] * SPP[20] + P[1][1] * SPP[12] + P[2][1] * SPP[11]);
	nextP[5][5] = P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[9] + P[1][5]*SPP[10] + P[2][5]*SPP[0] + SQ[4]*sq(SQ[7] + 2*q0*q1) + SQ[6]*sq(SQ[8] - 2*q0*q2) + SF[20]*(P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[9] + P[1][15]*SPP[10] + P[2][15]*SPP[0]) - SPP[9]*(P[5][0] + P[15][0]*SF[20] - P[0][0]*SPP[9] + P[1][0]*SPP[10] + P[2][0]*SPP[0]) + SPP[0]*(P[5][2] + P[15][2]*SF[20] - P[0][2]*SPP[9] + P[1][2]*SPP[10] + P[2][2]*SPP[0]) + SPP[10]*(P[5][1] + P[15][1]*SF[20] - P[0][1]*SPP[9] + P[1][1]*SPP[10] + P[2][1]*SPP[0]) + SQ[5]*sq(SG[1] - SG[2] - SG[3] + SG[4]);
	nextP[0][6] = P[0][6] * SPP[5] - P[1][6] * SPP[4] + P[2][6] * SPP[7] + P[9][6] * SPP[22] + P[12][6] * SPP[18] + dt *
		      (P[0][3] * SPP[5] - P[1][3] * SPP[4] + P[2][3] * SPP[7] + P[9][3] * SPP[22] + P[12][3] * SPP[18]);
	nextP[1][6] = P[1][6] * SPP[6] - P[0][6] * SPP[2] - P[2][6] * SPP[8] + P[10][6] * SPP[22] + P[13][6] * SPP[17] + dt *
		      (P[1][3] * SPP[6] - P[0][3] * SPP[2] - P[2][3] * SPP[8] + P[10][3] * SPP[22] + P[13][3] * SPP[17]);
	nextP[2][6] = P[0][6] * SPP[14] - P[1][6] * SPP[3] + P[2][6] * SPP[13] + P[11][6] * SPP[22] + P[14][6] * SPP[16] +
		      dt * (P[0][3] * SPP[14] - P[1][3] * SPP[3] + P[2][3] * SPP[13] + P[11][3] * SPP[22] + P[14][3] * SPP[16]);
	nextP[3][6] = P[3][6] + P[0][6] * SPP[1] + P[1][6] * SPP[19] + P[2][6] * SPP[15] - P[15][6] * SPP[21] + dt *
		      (P[3][3] + P[0][3] * SPP[1] + P[1][3] * SPP[19] + P[2][3] * SPP[15] - P[15][3] * SPP[21]);
	nextP[4][6] = P[4][6] + P[15][6] * SF[22] + P[0][6] * SPP[20] + P[1][6] * SPP[12] + P[2][6] * SPP[11] + dt *
		      (P[4][3] + P[15][3] * SF[22] + P[0][3] * SPP[20] + P[1][3] * SPP[12] + P[2][3] * SPP[11]);
	nextP[5][6] = P[5][6] + P[15][6] * SF[20] - P[0][6] * SPP[9] + P[1][6] * SPP[10] + P[2][6] * SPP[0] + dt *
		      (P[5][3] + P[15][3] * SF[20] - P[0][3] * SPP[9] + P[1][3] * SPP[10] + P[2][3] * SPP[0]);
	nextP[6][6] = P[6][6] + P[3][6] * dt + dt * (P[6][3] + P[3][3] * dt);
	nextP[0][7] = P[0][7] * SPP[5] - P[1][7] * SPP[4] + P[2][7] * SPP[7] + P[9][7] * SPP[22] + P[12][7] * SPP[18] + dt *
		      (P[0][4] * SPP[5] - P[1][4] * SPP[4] + P[2][4] * SPP[7] + P[9][4] * SPP[22] + P[12][4] * SPP[18]);
	nextP[1][7] = P[1][7] * SPP[6] - P[0][7] * SPP[2] - P[2][7] * SPP[8] + P[10][7] * SPP[22] + P[13][7] * SPP[17] + dt *
		      (P[1][4] * SPP[6] - P[0][4] * SPP[2] - P[2][4] * SPP[8] + P[10][4] * SPP[22] + P[13][4] * SPP[17]);
	nextP[2][7] = P[0][7] * SPP[14] - P[1][7] * SPP[3] + P[2][7] * SPP[13] + P[11][7] * SPP[22] + P[14][7] * SPP[16] +
		      dt * (P[0][4] * SPP[14] - P[1][4] * SPP[3] + P[2][4] * SPP[13] + P[11][4] * SPP[22] + P[14][4] * SPP[16]);
	nextP[3][7] = P[3][7] + P[0][7] * SPP[1] + P[1][7] * SPP[19] + P[2][7] * SPP[15] - P[15][7] * SPP[21] + dt *
		      (P[3][4] + P[0][4] * SPP[1] + P[1][4] * SPP[19] + P[2][4] * SPP[15] - P[15][4] * SPP[21]);
	nextP[4][7] = P[4][7] + P[15][7] * SF[22] + P[0][7] * SPP[20] + P[1][7] * SPP[12] + P[2][7] * SPP[11] + dt *
		      (P[4][4] + P[15][4] * SF[22] + P[0][4] * SPP[20] + P[1][4] * SPP[12] + P[2][4] * SPP[11]);
	nextP[5][7] = P[5][7] + P[15][7] * SF[20] - P[0][7] * SPP[9] + P[1][7] * SPP[10] + P[2][7] * SPP[0] + dt *
		      (P[5][4] + P[15][4] * SF[20] - P[0][4] * SPP[9] + P[1][4] * SPP[10] + P[2][4] * SPP[0]);
	nextP[6][7] = P[6][7] + P[3][7] * dt + dt * (P[6][4] + P[3][4] * dt);
	nextP[7][7] = P[7][7] + P[4][7] * dt + dt * (P[7][4] + P[4][4] * dt);
	nextP[0][8] = P[0][8] * SPP[5] - P[1][8] * SPP[4] + P[2][8] * SPP[7] + P[9][8] * SPP[22] + P[12][8] * SPP[18] + dt *
		      (P[0][5] * SPP[5] - P[1][5] * SPP[4] + P[2][5] * SPP[7] + P[9][5] * SPP[22] + P[12][5] * SPP[18]);
	nextP[1][8] = P[1][8] * SPP[6] - P[0][8] * SPP[2] - P[2][8] * SPP[8] + P[10][8] * SPP[22] + P[13][8] * SPP[17] + dt *
		      (P[1][5] * SPP[6] - P[0][5] * SPP[2] - P[2][5] * SPP[8] + P[10][5] * SPP[22] + P[13][5] * SPP[17]);
	nextP[2][8] = P[0][8] * SPP[14] - P[1][8] * SPP[3] + P[2][8] * SPP[13] + P[11][8] * SPP[22] + P[14][8] * SPP[16] +
		      dt * (P[0][5] * SPP[14] - P[1][5] * SPP[3] + P[2][5] * SPP[13] + P[11][5] * SPP[22] + P[14][5] * SPP[16]);
	nextP[3][8] = P[3][8] + P[0][8] * SPP[1] + P[1][8] * SPP[19] + P[2][8] * SPP[15] - P[15][8] * SPP[21] + dt *
		      (P[3][5] + P[0][5] * SPP[1] + P[1][5] * SPP[19] + P[2][5] * SPP[15] - P[15][5] * SPP[21]);
	nextP[4][8] = P[4][8] + P[15][8] * SF[22] + P[0][8] * SPP[20] + P[1][8] * SPP[12] + P[2][8] * SPP[11] + dt *
		      (P[4][5] + P[15][5] * SF[22] + P[0][5] * SPP[20] + P[1][5] * SPP[12] + P[2][5] * SPP[11]);
	nextP[5][8] = P[5][8] + P[15][8] * SF[20] - P[0][8] * SPP[9] + P[1][8] * SPP[10] + P[2][8] * SPP[0] + dt *
		      (P[5][5] + P[15][5] * SF[20] - P[0][5] * SPP[9] + P[1][5] * SPP[10] + P[2][5] * SPP[0]);
	nextP[6][8] = P[6][8] + P[3][8] * dt + dt * (P[6][5] + P[3][5] * dt);
	nextP[7][8] = P[7][8] + P[4][8] * dt + dt * (P[7][5] + P[4][5] * dt);
	nextP[8][8] = P[8][8] + P[5][8] * dt + dt * (P[8][5] + P[5][5] * dt);
	nextP[0][9] = P[0][9] * SPP[5] - P[1][9] * SPP[4] + P[2][9] * SPP[7] + P[9][9] * SPP[22] + P[12][9] * SPP[18];
	nextP[1][9] = P[1][9] * SPP[6] - P[0][9] * SPP[2] - P[2][9] * SPP[8] + P[10][9] * SPP[22] + P[13][9] * SPP[17];
	nextP[2][9] = P[0][9] * SPP[14] - P[1][9] * SPP[3] + P[2][9] * SPP[13] + P[11][9] * SPP[22] + P[14][9] * SPP[16];
	nextP[3][9] = P[3][9] + P[0][9] * SPP[1] + P[1][9] * SPP[19] + P[2][9] * SPP[15] - P[15][9] * SPP[21];
	nextP[4][9] = P[4][9] + P[15][9] * SF[22] + P[0][9] * SPP[20] + P[1][9] * SPP[12] + P[2][9] * SPP[11];
	nextP[5][9] = P[5][9] + P[15][9] * SF[20] - P[0][9] * SPP[9] + P[1][9] * SPP[10] + P[2][9] * SPP[0];
	nextP[6][9] = P[6][9] + P[3][9] * dt;
	nextP[7][9] = P[7][9] + P[4][9] * dt;
	nextP[8][9] = P[8][9] + P[5][9] * dt;
	nextP[9][9] = P[9][9];
	nextP[0][10] = P[0][10] * SPP[5] - P[1][10] * SPP[4] + P[2][10] * SPP[7] + P[9][10] * SPP[22] + P[12][10] * SPP[18];
	nextP[1][10] = P[1][10] * SPP[6] - P[0][10] * SPP[2] - P[2][10] * SPP[8] + P[10][10] * SPP[22] + P[13][10] * SPP[17];
	nextP[2][10] = P[0][10] * SPP[14] - P[1][10] * SPP[3] + P[2][10] * SPP[13] + P[11][10] * SPP[22] + P[14][10] * SPP[16];
	nextP[3][10] = P[3][10] + P[0][10] * SPP[1] + P[1][10] * SPP[19] + P[2][10] * SPP[15] - P[15][10] * SPP[21];
	nextP[4][10] = P[4][10] + P[15][10] * SF[22] + P[0][10] * SPP[20] + P[1][10] * SPP[12] + P[2][10] * SPP[11];
	nextP[5][10] = P[5][10] + P[15][10] * SF[20] - P[0][10] * SPP[9] + P[1][10] * SPP[10] + P[2][10] * SPP[0];
	nextP[6][10] = P[6][10] + P[3][10] * dt;
	nextP[7][10] = P[7][10] + P[4][10] * dt;
	nextP[8][10] = P[8][10] + P[5][10] * dt;
	nextP[9][10] = P[9][10];
	nextP[10][10] = P[10][10];
	nextP[0][11] = P[0][11] * SPP[5] - P[1][11] * SPP[4] + P[2][11] * SPP[7] + P[9][11] * SPP[22] + P[12][11] * SPP[18];
	nextP[1][11] = P[1][11] * SPP[6] - P[0][11] * SPP[2] - P[2][11] * SPP[8] + P[10][11] * SPP[22] + P[13][11] * SPP[17];
	nextP[2][11] = P[0][11] * SPP[14] - P[1][11] * SPP[3] + P[2][11] * SPP[13] + P[11][11] * SPP[22] + P[14][11] * SPP[16];
	nextP[3][11] = P[3][11] + P[0][11] * SPP[1] + P[1][11] * SPP[19] + P[2][11] * SPP[15] - P[15][11] * SPP[21];
	nextP[4][11] = P[4][11] + P[15][11] * SF[22] + P[0][11] * SPP[20] + P[1][11] * SPP[12] + P[2][11] * SPP[11];
	nextP[5][11] = P[5][11] + P[15][11] * SF[20] - P[0][11] * SPP[9] + P[1][11] * SPP[10] + P[2][11] * SPP[0];
	nextP[6][11] = P[6][11] + P[3][11] * dt;
	nextP[7][11] = P[7][11] + P[4][11] * dt;
	nextP[8][11] = P[8][11] + P[5][11] * dt;
	nextP[9][11] = P[9][11];
	nextP[10][11] = P[10][11];
	nextP[11][11] = P[11][11];
	nextP[0][12] = P[0][12] * SPP[5] - P[1][12] * SPP[4] + P[2][12] * SPP[7] + P[9][12] * SPP[22] + P[12][12] * SPP[18];
	nextP[1][12] = P[1][12] * SPP[6] - P[0][12] * SPP[2] - P[2][12] * SPP[8] + P[10][12] * SPP[22] + P[13][12] * SPP[17];
	nextP[2][12] = P[0][12] * SPP[14] - P[1][12] * SPP[3] + P[2][12] * SPP[13] + P[11][12] * SPP[22] + P[14][12] * SPP[16];
	nextP[3][12] = P[3][12] + P[0][12] * SPP[1] + P[1][12] * SPP[19] + P[2][12] * SPP[15] - P[15][12] * SPP[21];
	nextP[4][12] = P[4][12] + P[15][12] * SF[22] + P[0][12] * SPP[20] + P[1][12] * SPP[12] + P[2][12] * SPP[11];
	nextP[5][12] = P[5][12] + P[15][12] * SF[20] - P[0][12] * SPP[9] + P[1][12] * SPP[10] + P[2][12] * SPP[0];
	nextP[6][12] = P[6][12] + P[3][12] * dt;
	nextP[7][12] = P[7][12] + P[4][12] * dt;
	nextP[8][12] = P[8][12] + P[5][12] * dt;
	nextP[9][12] = P[9][12];
	nextP[10][12] = P[10][12];
	nextP[11][12] = P[11][12];
	nextP[12][12] = P[12][12];
	nextP[0][13] = P[0][13] * SPP[5] - P[1][13] * SPP[4] + P[2][13] * SPP[7] + P[9][13] * SPP[22] + P[12][13] * SPP[18];
	nextP[1][13] = P[1][13] * SPP[6] - P[0][13] * SPP[2] - P[2][13] * SPP[8] + P[10][13] * SPP[22] + P[13][13] * SPP[17];
	nextP[2][13] = P[0][13] * SPP[14] - P[1][13] * SPP[3] + P[2][13] * SPP[13] + P[11][13] * SPP[22] + P[14][13] * SPP[16];
	nextP[3][13] = P[3][13] + P[0][13] * SPP[1] + P[1][13] * SPP[19] + P[2][13] * SPP[15] - P[15][13] * SPP[21];
	nextP[4][13] = P[4][13] + P[15][13] * SF[22] + P[0][13] * SPP[20] + P[1][13] * SPP[12] + P[2][13] * SPP[11];
	nextP[5][13] = P[5][13] + P[15][13] * SF[20] - P[0][13] * SPP[9] + P[1][13] * SPP[10] + P[2][13] * SPP[0];
	nextP[6][13] = P[6][13] + P[3][13] * dt;
	nextP[7][13] = P[7][13] + P[4][13] * dt;
	nextP[8][13] = P[8][13] + P[5][13] * dt;
	nextP[9][13] = P[9][13];
	nextP[10][13] = P[10][13];
	nextP[11][13] = P[11][13];
	nextP[12][13] = P[12][13];
	nextP[13][13] = P[13][13];
	nextP[0][14] = P[0][14] * SPP[5] - P[1][14] * SPP[4] + P[2][14] * SPP[7] + P[9][14] * SPP[22] + P[12][14] * SPP[18];
	nextP[1][14] = P[1][14] * SPP[6] - P[0][14] * SPP[2] - P[2][14] * SPP[8] + P[10][14] * SPP[22] + P[13][14] * SPP[17];
	nextP[2][14] = P[0][14] * SPP[14] - P[1][14] * SPP[3] + P[2][14] * SPP[13] + P[11][14] * SPP[22] + P[14][14] * SPP[16];
	nextP[3][14] = P[3][14] + P[0][14] * SPP[1] + P[1][14] * SPP[19] + P[2][14] * SPP[15] - P[15][14] * SPP[21];
	nextP[4][14] = P[4][14] + P[15][14] * SF[22] + P[0][14] * SPP[20] + P[1][14] * SPP[12] + P[2][14] * SPP[11];
	nextP[5][14] = P[5][14] + P[15][14] * SF[20] - P[0][14] * SPP[9] + P[1][14] * SPP[10] + P[2][14] * SPP[0];
	nextP[6][14] = P[6][14] + P[3][14] * dt;
	nextP[7][14] = P[7][14] + P[4][14] * dt;
	nextP[8][14] = P[8][14] + P[5][14] * dt;
	nextP[9][14] = P[9][14];
	nextP[10][14] = P[10][14];
	nextP[11][14] = P[11][14];
	nextP[12][14] = P[12][14];
	nextP[13][14] = P[13][14];
	nextP[14][14] = P[14][14];
	nextP[0][15] = P[0][15] * SPP[5] - P[1][15] * SPP[4] + P[2][15] * SPP[7] + P[9][15] * SPP[22] + P[12][15] * SPP[18];
	nextP[1][15] = P[1][15] * SPP[6] - P[0][15] * SPP[2] - P[2][15] * SPP[8] + P[10][15] * SPP[22] + P[13][15] * SPP[17];
	nextP[2][15] = P[0][15] * SPP[14] - P[1][15] * SPP[3] + P[2][15] * SPP[13] + P[11][15] * SPP[22] + P[14][15] * SPP[16];
	nextP[3][15] = P[3][15] + P[0][15] * SPP[1] + P[1][15] * SPP[19] + P[2][15] * SPP[15] - P[15][15] * SPP[21];
	nextP[4][15] = P[4][15] + P[15][15] * SF[22] + P[0][15] * SPP[20] + P[1][15] * SPP[12] + P[2][15] * SPP[11];
	nextP[5][15] = P[5][15] + P[15][15] * SF[20] - P[0][15] * SPP[9] + P[1][15] * SPP[10] + P[2][15] * SPP[0];
	nextP[6][15] = P[6][15] + P[3][15] * dt;
	nextP[7][15] = P[7][15] + P[4][15] * dt;
	nextP[8][15] = P[8][15] + P[5][15] * dt;
	nextP[9][15] = P[9][15];
	nextP[10][15] = P[10][15];
	nextP[11][15] = P[11][15];
	nextP[12][15] = P[12][15];
	nextP[13][15] = P[13][15];
	nextP[14][15] = P[14][15];
	nextP[15][15] = P[15][15];

	// Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
	if (_control_status.flags.mag_3D) {
		// Check if we have just transitioned into 3-axis fusion and set the state variances
		if (!_control_status_prev.flags.mag_3D) {
			for (uint8_t index = 16; index <= 21; index++) {
				P[index][index] = sq(fmaxf(_params.mag_noise, 0.001f));
			}
		}

		nextP[0][16] = P[0][16] * SPP[5] - P[1][16] * SPP[4] + P[2][16] * SPP[7] + P[9][16] * SPP[22] + P[12][16] * SPP[18];
		nextP[1][16] = P[1][16] * SPP[6] - P[0][16] * SPP[2] - P[2][16] * SPP[8] + P[10][16] * SPP[22] + P[13][16] * SPP[17];
		nextP[2][16] = P[0][16] * SPP[14] - P[1][16] * SPP[3] + P[2][16] * SPP[13] + P[11][16] * SPP[22] + P[14][16] * SPP[16];
		nextP[3][16] = P[3][16] + P[0][16] * SPP[1] + P[1][16] * SPP[19] + P[2][16] * SPP[15] - P[15][16] * SPP[21];
		nextP[4][16] = P[4][16] + P[15][16] * SF[22] + P[0][16] * SPP[20] + P[1][16] * SPP[12] + P[2][16] * SPP[11];
		nextP[5][16] = P[5][16] + P[15][16] * SF[20] - P[0][16] * SPP[9] + P[1][16] * SPP[10] + P[2][16] * SPP[0];
		nextP[6][16] = P[6][16] + P[3][16] * dt;
		nextP[7][16] = P[7][16] + P[4][16] * dt;
		nextP[8][16] = P[8][16] + P[5][16] * dt;
		nextP[9][16] = P[9][16];
		nextP[10][16] = P[10][16];
		nextP[11][16] = P[11][16];
		nextP[12][16] = P[12][16];
		nextP[13][16] = P[13][16];
		nextP[14][16] = P[14][16];
		nextP[15][16] = P[15][16];
		nextP[16][16] = P[16][16];

		nextP[0][17] = P[0][17] * SPP[5] - P[1][17] * SPP[4] + P[2][17] * SPP[7] + P[9][17] * SPP[22] + P[12][17] * SPP[18];
		nextP[1][17] = P[1][17] * SPP[6] - P[0][17] * SPP[2] - P[2][17] * SPP[8] + P[10][17] * SPP[22] + P[13][17] * SPP[17];
		nextP[2][17] = P[0][17] * SPP[14] - P[1][17] * SPP[3] + P[2][17] * SPP[13] + P[11][17] * SPP[22] + P[14][17] * SPP[16];
		nextP[3][17] = P[3][17] + P[0][17] * SPP[1] + P[1][17] * SPP[19] + P[2][17] * SPP[15] - P[15][17] * SPP[21];
		nextP[4][17] = P[4][17] + P[15][17] * SF[22] + P[0][17] * SPP[20] + P[1][17] * SPP[12] + P[2][17] * SPP[11];
		nextP[5][17] = P[5][17] + P[15][17] * SF[20] - P[0][17] * SPP[9] + P[1][17] * SPP[10] + P[2][17] * SPP[0];
		nextP[6][17] = P[6][17] + P[3][17] * dt;
		nextP[7][17] = P[7][17] + P[4][17] * dt;
		nextP[8][17] = P[8][17] + P[5][17] * dt;
		nextP[9][17] = P[9][17];
		nextP[10][17] = P[10][17];
		nextP[11][17] = P[11][17];
		nextP[12][17] = P[12][17];
		nextP[13][17] = P[13][17];
		nextP[14][17] = P[14][17];
		nextP[15][17] = P[15][17];
		nextP[16][17] = P[16][17];
		nextP[17][17] = P[17][17];

		nextP[0][18] = P[0][18] * SPP[5] - P[1][18] * SPP[4] + P[2][18] * SPP[7] + P[9][18] * SPP[22] + P[12][18] * SPP[18];
		nextP[1][18] = P[1][18] * SPP[6] - P[0][18] * SPP[2] - P[2][18] * SPP[8] + P[10][18] * SPP[22] + P[13][18] * SPP[17];
		nextP[2][18] = P[0][18] * SPP[14] - P[1][18] * SPP[3] + P[2][18] * SPP[13] + P[11][18] * SPP[22] + P[14][18] * SPP[16];
		nextP[3][18] = P[3][18] + P[0][18] * SPP[1] + P[1][18] * SPP[19] + P[2][18] * SPP[15] - P[15][18] * SPP[21];
		nextP[4][18] = P[4][18] + P[15][18] * SF[22] + P[0][18] * SPP[20] + P[1][18] * SPP[12] + P[2][18] * SPP[11];
		nextP[5][18] = P[5][18] + P[15][18] * SF[20] - P[0][18] * SPP[9] + P[1][18] * SPP[10] + P[2][18] * SPP[0];
		nextP[6][18] = P[6][18] + P[3][18] * dt;
		nextP[7][18] = P[7][18] + P[4][18] * dt;
		nextP[8][18] = P[8][18] + P[5][18] * dt;
		nextP[9][18] = P[9][18];
		nextP[10][18] = P[10][18];
		nextP[11][18] = P[11][18];
		nextP[12][18] = P[12][18];
		nextP[13][18] = P[13][18];
		nextP[14][18] = P[14][18];
		nextP[15][18] = P[15][18];
		nextP[16][18] = P[16][18];
		nextP[17][18] = P[17][18];
		nextP[18][18] = P[18][18];

		nextP[0][19] = P[0][19] * SPP[5] - P[1][19] * SPP[4] + P[2][19] * SPP[7] + P[9][19] * SPP[22] + P[12][19] * SPP[18];
		nextP[1][19] = P[1][19] * SPP[6] - P[0][19] * SPP[2] - P[2][19] * SPP[8] + P[10][19] * SPP[22] + P[13][19] * SPP[17];
		nextP[2][19] = P[0][19] * SPP[14] - P[1][19] * SPP[3] + P[2][19] * SPP[13] + P[11][19] * SPP[22] + P[14][19] * SPP[16];
		nextP[3][19] = P[3][19] + P[0][19] * SPP[1] + P[1][19] * SPP[19] + P[2][19] * SPP[15] - P[15][19] * SPP[21];
		nextP[4][19] = P[4][19] + P[15][19] * SF[22] + P[0][19] * SPP[20] + P[1][19] * SPP[12] + P[2][19] * SPP[11];
		nextP[5][19] = P[5][19] + P[15][19] * SF[20] - P[0][19] * SPP[9] + P[1][19] * SPP[10] + P[2][19] * SPP[0];
		nextP[6][19] = P[6][19] + P[3][19] * dt;
		nextP[7][19] = P[7][19] + P[4][19] * dt;
		nextP[8][19] = P[8][19] + P[5][19] * dt;
		nextP[9][19] = P[9][19];
		nextP[10][19] = P[10][19];
		nextP[11][19] = P[11][19];
		nextP[12][19] = P[12][19];
		nextP[13][19] = P[13][19];
		nextP[14][19] = P[14][19];
		nextP[15][19] = P[15][19];
		nextP[16][19] = P[16][19];
		nextP[17][19] = P[17][19];
		nextP[18][19] = P[18][19];
		nextP[19][19] = P[19][19];

		nextP[0][20] = P[0][20] * SPP[5] - P[1][20] * SPP[4] + P[2][20] * SPP[7] + P[9][20] * SPP[22] + P[12][20] * SPP[18];
		nextP[1][20] = P[1][20] * SPP[6] - P[0][20] * SPP[2] - P[2][20] * SPP[8] + P[10][20] * SPP[22] + P[13][20] * SPP[17];
		nextP[2][20] = P[0][20] * SPP[14] - P[1][20] * SPP[3] + P[2][20] * SPP[13] + P[11][20] * SPP[22] + P[14][20] * SPP[16];
		nextP[3][20] = P[3][20] + P[0][20] * SPP[1] + P[1][20] * SPP[19] + P[2][20] * SPP[15] - P[15][20] * SPP[21];
		nextP[4][20] = P[4][20] + P[15][20] * SF[22] + P[0][20] * SPP[20] + P[1][20] * SPP[12] + P[2][20] * SPP[11];
		nextP[5][20] = P[5][20] + P[15][20] * SF[20] - P[0][20] * SPP[9] + P[1][20] * SPP[10] + P[2][20] * SPP[0];
		nextP[6][20] = P[6][20] + P[3][20] * dt;
		nextP[7][20] = P[7][20] + P[4][20] * dt;
		nextP[8][20] = P[8][20] + P[5][20] * dt;
		nextP[9][20] = P[9][20];
		nextP[10][20] = P[10][20];
		nextP[11][20] = P[11][20];
		nextP[12][20] = P[12][20];
		nextP[13][20] = P[13][20];
		nextP[14][20] = P[14][20];
		nextP[15][20] = P[15][20];
		nextP[16][20] = P[16][20];
		nextP[17][20] = P[17][20];
		nextP[18][20] = P[18][20];
		nextP[19][20] = P[19][20];
		nextP[20][20] = P[20][20];

		nextP[0][21] = P[0][21] * SPP[5] - P[1][21] * SPP[4] + P[2][21] * SPP[7] + P[9][21] * SPP[22] + P[12][21] * SPP[18];
		nextP[1][21] = P[1][21] * SPP[6] - P[0][21] * SPP[2] - P[2][21] * SPP[8] + P[10][21] * SPP[22] + P[13][21] * SPP[17];
		nextP[2][21] = P[0][21] * SPP[14] - P[1][21] * SPP[3] + P[2][21] * SPP[13] + P[11][21] * SPP[22] + P[14][21] * SPP[16];
		nextP[3][21] = P[3][21] + P[0][21] * SPP[1] + P[1][21] * SPP[19] + P[2][21] * SPP[15] - P[15][21] * SPP[21];
		nextP[4][21] = P[4][21] + P[15][21] * SF[22] + P[0][21] * SPP[20] + P[1][21] * SPP[12] + P[2][21] * SPP[11];
		nextP[5][21] = P[5][21] + P[15][21] * SF[20] - P[0][21] * SPP[9] + P[1][21] * SPP[10] + P[2][21] * SPP[0];
		nextP[6][21] = P[6][21] + P[3][21] * dt;
		nextP[7][21] = P[7][21] + P[4][21] * dt;
		nextP[8][21] = P[8][21] + P[5][21] * dt;
		nextP[9][21] = P[9][21];
		nextP[10][21] = P[10][21];
		nextP[11][21] = P[11][21];
		nextP[12][21] = P[12][21];
		nextP[13][21] = P[13][21];
		nextP[14][21] = P[14][21];
		nextP[15][21] = P[15][21];
		nextP[16][21] = P[16][21];
		nextP[17][21] = P[17][21];
		nextP[18][21] = P[18][21];
		nextP[19][21] = P[19][21];
		nextP[20][21] = P[20][21];
		nextP[21][21] = P[21][21];
	}

	// Don't do covariance prediction on wind states unless we are using them
	if (_control_status.flags.wind) {
		// Check if we have jsut transitioned to using wind states and set the variances accordingly
		if (!_control_status_prev.flags.mag_3D) {
			for (uint8_t index = 22; index <= 23; index++) {
				// TODO initialise wind states using ground speed and airspeed and set initial variance using sum of ground speed and airspeed variances
				P[index][index] = sq(5.0f);
			}
		}

		nextP[0][22] = P[0][22] * SPP[5] - P[1][22] * SPP[4] + P[2][22] * SPP[7] + P[9][22] * SPP[22] + P[12][22] * SPP[18];
		nextP[1][22] = P[1][22] * SPP[6] - P[0][22] * SPP[2] - P[2][22] * SPP[8] + P[10][22] * SPP[22] + P[13][22] * SPP[17];
		nextP[2][22] = P[0][22] * SPP[14] - P[1][22] * SPP[3] + P[2][22] * SPP[13] + P[11][22] * SPP[22] + P[14][22] * SPP[16];
		nextP[3][22] = P[3][22] + P[0][22] * SPP[1] + P[1][22] * SPP[19] + P[2][22] * SPP[15] - P[15][22] * SPP[21];
		nextP[4][22] = P[4][22] + P[15][22] * SF[22] + P[0][22] * SPP[20] + P[1][22] * SPP[12] + P[2][22] * SPP[11];
		nextP[5][22] = P[5][22] + P[15][22] * SF[20] - P[0][22] * SPP[9] + P[1][22] * SPP[10] + P[2][22] * SPP[0];
		nextP[6][22] = P[6][22] + P[3][22] * dt;
		nextP[7][22] = P[7][22] + P[4][22] * dt;
		nextP[8][22] = P[8][22] + P[5][22] * dt;
		nextP[9][22] = P[9][22];
		nextP[10][22] = P[10][22];
		nextP[11][22] = P[11][22];
		nextP[12][22] = P[12][22];
		nextP[13][22] = P[13][22];
		nextP[14][22] = P[14][22];
		nextP[15][22] = P[15][22];
		nextP[16][22] = P[16][22];
		nextP[17][22] = P[17][22];
		nextP[18][22] = P[18][22];
		nextP[19][22] = P[19][22];
		nextP[20][22] = P[20][22];
		nextP[21][22] = P[21][22];
		nextP[22][22] = P[22][22];

		nextP[0][23] = P[0][23] * SPP[5] - P[1][23] * SPP[4] + P[2][23] * SPP[7] + P[9][23] * SPP[22] + P[12][23] * SPP[18];
		nextP[1][23] = P[1][23] * SPP[6] - P[0][23] * SPP[2] - P[2][23] * SPP[8] + P[10][23] * SPP[22] + P[13][23] * SPP[17];
		nextP[2][23] = P[0][23] * SPP[14] - P[1][23] * SPP[3] + P[2][23] * SPP[13] + P[11][23] * SPP[22] + P[14][23] * SPP[16];
		nextP[3][23] = P[3][23] + P[0][23] * SPP[1] + P[1][23] * SPP[19] + P[2][23] * SPP[15] - P[15][23] * SPP[21];
		nextP[4][23] = P[4][23] + P[15][23] * SF[22] + P[0][23] * SPP[20] + P[1][23] * SPP[12] + P[2][23] * SPP[11];
		nextP[5][23] = P[5][23] + P[15][23] * SF[20] - P[0][23] * SPP[9] + P[1][23] * SPP[10] + P[2][23] * SPP[0];
		nextP[6][23] = P[6][23] + P[3][23] * dt;
		nextP[7][23] = P[7][23] + P[4][23] * dt;
		nextP[8][23] = P[8][23] + P[5][23] * dt;
		nextP[9][23] = P[9][23];
		nextP[10][23] = P[10][23];
		nextP[11][23] = P[11][23];
		nextP[12][23] = P[12][23];
		nextP[13][23] = P[13][23];
		nextP[14][23] = P[14][23];
		nextP[15][23] = P[15][23];
		nextP[16][23] = P[16][23];
		nextP[17][23] = P[17][23];
		nextP[18][23] = P[18][23];
		nextP[19][23] = P[19][23];
		nextP[20][23] = P[20][23];
		nextP[21][23] = P[21][23];
		nextP[22][23] = P[22][23];
		nextP[23][23] = P[23][23];

	}

	// add process noise
	for (unsigned i = 0; i < _k_num_states; i++) {
		nextP[i][i] += process_noise[i];
	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P[6][6] + P[7][7]) > 1e4f) {
		for (uint8_t i = 6; i < 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP[i][j] = P[i][j];
				nextP[j][i] = P[j][i];
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			nextP[row][column] = nextP[column][row];
		}
	}

	// copy variances (diagonals)
	for (unsigned i = 0; i < _k_num_states; i++) {
		P[i][i] = nextP[i][i];
	}

	// force symmetry
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0; column < row; column++) {
			P[row][column] = 0.5f * (nextP[row][column] + nextP[column][row]);
			P[column][row] = P[row][column];
		}
	}

	limitCov();
}

void Ekf::limitCov()
{
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)
	float P_lim[9] = {};
	P_lim[0] = 1.0f;		// angle error max var
	P_lim[1] = 1000.0f;		// velocity max var
	P_lim[2] = 1000000.0f;		// positiion max var
	P_lim[3] = 0.001f;		// gyro bias max var
	P_lim[4] = 0.01f;		// gyro scale max var
	P_lim[5] = 0.1f;		// delta velocity z bias max var
	P_lim[6] = 0.1f;		// earth mag field max var
	P_lim[7] = 0.1f;		// body mag field max var
	P_lim[8] = 1000.0f;		// wind max var

	for (int i = 0; i < 3; i++) {

		math::constrain(P[i][i], 0.0f, P_lim[0]);
	}

	for (int i = 3; i < 6; i++) {

		math::constrain(P[i][i], 0.0f, P_lim[1]);
	}

	for (int i = 6; i < 9; i++) {


		math::constrain(P[i][i], 0.0f, P_lim[2]);
	}

	for (int i = 9; i < 12; i++) {


		math::constrain(P[i][i], 0.0f, P_lim[3]);
	}

	for (int i = 12; i < 15; i++) {


		math::constrain(P[i][i], 0.0f, P_lim[4]);
	}


	math::constrain(P[15][15], 0.0f, P_lim[5]);

	for (int i = 16; i < 19; i++) {
		math::constrain(P[i][i], 0.0f, P_lim[6]);
	}

	for (int i = 19; i < 22; i++) {
		math::constrain(P[i][i], 0.0f, P_lim[7]);
	}

	for (int i = 22; i < 24; i++) {
		math::constrain(P[i][i], 0.0f, P_lim[8]);
	}
}
