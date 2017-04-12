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

#include "../ecl.h"
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
	float dt = 0.001f * (float)FILTER_UPDATE_PERIOD_MS;

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var(2) = rot_vec_var(1) = rot_vec_var(0) = sq(_params.initial_tilt_err);

	// update the quaternion state covariances
	initialiseQuatCovariances(rot_vec_var);

	// velocity
	P[4][4] = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P[5][5] = P[4][4];
	P[6][6] = sq(1.5f) * P[4][4];

	// position
	P[7][7] = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P[8][8] = P[7][7];
	if (_control_status.flags.rng_hgt) {
		P[9][9] = sq(fmaxf(_params.range_noise, 0.01f));
	} else if (_control_status.flags.gps_hgt) {
		float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
		float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
		P[9][9] = sq(1.5f * math::constrain(_gps_sample_delayed.vacc, lower_limit, upper_limit));
	} else {
		P[9][9] = sq(fmaxf(_params.baro_noise, 0.01f));
	}

	// gyro bias
	P[10][10] = sq(_params.switch_on_gyro_bias * dt);
	P[11][11] = P[10][10];
	P[12][12] = P[10][10];

	// accel bias
	_prev_dvel_bias_var(0) = P[13][13] = sq(_params.switch_on_accel_bias * dt);
	_prev_dvel_bias_var(1) = P[14][14] = P[13][13];
	_prev_dvel_bias_var(2) = P[15][15] = P[13][13];

	// variances for optional states

	// earth frame and body frame magnetic field
	// set to observation variance
	for (uint8_t index=16; index <= 21; index ++) {
		P[index][index] = sq(_params.mag_noise);
	}

	// wind
	P[22][22] = 1.0f;
	P[23][23] = 1.0f;

}

void Ekf::get_pos_var(Vector3f &pos_var)
{
	pos_var(0) = P[7][7];
	pos_var(1) = P[8][8];
	pos_var(2) = P[9][9];
}

void Ekf::get_vel_var(Vector3f &vel_var)
{
	vel_var(0) = P[4][4];
	vel_var(1) = P[5][5];
	vel_var(2) = P[6][6];
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

	float dvx_b = _state.accel_bias(0);
	float dvy_b = _state.accel_bias(1);
	float dvz_b = _state.accel_bias(2);

	float dt = math::constrain(_imu_sample_delayed.delta_ang_dt, 0.0005f * FILTER_UPDATE_PERIOD_MS, 0.002f * FILTER_UPDATE_PERIOD_MS);

	// compute noise variance for stationary processes
	float process_noise[_k_num_states] = {};

	// convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

	// convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);

	// inhibit learning of imu acccel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	float alpha = 1.0f - math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	_ang_rate_mag_filt = fmax(_imu_sample_delayed.delta_ang.norm(), alpha * _ang_rate_mag_filt);
	_accel_mag_filt = fmax(_imu_sample_delayed.delta_vel.norm(), alpha * _accel_mag_filt);
	if (_ang_rate_mag_filt > dt * _params.acc_bias_learn_gyr_lim
			|| _accel_mag_filt > dt * _params.acc_bias_learn_acc_lim
			|| _bad_vert_accel_detected) {
		// store the bias state variances to be reinstated later
		if (!_accel_bias_inhibit) {
			_prev_dvel_bias_var(0) = P[13][13];
			_prev_dvel_bias_var(1) = P[14][14];
			_prev_dvel_bias_var(2) = P[15][15];
		}
		_accel_bias_inhibit = true;
	} else {
		if (_accel_bias_inhibit) {
			// reinstate the bias state variances
			P[13][13] = _prev_dvel_bias_var(0);
			P[14][14] = _prev_dvel_bias_var(1);
			P[15][15] = _prev_dvel_bias_var(2);
		} else {
			// store the bias state variances to be reinstated later
			_prev_dvel_bias_var(0) = P[13][13];
			_prev_dvel_bias_var(1) = P[14][14];
			_prev_dvel_bias_var(2) = P[15][15];
		}
		_accel_bias_inhibit = false;
	}

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_I_sig;
	if (_control_status.flags.mag_3D && (P[16][16] + P[17][17] + P[18][18]) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;
	if (_control_status.flags.mag_3D && (P[19][19] + P[20][20] + P[21][21]) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1.0f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_sig;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && (P[22][22] + P[23][23]) < 1000.0f) {
		wind_vel_sig = dt * math::constrain(_params.wind_vel_p_noise, 0.0f, 1.0f);

	} else {
		wind_vel_sig = 0.0f;
	}

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances
	for (unsigned i = 0; i <= 9; i++) {
		process_noise[i] = 0.0f;
	}
	// delta angle bias states
	process_noise[12] = process_noise[11] = process_noise[10] = sq(d_ang_bias_sig);
	// delta_velocity bias states
	process_noise[15] = process_noise[14] = process_noise[13] = sq(d_vel_bias_sig);
	// earth frame magnetic field states
	process_noise[18] = process_noise[17] = process_noise[16] = sq(mag_I_sig);
	// body frame magnetic field states
	process_noise[21] = process_noise[20] = process_noise[19] = sq(mag_B_sig);
	// wind velocity states
	process_noise[23] = process_noise[22] = sq(wind_vel_sig);

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float daxVar, dayVar, dazVar;
	float dvxVar, dvyVar, dvzVar;
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	daxVar = dayVar = dazVar = sq(dt * gyro_noise);
	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);
	if (_bad_vert_accel_detected) {
		// Increase accelerometer process noise if bad accel data is detected. Measurement errors due to
		// vibration induced clipping commonly reach an equivalent 0.5g offset.
		accel_noise = BADACC_BIAS_PNOISE;
	}
	dvxVar = dvyVar = dvzVar = sq(dt * accel_noise);

	// predict the covariance

	// intermediate calculations
	float SF[21];
	SF[0] = dvz - dvz_b;
	SF[1] = dvy - dvy_b;
	SF[2] = dvx - dvx_b;
	SF[3] = 2*q1*SF[2] + 2*q2*SF[1] + 2*q3*SF[0];
	SF[4] = 2*q0*SF[1] - 2*q1*SF[0] + 2*q3*SF[2];
	SF[5] = 2*q0*SF[2] + 2*q2*SF[0] - 2*q3*SF[1];
	SF[6] = day/2 - day_b/2;
	SF[7] = daz/2 - daz_b/2;
	SF[8] = dax/2 - dax_b/2;
	SF[9] = dax_b/2 - dax/2;
	SF[10] = daz_b/2 - daz/2;
	SF[11] = day_b/2 - day/2;
	SF[12] = 2*q1*SF[1];
	SF[13] = 2*q0*SF[0];
	SF[14] = q1/2;
	SF[15] = q2/2;
	SF[16] = q3/2;
	SF[17] = sq(q3);
	SF[18] = sq(q2);
	SF[19] = sq(q1);
	SF[20] = sq(q0);

	float SG[8];
	SG[0] = q0/2;
	SG[1] = sq(q3);
	SG[2] = sq(q2);
	SG[3] = sq(q1);
	SG[4] = sq(q0);
	SG[5] = 2*q2*q3;
	SG[6] = 2*q1*q3;
	SG[7] = 2*q1*q2;

	float SQ[11];
	SQ[0] = dvzVar*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyVar*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxVar*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
	SQ[1] = dvzVar*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxVar*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyVar*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
	SQ[2] = dvzVar*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyVar*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxVar*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
	SQ[3] = (dayVar*q1*SG[0])/2 - (dazVar*q1*SG[0])/2 - (daxVar*q2*q3)/4;
	SQ[4] = (dazVar*q2*SG[0])/2 - (daxVar*q2*SG[0])/2 - (dayVar*q1*q3)/4;
	SQ[5] = (daxVar*q3*SG[0])/2 - (dayVar*q3*SG[0])/2 - (dazVar*q1*q2)/4;
	SQ[6] = (daxVar*q1*q2)/4 - (dazVar*q3*SG[0])/2 - (dayVar*q1*q2)/4;
	SQ[7] = (dazVar*q1*q3)/4 - (daxVar*q1*q3)/4 - (dayVar*q2*SG[0])/2;
	SQ[8] = (dayVar*q2*q3)/4 - (daxVar*q1*SG[0])/2 - (dazVar*q2*q3)/4;
	SQ[9] = sq(SG[0]);
	SQ[10] = sq(q1);

	float SPP[11] = {};
	SPP[0] = SF[12] + SF[13] - 2*q2*SF[2];
	SPP[1] = SF[17] - SF[18] - SF[19] + SF[20];
	SPP[2] = SF[17] - SF[18] + SF[19] - SF[20];
	SPP[3] = SF[17] + SF[18] - SF[19] - SF[20];
	SPP[4] = 2*q0*q2 - 2*q1*q3;
	SPP[5] = 2*q0*q1 - 2*q2*q3;
	SPP[6] = 2*q0*q3 - 2*q1*q2;
	SPP[7] = 2*q0*q1 + 2*q2*q3;
	SPP[8] = 2*q0*q3 + 2*q1*q2;
	SPP[9] = 2*q0*q2 + 2*q1*q3;
	SPP[10] = SF[16];

	// covariance update
	float nextP[24][24];

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	nextP[0][0] = P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10] + (daxVar*SQ[10])/4 + SF[9]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[11]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[10]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SF[14]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) + SF[15]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) + SPP[10]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) + (dayVar*sq(q2))/4 + (dazVar*sq(q3))/4;
	nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10] + SF[8]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[7]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[11]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) - SF[15]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) + SPP[10]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) - (q0*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]))/2;
	nextP[1][1] = P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] + daxVar*SQ[9] - (P[10][1]*q0)/2 + SF[8]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[7]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[11]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) - SF[15]*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2) + SPP[10]*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2) + (dayVar*sq(q3))/4 + (dazVar*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2))/2;
	nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10] + SF[6]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[10]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[8]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SF[14]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) - SPP[10]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) - (q0*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]))/2;
	nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2 + SF[6]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[10]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) + SF[8]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SF[14]*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2) - SPP[10]*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2))/2;
	nextP[2][2] = P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] + dayVar*SQ[9] + (dazVar*SQ[10])/4 - (P[11][2]*q0)/2 + SF[6]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[10]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) + SF[8]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SF[14]*(P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2) - SPP[10]*(P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2) + (daxVar*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2))/2;
	nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10] + SF[7]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[6]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[9]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[15]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) - SF[14]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) - (q0*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]))/2;
	nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2 + SF[7]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[6]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) + SF[9]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[15]*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2) - SF[14]*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2))/2;
	nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2 + SF[7]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[6]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) + SF[9]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[15]*(P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2) - SF[14]*(P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2))/2;
	nextP[3][3] = P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] + (dayVar*SQ[10])/4 + dazVar*SQ[9] - (P[12][3]*q0)/2 + SF[7]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[6]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) + SF[9]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[15]*(P[3][10] + P[0][10]*SF[7] + P[1][10]*SF[6] + P[2][10]*SF[9] + P[10][10]*SF[15] - P[11][10]*SF[14] - (P[12][10]*q0)/2) - SF[14]*(P[3][11] + P[0][11]*SF[7] + P[1][11]*SF[6] + P[2][11]*SF[9] + P[10][11]*SF[15] - P[11][11]*SF[14] - (P[12][11]*q0)/2) + (daxVar*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[7] + P[1][12]*SF[6] + P[2][12]*SF[9] + P[10][12]*SF[15] - P[11][12]*SF[14] - (P[12][12]*q0)/2))/2;
	nextP[0][4] = P[0][4] + P[1][4]*SF[9] + P[2][4]*SF[11] + P[3][4]*SF[10] + P[10][4]*SF[14] + P[11][4]*SF[15] + P[12][4]*SPP[10] + SF[5]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[3]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SF[4]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SPP[0]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SPP[3]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) + SPP[6]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) - SPP[9]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
	nextP[1][4] = P[1][4] + P[0][4]*SF[8] + P[2][4]*SF[7] + P[3][4]*SF[11] - P[12][4]*SF[15] + P[11][4]*SPP[10] - (P[10][4]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[3]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SF[4]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SPP[3]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) + SPP[6]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) - SPP[9]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
	nextP[2][4] = P[2][4] + P[0][4]*SF[6] + P[1][4]*SF[10] + P[3][4]*SF[8] + P[12][4]*SF[14] - P[10][4]*SPP[10] - (P[11][4]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[3]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SF[4]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SPP[3]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) + SPP[6]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) - SPP[9]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
	nextP[3][4] = P[3][4] + P[0][4]*SF[7] + P[1][4]*SF[6] + P[2][4]*SF[9] + P[10][4]*SF[15] - P[11][4]*SF[14] - (P[12][4]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[3]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SF[4]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SPP[3]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) + SPP[6]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) - SPP[9]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
	nextP[4][4] = P[4][4] + P[0][4]*SF[5] + P[1][4]*SF[3] - P[3][4]*SF[4] + P[2][4]*SPP[0] + P[13][4]*SPP[3] + P[14][4]*SPP[6] - P[15][4]*SPP[9] + dvyVar*sq(SG[7] - 2*q0*q3) + dvzVar*sq(SG[6] + 2*q0*q2) + SF[5]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SF[3]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SF[4]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) + SPP[0]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SPP[3]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) + SPP[6]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) - SPP[9]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]) + dvxVar*sq(SG[1] + SG[2] - SG[3] - SG[4]);
	nextP[0][5] = P[0][5] + P[1][5]*SF[9] + P[2][5]*SF[11] + P[3][5]*SF[10] + P[10][5]*SF[14] + P[11][5]*SF[15] + P[12][5]*SPP[10] + SF[4]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[3]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[5]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) - SPP[0]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SPP[8]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) + SPP[2]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) + SPP[5]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
	nextP[1][5] = P[1][5] + P[0][5]*SF[8] + P[2][5]*SF[7] + P[3][5]*SF[11] - P[12][5]*SF[15] + P[11][5]*SPP[10] - (P[10][5]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[3]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[5]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SPP[8]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) + SPP[2]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) + SPP[5]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
	nextP[2][5] = P[2][5] + P[0][5]*SF[6] + P[1][5]*SF[10] + P[3][5]*SF[8] + P[12][5]*SF[14] - P[10][5]*SPP[10] - (P[11][5]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[3]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[5]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SPP[8]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) + SPP[2]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) + SPP[5]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
	nextP[3][5] = P[3][5] + P[0][5]*SF[7] + P[1][5]*SF[6] + P[2][5]*SF[9] + P[10][5]*SF[15] - P[11][5]*SF[14] - (P[12][5]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[3]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[5]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SPP[8]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) + SPP[2]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) + SPP[5]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
	nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[5] + P[1][5]*SF[3] - P[3][5]*SF[4] + P[2][5]*SPP[0] + P[13][5]*SPP[3] + P[14][5]*SPP[6] - P[15][5]*SPP[9] + SF[4]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SF[3]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SF[5]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) - SPP[0]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SPP[8]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) + SPP[2]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) + SPP[5]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]);
	nextP[5][5] = P[5][5] + P[0][5]*SF[4] + P[2][5]*SF[3] + P[3][5]*SF[5] - P[1][5]*SPP[0] - P[13][5]*SPP[8] + P[14][5]*SPP[2] + P[15][5]*SPP[5] + dvxVar*sq(SG[7] + 2*q0*q3) + dvzVar*sq(SG[5] - 2*q0*q1) + SF[4]*(P[5][0] + P[0][0]*SF[4] + P[2][0]*SF[3] + P[3][0]*SF[5] - P[1][0]*SPP[0] - P[13][0]*SPP[8] + P[14][0]*SPP[2] + P[15][0]*SPP[5]) + SF[3]*(P[5][2] + P[0][2]*SF[4] + P[2][2]*SF[3] + P[3][2]*SF[5] - P[1][2]*SPP[0] - P[13][2]*SPP[8] + P[14][2]*SPP[2] + P[15][2]*SPP[5]) + SF[5]*(P[5][3] + P[0][3]*SF[4] + P[2][3]*SF[3] + P[3][3]*SF[5] - P[1][3]*SPP[0] - P[13][3]*SPP[8] + P[14][3]*SPP[2] + P[15][3]*SPP[5]) - SPP[0]*(P[5][1] + P[0][1]*SF[4] + P[2][1]*SF[3] + P[3][1]*SF[5] - P[1][1]*SPP[0] - P[13][1]*SPP[8] + P[14][1]*SPP[2] + P[15][1]*SPP[5]) - SPP[8]*(P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5]) + SPP[2]*(P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5]) + SPP[5]*(P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5]) + dvyVar*sq(SG[1] - SG[2] + SG[3] - SG[4]);
	nextP[0][6] = P[0][6] + P[1][6]*SF[9] + P[2][6]*SF[11] + P[3][6]*SF[10] + P[10][6]*SF[14] + P[11][6]*SF[15] + P[12][6]*SPP[10] + SF[4]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SF[5]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[3]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SPP[0]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SPP[4]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) - SPP[7]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) - SPP[1]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
	nextP[1][6] = P[1][6] + P[0][6]*SF[8] + P[2][6]*SF[7] + P[3][6]*SF[11] - P[12][6]*SF[15] + P[11][6]*SPP[10] - (P[10][6]*q0)/2 + SF[4]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SF[5]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[3]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SPP[4]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) - SPP[7]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) - SPP[1]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
	nextP[2][6] = P[2][6] + P[0][6]*SF[6] + P[1][6]*SF[10] + P[3][6]*SF[8] + P[12][6]*SF[14] - P[10][6]*SPP[10] - (P[11][6]*q0)/2 + SF[4]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SF[5]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[3]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SPP[4]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) - SPP[7]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) - SPP[1]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
	nextP[3][6] = P[3][6] + P[0][6]*SF[7] + P[1][6]*SF[6] + P[2][6]*SF[9] + P[10][6]*SF[15] - P[11][6]*SF[14] - (P[12][6]*q0)/2 + SF[4]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SF[5]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[3]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SPP[4]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) - SPP[7]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) - SPP[1]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
	nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[5] + P[1][6]*SF[3] - P[3][6]*SF[4] + P[2][6]*SPP[0] + P[13][6]*SPP[3] + P[14][6]*SPP[6] - P[15][6]*SPP[9] + SF[4]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SF[5]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SF[3]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) + SPP[0]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SPP[4]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) - SPP[7]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) - SPP[1]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]);
	nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[4] + P[2][6]*SF[3] + P[3][6]*SF[5] - P[1][6]*SPP[0] - P[13][6]*SPP[8] + P[14][6]*SPP[2] + P[15][6]*SPP[5] + SF[4]*(P[5][1] + P[0][1]*SF[4] + P[2][1]*SF[3] + P[3][1]*SF[5] - P[1][1]*SPP[0] - P[13][1]*SPP[8] + P[14][1]*SPP[2] + P[15][1]*SPP[5]) - SF[5]*(P[5][2] + P[0][2]*SF[4] + P[2][2]*SF[3] + P[3][2]*SF[5] - P[1][2]*SPP[0] - P[13][2]*SPP[8] + P[14][2]*SPP[2] + P[15][2]*SPP[5]) + SF[3]*(P[5][3] + P[0][3]*SF[4] + P[2][3]*SF[3] + P[3][3]*SF[5] - P[1][3]*SPP[0] - P[13][3]*SPP[8] + P[14][3]*SPP[2] + P[15][3]*SPP[5]) + SPP[0]*(P[5][0] + P[0][0]*SF[4] + P[2][0]*SF[3] + P[3][0]*SF[5] - P[1][0]*SPP[0] - P[13][0]*SPP[8] + P[14][0]*SPP[2] + P[15][0]*SPP[5]) + SPP[4]*(P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5]) - SPP[7]*(P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5]) - SPP[1]*(P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5]);
	nextP[6][6] = P[6][6] + P[1][6]*SF[4] - P[2][6]*SF[5] + P[3][6]*SF[3] + P[0][6]*SPP[0] + P[13][6]*SPP[4] - P[14][6]*SPP[7] - P[15][6]*SPP[1] + dvxVar*sq(SG[6] - 2*q0*q2) + dvyVar*sq(SG[5] + 2*q0*q1) + SF[4]*(P[6][1] + P[1][1]*SF[4] - P[2][1]*SF[5] + P[3][1]*SF[3] + P[0][1]*SPP[0] + P[13][1]*SPP[4] - P[14][1]*SPP[7] - P[15][1]*SPP[1]) - SF[5]*(P[6][2] + P[1][2]*SF[4] - P[2][2]*SF[5] + P[3][2]*SF[3] + P[0][2]*SPP[0] + P[13][2]*SPP[4] - P[14][2]*SPP[7] - P[15][2]*SPP[1]) + SF[3]*(P[6][3] + P[1][3]*SF[4] - P[2][3]*SF[5] + P[3][3]*SF[3] + P[0][3]*SPP[0] + P[13][3]*SPP[4] - P[14][3]*SPP[7] - P[15][3]*SPP[1]) + SPP[0]*(P[6][0] + P[1][0]*SF[4] - P[2][0]*SF[5] + P[3][0]*SF[3] + P[0][0]*SPP[0] + P[13][0]*SPP[4] - P[14][0]*SPP[7] - P[15][0]*SPP[1]) + SPP[4]*(P[6][13] + P[1][13]*SF[4] - P[2][13]*SF[5] + P[3][13]*SF[3] + P[0][13]*SPP[0] + P[13][13]*SPP[4] - P[14][13]*SPP[7] - P[15][13]*SPP[1]) - SPP[7]*(P[6][14] + P[1][14]*SF[4] - P[2][14]*SF[5] + P[3][14]*SF[3] + P[0][14]*SPP[0] + P[13][14]*SPP[4] - P[14][14]*SPP[7] - P[15][14]*SPP[1]) - SPP[1]*(P[6][15] + P[1][15]*SF[4] - P[2][15]*SF[5] + P[3][15]*SF[3] + P[0][15]*SPP[0] + P[13][15]*SPP[4] - P[14][15]*SPP[7] - P[15][15]*SPP[1]) + dvzVar*sq(SG[1] - SG[2] - SG[3] + SG[4]);
	nextP[0][7] = P[0][7] + P[1][7]*SF[9] + P[2][7]*SF[11] + P[3][7]*SF[10] + P[10][7]*SF[14] + P[11][7]*SF[15] + P[12][7]*SPP[10] + dt*(P[0][4] + P[1][4]*SF[9] + P[2][4]*SF[11] + P[3][4]*SF[10] + P[10][4]*SF[14] + P[11][4]*SF[15] + P[12][4]*SPP[10]);
	nextP[1][7] = P[1][7] + P[0][7]*SF[8] + P[2][7]*SF[7] + P[3][7]*SF[11] - P[12][7]*SF[15] + P[11][7]*SPP[10] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[8] + P[2][4]*SF[7] + P[3][4]*SF[11] - P[12][4]*SF[15] + P[11][4]*SPP[10] - (P[10][4]*q0)/2);
	nextP[2][7] = P[2][7] + P[0][7]*SF[6] + P[1][7]*SF[10] + P[3][7]*SF[8] + P[12][7]*SF[14] - P[10][7]*SPP[10] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[6] + P[1][4]*SF[10] + P[3][4]*SF[8] + P[12][4]*SF[14] - P[10][4]*SPP[10] - (P[11][4]*q0)/2);
	nextP[3][7] = P[3][7] + P[0][7]*SF[7] + P[1][7]*SF[6] + P[2][7]*SF[9] + P[10][7]*SF[15] - P[11][7]*SF[14] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[7] + P[1][4]*SF[6] + P[2][4]*SF[9] + P[10][4]*SF[15] - P[11][4]*SF[14] - (P[12][4]*q0)/2);
	nextP[4][7] = P[4][7] + P[0][7]*SF[5] + P[1][7]*SF[3] - P[3][7]*SF[4] + P[2][7]*SPP[0] + P[13][7]*SPP[3] + P[14][7]*SPP[6] - P[15][7]*SPP[9] + dt*(P[4][4] + P[0][4]*SF[5] + P[1][4]*SF[3] - P[3][4]*SF[4] + P[2][4]*SPP[0] + P[13][4]*SPP[3] + P[14][4]*SPP[6] - P[15][4]*SPP[9]);
	nextP[5][7] = P[5][7] + P[0][7]*SF[4] + P[2][7]*SF[3] + P[3][7]*SF[5] - P[1][7]*SPP[0] - P[13][7]*SPP[8] + P[14][7]*SPP[2] + P[15][7]*SPP[5] + dt*(P[5][4] + P[0][4]*SF[4] + P[2][4]*SF[3] + P[3][4]*SF[5] - P[1][4]*SPP[0] - P[13][4]*SPP[8] + P[14][4]*SPP[2] + P[15][4]*SPP[5]);
	nextP[6][7] = P[6][7] + P[1][7]*SF[4] - P[2][7]*SF[5] + P[3][7]*SF[3] + P[0][7]*SPP[0] + P[13][7]*SPP[4] - P[14][7]*SPP[7] - P[15][7]*SPP[1] + dt*(P[6][4] + P[1][4]*SF[4] - P[2][4]*SF[5] + P[3][4]*SF[3] + P[0][4]*SPP[0] + P[13][4]*SPP[4] - P[14][4]*SPP[7] - P[15][4]*SPP[1]);
	nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
	nextP[0][8] = P[0][8] + P[1][8]*SF[9] + P[2][8]*SF[11] + P[3][8]*SF[10] + P[10][8]*SF[14] + P[11][8]*SF[15] + P[12][8]*SPP[10] + dt*(P[0][5] + P[1][5]*SF[9] + P[2][5]*SF[11] + P[3][5]*SF[10] + P[10][5]*SF[14] + P[11][5]*SF[15] + P[12][5]*SPP[10]);
	nextP[1][8] = P[1][8] + P[0][8]*SF[8] + P[2][8]*SF[7] + P[3][8]*SF[11] - P[12][8]*SF[15] + P[11][8]*SPP[10] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[8] + P[2][5]*SF[7] + P[3][5]*SF[11] - P[12][5]*SF[15] + P[11][5]*SPP[10] - (P[10][5]*q0)/2);
	nextP[2][8] = P[2][8] + P[0][8]*SF[6] + P[1][8]*SF[10] + P[3][8]*SF[8] + P[12][8]*SF[14] - P[10][8]*SPP[10] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[6] + P[1][5]*SF[10] + P[3][5]*SF[8] + P[12][5]*SF[14] - P[10][5]*SPP[10] - (P[11][5]*q0)/2);
	nextP[3][8] = P[3][8] + P[0][8]*SF[7] + P[1][8]*SF[6] + P[2][8]*SF[9] + P[10][8]*SF[15] - P[11][8]*SF[14] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[7] + P[1][5]*SF[6] + P[2][5]*SF[9] + P[10][5]*SF[15] - P[11][5]*SF[14] - (P[12][5]*q0)/2);
	nextP[4][8] = P[4][8] + P[0][8]*SF[5] + P[1][8]*SF[3] - P[3][8]*SF[4] + P[2][8]*SPP[0] + P[13][8]*SPP[3] + P[14][8]*SPP[6] - P[15][8]*SPP[9] + dt*(P[4][5] + P[0][5]*SF[5] + P[1][5]*SF[3] - P[3][5]*SF[4] + P[2][5]*SPP[0] + P[13][5]*SPP[3] + P[14][5]*SPP[6] - P[15][5]*SPP[9]);
	nextP[5][8] = P[5][8] + P[0][8]*SF[4] + P[2][8]*SF[3] + P[3][8]*SF[5] - P[1][8]*SPP[0] - P[13][8]*SPP[8] + P[14][8]*SPP[2] + P[15][8]*SPP[5] + dt*(P[5][5] + P[0][5]*SF[4] + P[2][5]*SF[3] + P[3][5]*SF[5] - P[1][5]*SPP[0] - P[13][5]*SPP[8] + P[14][5]*SPP[2] + P[15][5]*SPP[5]);
	nextP[6][8] = P[6][8] + P[1][8]*SF[4] - P[2][8]*SF[5] + P[3][8]*SF[3] + P[0][8]*SPP[0] + P[13][8]*SPP[4] - P[14][8]*SPP[7] - P[15][8]*SPP[1] + dt*(P[6][5] + P[1][5]*SF[4] - P[2][5]*SF[5] + P[3][5]*SF[3] + P[0][5]*SPP[0] + P[13][5]*SPP[4] - P[14][5]*SPP[7] - P[15][5]*SPP[1]);
	nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
	nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
	nextP[0][9] = P[0][9] + P[1][9]*SF[9] + P[2][9]*SF[11] + P[3][9]*SF[10] + P[10][9]*SF[14] + P[11][9]*SF[15] + P[12][9]*SPP[10] + dt*(P[0][6] + P[1][6]*SF[9] + P[2][6]*SF[11] + P[3][6]*SF[10] + P[10][6]*SF[14] + P[11][6]*SF[15] + P[12][6]*SPP[10]);
	nextP[1][9] = P[1][9] + P[0][9]*SF[8] + P[2][9]*SF[7] + P[3][9]*SF[11] - P[12][9]*SF[15] + P[11][9]*SPP[10] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[8] + P[2][6]*SF[7] + P[3][6]*SF[11] - P[12][6]*SF[15] + P[11][6]*SPP[10] - (P[10][6]*q0)/2);
	nextP[2][9] = P[2][9] + P[0][9]*SF[6] + P[1][9]*SF[10] + P[3][9]*SF[8] + P[12][9]*SF[14] - P[10][9]*SPP[10] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[6] + P[1][6]*SF[10] + P[3][6]*SF[8] + P[12][6]*SF[14] - P[10][6]*SPP[10] - (P[11][6]*q0)/2);
	nextP[3][9] = P[3][9] + P[0][9]*SF[7] + P[1][9]*SF[6] + P[2][9]*SF[9] + P[10][9]*SF[15] - P[11][9]*SF[14] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[7] + P[1][6]*SF[6] + P[2][6]*SF[9] + P[10][6]*SF[15] - P[11][6]*SF[14] - (P[12][6]*q0)/2);
	nextP[4][9] = P[4][9] + P[0][9]*SF[5] + P[1][9]*SF[3] - P[3][9]*SF[4] + P[2][9]*SPP[0] + P[13][9]*SPP[3] + P[14][9]*SPP[6] - P[15][9]*SPP[9] + dt*(P[4][6] + P[0][6]*SF[5] + P[1][6]*SF[3] - P[3][6]*SF[4] + P[2][6]*SPP[0] + P[13][6]*SPP[3] + P[14][6]*SPP[6] - P[15][6]*SPP[9]);
	nextP[5][9] = P[5][9] + P[0][9]*SF[4] + P[2][9]*SF[3] + P[3][9]*SF[5] - P[1][9]*SPP[0] - P[13][9]*SPP[8] + P[14][9]*SPP[2] + P[15][9]*SPP[5] + dt*(P[5][6] + P[0][6]*SF[4] + P[2][6]*SF[3] + P[3][6]*SF[5] - P[1][6]*SPP[0] - P[13][6]*SPP[8] + P[14][6]*SPP[2] + P[15][6]*SPP[5]);
	nextP[6][9] = P[6][9] + P[1][9]*SF[4] - P[2][9]*SF[5] + P[3][9]*SF[3] + P[0][9]*SPP[0] + P[13][9]*SPP[4] - P[14][9]*SPP[7] - P[15][9]*SPP[1] + dt*(P[6][6] + P[1][6]*SF[4] - P[2][6]*SF[5] + P[3][6]*SF[3] + P[0][6]*SPP[0] + P[13][6]*SPP[4] - P[14][6]*SPP[7] - P[15][6]*SPP[1]);
	nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
	nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
	nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
	nextP[0][10] = P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10];
	nextP[1][10] = P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2;
	nextP[2][10] = P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2;
	nextP[3][10] = P[3][10] + P[0][10]*SF[7] + P[1][10]*SF[6] + P[2][10]*SF[9] + P[10][10]*SF[15] - P[11][10]*SF[14] - (P[12][10]*q0)/2;
	nextP[4][10] = P[4][10] + P[0][10]*SF[5] + P[1][10]*SF[3] - P[3][10]*SF[4] + P[2][10]*SPP[0] + P[13][10]*SPP[3] + P[14][10]*SPP[6] - P[15][10]*SPP[9];
	nextP[5][10] = P[5][10] + P[0][10]*SF[4] + P[2][10]*SF[3] + P[3][10]*SF[5] - P[1][10]*SPP[0] - P[13][10]*SPP[8] + P[14][10]*SPP[2] + P[15][10]*SPP[5];
	nextP[6][10] = P[6][10] + P[1][10]*SF[4] - P[2][10]*SF[5] + P[3][10]*SF[3] + P[0][10]*SPP[0] + P[13][10]*SPP[4] - P[14][10]*SPP[7] - P[15][10]*SPP[1];
	nextP[7][10] = P[7][10] + P[4][10]*dt;
	nextP[8][10] = P[8][10] + P[5][10]*dt;
	nextP[9][10] = P[9][10] + P[6][10]*dt;
	nextP[10][10] = P[10][10];
	nextP[0][11] = P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10];
	nextP[1][11] = P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2;
	nextP[2][11] = P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2;
	nextP[3][11] = P[3][11] + P[0][11]*SF[7] + P[1][11]*SF[6] + P[2][11]*SF[9] + P[10][11]*SF[15] - P[11][11]*SF[14] - (P[12][11]*q0)/2;
	nextP[4][11] = P[4][11] + P[0][11]*SF[5] + P[1][11]*SF[3] - P[3][11]*SF[4] + P[2][11]*SPP[0] + P[13][11]*SPP[3] + P[14][11]*SPP[6] - P[15][11]*SPP[9];
	nextP[5][11] = P[5][11] + P[0][11]*SF[4] + P[2][11]*SF[3] + P[3][11]*SF[5] - P[1][11]*SPP[0] - P[13][11]*SPP[8] + P[14][11]*SPP[2] + P[15][11]*SPP[5];
	nextP[6][11] = P[6][11] + P[1][11]*SF[4] - P[2][11]*SF[5] + P[3][11]*SF[3] + P[0][11]*SPP[0] + P[13][11]*SPP[4] - P[14][11]*SPP[7] - P[15][11]*SPP[1];
	nextP[7][11] = P[7][11] + P[4][11]*dt;
	nextP[8][11] = P[8][11] + P[5][11]*dt;
	nextP[9][11] = P[9][11] + P[6][11]*dt;
	nextP[10][11] = P[10][11];
	nextP[11][11] = P[11][11];
	nextP[0][12] = P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10];
	nextP[1][12] = P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2;
	nextP[2][12] = P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2;
	nextP[3][12] = P[3][12] + P[0][12]*SF[7] + P[1][12]*SF[6] + P[2][12]*SF[9] + P[10][12]*SF[15] - P[11][12]*SF[14] - (P[12][12]*q0)/2;
	nextP[4][12] = P[4][12] + P[0][12]*SF[5] + P[1][12]*SF[3] - P[3][12]*SF[4] + P[2][12]*SPP[0] + P[13][12]*SPP[3] + P[14][12]*SPP[6] - P[15][12]*SPP[9];
	nextP[5][12] = P[5][12] + P[0][12]*SF[4] + P[2][12]*SF[3] + P[3][12]*SF[5] - P[1][12]*SPP[0] - P[13][12]*SPP[8] + P[14][12]*SPP[2] + P[15][12]*SPP[5];
	nextP[6][12] = P[6][12] + P[1][12]*SF[4] - P[2][12]*SF[5] + P[3][12]*SF[3] + P[0][12]*SPP[0] + P[13][12]*SPP[4] - P[14][12]*SPP[7] - P[15][12]*SPP[1];
	nextP[7][12] = P[7][12] + P[4][12]*dt;
	nextP[8][12] = P[8][12] + P[5][12]*dt;
	nextP[9][12] = P[9][12] + P[6][12]*dt;
	nextP[10][12] = P[10][12];
	nextP[11][12] = P[11][12];
	nextP[12][12] = P[12][12];

	// add process noise that is not from the IMU
	for (unsigned i = 0; i <= 12; i++) {
		nextP[i][i] += process_noise[i];
	}

	// Don't calculate these covariance terms if IMU delta velocity bias estimation is inhibited
	if (!(_params.fusion_mode & MASK_INHIBIT_ACC_BIAS) && !_accel_bias_inhibit) {

		// calculate variances and upper diagonal covariances for IMU delta velocity bias states
		nextP[0][13] = P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10];
		nextP[1][13] = P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2;
		nextP[2][13] = P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2;
		nextP[3][13] = P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2;
		nextP[4][13] = P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9];
		nextP[5][13] = P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5];
		nextP[6][13] = P[6][13] + P[1][13]*SF[4] - P[2][13]*SF[5] + P[3][13]*SF[3] + P[0][13]*SPP[0] + P[13][13]*SPP[4] - P[14][13]*SPP[7] - P[15][13]*SPP[1];
		nextP[7][13] = P[7][13] + P[4][13]*dt;
		nextP[8][13] = P[8][13] + P[5][13]*dt;
		nextP[9][13] = P[9][13] + P[6][13]*dt;
		nextP[10][13] = P[10][13];
		nextP[11][13] = P[11][13];
		nextP[12][13] = P[12][13];
		nextP[13][13] = P[13][13];
		nextP[0][14] = P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10];
		nextP[1][14] = P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2;
		nextP[2][14] = P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2;
		nextP[3][14] = P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2;
		nextP[4][14] = P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9];
		nextP[5][14] = P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5];
		nextP[6][14] = P[6][14] + P[1][14]*SF[4] - P[2][14]*SF[5] + P[3][14]*SF[3] + P[0][14]*SPP[0] + P[13][14]*SPP[4] - P[14][14]*SPP[7] - P[15][14]*SPP[1];
		nextP[7][14] = P[7][14] + P[4][14]*dt;
		nextP[8][14] = P[8][14] + P[5][14]*dt;
		nextP[9][14] = P[9][14] + P[6][14]*dt;
		nextP[10][14] = P[10][14];
		nextP[11][14] = P[11][14];
		nextP[12][14] = P[12][14];
		nextP[13][14] = P[13][14];
		nextP[14][14] = P[14][14];
		nextP[0][15] = P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10];
		nextP[1][15] = P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2;
		nextP[2][15] = P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2;
		nextP[3][15] = P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2;
		nextP[4][15] = P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9];
		nextP[5][15] = P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5];
		nextP[6][15] = P[6][15] + P[1][15]*SF[4] - P[2][15]*SF[5] + P[3][15]*SF[3] + P[0][15]*SPP[0] + P[13][15]*SPP[4] - P[14][15]*SPP[7] - P[15][15]*SPP[1];
		nextP[7][15] = P[7][15] + P[4][15]*dt;
		nextP[8][15] = P[8][15] + P[5][15]*dt;
		nextP[9][15] = P[9][15] + P[6][15]*dt;
		nextP[10][15] = P[10][15];
		nextP[11][15] = P[11][15];
		nextP[12][15] = P[12][15];
		nextP[13][15] = P[13][15];
		nextP[14][15] = P[14][15];
		nextP[15][15] = P[15][15];

		// add process noise that is not from the IMU
		for (unsigned i = 13; i <= 15; i++) {
			nextP[i][i] += process_noise[i];
		}

	} else {
		// Inhibit delta velocity bias learning by zeroing the covariance terms
		zeroRows(nextP,13,15);
		zeroCols(nextP,13,15);
	}

	// Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
	if (_control_status.flags.mag_3D) {
		// Check if we have just transitioned into 3-axis fusion and set the state variances
		if (!_control_status_prev.flags.mag_3D) {
			for (uint8_t index = 16; index <= 21; index++) {
				P[index][index] = sq(fmaxf(_params.mag_noise, 0.001f));
			}
		}

		// calculate variances and upper diagonal covariances for earth and body magnetic field states
		nextP[0][16] = P[0][16] + P[1][16]*SF[9] + P[2][16]*SF[11] + P[3][16]*SF[10] + P[10][16]*SF[14] + P[11][16]*SF[15] + P[12][16]*SPP[10];
		nextP[1][16] = P[1][16] + P[0][16]*SF[8] + P[2][16]*SF[7] + P[3][16]*SF[11] - P[12][16]*SF[15] + P[11][16]*SPP[10] - (P[10][16]*q0)/2;
		nextP[2][16] = P[2][16] + P[0][16]*SF[6] + P[1][16]*SF[10] + P[3][16]*SF[8] + P[12][16]*SF[14] - P[10][16]*SPP[10] - (P[11][16]*q0)/2;
		nextP[3][16] = P[3][16] + P[0][16]*SF[7] + P[1][16]*SF[6] + P[2][16]*SF[9] + P[10][16]*SF[15] - P[11][16]*SF[14] - (P[12][16]*q0)/2;
		nextP[4][16] = P[4][16] + P[0][16]*SF[5] + P[1][16]*SF[3] - P[3][16]*SF[4] + P[2][16]*SPP[0] + P[13][16]*SPP[3] + P[14][16]*SPP[6] - P[15][16]*SPP[9];
		nextP[5][16] = P[5][16] + P[0][16]*SF[4] + P[2][16]*SF[3] + P[3][16]*SF[5] - P[1][16]*SPP[0] - P[13][16]*SPP[8] + P[14][16]*SPP[2] + P[15][16]*SPP[5];
		nextP[6][16] = P[6][16] + P[1][16]*SF[4] - P[2][16]*SF[5] + P[3][16]*SF[3] + P[0][16]*SPP[0] + P[13][16]*SPP[4] - P[14][16]*SPP[7] - P[15][16]*SPP[1];
		nextP[7][16] = P[7][16] + P[4][16]*dt;
		nextP[8][16] = P[8][16] + P[5][16]*dt;
		nextP[9][16] = P[9][16] + P[6][16]*dt;
		nextP[10][16] = P[10][16];
		nextP[11][16] = P[11][16];
		nextP[12][16] = P[12][16];
		nextP[13][16] = P[13][16];
		nextP[14][16] = P[14][16];
		nextP[15][16] = P[15][16];
		nextP[16][16] = P[16][16];
		nextP[0][17] = P[0][17] + P[1][17]*SF[9] + P[2][17]*SF[11] + P[3][17]*SF[10] + P[10][17]*SF[14] + P[11][17]*SF[15] + P[12][17]*SPP[10];
		nextP[1][17] = P[1][17] + P[0][17]*SF[8] + P[2][17]*SF[7] + P[3][17]*SF[11] - P[12][17]*SF[15] + P[11][17]*SPP[10] - (P[10][17]*q0)/2;
		nextP[2][17] = P[2][17] + P[0][17]*SF[6] + P[1][17]*SF[10] + P[3][17]*SF[8] + P[12][17]*SF[14] - P[10][17]*SPP[10] - (P[11][17]*q0)/2;
		nextP[3][17] = P[3][17] + P[0][17]*SF[7] + P[1][17]*SF[6] + P[2][17]*SF[9] + P[10][17]*SF[15] - P[11][17]*SF[14] - (P[12][17]*q0)/2;
		nextP[4][17] = P[4][17] + P[0][17]*SF[5] + P[1][17]*SF[3] - P[3][17]*SF[4] + P[2][17]*SPP[0] + P[13][17]*SPP[3] + P[14][17]*SPP[6] - P[15][17]*SPP[9];
		nextP[5][17] = P[5][17] + P[0][17]*SF[4] + P[2][17]*SF[3] + P[3][17]*SF[5] - P[1][17]*SPP[0] - P[13][17]*SPP[8] + P[14][17]*SPP[2] + P[15][17]*SPP[5];
		nextP[6][17] = P[6][17] + P[1][17]*SF[4] - P[2][17]*SF[5] + P[3][17]*SF[3] + P[0][17]*SPP[0] + P[13][17]*SPP[4] - P[14][17]*SPP[7] - P[15][17]*SPP[1];
		nextP[7][17] = P[7][17] + P[4][17]*dt;
		nextP[8][17] = P[8][17] + P[5][17]*dt;
		nextP[9][17] = P[9][17] + P[6][17]*dt;
		nextP[10][17] = P[10][17];
		nextP[11][17] = P[11][17];
		nextP[12][17] = P[12][17];
		nextP[13][17] = P[13][17];
		nextP[14][17] = P[14][17];
		nextP[15][17] = P[15][17];
		nextP[16][17] = P[16][17];
		nextP[17][17] = P[17][17];
		nextP[0][18] = P[0][18] + P[1][18]*SF[9] + P[2][18]*SF[11] + P[3][18]*SF[10] + P[10][18]*SF[14] + P[11][18]*SF[15] + P[12][18]*SPP[10];
		nextP[1][18] = P[1][18] + P[0][18]*SF[8] + P[2][18]*SF[7] + P[3][18]*SF[11] - P[12][18]*SF[15] + P[11][18]*SPP[10] - (P[10][18]*q0)/2;
		nextP[2][18] = P[2][18] + P[0][18]*SF[6] + P[1][18]*SF[10] + P[3][18]*SF[8] + P[12][18]*SF[14] - P[10][18]*SPP[10] - (P[11][18]*q0)/2;
		nextP[3][18] = P[3][18] + P[0][18]*SF[7] + P[1][18]*SF[6] + P[2][18]*SF[9] + P[10][18]*SF[15] - P[11][18]*SF[14] - (P[12][18]*q0)/2;
		nextP[4][18] = P[4][18] + P[0][18]*SF[5] + P[1][18]*SF[3] - P[3][18]*SF[4] + P[2][18]*SPP[0] + P[13][18]*SPP[3] + P[14][18]*SPP[6] - P[15][18]*SPP[9];
		nextP[5][18] = P[5][18] + P[0][18]*SF[4] + P[2][18]*SF[3] + P[3][18]*SF[5] - P[1][18]*SPP[0] - P[13][18]*SPP[8] + P[14][18]*SPP[2] + P[15][18]*SPP[5];
		nextP[6][18] = P[6][18] + P[1][18]*SF[4] - P[2][18]*SF[5] + P[3][18]*SF[3] + P[0][18]*SPP[0] + P[13][18]*SPP[4] - P[14][18]*SPP[7] - P[15][18]*SPP[1];
		nextP[7][18] = P[7][18] + P[4][18]*dt;
		nextP[8][18] = P[8][18] + P[5][18]*dt;
		nextP[9][18] = P[9][18] + P[6][18]*dt;
		nextP[10][18] = P[10][18];
		nextP[11][18] = P[11][18];
		nextP[12][18] = P[12][18];
		nextP[13][18] = P[13][18];
		nextP[14][18] = P[14][18];
		nextP[15][18] = P[15][18];
		nextP[16][18] = P[16][18];
		nextP[17][18] = P[17][18];
		nextP[18][18] = P[18][18];
		nextP[0][19] = P[0][19] + P[1][19]*SF[9] + P[2][19]*SF[11] + P[3][19]*SF[10] + P[10][19]*SF[14] + P[11][19]*SF[15] + P[12][19]*SPP[10];
		nextP[1][19] = P[1][19] + P[0][19]*SF[8] + P[2][19]*SF[7] + P[3][19]*SF[11] - P[12][19]*SF[15] + P[11][19]*SPP[10] - (P[10][19]*q0)/2;
		nextP[2][19] = P[2][19] + P[0][19]*SF[6] + P[1][19]*SF[10] + P[3][19]*SF[8] + P[12][19]*SF[14] - P[10][19]*SPP[10] - (P[11][19]*q0)/2;
		nextP[3][19] = P[3][19] + P[0][19]*SF[7] + P[1][19]*SF[6] + P[2][19]*SF[9] + P[10][19]*SF[15] - P[11][19]*SF[14] - (P[12][19]*q0)/2;
		nextP[4][19] = P[4][19] + P[0][19]*SF[5] + P[1][19]*SF[3] - P[3][19]*SF[4] + P[2][19]*SPP[0] + P[13][19]*SPP[3] + P[14][19]*SPP[6] - P[15][19]*SPP[9];
		nextP[5][19] = P[5][19] + P[0][19]*SF[4] + P[2][19]*SF[3] + P[3][19]*SF[5] - P[1][19]*SPP[0] - P[13][19]*SPP[8] + P[14][19]*SPP[2] + P[15][19]*SPP[5];
		nextP[6][19] = P[6][19] + P[1][19]*SF[4] - P[2][19]*SF[5] + P[3][19]*SF[3] + P[0][19]*SPP[0] + P[13][19]*SPP[4] - P[14][19]*SPP[7] - P[15][19]*SPP[1];
		nextP[7][19] = P[7][19] + P[4][19]*dt;
		nextP[8][19] = P[8][19] + P[5][19]*dt;
		nextP[9][19] = P[9][19] + P[6][19]*dt;
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
		nextP[0][20] = P[0][20] + P[1][20]*SF[9] + P[2][20]*SF[11] + P[3][20]*SF[10] + P[10][20]*SF[14] + P[11][20]*SF[15] + P[12][20]*SPP[10];
		nextP[1][20] = P[1][20] + P[0][20]*SF[8] + P[2][20]*SF[7] + P[3][20]*SF[11] - P[12][20]*SF[15] + P[11][20]*SPP[10] - (P[10][20]*q0)/2;
		nextP[2][20] = P[2][20] + P[0][20]*SF[6] + P[1][20]*SF[10] + P[3][20]*SF[8] + P[12][20]*SF[14] - P[10][20]*SPP[10] - (P[11][20]*q0)/2;
		nextP[3][20] = P[3][20] + P[0][20]*SF[7] + P[1][20]*SF[6] + P[2][20]*SF[9] + P[10][20]*SF[15] - P[11][20]*SF[14] - (P[12][20]*q0)/2;
		nextP[4][20] = P[4][20] + P[0][20]*SF[5] + P[1][20]*SF[3] - P[3][20]*SF[4] + P[2][20]*SPP[0] + P[13][20]*SPP[3] + P[14][20]*SPP[6] - P[15][20]*SPP[9];
		nextP[5][20] = P[5][20] + P[0][20]*SF[4] + P[2][20]*SF[3] + P[3][20]*SF[5] - P[1][20]*SPP[0] - P[13][20]*SPP[8] + P[14][20]*SPP[2] + P[15][20]*SPP[5];
		nextP[6][20] = P[6][20] + P[1][20]*SF[4] - P[2][20]*SF[5] + P[3][20]*SF[3] + P[0][20]*SPP[0] + P[13][20]*SPP[4] - P[14][20]*SPP[7] - P[15][20]*SPP[1];
		nextP[7][20] = P[7][20] + P[4][20]*dt;
		nextP[8][20] = P[8][20] + P[5][20]*dt;
		nextP[9][20] = P[9][20] + P[6][20]*dt;
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
		nextP[0][21] = P[0][21] + P[1][21]*SF[9] + P[2][21]*SF[11] + P[3][21]*SF[10] + P[10][21]*SF[14] + P[11][21]*SF[15] + P[12][21]*SPP[10];
		nextP[1][21] = P[1][21] + P[0][21]*SF[8] + P[2][21]*SF[7] + P[3][21]*SF[11] - P[12][21]*SF[15] + P[11][21]*SPP[10] - (P[10][21]*q0)/2;
		nextP[2][21] = P[2][21] + P[0][21]*SF[6] + P[1][21]*SF[10] + P[3][21]*SF[8] + P[12][21]*SF[14] - P[10][21]*SPP[10] - (P[11][21]*q0)/2;
		nextP[3][21] = P[3][21] + P[0][21]*SF[7] + P[1][21]*SF[6] + P[2][21]*SF[9] + P[10][21]*SF[15] - P[11][21]*SF[14] - (P[12][21]*q0)/2;
		nextP[4][21] = P[4][21] + P[0][21]*SF[5] + P[1][21]*SF[3] - P[3][21]*SF[4] + P[2][21]*SPP[0] + P[13][21]*SPP[3] + P[14][21]*SPP[6] - P[15][21]*SPP[9];
		nextP[5][21] = P[5][21] + P[0][21]*SF[4] + P[2][21]*SF[3] + P[3][21]*SF[5] - P[1][21]*SPP[0] - P[13][21]*SPP[8] + P[14][21]*SPP[2] + P[15][21]*SPP[5];
		nextP[6][21] = P[6][21] + P[1][21]*SF[4] - P[2][21]*SF[5] + P[3][21]*SF[3] + P[0][21]*SPP[0] + P[13][21]*SPP[4] - P[14][21]*SPP[7] - P[15][21]*SPP[1];
		nextP[7][21] = P[7][21] + P[4][21]*dt;
		nextP[8][21] = P[8][21] + P[5][21]*dt;
		nextP[9][21] = P[9][21] + P[6][21]*dt;
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

		// add process noise that is not from the IMU
		for (unsigned i = 16; i <= 21; i++) {
			nextP[i][i] += process_noise[i];
		}

	}

	// Don't do covariance prediction on wind states unless we are using them
	if (_control_status.flags.wind) {

		// calculate variances and upper diagonal covariances for wind states
		nextP[0][22] = P[0][22] + P[1][22]*SF[9] + P[2][22]*SF[11] + P[3][22]*SF[10] + P[10][22]*SF[14] + P[11][22]*SF[15] + P[12][22]*SPP[10];
		nextP[1][22] = P[1][22] + P[0][22]*SF[8] + P[2][22]*SF[7] + P[3][22]*SF[11] - P[12][22]*SF[15] + P[11][22]*SPP[10] - (P[10][22]*q0)/2;
		nextP[2][22] = P[2][22] + P[0][22]*SF[6] + P[1][22]*SF[10] + P[3][22]*SF[8] + P[12][22]*SF[14] - P[10][22]*SPP[10] - (P[11][22]*q0)/2;
		nextP[3][22] = P[3][22] + P[0][22]*SF[7] + P[1][22]*SF[6] + P[2][22]*SF[9] + P[10][22]*SF[15] - P[11][22]*SF[14] - (P[12][22]*q0)/2;
		nextP[4][22] = P[4][22] + P[0][22]*SF[5] + P[1][22]*SF[3] - P[3][22]*SF[4] + P[2][22]*SPP[0] + P[13][22]*SPP[3] + P[14][22]*SPP[6] - P[15][22]*SPP[9];
		nextP[5][22] = P[5][22] + P[0][22]*SF[4] + P[2][22]*SF[3] + P[3][22]*SF[5] - P[1][22]*SPP[0] - P[13][22]*SPP[8] + P[14][22]*SPP[2] + P[15][22]*SPP[5];
		nextP[6][22] = P[6][22] + P[1][22]*SF[4] - P[2][22]*SF[5] + P[3][22]*SF[3] + P[0][22]*SPP[0] + P[13][22]*SPP[4] - P[14][22]*SPP[7] - P[15][22]*SPP[1];
		nextP[7][22] = P[7][22] + P[4][22]*dt;
		nextP[8][22] = P[8][22] + P[5][22]*dt;
		nextP[9][22] = P[9][22] + P[6][22]*dt;
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
		nextP[0][23] = P[0][23] + P[1][23]*SF[9] + P[2][23]*SF[11] + P[3][23]*SF[10] + P[10][23]*SF[14] + P[11][23]*SF[15] + P[12][23]*SPP[10];
		nextP[1][23] = P[1][23] + P[0][23]*SF[8] + P[2][23]*SF[7] + P[3][23]*SF[11] - P[12][23]*SF[15] + P[11][23]*SPP[10] - (P[10][23]*q0)/2;
		nextP[2][23] = P[2][23] + P[0][23]*SF[6] + P[1][23]*SF[10] + P[3][23]*SF[8] + P[12][23]*SF[14] - P[10][23]*SPP[10] - (P[11][23]*q0)/2;
		nextP[3][23] = P[3][23] + P[0][23]*SF[7] + P[1][23]*SF[6] + P[2][23]*SF[9] + P[10][23]*SF[15] - P[11][23]*SF[14] - (P[12][23]*q0)/2;
		nextP[4][23] = P[4][23] + P[0][23]*SF[5] + P[1][23]*SF[3] - P[3][23]*SF[4] + P[2][23]*SPP[0] + P[13][23]*SPP[3] + P[14][23]*SPP[6] - P[15][23]*SPP[9];
		nextP[5][23] = P[5][23] + P[0][23]*SF[4] + P[2][23]*SF[3] + P[3][23]*SF[5] - P[1][23]*SPP[0] - P[13][23]*SPP[8] + P[14][23]*SPP[2] + P[15][23]*SPP[5];
		nextP[6][23] = P[6][23] + P[1][23]*SF[4] - P[2][23]*SF[5] + P[3][23]*SF[3] + P[0][23]*SPP[0] + P[13][23]*SPP[4] - P[14][23]*SPP[7] - P[15][23]*SPP[1];
		nextP[7][23] = P[7][23] + P[4][23]*dt;
		nextP[8][23] = P[8][23] + P[5][23]*dt;
		nextP[9][23] = P[9][23] + P[6][23]*dt;
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

		// add process noise that is not from the IMU
		for (unsigned i = 22; i <= 23; i++) {
			nextP[i][i] += process_noise[i];
		}

	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P[7][7] + P[8][8]) > 1e4f) {
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP[i][j] = P[i][j];
				nextP[j][i] = P[j][i];
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P[row][column] = P[column][row] = nextP[column][row];
		}
	}

	// copy variances (diagonals)
	for (unsigned i = 0; i < _k_num_states; i++) {
		P[i][i] = nextP[i][i];
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors();

}

void Ekf::fixCovarianceErrors()
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)
	float P_lim[8] = {};
	P_lim[0] = 1.0f;		// quaternion max var
	P_lim[1] = 1e6f;		// velocity max var
	P_lim[2] = 1e6f;		// positiion max var
	P_lim[3] = 1.0f;		// gyro bias max var
	P_lim[4] = 1.0f;		// delta velocity z bias max var
	P_lim[5] = 1.0f;		// earth mag field max var
	P_lim[6] = 1.0f;		// body mag field max var
	P_lim[7] = 1e6f;		// wind max var

	for (int i = 0; i <= 3; i++) {
		// quaternion states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[0]);
	}

	for (int i = 4; i <= 6; i++) {
		// NED velocity states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[1]);
	}

	for (int i = 7; i <= 9; i++) {
		// NED position states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[2]);
	}

	for (int i = 10; i <= 12; i++) {
		// gyro bias states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[3]);
	}

	// force symmetry on the quaternion, velocity and positon state covariances
	makeSymmetrical(P,0,12);

	// the following states are optional and are deactivaed when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if ((_params.fusion_mode & MASK_INHIBIT_ACC_BIAS) || _accel_bias_inhibit) {
		zeroRows(P,13,15);
		zeroCols(P,13,15);
	} else {
		// constrain variances
		for (int i = 13; i <= 15; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[4]);
		}

		// calculate accel bias term aligned with the gravity vector
		float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		float down_dvel_bias = 0.0f;
		for (uint8_t axis_index=0; axis_index < 3; axis_index++) {
			down_dvel_bias += _state.accel_bias(axis_index) * _R_to_earth(2,axis_index);
		}

		// check that the vertical componenent of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = (fabsf(down_dvel_bias) > dVel_bias_lim
				     && down_dvel_bias * _vel_pos_innov[2] < 0.0f
				     && down_dvel_bias * _vel_pos_innov[5] < 0.0f);

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_last_imu;
		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (_time_last_imu - _time_acc_bias_check > 7E6) {
			float varX = P[13][13];
			float varY = P[14][14];
			float varZ = P[15][15];
			zeroRows(P,13,15);
			zeroCols(P,13,15);
			P[13][13] = varX;
			P[14][14] = varY;
			P[15][15] = varZ;
			_time_acc_bias_check = _time_last_imu;
			_fault_status.flags.bad_acc_bias = false;
			ECL_WARN("EKF invalid accel bias - resetting covariance");
		} else {
			// ensure the covariance values are symmetrical
			makeSymmetrical(P,13,15);
		}

	}

	// magnetic field states
	if (!_control_status.flags.mag_3D) {
		zeroRows(P,16,21);
		zeroCols(P,16,21);
	} else {
		// constrain variances
		for (int i = 16; i <= 18; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[5]);
		}
		for (int i = 19; i <= 21; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[6]);
		}
		// force symmetry
		makeSymmetrical(P,16,21);
	}

	// wind velocity states
	if (!_control_status.flags.wind) {
		zeroRows(P,22,23);
		zeroCols(P,22,23);
	} else {
		// constrain variances
		for (int i = 22; i <= 23; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[7]);
		}
		// force symmetry
		makeSymmetrical(P,22,23);
	}
}

void Ekf::resetMagCovariance()
{	
	// set the quaternion covariance terms to zero
	zeroRows(P,0,3);
	zeroCols(P,0,3);

	// set the magnetic field covariance terms to zero
	zeroRows(P,16,21);
	zeroCols(P,16,21);

	// set the field state variance to the observation variance
	for (uint8_t rc_index=16; rc_index <= 21; rc_index ++) {
		P[rc_index][rc_index] = sq(_params.mag_noise);
	}
}

void Ekf::resetWindCovariance()
{
	// set the wind  covariance terms to zero
	zeroRows(P,22,23);
	zeroCols(P,22,23);

	if (_tas_data_ready && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us < 5e5)) {
		// Use airspeed and zer sideslip assumption to set initial covariance values for wind states

		// calculate the wind speed and bearing
		float spd = sqrtf(sq(_state.wind_vel(0))+sq(_state.wind_vel(1)));
		float yaw = atan2f(_state.wind_vel(1),_state.wind_vel(0));

		// calculate the uncertainty in wind speed and direction using the uncertainty in airspeed and sideslip angle
		// used to calculate the initial wind speed
		float R_spd = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));
		float R_yaw = sq(0.1745f);

		// calculate the variance and covariance terms for the wind states
		float cos_yaw = cosf(yaw);
		float sin_yaw = sinf(yaw);
		float cos_yaw_2 = sq(cos_yaw);
		float sin_yaw_2 = sq(sin_yaw);
		float sin_cos_yaw = sin_yaw*cos_yaw;
		float spd_2 = sq(spd);
		P[22][22] = R_yaw*spd_2*sin_yaw_2 + R_spd*cos_yaw_2;
		P[22][23] = - R_yaw*sin_cos_yaw*spd_2 + R_spd*sin_cos_yaw;
		P[23][22] = P[22][23];
		P[23][23] = R_yaw*spd_2*cos_yaw_2 + R_spd*sin_yaw_2;

		// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
		P[22][22] += P[4][4];
		P[23][23] += P[5][5];

	} else {
		// without airspeed, start with a small initial uncertainty to improve the initial estimate
		P[22][22] = sq(5.0f);
		P[23][23] = sq(5.0f);

	}

}
