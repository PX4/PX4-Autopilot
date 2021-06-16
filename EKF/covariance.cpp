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
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include "utils.hpp"

#include <ecl.h>
#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	P.zero();

	_delta_angle_bias_var_accum.setZero();
	_delta_vel_bias_var_accum.setZero();

	const float dt = FILTER_UPDATE_PERIOD_S;

	resetQuatCov();

	// velocity
	P(4,4) = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P(5,5) = P(4,4);
	P(6,6) = sq(1.5f) * P(4,4);

	// position
	P(7,7) = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P(8,8) = P(7,7);

	if (_control_status.flags.rng_hgt) {
		P(9,9) = sq(fmaxf(_params.range_noise, 0.01f));

	} else if (_control_status.flags.gps_hgt) {
		P(9,9) = getGpsHeightVariance();

	} else {
		P(9,9) = sq(fmaxf(_params.baro_noise, 0.01f));
	}

	// gyro bias
	P(10,10) = sq(_params.switch_on_gyro_bias * dt);
	P(11,11) = P(10,10);
	P(12,12) = P(10,10);

	// accel bias
	_prev_dvel_bias_var(0) = P(13,13) = sq(_params.switch_on_accel_bias * dt);
	_prev_dvel_bias_var(1) = P(14,14) = P(13,13);
	_prev_dvel_bias_var(2) = P(15,15) = P(13,13);

	resetMagCov();

	// wind
	P(22,22) = sq(_params.initial_wind_uncertainty);
	P(23,23) = P(22,22);

}

void Ekf::predictCovariance()
{
	// assign intermediate state variables
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	const float &dax = _imu_sample_delayed.delta_ang(0);
	const float &day = _imu_sample_delayed.delta_ang(1);
	const float &daz = _imu_sample_delayed.delta_ang(2);

	const float &dvx = _imu_sample_delayed.delta_vel(0);
	const float &dvy = _imu_sample_delayed.delta_vel(1);
	const float &dvz = _imu_sample_delayed.delta_vel(2);

	const float &dax_b = _state.delta_ang_bias(0);
	const float &day_b = _state.delta_ang_bias(1);
	const float &daz_b = _state.delta_ang_bias(2);

	const float &dvx_b = _state.delta_vel_bias(0);
	const float &dvy_b = _state.delta_vel_bias(1);
	const float &dvz_b = _state.delta_vel_bias(2);

	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = FILTER_UPDATE_PERIOD_S;
	const float dt_inv = 1.0f / dt;

	// convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	const float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

	// convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	const float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	const float beta = 1.0f - alpha;
	_ang_rate_magnitude_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_ang.norm(), beta * _ang_rate_magnitude_filt);
	_accel_magnitude_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_vel.norm(), beta * _accel_magnitude_filt);
	_accel_vec_filt = alpha * dt_inv * _imu_sample_delayed.delta_vel + beta * _accel_vec_filt;

	const bool is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim
					    || _accel_magnitude_filt > _params.acc_bias_learn_acc_lim;

	const bool do_inhibit_all_axes = (_params.fusion_mode & MASK_INHIBIT_ACC_BIAS)
					 || is_manoeuvre_level_high
					 || _fault_status.flags.bad_acc_vertical;

	for (unsigned stateIndex = 13; stateIndex <= 15; stateIndex++) {
		const unsigned index = stateIndex - 13;

		// When on ground, only consider an accel bias observable if aligned with the gravity vector
		const bool is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.8f) || _control_status.flags.in_air;
		const bool do_inhibit_axis = do_inhibit_all_axes || !is_bias_observable || _imu_sample_delayed.delta_vel_clipping[index];

		if (do_inhibit_axis) {
			// store the bias state variances to be reinstated later
			if (!_accel_bias_inhibit[index]) {
				_prev_dvel_bias_var(index) = P(stateIndex, stateIndex);
				_accel_bias_inhibit[index] = true;
			}

		} else {
			if (_accel_bias_inhibit[index]) {
				// reinstate the bias state variances
				P(stateIndex, stateIndex) = _prev_dvel_bias_var(index);
				_accel_bias_inhibit[index] = false;
			}
		}
	}

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_I_sig;

	if (_control_status.flags.mag_3D && (P(16,16) + P(17,17) + P(18,18)) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;

	if (_control_status.flags.mag_3D && (P(19,19) + P(20,20) + P(21,21)) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1.0f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_sig;

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && (P(22,22) + P(23,23)) < sq(_params.initial_wind_uncertainty)) {
		wind_vel_sig = dt * math::constrain(_params.wind_vel_p_noise, 0.0f, 1.0f) * (1.0f + _params.wind_vel_p_noise_scaler * fabsf(_height_rate_lpf));

	} else {
		wind_vel_sig = 0.0f;
	}

	// compute noise variance for stationary processes
	Vector24f process_noise;

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// delta angle bias states
	process_noise.slice<3,1>(10,0) = sq(d_ang_bias_sig);
	// delta_velocity bias states
	process_noise.slice<3,1>(13,0) = sq(d_vel_bias_sig);
	// earth frame magnetic field states
	process_noise.slice<3,1>(16,0) = sq(mag_I_sig);
	// body frame magnetic field states
	process_noise.slice<3,1>(19,0) = sq(mag_B_sig);
	// wind velocity states
	process_noise.slice<2,1>(22,0) = sq(wind_vel_sig);

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	const float daxVar = sq(dt * gyro_noise);
	const float dayVar = daxVar;
	const float dazVar = daxVar;

	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	if (_fault_status.flags.bad_acc_vertical) {
		// Increase accelerometer process noise if bad accel data is detected. Measurement errors due to
		// vibration induced clipping commonly reach an equivalent 0.5g offset.
		accel_noise = BADACC_BIAS_PNOISE;
	}

	float dvxVar, dvyVar, dvzVar;
	dvxVar = dvyVar = dvzVar = sq(dt * accel_noise);

	// Accelerometer Clipping
	_fault_status.flags.bad_acc_clipping = false; // reset flag

	// delta velocity X: increase process noise if sample contained any X axis clipping
	if (_imu_sample_delayed.delta_vel_clipping[0]) {
		dvxVar = sq(dt * BADACC_BIAS_PNOISE);
		_fault_status.flags.bad_acc_clipping = true;
	}
	// delta velocity Y: increase process noise if sample contained any Y axis clipping
	if (_imu_sample_delayed.delta_vel_clipping[1]) {
		dvyVar = sq(dt * BADACC_BIAS_PNOISE);
		_fault_status.flags.bad_acc_clipping = true;
	}
	// delta velocity Z: increase process noise if sample contained any Z axis clipping
	if (_imu_sample_delayed.delta_vel_clipping[2]) {
		dvzVar = sq(dt * BADACC_BIAS_PNOISE);
		_fault_status.flags.bad_acc_clipping = true;
	}

	// predict the covariance
	// equations generated using EKF/python/ekf_derivation/main.py

	// intermediate calculations
	const float PS0 = ecl::powf(q1, 2);
	const float PS1 = 0.25F*daxVar;
	const float PS2 = ecl::powf(q2, 2);
	const float PS3 = 0.25F*dayVar;
	const float PS4 = ecl::powf(q3, 2);
	const float PS5 = 0.25F*dazVar;
	const float PS6 = 0.5F*q1;
	const float PS7 = 0.5F*q2;
	const float PS8 = P(10,11)*PS7;
	const float PS9 = 0.5F*q3;
	const float PS10 = P(10,12)*PS9;
	const float PS11 = 0.5F*dax - 0.5F*dax_b;
	const float PS12 = 0.5F*day - 0.5F*day_b;
	const float PS13 = 0.5F*daz - 0.5F*daz_b;
	const float PS14 = P(0,10) - P(1,10)*PS11 + P(10,10)*PS6 - P(2,10)*PS12 - P(3,10)*PS13 + PS10 + PS8;
	const float PS15 = P(10,11)*PS6;
	const float PS16 = P(11,12)*PS9;
	const float PS17 = P(0,11) - P(1,11)*PS11 + P(11,11)*PS7 - P(2,11)*PS12 - P(3,11)*PS13 + PS15 + PS16;
	const float PS18 = P(10,12)*PS6;
	const float PS19 = P(11,12)*PS7;
	const float PS20 = P(0,12) - P(1,12)*PS11 + P(12,12)*PS9 - P(2,12)*PS12 - P(3,12)*PS13 + PS18 + PS19;
	const float PS21 = P(1,2)*PS12;
	const float PS22 = -P(1,3)*PS13;
	const float PS23 = P(0,1) - P(1,1)*PS11 + P(1,10)*PS6 + P(1,11)*PS7 + P(1,12)*PS9 - PS21 + PS22;
	const float PS24 = -P(1,2)*PS11;
	const float PS25 = P(2,3)*PS13;
	const float PS26 = P(0,2) + P(2,10)*PS6 + P(2,11)*PS7 + P(2,12)*PS9 - P(2,2)*PS12 + PS24 - PS25;
	const float PS27 = P(1,3)*PS11;
	const float PS28 = -P(2,3)*PS12;
	const float PS29 = P(0,3) + P(3,10)*PS6 + P(3,11)*PS7 + P(3,12)*PS9 - P(3,3)*PS13 - PS27 + PS28;
	const float PS30 = P(0,1)*PS11;
	const float PS31 = P(0,2)*PS12;
	const float PS32 = P(0,3)*PS13;
	const float PS33 = P(0,0) + P(0,10)*PS6 + P(0,11)*PS7 + P(0,12)*PS9 - PS30 - PS31 - PS32;
	const float PS34 = 0.5F*q0;
	const float PS35 = q2*q3;
	const float PS36 = q0*q1;
	const float PS37 = q1*q3;
	const float PS38 = q0*q2;
	const float PS39 = q1*q2;
	const float PS40 = q0*q3;
	const float PS41 = -PS2;
	const float PS42 = ecl::powf(q0, 2);
	const float PS43 = -PS4 + PS42;
	const float PS44 = PS0 + PS41 + PS43;
	const float PS45 = P(0,13) - P(1,13)*PS11 + P(10,13)*PS6 + P(11,13)*PS7 + P(12,13)*PS9 - P(2,13)*PS12 - P(3,13)*PS13;
	const float PS46 = PS37 + PS38;
	const float PS47 = P(0,15) - P(1,15)*PS11 + P(10,15)*PS6 + P(11,15)*PS7 + P(12,15)*PS9 - P(2,15)*PS12 - P(3,15)*PS13;
	const float PS48 = 2*PS47;
	const float PS49 = dvy - dvy_b;
	const float PS50 = dvx - dvx_b;
	const float PS51 = dvz - dvz_b;
	const float PS52 = PS49*q0 + PS50*q3 - PS51*q1;
	const float PS53 = 2*PS29;
	const float PS54 = -PS39 + PS40;
	const float PS55 = P(0,14) - P(1,14)*PS11 + P(10,14)*PS6 + P(11,14)*PS7 + P(12,14)*PS9 - P(2,14)*PS12 - P(3,14)*PS13;
	const float PS56 = 2*PS55;
	const float PS57 = -PS49*q3 + PS50*q0 + PS51*q2;
	const float PS58 = 2*PS33;
	const float PS59 = PS49*q1 - PS50*q2 + PS51*q0;
	const float PS60 = 2*PS59;
	const float PS61 = PS49*q2 + PS50*q1 + PS51*q3;
	const float PS62 = 2*PS61;
	const float PS63 = P(0,4) - P(1,4)*PS11 - P(2,4)*PS12 - P(3,4)*PS13 + P(4,10)*PS6 + P(4,11)*PS7 + P(4,12)*PS9;
	const float PS64 = -PS0;
	const float PS65 = PS2 + PS43 + PS64;
	const float PS66 = PS39 + PS40;
	const float PS67 = 2*PS45;
	const float PS68 = -PS35 + PS36;
	const float PS69 = P(0,5) - P(1,5)*PS11 - P(2,5)*PS12 - P(3,5)*PS13 + P(5,10)*PS6 + P(5,11)*PS7 + P(5,12)*PS9;
	const float PS70 = PS4 + PS41 + PS42 + PS64;
	const float PS71 = PS35 + PS36;
	const float PS72 = 2*PS57;
	const float PS73 = -PS37 + PS38;
	const float PS74 = 2*PS52;
	const float PS75 = P(0,6) - P(1,6)*PS11 - P(2,6)*PS12 - P(3,6)*PS13 + P(6,10)*PS6 + P(6,11)*PS7 + P(6,12)*PS9;
	const float PS76 = -P(10,11)*PS34;
	const float PS77 = P(0,11)*PS11 + P(1,11) + P(11,11)*PS9 + P(2,11)*PS13 - P(3,11)*PS12 - PS19 + PS76;
	const float PS78 = P(0,2)*PS13;
	const float PS79 = P(0,3)*PS12;
	const float PS80 = P(0,0)*PS11 + P(0,1) - P(0,10)*PS34 + P(0,11)*PS9 - P(0,12)*PS7 + PS78 - PS79;
	const float PS81 = P(0,2)*PS11;
	const float PS82 = P(1,2) - P(2,10)*PS34 + P(2,11)*PS9 - P(2,12)*PS7 + P(2,2)*PS13 + PS28 + PS81;
	const float PS83 = P(10,11)*PS9;
	const float PS84 = P(10,12)*PS7;
	const float PS85 = P(0,10)*PS11 + P(1,10) - P(10,10)*PS34 + P(2,10)*PS13 - P(3,10)*PS12 + PS83 - PS84;
	const float PS86 = -P(10,12)*PS34;
	const float PS87 = P(0,12)*PS11 + P(1,12) - P(12,12)*PS7 + P(2,12)*PS13 - P(3,12)*PS12 + PS16 + PS86;
	const float PS88 = P(0,3)*PS11;
	const float PS89 = P(1,3) - P(3,10)*PS34 + P(3,11)*PS9 - P(3,12)*PS7 - P(3,3)*PS12 + PS25 + PS88;
	const float PS90 = P(1,2)*PS13;
	const float PS91 = P(1,3)*PS12;
	const float PS92 = P(1,1) - P(1,10)*PS34 + P(1,11)*PS9 - P(1,12)*PS7 + PS30 + PS90 - PS91;
	const float PS93 = P(0,13)*PS11 + P(1,13) - P(10,13)*PS34 + P(11,13)*PS9 - P(12,13)*PS7 + P(2,13)*PS13 - P(3,13)*PS12;
	const float PS94 = P(0,15)*PS11 + P(1,15) - P(10,15)*PS34 + P(11,15)*PS9 - P(12,15)*PS7 + P(2,15)*PS13 - P(3,15)*PS12;
	const float PS95 = 2*PS94;
	const float PS96 = P(0,14)*PS11 + P(1,14) - P(10,14)*PS34 + P(11,14)*PS9 - P(12,14)*PS7 + P(2,14)*PS13 - P(3,14)*PS12;
	const float PS97 = 2*PS96;
	const float PS98 = P(0,4)*PS11 + P(1,4) + P(2,4)*PS13 - P(3,4)*PS12 - P(4,10)*PS34 + P(4,11)*PS9 - P(4,12)*PS7;
	const float PS99 = 2*PS93;
	const float PS100 = P(0,5)*PS11 + P(1,5) + P(2,5)*PS13 - P(3,5)*PS12 - P(5,10)*PS34 + P(5,11)*PS9 - P(5,12)*PS7;
	const float PS101 = P(0,6)*PS11 + P(1,6) + P(2,6)*PS13 - P(3,6)*PS12 - P(6,10)*PS34 + P(6,11)*PS9 - P(6,12)*PS7;
	const float PS102 = -P(11,12)*PS34;
	const float PS103 = P(0,12)*PS12 - P(1,12)*PS13 + P(12,12)*PS6 + P(2,12) + P(3,12)*PS11 - PS10 + PS102;
	const float PS104 = P(2,3) - P(3,10)*PS9 - P(3,11)*PS34 + P(3,12)*PS6 + P(3,3)*PS11 + PS22 + PS79;
	const float PS105 = P(0,1)*PS13;
	const float PS106 = P(0,0)*PS12 - P(0,10)*PS9 - P(0,11)*PS34 + P(0,12)*PS6 + P(0,2) - PS105 + PS88;
	const float PS107 = P(11,12)*PS6;
	const float PS108 = P(0,11)*PS12 - P(1,11)*PS13 - P(11,11)*PS34 + P(2,11) + P(3,11)*PS11 + PS107 - PS83;
	const float PS109 = P(0,10)*PS12 - P(1,10)*PS13 - P(10,10)*PS9 + P(2,10) + P(3,10)*PS11 + PS18 + PS76;
	const float PS110 = P(0,1)*PS12;
	const float PS111 = -P(1,1)*PS13 - P(1,10)*PS9 - P(1,11)*PS34 + P(1,12)*PS6 + P(1,2) + PS110 + PS27;
	const float PS112 = P(2,3)*PS11;
	const float PS113 = -P(2,10)*PS9 - P(2,11)*PS34 + P(2,12)*PS6 + P(2,2) + PS112 + PS31 - PS90;
	const float PS114 = P(0,13)*PS12 - P(1,13)*PS13 - P(10,13)*PS9 - P(11,13)*PS34 + P(12,13)*PS6 + P(2,13) + P(3,13)*PS11;
	const float PS115 = P(0,15)*PS12 - P(1,15)*PS13 - P(10,15)*PS9 - P(11,15)*PS34 + P(12,15)*PS6 + P(2,15) + P(3,15)*PS11;
	const float PS116 = 2*PS115;
	const float PS117 = P(0,14)*PS12 - P(1,14)*PS13 - P(10,14)*PS9 - P(11,14)*PS34 + P(12,14)*PS6 + P(2,14) + P(3,14)*PS11;
	const float PS118 = 2*PS117;
	const float PS119 = P(0,4)*PS12 - P(1,4)*PS13 + P(2,4) + P(3,4)*PS11 - P(4,10)*PS9 - P(4,11)*PS34 + P(4,12)*PS6;
	const float PS120 = 2*PS114;
	const float PS121 = P(0,5)*PS12 - P(1,5)*PS13 + P(2,5) + P(3,5)*PS11 - P(5,10)*PS9 - P(5,11)*PS34 + P(5,12)*PS6;
	const float PS122 = P(0,6)*PS12 - P(1,6)*PS13 + P(2,6) + P(3,6)*PS11 - P(6,10)*PS9 - P(6,11)*PS34 + P(6,12)*PS6;
	const float PS123 = P(0,10)*PS13 + P(1,10)*PS12 + P(10,10)*PS7 - P(2,10)*PS11 + P(3,10) - PS15 + PS86;
	const float PS124 = P(1,1)*PS12 + P(1,10)*PS7 - P(1,11)*PS6 - P(1,12)*PS34 + P(1,3) + PS105 + PS24;
	const float PS125 = P(0,0)*PS13 + P(0,10)*PS7 - P(0,11)*PS6 - P(0,12)*PS34 + P(0,3) + PS110 - PS81;
	const float PS126 = P(0,12)*PS13 + P(1,12)*PS12 - P(12,12)*PS34 - P(2,12)*PS11 + P(3,12) - PS107 + PS84;
	const float PS127 = P(0,11)*PS13 + P(1,11)*PS12 - P(11,11)*PS6 - P(2,11)*PS11 + P(3,11) + PS102 + PS8;
	const float PS128 = P(2,10)*PS7 - P(2,11)*PS6 - P(2,12)*PS34 - P(2,2)*PS11 + P(2,3) + PS21 + PS78;
	const float PS129 = P(3,10)*PS7 - P(3,11)*PS6 - P(3,12)*PS34 + P(3,3) - PS112 + PS32 + PS91;
	const float PS130 = P(0,13)*PS13 + P(1,13)*PS12 + P(10,13)*PS7 - P(11,13)*PS6 - P(12,13)*PS34 - P(2,13)*PS11 + P(3,13);
	const float PS131 = P(0,15)*PS13 + P(1,15)*PS12 + P(10,15)*PS7 - P(11,15)*PS6 - P(12,15)*PS34 - P(2,15)*PS11 + P(3,15);
	const float PS132 = 2*PS131;
	const float PS133 = P(0,14)*PS13 + P(1,14)*PS12 + P(10,14)*PS7 - P(11,14)*PS6 - P(12,14)*PS34 - P(2,14)*PS11 + P(3,14);
	const float PS134 = 2*PS133;
	const float PS135 = P(0,4)*PS13 + P(1,4)*PS12 - P(2,4)*PS11 + P(3,4) + P(4,10)*PS7 - P(4,11)*PS6 - P(4,12)*PS34;
	const float PS136 = 2*PS130;
	const float PS137 = P(0,5)*PS13 + P(1,5)*PS12 - P(2,5)*PS11 + P(3,5) + P(5,10)*PS7 - P(5,11)*PS6 - P(5,12)*PS34;
	const float PS138 = P(0,6)*PS13 + P(1,6)*PS12 - P(2,6)*PS11 + P(3,6) + P(6,10)*PS7 - P(6,11)*PS6 - P(6,12)*PS34;
	const float PS139 = 2*PS46;
	const float PS140 = 2*PS54;
	const float PS141 = P(0,13)*PS72 + P(1,13)*PS62 - P(13,13)*PS44 + P(13,14)*PS140 - P(13,15)*PS139 + P(2,13)*PS60 - P(3,13)*PS74 + P(4,13);
	const float PS142 = P(0,15)*PS72 + P(1,15)*PS62 - P(13,15)*PS44 + P(14,15)*PS140 - P(15,15)*PS139 + P(2,15)*PS60 - P(3,15)*PS74 + P(4,15);
	const float PS143 = P(1,3)*PS62;
	const float PS144 = P(0,3)*PS72;
	const float PS145 = P(2,3)*PS60 - P(3,13)*PS44 + P(3,14)*PS140 - P(3,15)*PS139 - P(3,3)*PS74 + P(3,4) + PS143 + PS144;
	const float PS146 = P(0,14)*PS72 + P(1,14)*PS62 - P(13,14)*PS44 + P(14,14)*PS140 - P(14,15)*PS139 + P(2,14)*PS60 - P(3,14)*PS74 + P(4,14);
	const float PS147 = P(0,2)*PS60;
	const float PS148 = P(0,3)*PS74;
	const float PS149 = P(0,0)*PS72 + P(0,1)*PS62 - P(0,13)*PS44 + P(0,14)*PS140 - P(0,15)*PS139 + P(0,4) + PS147 - PS148;
	const float PS150 = P(1,2)*PS62;
	const float PS151 = P(0,2)*PS72;
	const float PS152 = -P(2,13)*PS44 + P(2,14)*PS140 - P(2,15)*PS139 + P(2,2)*PS60 - P(2,3)*PS74 + P(2,4) + PS150 + PS151;
	const float PS153 = P(1,2)*PS60;
	const float PS154 = P(1,3)*PS74;
	const float PS155 = P(0,1)*PS72 + P(1,1)*PS62 - P(1,13)*PS44 + P(1,14)*PS140 - P(1,15)*PS139 + P(1,4) + PS153 - PS154;
	const float PS156 = 4*dvyVar;
	const float PS157 = 4*dvzVar;
	const float PS158 = P(0,4)*PS72 + P(1,4)*PS62 + P(2,4)*PS60 - P(3,4)*PS74 - P(4,13)*PS44 + P(4,14)*PS140 - P(4,15)*PS139 + P(4,4);
	const float PS159 = 2*PS141;
	const float PS160 = 2*PS68;
	const float PS161 = PS65*dvyVar;
	const float PS162 = 2*PS66;
	const float PS163 = PS44*dvxVar;
	const float PS164 = P(0,5)*PS72 + P(1,5)*PS62 + P(2,5)*PS60 - P(3,5)*PS74 + P(4,5) - P(5,13)*PS44 + P(5,14)*PS140 - P(5,15)*PS139;
	const float PS165 = 2*PS71;
	const float PS166 = 2*PS73;
	const float PS167 = PS70*dvzVar;
	const float PS168 = P(0,6)*PS72 + P(1,6)*PS62 + P(2,6)*PS60 - P(3,6)*PS74 + P(4,6) - P(6,13)*PS44 + P(6,14)*PS140 - P(6,15)*PS139;
	const float PS169 = P(0,14)*PS74 - P(1,14)*PS60 - P(13,14)*PS162 - P(14,14)*PS65 + P(14,15)*PS160 + P(2,14)*PS62 + P(3,14)*PS72 + P(5,14);
	const float PS170 = P(0,13)*PS74 - P(1,13)*PS60 - P(13,13)*PS162 - P(13,14)*PS65 + P(13,15)*PS160 + P(2,13)*PS62 + P(3,13)*PS72 + P(5,13);
	const float PS171 = P(0,1)*PS74;
	const float PS172 = -P(1,1)*PS60 - P(1,13)*PS162 - P(1,14)*PS65 + P(1,15)*PS160 + P(1,3)*PS72 + P(1,5) + PS150 + PS171;
	const float PS173 = P(0,15)*PS74 - P(1,15)*PS60 - P(13,15)*PS162 - P(14,15)*PS65 + P(15,15)*PS160 + P(2,15)*PS62 + P(3,15)*PS72 + P(5,15);
	const float PS174 = P(2,3)*PS62;
	const float PS175 = -P(1,3)*PS60 - P(3,13)*PS162 - P(3,14)*PS65 + P(3,15)*PS160 + P(3,3)*PS72 + P(3,5) + PS148 + PS174;
	const float PS176 = P(0,1)*PS60;
	const float PS177 = P(0,0)*PS74 - P(0,13)*PS162 - P(0,14)*PS65 + P(0,15)*PS160 + P(0,2)*PS62 + P(0,5) + PS144 - PS176;
	const float PS178 = P(2,3)*PS72;
	const float PS179 = P(0,2)*PS74 - P(2,13)*PS162 - P(2,14)*PS65 + P(2,15)*PS160 + P(2,2)*PS62 + P(2,5) - PS153 + PS178;
	const float PS180 = 4*dvxVar;
	const float PS181 = P(0,5)*PS74 - P(1,5)*PS60 + P(2,5)*PS62 + P(3,5)*PS72 - P(5,13)*PS162 - P(5,14)*PS65 + P(5,15)*PS160 + P(5,5);
	const float PS182 = P(0,6)*PS74 - P(1,6)*PS60 + P(2,6)*PS62 + P(3,6)*PS72 + P(5,6) - P(6,13)*PS162 - P(6,14)*PS65 + P(6,15)*PS160;
	const float PS183 = P(0,15)*PS60 + P(1,15)*PS74 + P(13,15)*PS166 - P(14,15)*PS165 - P(15,15)*PS70 - P(2,15)*PS72 + P(3,15)*PS62 + P(6,15);
	const float PS184 = P(0,14)*PS60 + P(1,14)*PS74 + P(13,14)*PS166 - P(14,14)*PS165 - P(14,15)*PS70 - P(2,14)*PS72 + P(3,14)*PS62 + P(6,14);
	const float PS185 = P(0,13)*PS60 + P(1,13)*PS74 + P(13,13)*PS166 - P(13,14)*PS165 - P(13,15)*PS70 - P(2,13)*PS72 + P(3,13)*PS62 + P(6,13);
	const float PS186 = P(0,6)*PS60 + P(1,6)*PS74 - P(2,6)*PS72 + P(3,6)*PS62 + P(6,13)*PS166 - P(6,14)*PS165 - P(6,15)*PS70 + P(6,6);

	// covariance update
	SquareMatrix24f nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	nextP(0,0) = PS0*PS1 - PS11*PS23 - PS12*PS26 - PS13*PS29 + PS14*PS6 + PS17*PS7 + PS2*PS3 + PS20*PS9 + PS33 + PS4*PS5;
	nextP(0,1) = -PS1*PS36 + PS11*PS33 - PS12*PS29 + PS13*PS26 - PS14*PS34 + PS17*PS9 - PS20*PS7 + PS23 + PS3*PS35 - PS35*PS5;
	nextP(1,1) = PS1*PS42 + PS11*PS80 - PS12*PS89 + PS13*PS82 + PS2*PS5 + PS3*PS4 - PS34*PS85 - PS7*PS87 + PS77*PS9 + PS92;
	nextP(0,2) = -PS1*PS37 + PS11*PS29 + PS12*PS33 - PS13*PS23 - PS14*PS9 - PS17*PS34 + PS20*PS6 + PS26 - PS3*PS38 + PS37*PS5;
	nextP(1,2) = PS1*PS40 + PS11*PS89 + PS12*PS80 - PS13*PS92 - PS3*PS40 - PS34*PS77 - PS39*PS5 + PS6*PS87 + PS82 - PS85*PS9;
	nextP(2,2) = PS0*PS5 + PS1*PS4 + PS103*PS6 + PS104*PS11 + PS106*PS12 - PS108*PS34 - PS109*PS9 - PS111*PS13 + PS113 + PS3*PS42;
	nextP(0,3) = PS1*PS39 - PS11*PS26 + PS12*PS23 + PS13*PS33 + PS14*PS7 - PS17*PS6 - PS20*PS34 + PS29 - PS3*PS39 - PS40*PS5;
	nextP(1,3) = -PS1*PS38 - PS11*PS82 + PS12*PS92 + PS13*PS80 - PS3*PS37 - PS34*PS87 + PS38*PS5 - PS6*PS77 + PS7*PS85 + PS89;
	nextP(2,3) = -PS1*PS35 - PS103*PS34 + PS104 + PS106*PS13 - PS108*PS6 + PS109*PS7 - PS11*PS113 + PS111*PS12 + PS3*PS36 - PS36*PS5;
	nextP(3,3) = PS0*PS3 + PS1*PS2 - PS11*PS128 + PS12*PS124 + PS123*PS7 + PS125*PS13 - PS126*PS34 - PS127*PS6 + PS129 + PS42*PS5;
	nextP(0,4) = PS23*PS62 + PS26*PS60 - PS44*PS45 - PS46*PS48 - PS52*PS53 + PS54*PS56 + PS57*PS58 + PS63;
	nextP(1,4) = -PS44*PS93 - PS46*PS95 + PS54*PS97 + PS60*PS82 + PS62*PS92 + PS72*PS80 - PS74*PS89 + PS98;
	nextP(2,4) = -PS104*PS74 + PS106*PS72 + PS111*PS62 + PS113*PS60 - PS114*PS44 - PS116*PS46 + PS118*PS54 + PS119;
	nextP(3,4) = PS124*PS62 + PS125*PS72 + PS128*PS60 - PS129*PS74 - PS130*PS44 - PS132*PS46 + PS134*PS54 + PS135;
	nextP(4,4) = -PS139*PS142 + PS140*PS146 - PS141*PS44 - PS145*PS74 + PS149*PS72 + PS152*PS60 + PS155*PS62 + PS156*ecl::powf(PS54, 2) + PS157*ecl::powf(PS46, 2) + PS158 + ecl::powf(PS44, 2)*dvxVar;
	nextP(0,5) = -PS23*PS60 + PS26*PS62 + PS48*PS68 + PS52*PS58 + PS53*PS57 - PS55*PS65 - PS66*PS67 + PS69;
	nextP(1,5) = PS100 - PS60*PS92 + PS62*PS82 - PS65*PS96 - PS66*PS99 + PS68*PS95 + PS72*PS89 + PS74*PS80;
	nextP(2,5) = PS104*PS72 + PS106*PS74 - PS111*PS60 + PS113*PS62 + PS116*PS68 - PS117*PS65 - PS120*PS66 + PS121;
	nextP(3,5) = -PS124*PS60 + PS125*PS74 + PS128*PS62 + PS129*PS72 + PS132*PS68 - PS133*PS65 - PS136*PS66 + PS137;
	nextP(4,5) = -PS140*PS161 + PS142*PS160 + PS145*PS72 - PS146*PS65 + PS149*PS74 + PS152*PS62 - PS155*PS60 - PS157*PS46*PS68 - PS159*PS66 + PS162*PS163 + PS164;
	nextP(5,5) = PS157*ecl::powf(PS68, 2) + PS160*PS173 - PS162*PS170 - PS169*PS65 - PS172*PS60 + PS175*PS72 + PS177*PS74 + PS179*PS62 + PS180*ecl::powf(PS66, 2) + PS181 + ecl::powf(PS65, 2)*dvyVar;
	nextP(0,6) = PS23*PS74 - PS26*PS72 - PS47*PS70 + PS53*PS61 - PS56*PS71 + PS58*PS59 + PS67*PS73 + PS75;
	nextP(1,6) = PS101 + PS60*PS80 + PS62*PS89 - PS70*PS94 - PS71*PS97 - PS72*PS82 + PS73*PS99 + PS74*PS92;
	nextP(2,6) = PS104*PS62 + PS106*PS60 + PS111*PS74 - PS113*PS72 - PS115*PS70 - PS118*PS71 + PS120*PS73 + PS122;
	nextP(3,6) = PS124*PS74 + PS125*PS60 - PS128*PS72 + PS129*PS62 - PS131*PS70 - PS134*PS71 + PS136*PS73 + PS138;
	nextP(4,6) = PS139*PS167 - PS142*PS70 + PS145*PS62 - PS146*PS165 + PS149*PS60 - PS152*PS72 + PS155*PS74 - PS156*PS54*PS71 + PS159*PS73 - PS163*PS166 + PS168;
	nextP(5,6) = -PS160*PS167 + PS161*PS165 - PS165*PS169 + PS166*PS170 + PS172*PS74 - PS173*PS70 + PS175*PS62 + PS177*PS60 - PS179*PS72 - PS180*PS66*PS73 + PS182;
	nextP(6,6) = PS156*ecl::powf(PS71, 2) - PS165*PS184 + PS166*PS185 + PS180*ecl::powf(PS73, 2) - PS183*PS70 + PS186 + PS60*(P(0,0)*PS60 + P(0,13)*PS166 - P(0,14)*PS165 - P(0,15)*PS70 + P(0,3)*PS62 + P(0,6) - PS151 + PS171) + PS62*(P(0,3)*PS60 + P(3,13)*PS166 - P(3,14)*PS165 - P(3,15)*PS70 + P(3,3)*PS62 + P(3,6) + PS154 - PS178) + ecl::powf(PS70, 2)*dvzVar - PS72*(P(1,2)*PS74 + P(2,13)*PS166 - P(2,14)*PS165 - P(2,15)*PS70 - P(2,2)*PS72 + P(2,6) + PS147 + PS174) + PS74*(P(1,1)*PS74 + P(1,13)*PS166 - P(1,14)*PS165 - P(1,15)*PS70 - P(1,2)*PS72 + P(1,6) + PS143 + PS176);
	nextP(0,7) = P(0,7) - P(1,7)*PS11 - P(2,7)*PS12 - P(3,7)*PS13 + P(7,10)*PS6 + P(7,11)*PS7 + P(7,12)*PS9 + PS63*dt;
	nextP(1,7) = P(0,7)*PS11 + P(1,7) + P(2,7)*PS13 - P(3,7)*PS12 - P(7,10)*PS34 + P(7,11)*PS9 - P(7,12)*PS7 + PS98*dt;
	nextP(2,7) = P(0,7)*PS12 - P(1,7)*PS13 + P(2,7) + P(3,7)*PS11 - P(7,10)*PS9 - P(7,11)*PS34 + P(7,12)*PS6 + PS119*dt;
	nextP(3,7) = P(0,7)*PS13 + P(1,7)*PS12 - P(2,7)*PS11 + P(3,7) + P(7,10)*PS7 - P(7,11)*PS6 - P(7,12)*PS34 + PS135*dt;
	nextP(4,7) = P(0,7)*PS72 + P(1,7)*PS62 + P(2,7)*PS60 - P(3,7)*PS74 + P(4,7) - P(7,13)*PS44 + P(7,14)*PS140 - P(7,15)*PS139 + PS158*dt;
	nextP(5,7) = P(0,7)*PS74 - P(1,7)*PS60 + P(2,7)*PS62 + P(3,7)*PS72 + P(5,7) - P(7,13)*PS162 - P(7,14)*PS65 + P(7,15)*PS160 + dt*(P(0,4)*PS74 - P(1,4)*PS60 + P(2,4)*PS62 + P(3,4)*PS72 - P(4,13)*PS162 - P(4,14)*PS65 + P(4,15)*PS160 + P(4,5));
	nextP(6,7) = P(0,7)*PS60 + P(1,7)*PS74 - P(2,7)*PS72 + P(3,7)*PS62 + P(6,7) + P(7,13)*PS166 - P(7,14)*PS165 - P(7,15)*PS70 + dt*(P(0,4)*PS60 + P(1,4)*PS74 - P(2,4)*PS72 + P(3,4)*PS62 + P(4,13)*PS166 - P(4,14)*PS165 - P(4,15)*PS70 + P(4,6));
	nextP(7,7) = P(4,7)*dt + P(7,7) + dt*(P(4,4)*dt + P(4,7));
	nextP(0,8) = P(0,8) - P(1,8)*PS11 - P(2,8)*PS12 - P(3,8)*PS13 + P(8,10)*PS6 + P(8,11)*PS7 + P(8,12)*PS9 + PS69*dt;
	nextP(1,8) = P(0,8)*PS11 + P(1,8) + P(2,8)*PS13 - P(3,8)*PS12 - P(8,10)*PS34 + P(8,11)*PS9 - P(8,12)*PS7 + PS100*dt;
	nextP(2,8) = P(0,8)*PS12 - P(1,8)*PS13 + P(2,8) + P(3,8)*PS11 - P(8,10)*PS9 - P(8,11)*PS34 + P(8,12)*PS6 + PS121*dt;
	nextP(3,8) = P(0,8)*PS13 + P(1,8)*PS12 - P(2,8)*PS11 + P(3,8) + P(8,10)*PS7 - P(8,11)*PS6 - P(8,12)*PS34 + PS137*dt;
	nextP(4,8) = P(0,8)*PS72 + P(1,8)*PS62 + P(2,8)*PS60 - P(3,8)*PS74 + P(4,8) - P(8,13)*PS44 + P(8,14)*PS140 - P(8,15)*PS139 + PS164*dt;
	nextP(5,8) = P(0,8)*PS74 - P(1,8)*PS60 + P(2,8)*PS62 + P(3,8)*PS72 + P(5,8) - P(8,13)*PS162 - P(8,14)*PS65 + P(8,15)*PS160 + PS181*dt;
	nextP(6,8) = P(0,8)*PS60 + P(1,8)*PS74 - P(2,8)*PS72 + P(3,8)*PS62 + P(6,8) + P(8,13)*PS166 - P(8,14)*PS165 - P(8,15)*PS70 + dt*(P(0,5)*PS60 + P(1,5)*PS74 - P(2,5)*PS72 + P(3,5)*PS62 + P(5,13)*PS166 - P(5,14)*PS165 - P(5,15)*PS70 + P(5,6));
	nextP(7,8) = P(4,8)*dt + P(7,8) + dt*(P(4,5)*dt + P(5,7));
	nextP(8,8) = P(5,8)*dt + P(8,8) + dt*(P(5,5)*dt + P(5,8));
	nextP(0,9) = P(0,9) - P(1,9)*PS11 - P(2,9)*PS12 - P(3,9)*PS13 + P(9,10)*PS6 + P(9,11)*PS7 + P(9,12)*PS9 + PS75*dt;
	nextP(1,9) = P(0,9)*PS11 + P(1,9) + P(2,9)*PS13 - P(3,9)*PS12 - P(9,10)*PS34 + P(9,11)*PS9 - P(9,12)*PS7 + PS101*dt;
	nextP(2,9) = P(0,9)*PS12 - P(1,9)*PS13 + P(2,9) + P(3,9)*PS11 - P(9,10)*PS9 - P(9,11)*PS34 + P(9,12)*PS6 + PS122*dt;
	nextP(3,9) = P(0,9)*PS13 + P(1,9)*PS12 - P(2,9)*PS11 + P(3,9) + P(9,10)*PS7 - P(9,11)*PS6 - P(9,12)*PS34 + PS138*dt;
	nextP(4,9) = P(0,9)*PS72 + P(1,9)*PS62 + P(2,9)*PS60 - P(3,9)*PS74 + P(4,9) - P(9,13)*PS44 + P(9,14)*PS140 - P(9,15)*PS139 + PS168*dt;
	nextP(5,9) = P(0,9)*PS74 - P(1,9)*PS60 + P(2,9)*PS62 + P(3,9)*PS72 + P(5,9) - P(9,13)*PS162 - P(9,14)*PS65 + P(9,15)*PS160 + PS182*dt;
	nextP(6,9) = P(0,9)*PS60 + P(1,9)*PS74 - P(2,9)*PS72 + P(3,9)*PS62 + P(6,9) + P(9,13)*PS166 - P(9,14)*PS165 - P(9,15)*PS70 + PS186*dt;
	nextP(7,9) = P(4,9)*dt + P(7,9) + dt*(P(4,6)*dt + P(6,7));
	nextP(8,9) = P(5,9)*dt + P(8,9) + dt*(P(5,6)*dt + P(6,8));
	nextP(9,9) = P(6,9)*dt + P(9,9) + dt*(P(6,6)*dt + P(6,9));
	nextP(0,10) = PS14;
	nextP(1,10) = PS85;
	nextP(2,10) = PS109;
	nextP(3,10) = PS123;
	nextP(4,10) = P(0,10)*PS72 + P(1,10)*PS62 - P(10,13)*PS44 + P(10,14)*PS140 - P(10,15)*PS139 + P(2,10)*PS60 - P(3,10)*PS74 + P(4,10);
	nextP(5,10) = P(0,10)*PS74 - P(1,10)*PS60 - P(10,13)*PS162 - P(10,14)*PS65 + P(10,15)*PS160 + P(2,10)*PS62 + P(3,10)*PS72 + P(5,10);
	nextP(6,10) = P(0,10)*PS60 + P(1,10)*PS74 + P(10,13)*PS166 - P(10,14)*PS165 - P(10,15)*PS70 - P(2,10)*PS72 + P(3,10)*PS62 + P(6,10);
	nextP(7,10) = P(4,10)*dt + P(7,10);
	nextP(8,10) = P(5,10)*dt + P(8,10);
	nextP(9,10) = P(6,10)*dt + P(9,10);
	nextP(10,10) = P(10,10);
	nextP(0,11) = PS17;
	nextP(1,11) = PS77;
	nextP(2,11) = PS108;
	nextP(3,11) = PS127;
	nextP(4,11) = P(0,11)*PS72 + P(1,11)*PS62 - P(11,13)*PS44 + P(11,14)*PS140 - P(11,15)*PS139 + P(2,11)*PS60 - P(3,11)*PS74 + P(4,11);
	nextP(5,11) = P(0,11)*PS74 - P(1,11)*PS60 - P(11,13)*PS162 - P(11,14)*PS65 + P(11,15)*PS160 + P(2,11)*PS62 + P(3,11)*PS72 + P(5,11);
	nextP(6,11) = P(0,11)*PS60 + P(1,11)*PS74 + P(11,13)*PS166 - P(11,14)*PS165 - P(11,15)*PS70 - P(2,11)*PS72 + P(3,11)*PS62 + P(6,11);
	nextP(7,11) = P(4,11)*dt + P(7,11);
	nextP(8,11) = P(5,11)*dt + P(8,11);
	nextP(9,11) = P(6,11)*dt + P(9,11);
	nextP(10,11) = P(10,11);
	nextP(11,11) = P(11,11);
	nextP(0,12) = PS20;
	nextP(1,12) = PS87;
	nextP(2,12) = PS103;
	nextP(3,12) = PS126;
	nextP(4,12) = P(0,12)*PS72 + P(1,12)*PS62 - P(12,13)*PS44 + P(12,14)*PS140 - P(12,15)*PS139 + P(2,12)*PS60 - P(3,12)*PS74 + P(4,12);
	nextP(5,12) = P(0,12)*PS74 - P(1,12)*PS60 - P(12,13)*PS162 - P(12,14)*PS65 + P(12,15)*PS160 + P(2,12)*PS62 + P(3,12)*PS72 + P(5,12);
	nextP(6,12) = P(0,12)*PS60 + P(1,12)*PS74 + P(12,13)*PS166 - P(12,14)*PS165 - P(12,15)*PS70 - P(2,12)*PS72 + P(3,12)*PS62 + P(6,12);
	nextP(7,12) = P(4,12)*dt + P(7,12);
	nextP(8,12) = P(5,12)*dt + P(8,12);
	nextP(9,12) = P(6,12)*dt + P(9,12);
	nextP(10,12) = P(10,12);
	nextP(11,12) = P(11,12);
	nextP(12,12) = P(12,12);

	// process noise contribution for delta angle states can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (unsigned i = 10; i <=12; i++) {
		const int index = i-10;
		nextP(i,i) = kahanSummation(nextP(i,i), process_noise(i), _delta_angle_bias_var_accum(index));
	}

	if (!_accel_bias_inhibit[0]) {
		// calculate variances and upper diagonal covariances for IMU X axis delta velocity bias state
		nextP(0,13) = PS45;
		nextP(1,13) = PS93;
		nextP(2,13) = PS114;
		nextP(3,13) = PS130;
		nextP(4,13) = PS141;
		nextP(5,13) = PS170;
		nextP(6,13) = PS185;
		nextP(7,13) = P(4,13)*dt + P(7,13);
		nextP(8,13) = P(5,13)*dt + P(8,13);
		nextP(9,13) = P(6,13)*dt + P(9,13);
		nextP(10,13) = P(10,13);
		nextP(11,13) = P(11,13);
		nextP(12,13) = P(12,13);
		nextP(13,13) = P(13,13);

		// add process noise that is not from the IMU
		// process noise contribution for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		nextP(13,13) = kahanSummation(nextP(13,13), process_noise(13), _delta_vel_bias_var_accum(0));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(13, _prev_dvel_bias_var(0));
		_delta_vel_bias_var_accum(0) = 0.f;

	}

	if (!_accel_bias_inhibit[1]) {
		// calculate variances and upper diagonal covariances for IMU Y axis delta velocity bias state
		nextP(0,14) = PS55;
		nextP(1,14) = PS96;
		nextP(2,14) = PS117;
		nextP(3,14) = PS133;
		nextP(4,14) = PS146;
		nextP(5,14) = PS169;
		nextP(6,14) = PS184;
		nextP(7,14) = P(4,14)*dt + P(7,14);
		nextP(8,14) = P(5,14)*dt + P(8,14);
		nextP(9,14) = P(6,14)*dt + P(9,14);
		nextP(10,14) = P(10,14);
		nextP(11,14) = P(11,14);
		nextP(12,14) = P(12,14);
		nextP(13,14) = P(13,14);
		nextP(14,14) = P(14,14);

		// add process noise that is not from the IMU
		// process noise contribution for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		nextP(14,14) = kahanSummation(nextP(14,14), process_noise(14), _delta_vel_bias_var_accum(1));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(14, _prev_dvel_bias_var(1));
		_delta_vel_bias_var_accum(1) = 0.f;

	}

	if (!_accel_bias_inhibit[2]) {
		// calculate variances and upper diagonal covariances for IMU Z axis delta velocity bias state
		nextP(0,15) = PS47;
		nextP(1,15) = PS94;
		nextP(2,15) = PS115;
		nextP(3,15) = PS131;
		nextP(4,15) = PS142;
		nextP(5,15) = PS173;
		nextP(6,15) = PS183;
		nextP(7,15) = P(4,15)*dt + P(7,15);
		nextP(8,15) = P(5,15)*dt + P(8,15);
		nextP(9,15) = P(6,15)*dt + P(9,15);
		nextP(10,15) = P(10,15);
		nextP(11,15) = P(11,15);
		nextP(12,15) = P(12,15);
		nextP(13,15) = P(13,15);
		nextP(14,15) = P(14,15);
		nextP(15,15) = P(15,15);

		// add process noise that is not from the IMU
		// process noise contribution for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		nextP(15,15) = kahanSummation(nextP(15,15), process_noise(15), _delta_vel_bias_var_accum(2));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(15, _prev_dvel_bias_var(2));
		_delta_vel_bias_var_accum(2) = 0.f;

	}

	// Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
	if (_control_status.flags.mag_3D) {
		// calculate variances and upper diagonal covariances for earth and body magnetic field states
		nextP(0,16) = P(0,16) - P(1,16)*PS11 + P(10,16)*PS6 + P(11,16)*PS7 + P(12,16)*PS9 - P(2,16)*PS12 - P(3,16)*PS13;
		nextP(1,16) = P(0,16)*PS11 + P(1,16) - P(10,16)*PS34 + P(11,16)*PS9 - P(12,16)*PS7 + P(2,16)*PS13 - P(3,16)*PS12;
		nextP(2,16) = P(0,16)*PS12 - P(1,16)*PS13 - P(10,16)*PS9 - P(11,16)*PS34 + P(12,16)*PS6 + P(2,16) + P(3,16)*PS11;
		nextP(3,16) = P(0,16)*PS13 + P(1,16)*PS12 + P(10,16)*PS7 - P(11,16)*PS6 - P(12,16)*PS34 - P(2,16)*PS11 + P(3,16);
		nextP(4,16) = P(0,16)*PS72 + P(1,16)*PS62 - P(13,16)*PS44 + P(14,16)*PS140 - P(15,16)*PS139 + P(2,16)*PS60 - P(3,16)*PS74 + P(4,16);
		nextP(5,16) = P(0,16)*PS74 - P(1,16)*PS60 - P(13,16)*PS162 - P(14,16)*PS65 + P(15,16)*PS160 + P(2,16)*PS62 + P(3,16)*PS72 + P(5,16);
		nextP(6,16) = P(0,16)*PS60 + P(1,16)*PS74 + P(13,16)*PS166 - P(14,16)*PS165 - P(15,16)*PS70 - P(2,16)*PS72 + P(3,16)*PS62 + P(6,16);
		nextP(7,16) = P(4,16)*dt + P(7,16);
		nextP(8,16) = P(5,16)*dt + P(8,16);
		nextP(9,16) = P(6,16)*dt + P(9,16);
		nextP(10,16) = P(10,16);
		nextP(11,16) = P(11,16);
		nextP(12,16) = P(12,16);
		nextP(13,16) = P(13,16);
		nextP(14,16) = P(14,16);
		nextP(15,16) = P(15,16);
		nextP(16,16) = P(16,16);
		nextP(0,17) = P(0,17) - P(1,17)*PS11 + P(10,17)*PS6 + P(11,17)*PS7 + P(12,17)*PS9 - P(2,17)*PS12 - P(3,17)*PS13;
		nextP(1,17) = P(0,17)*PS11 + P(1,17) - P(10,17)*PS34 + P(11,17)*PS9 - P(12,17)*PS7 + P(2,17)*PS13 - P(3,17)*PS12;
		nextP(2,17) = P(0,17)*PS12 - P(1,17)*PS13 - P(10,17)*PS9 - P(11,17)*PS34 + P(12,17)*PS6 + P(2,17) + P(3,17)*PS11;
		nextP(3,17) = P(0,17)*PS13 + P(1,17)*PS12 + P(10,17)*PS7 - P(11,17)*PS6 - P(12,17)*PS34 - P(2,17)*PS11 + P(3,17);
		nextP(4,17) = P(0,17)*PS72 + P(1,17)*PS62 - P(13,17)*PS44 + P(14,17)*PS140 - P(15,17)*PS139 + P(2,17)*PS60 - P(3,17)*PS74 + P(4,17);
		nextP(5,17) = P(0,17)*PS74 - P(1,17)*PS60 - P(13,17)*PS162 - P(14,17)*PS65 + P(15,17)*PS160 + P(2,17)*PS62 + P(3,17)*PS72 + P(5,17);
		nextP(6,17) = P(0,17)*PS60 + P(1,17)*PS74 + P(13,17)*PS166 - P(14,17)*PS165 - P(15,17)*PS70 - P(2,17)*PS72 + P(3,17)*PS62 + P(6,17);
		nextP(7,17) = P(4,17)*dt + P(7,17);
		nextP(8,17) = P(5,17)*dt + P(8,17);
		nextP(9,17) = P(6,17)*dt + P(9,17);
		nextP(10,17) = P(10,17);
		nextP(11,17) = P(11,17);
		nextP(12,17) = P(12,17);
		nextP(13,17) = P(13,17);
		nextP(14,17) = P(14,17);
		nextP(15,17) = P(15,17);
		nextP(16,17) = P(16,17);
		nextP(17,17) = P(17,17);
		nextP(0,18) = P(0,18) - P(1,18)*PS11 + P(10,18)*PS6 + P(11,18)*PS7 + P(12,18)*PS9 - P(2,18)*PS12 - P(3,18)*PS13;
		nextP(1,18) = P(0,18)*PS11 + P(1,18) - P(10,18)*PS34 + P(11,18)*PS9 - P(12,18)*PS7 + P(2,18)*PS13 - P(3,18)*PS12;
		nextP(2,18) = P(0,18)*PS12 - P(1,18)*PS13 - P(10,18)*PS9 - P(11,18)*PS34 + P(12,18)*PS6 + P(2,18) + P(3,18)*PS11;
		nextP(3,18) = P(0,18)*PS13 + P(1,18)*PS12 + P(10,18)*PS7 - P(11,18)*PS6 - P(12,18)*PS34 - P(2,18)*PS11 + P(3,18);
		nextP(4,18) = P(0,18)*PS72 + P(1,18)*PS62 - P(13,18)*PS44 + P(14,18)*PS140 - P(15,18)*PS139 + P(2,18)*PS60 - P(3,18)*PS74 + P(4,18);
		nextP(5,18) = P(0,18)*PS74 - P(1,18)*PS60 - P(13,18)*PS162 - P(14,18)*PS65 + P(15,18)*PS160 + P(2,18)*PS62 + P(3,18)*PS72 + P(5,18);
		nextP(6,18) = P(0,18)*PS60 + P(1,18)*PS74 + P(13,18)*PS166 - P(14,18)*PS165 - P(15,18)*PS70 - P(2,18)*PS72 + P(3,18)*PS62 + P(6,18);
		nextP(7,18) = P(4,18)*dt + P(7,18);
		nextP(8,18) = P(5,18)*dt + P(8,18);
		nextP(9,18) = P(6,18)*dt + P(9,18);
		nextP(10,18) = P(10,18);
		nextP(11,18) = P(11,18);
		nextP(12,18) = P(12,18);
		nextP(13,18) = P(13,18);
		nextP(14,18) = P(14,18);
		nextP(15,18) = P(15,18);
		nextP(16,18) = P(16,18);
		nextP(17,18) = P(17,18);
		nextP(18,18) = P(18,18);
		nextP(0,19) = P(0,19) - P(1,19)*PS11 + P(10,19)*PS6 + P(11,19)*PS7 + P(12,19)*PS9 - P(2,19)*PS12 - P(3,19)*PS13;
		nextP(1,19) = P(0,19)*PS11 + P(1,19) - P(10,19)*PS34 + P(11,19)*PS9 - P(12,19)*PS7 + P(2,19)*PS13 - P(3,19)*PS12;
		nextP(2,19) = P(0,19)*PS12 - P(1,19)*PS13 - P(10,19)*PS9 - P(11,19)*PS34 + P(12,19)*PS6 + P(2,19) + P(3,19)*PS11;
		nextP(3,19) = P(0,19)*PS13 + P(1,19)*PS12 + P(10,19)*PS7 - P(11,19)*PS6 - P(12,19)*PS34 - P(2,19)*PS11 + P(3,19);
		nextP(4,19) = P(0,19)*PS72 + P(1,19)*PS62 - P(13,19)*PS44 + P(14,19)*PS140 - P(15,19)*PS139 + P(2,19)*PS60 - P(3,19)*PS74 + P(4,19);
		nextP(5,19) = P(0,19)*PS74 - P(1,19)*PS60 - P(13,19)*PS162 - P(14,19)*PS65 + P(15,19)*PS160 + P(2,19)*PS62 + P(3,19)*PS72 + P(5,19);
		nextP(6,19) = P(0,19)*PS60 + P(1,19)*PS74 + P(13,19)*PS166 - P(14,19)*PS165 - P(15,19)*PS70 - P(2,19)*PS72 + P(3,19)*PS62 + P(6,19);
		nextP(7,19) = P(4,19)*dt + P(7,19);
		nextP(8,19) = P(5,19)*dt + P(8,19);
		nextP(9,19) = P(6,19)*dt + P(9,19);
		nextP(10,19) = P(10,19);
		nextP(11,19) = P(11,19);
		nextP(12,19) = P(12,19);
		nextP(13,19) = P(13,19);
		nextP(14,19) = P(14,19);
		nextP(15,19) = P(15,19);
		nextP(16,19) = P(16,19);
		nextP(17,19) = P(17,19);
		nextP(18,19) = P(18,19);
		nextP(19,19) = P(19,19);
		nextP(0,20) = P(0,20) - P(1,20)*PS11 + P(10,20)*PS6 + P(11,20)*PS7 + P(12,20)*PS9 - P(2,20)*PS12 - P(3,20)*PS13;
		nextP(1,20) = P(0,20)*PS11 + P(1,20) - P(10,20)*PS34 + P(11,20)*PS9 - P(12,20)*PS7 + P(2,20)*PS13 - P(3,20)*PS12;
		nextP(2,20) = P(0,20)*PS12 - P(1,20)*PS13 - P(10,20)*PS9 - P(11,20)*PS34 + P(12,20)*PS6 + P(2,20) + P(3,20)*PS11;
		nextP(3,20) = P(0,20)*PS13 + P(1,20)*PS12 + P(10,20)*PS7 - P(11,20)*PS6 - P(12,20)*PS34 - P(2,20)*PS11 + P(3,20);
		nextP(4,20) = P(0,20)*PS72 + P(1,20)*PS62 - P(13,20)*PS44 + P(14,20)*PS140 - P(15,20)*PS139 + P(2,20)*PS60 - P(3,20)*PS74 + P(4,20);
		nextP(5,20) = P(0,20)*PS74 - P(1,20)*PS60 - P(13,20)*PS162 - P(14,20)*PS65 + P(15,20)*PS160 + P(2,20)*PS62 + P(3,20)*PS72 + P(5,20);
		nextP(6,20) = P(0,20)*PS60 + P(1,20)*PS74 + P(13,20)*PS166 - P(14,20)*PS165 - P(15,20)*PS70 - P(2,20)*PS72 + P(3,20)*PS62 + P(6,20);
		nextP(7,20) = P(4,20)*dt + P(7,20);
		nextP(8,20) = P(5,20)*dt + P(8,20);
		nextP(9,20) = P(6,20)*dt + P(9,20);
		nextP(10,20) = P(10,20);
		nextP(11,20) = P(11,20);
		nextP(12,20) = P(12,20);
		nextP(13,20) = P(13,20);
		nextP(14,20) = P(14,20);
		nextP(15,20) = P(15,20);
		nextP(16,20) = P(16,20);
		nextP(17,20) = P(17,20);
		nextP(18,20) = P(18,20);
		nextP(19,20) = P(19,20);
		nextP(20,20) = P(20,20);
		nextP(0,21) = P(0,21) - P(1,21)*PS11 + P(10,21)*PS6 + P(11,21)*PS7 + P(12,21)*PS9 - P(2,21)*PS12 - P(3,21)*PS13;
		nextP(1,21) = P(0,21)*PS11 + P(1,21) - P(10,21)*PS34 + P(11,21)*PS9 - P(12,21)*PS7 + P(2,21)*PS13 - P(3,21)*PS12;
		nextP(2,21) = P(0,21)*PS12 - P(1,21)*PS13 - P(10,21)*PS9 - P(11,21)*PS34 + P(12,21)*PS6 + P(2,21) + P(3,21)*PS11;
		nextP(3,21) = P(0,21)*PS13 + P(1,21)*PS12 + P(10,21)*PS7 - P(11,21)*PS6 - P(12,21)*PS34 - P(2,21)*PS11 + P(3,21);
		nextP(4,21) = P(0,21)*PS72 + P(1,21)*PS62 - P(13,21)*PS44 + P(14,21)*PS140 - P(15,21)*PS139 + P(2,21)*PS60 - P(3,21)*PS74 + P(4,21);
		nextP(5,21) = P(0,21)*PS74 - P(1,21)*PS60 - P(13,21)*PS162 - P(14,21)*PS65 + P(15,21)*PS160 + P(2,21)*PS62 + P(3,21)*PS72 + P(5,21);
		nextP(6,21) = P(0,21)*PS60 + P(1,21)*PS74 + P(13,21)*PS166 - P(14,21)*PS165 - P(15,21)*PS70 - P(2,21)*PS72 + P(3,21)*PS62 + P(6,21);
		nextP(7,21) = P(4,21)*dt + P(7,21);
		nextP(8,21) = P(5,21)*dt + P(8,21);
		nextP(9,21) = P(6,21)*dt + P(9,21);
		nextP(10,21) = P(10,21);
		nextP(11,21) = P(11,21);
		nextP(12,21) = P(12,21);
		nextP(13,21) = P(13,21);
		nextP(14,21) = P(14,21);
		nextP(15,21) = P(15,21);
		nextP(16,21) = P(16,21);
		nextP(17,21) = P(17,21);
		nextP(18,21) = P(18,21);
		nextP(19,21) = P(19,21);
		nextP(20,21) = P(20,21);
		nextP(21,21) = P(21,21);

		// add process noise that is not from the IMU
		for (unsigned i = 16; i <= 21; i++) {
			nextP(i,i) += process_noise(i);
		}

	}

	// Don't do covariance prediction on wind states unless we are using them
	if (_control_status.flags.wind) {

		// calculate variances and upper diagonal covariances for wind states
		nextP(0,22) = P(0,22) - P(1,22)*PS11 + P(10,22)*PS6 + P(11,22)*PS7 + P(12,22)*PS9 - P(2,22)*PS12 - P(3,22)*PS13;
		nextP(1,22) = P(0,22)*PS11 + P(1,22) - P(10,22)*PS34 + P(11,22)*PS9 - P(12,22)*PS7 + P(2,22)*PS13 - P(3,22)*PS12;
		nextP(2,22) = P(0,22)*PS12 - P(1,22)*PS13 - P(10,22)*PS9 - P(11,22)*PS34 + P(12,22)*PS6 + P(2,22) + P(3,22)*PS11;
		nextP(3,22) = P(0,22)*PS13 + P(1,22)*PS12 + P(10,22)*PS7 - P(11,22)*PS6 - P(12,22)*PS34 - P(2,22)*PS11 + P(3,22);
		nextP(4,22) = P(0,22)*PS72 + P(1,22)*PS62 - P(13,22)*PS44 + P(14,22)*PS140 - P(15,22)*PS139 + P(2,22)*PS60 - P(3,22)*PS74 + P(4,22);
		nextP(5,22) = P(0,22)*PS74 - P(1,22)*PS60 - P(13,22)*PS162 - P(14,22)*PS65 + P(15,22)*PS160 + P(2,22)*PS62 + P(3,22)*PS72 + P(5,22);
		nextP(6,22) = P(0,22)*PS60 + P(1,22)*PS74 + P(13,22)*PS166 - P(14,22)*PS165 - P(15,22)*PS70 - P(2,22)*PS72 + P(3,22)*PS62 + P(6,22);
		nextP(7,22) = P(4,22)*dt + P(7,22);
		nextP(8,22) = P(5,22)*dt + P(8,22);
		nextP(9,22) = P(6,22)*dt + P(9,22);
		nextP(10,22) = P(10,22);
		nextP(11,22) = P(11,22);
		nextP(12,22) = P(12,22);
		nextP(13,22) = P(13,22);
		nextP(14,22) = P(14,22);
		nextP(15,22) = P(15,22);
		nextP(16,22) = P(16,22);
		nextP(17,22) = P(17,22);
		nextP(18,22) = P(18,22);
		nextP(19,22) = P(19,22);
		nextP(20,22) = P(20,22);
		nextP(21,22) = P(21,22);
		nextP(22,22) = P(22,22);
		nextP(0,23) = P(0,23) - P(1,23)*PS11 + P(10,23)*PS6 + P(11,23)*PS7 + P(12,23)*PS9 - P(2,23)*PS12 - P(3,23)*PS13;
		nextP(1,23) = P(0,23)*PS11 + P(1,23) - P(10,23)*PS34 + P(11,23)*PS9 - P(12,23)*PS7 + P(2,23)*PS13 - P(3,23)*PS12;
		nextP(2,23) = P(0,23)*PS12 - P(1,23)*PS13 - P(10,23)*PS9 - P(11,23)*PS34 + P(12,23)*PS6 + P(2,23) + P(3,23)*PS11;
		nextP(3,23) = P(0,23)*PS13 + P(1,23)*PS12 + P(10,23)*PS7 - P(11,23)*PS6 - P(12,23)*PS34 - P(2,23)*PS11 + P(3,23);
		nextP(4,23) = P(0,23)*PS72 + P(1,23)*PS62 - P(13,23)*PS44 + P(14,23)*PS140 - P(15,23)*PS139 + P(2,23)*PS60 - P(3,23)*PS74 + P(4,23);
		nextP(5,23) = P(0,23)*PS74 - P(1,23)*PS60 - P(13,23)*PS162 - P(14,23)*PS65 + P(15,23)*PS160 + P(2,23)*PS62 + P(3,23)*PS72 + P(5,23);
		nextP(6,23) = P(0,23)*PS60 + P(1,23)*PS74 + P(13,23)*PS166 - P(14,23)*PS165 - P(15,23)*PS70 - P(2,23)*PS72 + P(3,23)*PS62 + P(6,23);
		nextP(7,23) = P(4,23)*dt + P(7,23);
		nextP(8,23) = P(5,23)*dt + P(8,23);
		nextP(9,23) = P(6,23)*dt + P(9,23);
		nextP(10,23) = P(10,23);
		nextP(11,23) = P(11,23);
		nextP(12,23) = P(12,23);
		nextP(13,23) = P(13,23);
		nextP(14,23) = P(14,23);
		nextP(15,23) = P(15,23);
		nextP(16,23) = P(16,23);
		nextP(17,23) = P(17,23);
		nextP(18,23) = P(18,23);
		nextP(19,23) = P(19,23);
		nextP(20,23) = P(20,23);
		nextP(21,23) = P(21,23);
		nextP(22,23) = P(22,23);
		nextP(23,23) = P(23,23);

		// add process noise that is not from the IMU
		for (unsigned i = 22; i <= 23; i++) {
			nextP(i,i) += process_noise(i);
		}

	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P(7,7) + P(8,8)) > 1e4f) {
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP(i,j) = P(i,j);
				nextP(j,i) = P(j,i);
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P(row,column) = P(column,row) = nextP(column,row);
		}
	}

	// copy variances (diagonals)
	for (unsigned i = 0; i < _k_num_states; i++) {
		P(i,i) = nextP(i,i);
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);

}

void Ekf::fixCovarianceErrors(bool force_symmetry)
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
		P(i,i) = math::constrain(P(i,i), 0.0f, P_lim[0]);
	}

	for (int i = 4; i <= 6; i++) {
		// NED velocity states
		P(i,i) = math::constrain(P(i,i), 1e-6f, P_lim[1]);
	}

	for (int i = 7; i <= 9; i++) {
		// NED position states
		P(i,i) = math::constrain(P(i,i), 1e-6f, P_lim[2]);
	}

	for (int i = 10; i <= 12; i++) {
		// gyro bias states
		P(i,i) = math::constrain(P(i,i), 0.0f, P_lim[3]);
	}

	// force symmetry on the quaternion, velocity and position state covariances
	if (force_symmetry) {
		P.makeRowColSymmetric<13>(0);
	}

	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if (!_accel_bias_inhibit[0] || !_accel_bias_inhibit[1] || !_accel_bias_inhibit[2]) {
		// Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		const float minSafeStateVar = 1e-9f;
		float maxStateVar = minSafeStateVar;
		bool resetRequired = false;

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_accel_bias_inhibit[stateIndex - 13]) {
				// Skip the check for the inhibited axis
				continue;
			}
			if (P(stateIndex,stateIndex) > maxStateVar) {
				maxStateVar = P(stateIndex,stateIndex);

			} else if (P(stateIndex,stateIndex) < minSafeStateVar) {
				resetRequired = true;
			}
		}

		// To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
		// not exceed 100 and the minimum variance must not fall below the target minimum
		// Also limit variance to a maximum equivalent to a 0.1g uncertainty
		const float minStateVarTarget = 5E-8f;
		float minAllowedStateVar = fmaxf(0.01f * maxStateVar, minStateVarTarget);

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_accel_bias_inhibit[stateIndex - 13]) {
				// Skip the check for the inhibited axis
				continue;
			}
			P(stateIndex,stateIndex) = math::constrain(P(stateIndex,stateIndex), minAllowedStateVar, sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
		}

		// If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) {
			P.uncorrelateCovariance<3>(13);
		}

		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		const float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		const float down_dvel_bias = _state.delta_vel_bias.dot(Vector3f(_R_to_earth.row(2)));

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = (fabsf(down_dvel_bias) > dVel_bias_lim
				     && ( (down_dvel_bias * _gps_vel_innov(2) < 0.0f && _control_status.flags.gps)
				     ||   (down_dvel_bias * _ev_vel_innov(2) < 0.0f && _control_status.flags.ev_vel) )
				     && ( (down_dvel_bias * _gps_pos_innov(2) < 0.0f && _control_status.flags.gps_hgt)
				     ||   (down_dvel_bias * _baro_hgt_innov(2) < 0.0f && _control_status.flags.baro_hgt)
				     ||   (down_dvel_bias * _rng_hgt_innov(2) < 0.0f && _control_status.flags.rng_hgt)
				     ||   (down_dvel_bias * _ev_pos_innov(2) < 0.0f && _control_status.flags.ev_hgt) ) );

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_last_imu;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(_time_acc_bias_check, (uint64_t)7e6)) {

			P.uncorrelateCovariance<3>(13);

			_time_acc_bias_check = _time_last_imu;
			_fault_status.flags.bad_acc_bias = false;
			_warning_events.flags.invalid_accel_bias_cov_reset = true;
			ECL_WARN("invalid accel bias - covariance reset");

		} else if (force_symmetry) {
			// ensure the covariance values are symmetrical
			P.makeRowColSymmetric<3>(13);
		}

	}

	// magnetic field states
	if (!_control_status.flags.mag_3D) {
		zeroMagCov();

	} else {
		// constrain variances
		for (int i = 16; i <= 18; i++) {
			P(i,i) = math::constrain(P(i,i), 0.0f, P_lim[5]);
		}

		for (int i = 19; i <= 21; i++) {
			P(i,i) = math::constrain(P(i,i), 0.0f, P_lim[6]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<3>(16);
			P.makeRowColSymmetric<3>(19);
		}

	}

	// wind velocity states
	if (!_control_status.flags.wind) {
		P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

	} else {
		// constrain variances
		for (int i = 22; i <= 23; i++) {
			P(i,i) = math::constrain(P(i,i), 0.0f, P_lim[7]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<2>(22);
		}
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrix24f& KHP) {
	bool healthy = true;
	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);
			healthy = false;
		}
	}
	return healthy;
}

void Ekf::resetMagRelatedCovariances()
{
	resetQuatCov();
	resetMagCov();
}

void Ekf::resetQuatCov(){
	zeroQuatCov();

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var.setAll(sq(_params.initial_tilt_err));

	initialiseQuatCovariances(rot_vec_var);
}

void Ekf::zeroQuatCov()
{
	P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
	P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);
}

void Ekf::resetMagCov()
{
	// reset the corresponding rows and columns in the covariance matrix and
	// set the variances on the magnetic field states to the measurement variance
	clearMagCov();

	P.uncorrelateCovarianceSetVariance<3>(16, sq(_params.mag_noise));
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

	if (!_control_status.flags.mag_3D) {
		// save covariance data for re-use when auto-switching between heading and 3-axis fusion
		// if already in 3-axis fusion mode, the covariances are automatically saved when switching out
		// of this mode
		saveMagCovData();
	}
}

void Ekf::clearMagCov()
{
	zeroMagCov();
	_mag_decl_cov_reset = false;
}

void Ekf::zeroMagCov()
{
	P.uncorrelateCovarianceSetVariance<3>(16, 0.0f);
	P.uncorrelateCovarianceSetVariance<3>(19, 0.0f);
}

void Ekf::resetZDeltaAngBiasCov()
{
	const float init_delta_ang_bias_var = sq(_params.switch_on_gyro_bias * _dt_ekf_avg);

	P.uncorrelateCovarianceSetVariance<1>(12, init_delta_ang_bias_var);
}

void Ekf::resetWindCovariance()
{
	if (_tas_data_ready && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us < (uint64_t)5e5)) {
		// Derived using EKF/matlab/scripts/Inertial Nav EKF/wind_cov.py
		// TODO: explicitly include the sideslip angle in the derivation
		const float euler_yaw = getEuler321Yaw(_state.quat_nominal);
		const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));
		constexpr float initial_sideslip_uncertainty = math::radians(15.0f);
		const float initial_wind_var_body_y = sq(_airspeed_sample_delayed.true_airspeed * sinf(initial_sideslip_uncertainty));
		constexpr float R_yaw = sq(math::radians(10.0f));

		const float cos_yaw = cosf(euler_yaw);
		const float sin_yaw = sinf(euler_yaw);

		// rotate wind velocity into earth frame aligned with vehicle yaw
		const float Wx = _state.wind_vel(0) * cos_yaw + _state.wind_vel(1) * sin_yaw;
		const float Wy = -_state.wind_vel(0) * sin_yaw + _state.wind_vel(1) * cos_yaw;

		// it is safer to remove all existing correlations to other states at this time
		P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

		P(22,22) = R_TAS*sq(cos_yaw) + R_yaw*sq(-Wx*sin_yaw - Wy*cos_yaw) + initial_wind_var_body_y*sq(sin_yaw);
		P(22,23) = R_TAS*sin_yaw*cos_yaw + R_yaw*(-Wx*sin_yaw - Wy*cos_yaw)*(Wx*cos_yaw - Wy*sin_yaw) - initial_wind_var_body_y*sin_yaw*cos_yaw;
		P(23,22) = P(22,23);
		P(23,23) = R_TAS*sq(sin_yaw) + R_yaw*sq(Wx*cos_yaw - Wy*sin_yaw) + initial_wind_var_body_y*sq(cos_yaw);

		// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
		P(22,22) += P(4,4);
		P(23,23) += P(5,5);

	} else {
		// without airspeed, start with a small initial uncertainty to improve the initial estimate
		P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
	}
}
