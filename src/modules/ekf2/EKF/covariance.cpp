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

	if (_control_status.flags.mag_3D && (P(16, 16) + P(17, 17) + P(18, 18)) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;

	if (_control_status.flags.mag_3D && (P(19, 19) + P(20, 20) + P(21, 21)) < 0.1f) {
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
	process_noise.slice<3, 1>(10, 0) = sq(d_ang_bias_sig);
	// delta_velocity bias states
	process_noise.slice<3, 1>(13, 0) = sq(d_vel_bias_sig);
	// earth frame magnetic field states
	process_noise.slice<3, 1>(16, 0) = sq(mag_I_sig);
	// body frame magnetic field states
	process_noise.slice<3, 1>(19, 0) = sq(mag_B_sig);
	// wind velocity states
	process_noise.slice<2, 1>(22, 0) = sq(wind_vel_sig);

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
	const float PS41 = 2*PS2;
	const float PS42 = 2*PS4 - 1;
	const float PS43 = PS41 + PS42;
	const float PS44 = P(0,13) - P(1,13)*PS11 + P(10,13)*PS6 + P(11,13)*PS7 + P(12,13)*PS9 - P(2,13)*PS12 - P(3,13)*PS13;
	const float PS45 = PS37 + PS38;
	const float PS46 = P(0,15) - P(1,15)*PS11 + P(10,15)*PS6 + P(11,15)*PS7 + P(12,15)*PS9 - P(2,15)*PS12 - P(3,15)*PS13;
	const float PS47 = 2*PS46;
	const float PS48 = dvy - dvy_b;
	const float PS49 = PS48*q0;
	const float PS50 = dvz - dvz_b;
	const float PS51 = PS50*q1;
	const float PS52 = dvx - dvx_b;
	const float PS53 = PS52*q3;
	const float PS54 = PS49 - PS51 + 2*PS53;
	const float PS55 = 2*PS29;
	const float PS56 = -PS39 + PS40;
	const float PS57 = P(0,14) - P(1,14)*PS11 + P(10,14)*PS6 + P(11,14)*PS7 + P(12,14)*PS9 - P(2,14)*PS12 - P(3,14)*PS13;
	const float PS58 = 2*PS57;
	const float PS59 = PS48*q2;
	const float PS60 = PS50*q3;
	const float PS61 = PS59 + PS60;
	const float PS62 = 2*PS23;
	const float PS63 = PS50*q2;
	const float PS64 = PS48*q3;
	const float PS65 = -PS64;
	const float PS66 = PS63 + PS65;
	const float PS67 = 2*PS33;
	const float PS68 = PS50*q0;
	const float PS69 = PS48*q1;
	const float PS70 = PS52*q2;
	const float PS71 = PS68 + PS69 - 2*PS70;
	const float PS72 = 2*PS26;
	const float PS73 = P(0,4) - P(1,4)*PS11 - P(2,4)*PS12 - P(3,4)*PS13 + P(4,10)*PS6 + P(4,11)*PS7 + P(4,12)*PS9;
	const float PS74 = 2*PS0;
	const float PS75 = PS42 + PS74;
	const float PS76 = PS39 + PS40;
	const float PS77 = 2*PS44;
	const float PS78 = PS51 - PS53;
	const float PS79 = -PS70;
	const float PS80 = PS68 + 2*PS69 + PS79;
	const float PS81 = -PS35 + PS36;
	const float PS82 = PS52*q1;
	const float PS83 = PS60 + PS82;
	const float PS84 = PS52*q0;
	const float PS85 = PS63 - 2*PS64 + PS84;
	const float PS86 = P(0,5) - P(1,5)*PS11 - P(2,5)*PS12 - P(3,5)*PS13 + P(5,10)*PS6 + P(5,11)*PS7 + P(5,12)*PS9;
	const float PS87 = PS41 + PS74 - 1;
	const float PS88 = PS35 + PS36;
	const float PS89 = 2*PS63 + PS65 + PS84;
	const float PS90 = -PS37 + PS38;
	const float PS91 = PS59 + PS82;
	const float PS92 = PS69 + PS79;
	const float PS93 = PS49 - 2*PS51 + PS53;
	const float PS94 = P(0,6) - P(1,6)*PS11 - P(2,6)*PS12 - P(3,6)*PS13 + P(6,10)*PS6 + P(6,11)*PS7 + P(6,12)*PS9;
	const float PS95 = ecl::powf(q0, 2);
	const float PS96 = -P(10,11)*PS34;
	const float PS97 = P(0,11)*PS11 + P(1,11) + P(11,11)*PS9 + P(2,11)*PS13 - P(3,11)*PS12 - PS19 + PS96;
	const float PS98 = P(0,2)*PS13;
	const float PS99 = P(0,3)*PS12;
	const float PS100 = P(0,0)*PS11 + P(0,1) - P(0,10)*PS34 + P(0,11)*PS9 - P(0,12)*PS7 + PS98 - PS99;
	const float PS101 = P(0,2)*PS11;
	const float PS102 = P(1,2) - P(2,10)*PS34 + P(2,11)*PS9 - P(2,12)*PS7 + P(2,2)*PS13 + PS101 + PS28;
	const float PS103 = P(10,11)*PS9;
	const float PS104 = P(10,12)*PS7;
	const float PS105 = P(0,10)*PS11 + P(1,10) - P(10,10)*PS34 + P(2,10)*PS13 - P(3,10)*PS12 + PS103 - PS104;
	const float PS106 = -P(10,12)*PS34;
	const float PS107 = P(0,12)*PS11 + P(1,12) - P(12,12)*PS7 + P(2,12)*PS13 - P(3,12)*PS12 + PS106 + PS16;
	const float PS108 = P(0,3)*PS11;
	const float PS109 = P(1,3) - P(3,10)*PS34 + P(3,11)*PS9 - P(3,12)*PS7 - P(3,3)*PS12 + PS108 + PS25;
	const float PS110 = P(1,2)*PS13;
	const float PS111 = P(1,3)*PS12;
	const float PS112 = P(1,1) - P(1,10)*PS34 + P(1,11)*PS9 - P(1,12)*PS7 + PS110 - PS111 + PS30;
	const float PS113 = P(0,13)*PS11 + P(1,13) - P(10,13)*PS34 + P(11,13)*PS9 - P(12,13)*PS7 + P(2,13)*PS13 - P(3,13)*PS12;
	const float PS114 = P(0,15)*PS11 + P(1,15) - P(10,15)*PS34 + P(11,15)*PS9 - P(12,15)*PS7 + P(2,15)*PS13 - P(3,15)*PS12;
	const float PS115 = 2*PS114;
	const float PS116 = 2*PS109;
	const float PS117 = P(0,14)*PS11 + P(1,14) - P(10,14)*PS34 + P(11,14)*PS9 - P(12,14)*PS7 + P(2,14)*PS13 - P(3,14)*PS12;
	const float PS118 = 2*PS117;
	const float PS119 = 2*PS112;
	const float PS120 = 2*PS100;
	const float PS121 = 2*PS102;
	const float PS122 = P(0,4)*PS11 + P(1,4) + P(2,4)*PS13 - P(3,4)*PS12 - P(4,10)*PS34 + P(4,11)*PS9 - P(4,12)*PS7;
	const float PS123 = 2*PS113;
	const float PS124 = P(0,5)*PS11 + P(1,5) + P(2,5)*PS13 - P(3,5)*PS12 - P(5,10)*PS34 + P(5,11)*PS9 - P(5,12)*PS7;
	const float PS125 = P(0,6)*PS11 + P(1,6) + P(2,6)*PS13 - P(3,6)*PS12 - P(6,10)*PS34 + P(6,11)*PS9 - P(6,12)*PS7;
	const float PS126 = -P(11,12)*PS34;
	const float PS127 = P(0,12)*PS12 - P(1,12)*PS13 + P(12,12)*PS6 + P(2,12) + P(3,12)*PS11 - PS10 + PS126;
	const float PS128 = P(2,3) - P(3,10)*PS9 - P(3,11)*PS34 + P(3,12)*PS6 + P(3,3)*PS11 + PS22 + PS99;
	const float PS129 = P(0,1)*PS13;
	const float PS130 = P(0,0)*PS12 - P(0,10)*PS9 - P(0,11)*PS34 + P(0,12)*PS6 + P(0,2) + PS108 - PS129;
	const float PS131 = P(11,12)*PS6;
	const float PS132 = P(0,11)*PS12 - P(1,11)*PS13 - P(11,11)*PS34 + P(2,11) + P(3,11)*PS11 - PS103 + PS131;
	const float PS133 = P(0,10)*PS12 - P(1,10)*PS13 - P(10,10)*PS9 + P(2,10) + P(3,10)*PS11 + PS18 + PS96;
	const float PS134 = P(0,1)*PS12;
	const float PS135 = -P(1,1)*PS13 - P(1,10)*PS9 - P(1,11)*PS34 + P(1,12)*PS6 + P(1,2) + PS134 + PS27;
	const float PS136 = P(2,3)*PS11;
	const float PS137 = -P(2,10)*PS9 - P(2,11)*PS34 + P(2,12)*PS6 + P(2,2) - PS110 + PS136 + PS31;
	const float PS138 = P(0,13)*PS12 - P(1,13)*PS13 - P(10,13)*PS9 - P(11,13)*PS34 + P(12,13)*PS6 + P(2,13) + P(3,13)*PS11;
	const float PS139 = P(0,15)*PS12 - P(1,15)*PS13 - P(10,15)*PS9 - P(11,15)*PS34 + P(12,15)*PS6 + P(2,15) + P(3,15)*PS11;
	const float PS140 = 2*PS139;
	const float PS141 = 2*PS128;
	const float PS142 = P(0,14)*PS12 - P(1,14)*PS13 - P(10,14)*PS9 - P(11,14)*PS34 + P(12,14)*PS6 + P(2,14) + P(3,14)*PS11;
	const float PS143 = 2*PS142;
	const float PS144 = 2*PS135;
	const float PS145 = 2*PS130;
	const float PS146 = 2*PS137;
	const float PS147 = P(0,4)*PS12 - P(1,4)*PS13 + P(2,4) + P(3,4)*PS11 - P(4,10)*PS9 - P(4,11)*PS34 + P(4,12)*PS6;
	const float PS148 = 2*PS138;
	const float PS149 = P(0,5)*PS12 - P(1,5)*PS13 + P(2,5) + P(3,5)*PS11 - P(5,10)*PS9 - P(5,11)*PS34 + P(5,12)*PS6;
	const float PS150 = P(0,6)*PS12 - P(1,6)*PS13 + P(2,6) + P(3,6)*PS11 - P(6,10)*PS9 - P(6,11)*PS34 + P(6,12)*PS6;
	const float PS151 = P(0,10)*PS13 + P(1,10)*PS12 + P(10,10)*PS7 - P(2,10)*PS11 + P(3,10) + PS106 - PS15;
	const float PS152 = P(1,1)*PS12 + P(1,10)*PS7 - P(1,11)*PS6 - P(1,12)*PS34 + P(1,3) + PS129 + PS24;
	const float PS153 = P(0,0)*PS13 + P(0,10)*PS7 - P(0,11)*PS6 - P(0,12)*PS34 + P(0,3) - PS101 + PS134;
	const float PS154 = P(0,12)*PS13 + P(1,12)*PS12 - P(12,12)*PS34 - P(2,12)*PS11 + P(3,12) + PS104 - PS131;
	const float PS155 = P(0,11)*PS13 + P(1,11)*PS12 - P(11,11)*PS6 - P(2,11)*PS11 + P(3,11) + PS126 + PS8;
	const float PS156 = P(2,10)*PS7 - P(2,11)*PS6 - P(2,12)*PS34 - P(2,2)*PS11 + P(2,3) + PS21 + PS98;
	const float PS157 = P(3,10)*PS7 - P(3,11)*PS6 - P(3,12)*PS34 + P(3,3) + PS111 - PS136 + PS32;
	const float PS158 = P(0,13)*PS13 + P(1,13)*PS12 + P(10,13)*PS7 - P(11,13)*PS6 - P(12,13)*PS34 - P(2,13)*PS11 + P(3,13);
	const float PS159 = P(0,15)*PS13 + P(1,15)*PS12 + P(10,15)*PS7 - P(11,15)*PS6 - P(12,15)*PS34 - P(2,15)*PS11 + P(3,15);
	const float PS160 = 2*PS159;
	const float PS161 = 2*PS157;
	const float PS162 = P(0,14)*PS13 + P(1,14)*PS12 + P(10,14)*PS7 - P(11,14)*PS6 - P(12,14)*PS34 - P(2,14)*PS11 + P(3,14);
	const float PS163 = 2*PS162;
	const float PS164 = 2*PS152;
	const float PS165 = 2*PS153;
	const float PS166 = 2*PS156;
	const float PS167 = P(0,4)*PS13 + P(1,4)*PS12 - P(2,4)*PS11 + P(3,4) + P(4,10)*PS7 - P(4,11)*PS6 - P(4,12)*PS34;
	const float PS168 = 2*PS158;
	const float PS169 = P(0,5)*PS13 + P(1,5)*PS12 - P(2,5)*PS11 + P(3,5) + P(5,10)*PS7 - P(5,11)*PS6 - P(5,12)*PS34;
	const float PS170 = P(0,6)*PS13 + P(1,6)*PS12 - P(2,6)*PS11 + P(3,6) + P(6,10)*PS7 - P(6,11)*PS6 - P(6,12)*PS34;
	const float PS171 = 2*PS45;
	const float PS172 = 2*PS56;
	const float PS173 = 2*PS61;
	const float PS174 = 2*PS66;
	const float PS175 = 2*PS71;
	const float PS176 = 2*PS54;
	const float PS177 = P(0,13)*PS174 + P(1,13)*PS173 + P(13,13)*PS43 + P(13,14)*PS172 - P(13,15)*PS171 + P(2,13)*PS175 - P(3,13)*PS176 + P(4,13);
	const float PS178 = P(0,15)*PS174 + P(1,15)*PS173 + P(13,15)*PS43 + P(14,15)*PS172 - P(15,15)*PS171 + P(2,15)*PS175 - P(3,15)*PS176 + P(4,15);
	const float PS179 = P(0,3)*PS174 + P(1,3)*PS173 + P(2,3)*PS175 + P(3,13)*PS43 + P(3,14)*PS172 - P(3,15)*PS171 - P(3,3)*PS176 + P(3,4);
	const float PS180 = P(0,14)*PS174 + P(1,14)*PS173 + P(13,14)*PS43 + P(14,14)*PS172 - P(14,15)*PS171 + P(2,14)*PS175 - P(3,14)*PS176 + P(4,14);
	const float PS181 = P(0,1)*PS174 + P(1,1)*PS173 + P(1,13)*PS43 + P(1,14)*PS172 - P(1,15)*PS171 + P(1,2)*PS175 - P(1,3)*PS176 + P(1,4);
	const float PS182 = P(0,0)*PS174 + P(0,1)*PS173 + P(0,13)*PS43 + P(0,14)*PS172 - P(0,15)*PS171 + P(0,2)*PS175 - P(0,3)*PS176 + P(0,4);
	const float PS183 = P(0,2)*PS174 + P(1,2)*PS173 + P(2,13)*PS43 + P(2,14)*PS172 - P(2,15)*PS171 + P(2,2)*PS175 - P(2,3)*PS176 + P(2,4);
	const float PS184 = 4*dvyVar;
	const float PS185 = 4*dvzVar;
	const float PS186 = P(0,4)*PS174 + P(1,4)*PS173 + P(2,4)*PS175 - P(3,4)*PS176 + P(4,13)*PS43 + P(4,14)*PS172 - P(4,15)*PS171 + P(4,4);
	const float PS187 = 2*PS177;
	const float PS188 = 2*PS182;
	const float PS189 = 2*PS181;
	const float PS190 = 2*PS81;
	const float PS191 = 2*PS183;
	const float PS192 = 2*PS179;
	const float PS193 = 2*PS76;
	const float PS194 = PS43*dvxVar;
	const float PS195 = PS75*dvyVar;
	const float PS196 = P(0,5)*PS174 + P(1,5)*PS173 + P(2,5)*PS175 - P(3,5)*PS176 + P(4,5) + P(5,13)*PS43 + P(5,14)*PS172 - P(5,15)*PS171;
	const float PS197 = 2*PS88;
	const float PS198 = PS87*dvzVar;
	const float PS199 = 2*PS90;
	const float PS200 = P(0,6)*PS174 + P(1,6)*PS173 + P(2,6)*PS175 - P(3,6)*PS176 + P(4,6) + P(6,13)*PS43 + P(6,14)*PS172 - P(6,15)*PS171;
	const float PS201 = 2*PS83;
	const float PS202 = 2*PS78;
	const float PS203 = 2*PS85;
	const float PS204 = 2*PS80;
	const float PS205 = -P(0,14)*PS202 - P(1,14)*PS204 - P(13,14)*PS193 + P(14,14)*PS75 + P(14,15)*PS190 + P(2,14)*PS201 + P(3,14)*PS203 + P(5,14);
	const float PS206 = -P(0,13)*PS202 - P(1,13)*PS204 - P(13,13)*PS193 + P(13,14)*PS75 + P(13,15)*PS190 + P(2,13)*PS201 + P(3,13)*PS203 + P(5,13);
	const float PS207 = -P(0,0)*PS202 - P(0,1)*PS204 - P(0,13)*PS193 + P(0,14)*PS75 + P(0,15)*PS190 + P(0,2)*PS201 + P(0,3)*PS203 + P(0,5);
	const float PS208 = -P(0,1)*PS202 - P(1,1)*PS204 - P(1,13)*PS193 + P(1,14)*PS75 + P(1,15)*PS190 + P(1,2)*PS201 + P(1,3)*PS203 + P(1,5);
	const float PS209 = -P(0,15)*PS202 - P(1,15)*PS204 - P(13,15)*PS193 + P(14,15)*PS75 + P(15,15)*PS190 + P(2,15)*PS201 + P(3,15)*PS203 + P(5,15);
	const float PS210 = -P(0,2)*PS202 - P(1,2)*PS204 - P(2,13)*PS193 + P(2,14)*PS75 + P(2,15)*PS190 + P(2,2)*PS201 + P(2,3)*PS203 + P(2,5);
	const float PS211 = -P(0,3)*PS202 - P(1,3)*PS204 + P(2,3)*PS201 - P(3,13)*PS193 + P(3,14)*PS75 + P(3,15)*PS190 + P(3,3)*PS203 + P(3,5);
	const float PS212 = 4*dvxVar;
	const float PS213 = -P(0,5)*PS202 - P(1,5)*PS204 + P(2,5)*PS201 + P(3,5)*PS203 - P(5,13)*PS193 + P(5,14)*PS75 + P(5,15)*PS190 + P(5,5);
	const float PS214 = 2*PS89;
	const float PS215 = 2*PS91;
	const float PS216 = 2*PS92;
	const float PS217 = 2*PS93;
	const float PS218 = -P(0,6)*PS202 - P(1,6)*PS204 + P(2,6)*PS201 + P(3,6)*PS203 + P(5,6) - P(6,13)*PS193 + P(6,14)*PS75 + P(6,15)*PS190;
	const float PS219 = P(0,15)*PS216 + P(1,15)*PS217 + P(13,15)*PS199 - P(14,15)*PS197 + P(15,15)*PS87 - P(2,15)*PS214 + P(3,15)*PS215 + P(6,15);
	const float PS220 = P(0,14)*PS216 + P(1,14)*PS217 + P(13,14)*PS199 - P(14,14)*PS197 + P(14,15)*PS87 - P(2,14)*PS214 + P(3,14)*PS215 + P(6,14);
	const float PS221 = P(0,13)*PS216 + P(1,13)*PS217 + P(13,13)*PS199 - P(13,14)*PS197 + P(13,15)*PS87 - P(2,13)*PS214 + P(3,13)*PS215 + P(6,13);
	const float PS222 = P(0,6)*PS216 + P(1,6)*PS217 - P(2,6)*PS214 + P(3,6)*PS215 + P(6,13)*PS199 - P(6,14)*PS197 + P(6,15)*PS87 + P(6,6);


	// covariance update
	SquareMatrix24f nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states

	nextP(0,0) = PS0*PS1 - PS11*PS23 - PS12*PS26 - PS13*PS29 + PS14*PS6 + PS17*PS7 + PS2*PS3 + PS20*PS9 + PS33 + PS4*PS5;
	nextP(0,1) = -PS1*PS36 + PS11*PS33 - PS12*PS29 + PS13*PS26 - PS14*PS34 + PS17*PS9 - PS20*PS7 + PS23 + PS3*PS35 - PS35*PS5;
	nextP(1,1) = PS1*PS95 + PS100*PS11 + PS102*PS13 - PS105*PS34 - PS107*PS7 - PS109*PS12 + PS112 + PS2*PS5 + PS3*PS4 + PS9*PS97;
	nextP(0,2) = -PS1*PS37 + PS11*PS29 + PS12*PS33 - PS13*PS23 - PS14*PS9 - PS17*PS34 + PS20*PS6 + PS26 - PS3*PS38 + PS37*PS5;
	nextP(1,2) = PS1*PS40 + PS100*PS12 + PS102 - PS105*PS9 + PS107*PS6 + PS109*PS11 - PS112*PS13 - PS3*PS40 - PS34*PS97 - PS39*PS5;
	nextP(2,2) = PS0*PS5 + PS1*PS4 + PS11*PS128 + PS12*PS130 + PS127*PS6 - PS13*PS135 - PS132*PS34 - PS133*PS9 + PS137 + PS3*PS95;
	nextP(0,3) = PS1*PS39 - PS11*PS26 + PS12*PS23 + PS13*PS33 + PS14*PS7 - PS17*PS6 - PS20*PS34 + PS29 - PS3*PS39 - PS40*PS5;
	nextP(1,3) = -PS1*PS38 + PS100*PS13 - PS102*PS11 + PS105*PS7 - PS107*PS34 + PS109 + PS112*PS12 - PS3*PS37 + PS38*PS5 - PS6*PS97;
	nextP(2,3) = -PS1*PS35 - PS11*PS137 + PS12*PS135 - PS127*PS34 + PS128 + PS13*PS130 - PS132*PS6 + PS133*PS7 + PS3*PS36 - PS36*PS5;
	nextP(3,3) = PS0*PS3 + PS1*PS2 - PS11*PS156 + PS12*PS152 + PS13*PS153 + PS151*PS7 - PS154*PS34 - PS155*PS6 + PS157 + PS5*PS95;
	nextP(0,4) = PS43*PS44 - PS45*PS47 - PS54*PS55 + PS56*PS58 + PS61*PS62 + PS66*PS67 + PS71*PS72 + PS73;
	nextP(1,4) = PS113*PS43 - PS115*PS45 - PS116*PS54 + PS118*PS56 + PS119*PS61 + PS120*PS66 + PS121*PS71 + PS122;
	nextP(2,4) = PS138*PS43 - PS140*PS45 - PS141*PS54 + PS143*PS56 + PS144*PS61 + PS145*PS66 + PS146*PS71 + PS147;
	nextP(3,4) = PS158*PS43 - PS160*PS45 - PS161*PS54 + PS163*PS56 + PS164*PS61 + PS165*PS66 + PS166*PS71 + PS167;
	nextP(4,4) = -PS171*PS178 + PS172*PS180 + PS173*PS181 + PS174*PS182 + PS175*PS183 - PS176*PS179 + PS177*PS43 + PS184*ecl::powf(PS56, 2) + PS185*ecl::powf(PS45, 2) + PS186 + ecl::powf(PS43, 2)*dvxVar;
	nextP(0,5) = PS47*PS81 + PS55*PS85 + PS57*PS75 - PS62*PS80 - PS67*PS78 + PS72*PS83 - PS76*PS77 + PS86;
	nextP(1,5) = PS115*PS81 + PS116*PS85 + PS117*PS75 - PS119*PS80 - PS120*PS78 + PS121*PS83 - PS123*PS76 + PS124;
	nextP(2,5) = PS140*PS81 + PS141*PS85 + PS142*PS75 - PS144*PS80 - PS145*PS78 + PS146*PS83 - PS148*PS76 + PS149;
	nextP(3,5) = PS160*PS81 + PS161*PS85 + PS162*PS75 - PS164*PS80 - PS165*PS78 + PS166*PS83 - PS168*PS76 + PS169;
	nextP(4,5) = PS172*PS195 + PS178*PS190 + PS180*PS75 - PS185*PS45*PS81 - PS187*PS76 - PS188*PS78 - PS189*PS80 + PS191*PS83 + PS192*PS85 - PS193*PS194 + PS196;
	nextP(5,5) = PS185*ecl::powf(PS81, 2) + PS190*PS209 - PS193*PS206 + PS201*PS210 - PS202*PS207 + PS203*PS211 - PS204*PS208 + PS205*PS75 + PS212*ecl::powf(PS76, 2) + PS213 + ecl::powf(PS75, 2)*dvyVar;
	nextP(0,6) = PS46*PS87 + PS55*PS91 - PS58*PS88 + PS62*PS93 + PS67*PS92 - PS72*PS89 + PS77*PS90 + PS94;
	nextP(1,6) = PS114*PS87 + PS116*PS91 - PS118*PS88 + PS119*PS93 + PS120*PS92 - PS121*PS89 + PS123*PS90 + PS125;
	nextP(2,6) = PS139*PS87 + PS141*PS91 - PS143*PS88 + PS144*PS93 + PS145*PS92 - PS146*PS89 + PS148*PS90 + PS150;
	nextP(3,6) = PS159*PS87 + PS161*PS91 - PS163*PS88 + PS164*PS93 + PS165*PS92 - PS166*PS89 + PS168*PS90 + PS170;
	nextP(4,6) = -PS171*PS198 + PS178*PS87 - PS180*PS197 - PS184*PS56*PS88 + PS187*PS90 + PS188*PS92 + PS189*PS93 - PS191*PS89 + PS192*PS91 + PS194*PS199 + PS200;
	nextP(5,6) = PS190*PS198 - PS195*PS197 - PS197*PS205 + PS199*PS206 + PS207*PS216 + PS208*PS217 + PS209*PS87 - PS210*PS214 + PS211*PS215 - PS212*PS76*PS90 + PS218;
	nextP(6,6) = PS184*ecl::powf(PS88, 2) - PS197*PS220 + PS199*PS221 + PS212*ecl::powf(PS90, 2) - PS214*(P(0,2)*PS216 + P(1,2)*PS217 + P(2,13)*PS199 - P(2,14)*PS197 + P(2,15)*PS87 - P(2,2)*PS214 + P(2,3)*PS215 + P(2,6)) + PS215*(P(0,3)*PS216 + P(1,3)*PS217 - P(2,3)*PS214 + P(3,13)*PS199 - P(3,14)*PS197 + P(3,15)*PS87 + P(3,3)*PS215 + P(3,6)) + PS216*(P(0,0)*PS216 + P(0,1)*PS217 + P(0,13)*PS199 - P(0,14)*PS197 + P(0,15)*PS87 - P(0,2)*PS214 + P(0,3)*PS215 + P(0,6)) + PS217*(P(0,1)*PS216 + P(1,1)*PS217 + P(1,13)*PS199 - P(1,14)*PS197 + P(1,15)*PS87 - P(1,2)*PS214 + P(1,3)*PS215 + P(1,6)) + PS219*PS87 + PS222 + ecl::powf(PS87, 2)*dvzVar;
	nextP(0,7) = P(0,7) - P(1,7)*PS11 - P(2,7)*PS12 - P(3,7)*PS13 + P(7,10)*PS6 + P(7,11)*PS7 + P(7,12)*PS9 + PS73*dt;
	nextP(1,7) = P(0,7)*PS11 + P(1,7) + P(2,7)*PS13 - P(3,7)*PS12 - P(7,10)*PS34 + P(7,11)*PS9 - P(7,12)*PS7 + PS122*dt;
	nextP(2,7) = P(0,7)*PS12 - P(1,7)*PS13 + P(2,7) + P(3,7)*PS11 - P(7,10)*PS9 - P(7,11)*PS34 + P(7,12)*PS6 + PS147*dt;
	nextP(3,7) = P(0,7)*PS13 + P(1,7)*PS12 - P(2,7)*PS11 + P(3,7) + P(7,10)*PS7 - P(7,11)*PS6 - P(7,12)*PS34 + PS167*dt;
	nextP(4,7) = P(0,7)*PS174 + P(1,7)*PS173 + P(2,7)*PS175 - P(3,7)*PS176 + P(4,7) + P(7,13)*PS43 + P(7,14)*PS172 - P(7,15)*PS171 + PS186*dt;
	nextP(5,7) = -P(0,7)*PS202 - P(1,7)*PS204 + P(2,7)*PS201 + P(3,7)*PS203 + P(5,7) - P(7,13)*PS193 + P(7,14)*PS75 + P(7,15)*PS190 + dt*(-P(0,4)*PS202 - P(1,4)*PS204 + P(2,4)*PS201 + P(3,4)*PS203 - P(4,13)*PS193 + P(4,14)*PS75 + P(4,15)*PS190 + P(4,5));
	nextP(6,7) = P(0,7)*PS216 + P(1,7)*PS217 - P(2,7)*PS214 + P(3,7)*PS215 + P(6,7) + P(7,13)*PS199 - P(7,14)*PS197 + P(7,15)*PS87 + dt*(P(0,4)*PS216 + P(1,4)*PS217 - P(2,4)*PS214 + P(3,4)*PS215 + P(4,13)*PS199 - P(4,14)*PS197 + P(4,15)*PS87 + P(4,6));
	nextP(7,7) = P(4,7)*dt + P(7,7) + dt*(P(4,4)*dt + P(4,7));
	nextP(0,8) = P(0,8) - P(1,8)*PS11 - P(2,8)*PS12 - P(3,8)*PS13 + P(8,10)*PS6 + P(8,11)*PS7 + P(8,12)*PS9 + PS86*dt;
	nextP(1,8) = P(0,8)*PS11 + P(1,8) + P(2,8)*PS13 - P(3,8)*PS12 - P(8,10)*PS34 + P(8,11)*PS9 - P(8,12)*PS7 + PS124*dt;
	nextP(2,8) = P(0,8)*PS12 - P(1,8)*PS13 + P(2,8) + P(3,8)*PS11 - P(8,10)*PS9 - P(8,11)*PS34 + P(8,12)*PS6 + PS149*dt;
	nextP(3,8) = P(0,8)*PS13 + P(1,8)*PS12 - P(2,8)*PS11 + P(3,8) + P(8,10)*PS7 - P(8,11)*PS6 - P(8,12)*PS34 + PS169*dt;
	nextP(4,8) = P(0,8)*PS174 + P(1,8)*PS173 + P(2,8)*PS175 - P(3,8)*PS176 + P(4,8) + P(8,13)*PS43 + P(8,14)*PS172 - P(8,15)*PS171 + PS196*dt;
	nextP(5,8) = -P(0,8)*PS202 - P(1,8)*PS204 + P(2,8)*PS201 + P(3,8)*PS203 + P(5,8) - P(8,13)*PS193 + P(8,14)*PS75 + P(8,15)*PS190 + PS213*dt;
	nextP(6,8) = P(0,8)*PS216 + P(1,8)*PS217 - P(2,8)*PS214 + P(3,8)*PS215 + P(6,8) + P(8,13)*PS199 - P(8,14)*PS197 + P(8,15)*PS87 + dt*(P(0,5)*PS216 + P(1,5)*PS217 - P(2,5)*PS214 + P(3,5)*PS215 + P(5,13)*PS199 - P(5,14)*PS197 + P(5,15)*PS87 + P(5,6));
	nextP(7,8) = P(4,8)*dt + P(7,8) + dt*(P(4,5)*dt + P(5,7));
	nextP(8,8) = P(5,8)*dt + P(8,8) + dt*(P(5,5)*dt + P(5,8));
	nextP(0,9) = P(0,9) - P(1,9)*PS11 - P(2,9)*PS12 - P(3,9)*PS13 + P(9,10)*PS6 + P(9,11)*PS7 + P(9,12)*PS9 + PS94*dt;
	nextP(1,9) = P(0,9)*PS11 + P(1,9) + P(2,9)*PS13 - P(3,9)*PS12 - P(9,10)*PS34 + P(9,11)*PS9 - P(9,12)*PS7 + PS125*dt;
	nextP(2,9) = P(0,9)*PS12 - P(1,9)*PS13 + P(2,9) + P(3,9)*PS11 - P(9,10)*PS9 - P(9,11)*PS34 + P(9,12)*PS6 + PS150*dt;
	nextP(3,9) = P(0,9)*PS13 + P(1,9)*PS12 - P(2,9)*PS11 + P(3,9) + P(9,10)*PS7 - P(9,11)*PS6 - P(9,12)*PS34 + PS170*dt;
	nextP(4,9) = P(0,9)*PS174 + P(1,9)*PS173 + P(2,9)*PS175 - P(3,9)*PS176 + P(4,9) + P(9,13)*PS43 + P(9,14)*PS172 - P(9,15)*PS171 + PS200*dt;
	nextP(5,9) = -P(0,9)*PS202 - P(1,9)*PS204 + P(2,9)*PS201 + P(3,9)*PS203 + P(5,9) - P(9,13)*PS193 + P(9,14)*PS75 + P(9,15)*PS190 + PS218*dt;
	nextP(6,9) = P(0,9)*PS216 + P(1,9)*PS217 - P(2,9)*PS214 + P(3,9)*PS215 + P(6,9) + P(9,13)*PS199 - P(9,14)*PS197 + P(9,15)*PS87 + PS222*dt;
	nextP(7,9) = P(4,9)*dt + P(7,9) + dt*(P(4,6)*dt + P(6,7));
	nextP(8,9) = P(5,9)*dt + P(8,9) + dt*(P(5,6)*dt + P(6,8));
	nextP(9,9) = P(6,9)*dt + P(9,9) + dt*(P(6,6)*dt + P(6,9));
	nextP(0,10) = PS14;
	nextP(1,10) = PS105;
	nextP(2,10) = PS133;
	nextP(3,10) = PS151;
	nextP(4,10) = P(0,10)*PS174 + P(1,10)*PS173 + P(10,13)*PS43 + P(10,14)*PS172 - P(10,15)*PS171 + P(2,10)*PS175 - P(3,10)*PS176 + P(4,10);
	nextP(5,10) = -P(0,10)*PS202 - P(1,10)*PS204 - P(10,13)*PS193 + P(10,14)*PS75 + P(10,15)*PS190 + P(2,10)*PS201 + P(3,10)*PS203 + P(5,10);
	nextP(6,10) = P(0,10)*PS216 + P(1,10)*PS217 + P(10,13)*PS199 - P(10,14)*PS197 + P(10,15)*PS87 - P(2,10)*PS214 + P(3,10)*PS215 + P(6,10);
	nextP(7,10) = P(4,10)*dt + P(7,10);
	nextP(8,10) = P(5,10)*dt + P(8,10);
	nextP(9,10) = P(6,10)*dt + P(9,10);
	nextP(10,10) = P(10,10);
	nextP(0,11) = PS17;
	nextP(1,11) = PS97;
	nextP(2,11) = PS132;
	nextP(3,11) = PS155;
	nextP(4,11) = P(0,11)*PS174 + P(1,11)*PS173 + P(11,13)*PS43 + P(11,14)*PS172 - P(11,15)*PS171 + P(2,11)*PS175 - P(3,11)*PS176 + P(4,11);
	nextP(5,11) = -P(0,11)*PS202 - P(1,11)*PS204 - P(11,13)*PS193 + P(11,14)*PS75 + P(11,15)*PS190 + P(2,11)*PS201 + P(3,11)*PS203 + P(5,11);
	nextP(6,11) = P(0,11)*PS216 + P(1,11)*PS217 + P(11,13)*PS199 - P(11,14)*PS197 + P(11,15)*PS87 - P(2,11)*PS214 + P(3,11)*PS215 + P(6,11);
	nextP(7,11) = P(4,11)*dt + P(7,11);
	nextP(8,11) = P(5,11)*dt + P(8,11);
	nextP(9,11) = P(6,11)*dt + P(9,11);
	nextP(10,11) = P(10,11);
	nextP(11,11) = P(11,11);
	nextP(0,12) = PS20;
	nextP(1,12) = PS107;
	nextP(2,12) = PS127;
	nextP(3,12) = PS154;
	nextP(4,12) = P(0,12)*PS174 + P(1,12)*PS173 + P(12,13)*PS43 + P(12,14)*PS172 - P(12,15)*PS171 + P(2,12)*PS175 - P(3,12)*PS176 + P(4,12);
	nextP(5,12) = -P(0,12)*PS202 - P(1,12)*PS204 - P(12,13)*PS193 + P(12,14)*PS75 + P(12,15)*PS190 + P(2,12)*PS201 + P(3,12)*PS203 + P(5,12);
	nextP(6,12) = P(0,12)*PS216 + P(1,12)*PS217 + P(12,13)*PS199 - P(12,14)*PS197 + P(12,15)*PS87 - P(2,12)*PS214 + P(3,12)*PS215 + P(6,12);
	nextP(7,12) = P(4,12)*dt + P(7,12);
	nextP(8,12) = P(5,12)*dt + P(8,12);
	nextP(9,12) = P(6,12)*dt + P(9,12);
	nextP(10,12) = P(10,12);
	nextP(11,12) = P(11,12);
	nextP(12,12) = P(12,12);

	// process noise contribution for delta angle states can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (unsigned i = 10; i <= 12; i++) {
		const int index = i - 10;
		nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_angle_bias_var_accum(index));
	}

	if (!_accel_bias_inhibit[0]) {
		// calculate variances and upper diagonal covariances for IMU X axis delta velocity bias state
		nextP(0,13) = PS44;
		nextP(1,13) = PS113;
		nextP(2,13) = PS138;
		nextP(3,13) = PS158;
		nextP(4,13) = PS177;
		nextP(5,13) = PS206;
		nextP(6,13) = PS221;
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
		nextP(13, 13) = kahanSummation(nextP(13, 13), process_noise(13), _delta_vel_bias_var_accum(0));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(13, _prev_dvel_bias_var(0));
		_delta_vel_bias_var_accum(0) = 0.f;

	}

	if (!_accel_bias_inhibit[1]) {
		// calculate variances and upper diagonal covariances for IMU Y axis delta velocity bias state

		nextP(0,14) = PS57;
		nextP(1,14) = PS117;
		nextP(2,14) = PS142;
		nextP(3,14) = PS162;
		nextP(4,14) = PS180;
		nextP(5,14) = PS205;
		nextP(6,14) = PS220;
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
		nextP(14, 14) = kahanSummation(nextP(14, 14), process_noise(14), _delta_vel_bias_var_accum(1));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(14, _prev_dvel_bias_var(1));
		_delta_vel_bias_var_accum(1) = 0.f;

	}

	if (!_accel_bias_inhibit[2]) {
		// calculate variances and upper diagonal covariances for IMU Z axis delta velocity bias state
		nextP(0,15) = PS46;
		nextP(1,15) = PS114;
		nextP(2,15) = PS139;
		nextP(3,15) = PS159;
		nextP(4,15) = PS178;
		nextP(5,15) = PS209;
		nextP(6,15) = PS219;
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
		nextP(15, 15) = kahanSummation(nextP(15, 15), process_noise(15), _delta_vel_bias_var_accum(2));

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
		nextP(4,16) = P(0,16)*PS174 + P(1,16)*PS173 + P(13,16)*PS43 + P(14,16)*PS172 - P(15,16)*PS171 + P(2,16)*PS175 - P(3,16)*PS176 + P(4,16);
		nextP(5,16) = -P(0,16)*PS202 - P(1,16)*PS204 - P(13,16)*PS193 + P(14,16)*PS75 + P(15,16)*PS190 + P(2,16)*PS201 + P(3,16)*PS203 + P(5,16);
		nextP(6,16) = P(0,16)*PS216 + P(1,16)*PS217 + P(13,16)*PS199 - P(14,16)*PS197 + P(15,16)*PS87 - P(2,16)*PS214 + P(3,16)*PS215 + P(6,16);
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
		nextP(4,17) = P(0,17)*PS174 + P(1,17)*PS173 + P(13,17)*PS43 + P(14,17)*PS172 - P(15,17)*PS171 + P(2,17)*PS175 - P(3,17)*PS176 + P(4,17);
		nextP(5,17) = -P(0,17)*PS202 - P(1,17)*PS204 - P(13,17)*PS193 + P(14,17)*PS75 + P(15,17)*PS190 + P(2,17)*PS201 + P(3,17)*PS203 + P(5,17);
		nextP(6,17) = P(0,17)*PS216 + P(1,17)*PS217 + P(13,17)*PS199 - P(14,17)*PS197 + P(15,17)*PS87 - P(2,17)*PS214 + P(3,17)*PS215 + P(6,17);
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
		nextP(4,18) = P(0,18)*PS174 + P(1,18)*PS173 + P(13,18)*PS43 + P(14,18)*PS172 - P(15,18)*PS171 + P(2,18)*PS175 - P(3,18)*PS176 + P(4,18);
		nextP(5,18) = -P(0,18)*PS202 - P(1,18)*PS204 - P(13,18)*PS193 + P(14,18)*PS75 + P(15,18)*PS190 + P(2,18)*PS201 + P(3,18)*PS203 + P(5,18);
		nextP(6,18) = P(0,18)*PS216 + P(1,18)*PS217 + P(13,18)*PS199 - P(14,18)*PS197 + P(15,18)*PS87 - P(2,18)*PS214 + P(3,18)*PS215 + P(6,18);
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
		nextP(4,19) = P(0,19)*PS174 + P(1,19)*PS173 + P(13,19)*PS43 + P(14,19)*PS172 - P(15,19)*PS171 + P(2,19)*PS175 - P(3,19)*PS176 + P(4,19);
		nextP(5,19) = -P(0,19)*PS202 - P(1,19)*PS204 - P(13,19)*PS193 + P(14,19)*PS75 + P(15,19)*PS190 + P(2,19)*PS201 + P(3,19)*PS203 + P(5,19);
		nextP(6,19) = P(0,19)*PS216 + P(1,19)*PS217 + P(13,19)*PS199 - P(14,19)*PS197 + P(15,19)*PS87 - P(2,19)*PS214 + P(3,19)*PS215 + P(6,19);
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
		nextP(4,20) = P(0,20)*PS174 + P(1,20)*PS173 + P(13,20)*PS43 + P(14,20)*PS172 - P(15,20)*PS171 + P(2,20)*PS175 - P(3,20)*PS176 + P(4,20);
		nextP(5,20) = -P(0,20)*PS202 - P(1,20)*PS204 - P(13,20)*PS193 + P(14,20)*PS75 + P(15,20)*PS190 + P(2,20)*PS201 + P(3,20)*PS203 + P(5,20);
		nextP(6,20) = P(0,20)*PS216 + P(1,20)*PS217 + P(13,20)*PS199 - P(14,20)*PS197 + P(15,20)*PS87 - P(2,20)*PS214 + P(3,20)*PS215 + P(6,20);
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
		nextP(4,21) = P(0,21)*PS174 + P(1,21)*PS173 + P(13,21)*PS43 + P(14,21)*PS172 - P(15,21)*PS171 + P(2,21)*PS175 - P(3,21)*PS176 + P(4,21);
		nextP(5,21) = -P(0,21)*PS202 - P(1,21)*PS204 - P(13,21)*PS193 + P(14,21)*PS75 + P(15,21)*PS190 + P(2,21)*PS201 + P(3,21)*PS203 + P(5,21);
		nextP(6,21) = P(0,21)*PS216 + P(1,21)*PS217 + P(13,21)*PS199 - P(14,21)*PS197 + P(15,21)*PS87 - P(2,21)*PS214 + P(3,21)*PS215 + P(6,21);
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
			nextP(i, i) += process_noise(i);
		}

	}

	// Don't do covariance prediction on wind states unless we are using them
	if (_control_status.flags.wind) {

		// calculate variances and upper diagonal covariances for wind states

		nextP(0,22) = P(0,22) - P(1,22)*PS11 + P(10,22)*PS6 + P(11,22)*PS7 + P(12,22)*PS9 - P(2,22)*PS12 - P(3,22)*PS13;
		nextP(1,22) = P(0,22)*PS11 + P(1,22) - P(10,22)*PS34 + P(11,22)*PS9 - P(12,22)*PS7 + P(2,22)*PS13 - P(3,22)*PS12;
		nextP(2,22) = P(0,22)*PS12 - P(1,22)*PS13 - P(10,22)*PS9 - P(11,22)*PS34 + P(12,22)*PS6 + P(2,22) + P(3,22)*PS11;
		nextP(3,22) = P(0,22)*PS13 + P(1,22)*PS12 + P(10,22)*PS7 - P(11,22)*PS6 - P(12,22)*PS34 - P(2,22)*PS11 + P(3,22);
		nextP(4,22) = P(0,22)*PS174 + P(1,22)*PS173 + P(13,22)*PS43 + P(14,22)*PS172 - P(15,22)*PS171 + P(2,22)*PS175 - P(3,22)*PS176 + P(4,22);
		nextP(5,22) = -P(0,22)*PS202 - P(1,22)*PS204 - P(13,22)*PS193 + P(14,22)*PS75 + P(15,22)*PS190 + P(2,22)*PS201 + P(3,22)*PS203 + P(5,22);
		nextP(6,22) = P(0,22)*PS216 + P(1,22)*PS217 + P(13,22)*PS199 - P(14,22)*PS197 + P(15,22)*PS87 - P(2,22)*PS214 + P(3,22)*PS215 + P(6,22);
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
		nextP(4,23) = P(0,23)*PS174 + P(1,23)*PS173 + P(13,23)*PS43 + P(14,23)*PS172 - P(15,23)*PS171 + P(2,23)*PS175 - P(3,23)*PS176 + P(4,23);
		nextP(5,23) = -P(0,23)*PS202 - P(1,23)*PS204 - P(13,23)*PS193 + P(14,23)*PS75 + P(15,23)*PS190 + P(2,23)*PS201 + P(3,23)*PS203 + P(5,23);
		nextP(6,23) = P(0,23)*PS216 + P(1,23)*PS217 + P(13,23)*PS199 - P(14,23)*PS197 + P(15,23)*PS87 - P(2,23)*PS214 + P(3,23)*PS215 + P(6,23);
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
			nextP(i, i) += process_noise(i);
		}

	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P(7, 7) + P(8, 8)) > 1e4f) {
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P(row, column) = P(column, row) = nextP(column, row);
		}
	}

	// copy variances (diagonals)
	for (unsigned i = 0; i < _k_num_states; i++) {
		P(i, i) = nextP(i, i);
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
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[0]);
	}

	for (int i = 4; i <= 6; i++) {
		// NED velocity states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[1]);
	}

	for (int i = 7; i <= 9; i++) {
		// NED position states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[2]);
	}

	for (int i = 10; i <= 12; i++) {
		// gyro bias states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[3]);
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

			if (P(stateIndex, stateIndex) > maxStateVar) {
				maxStateVar = P(stateIndex, stateIndex);

			} else if (P(stateIndex, stateIndex) < minSafeStateVar) {
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

			P(stateIndex, stateIndex) = math::constrain(P(stateIndex, stateIndex), minAllowedStateVar,
						    sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
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
				     && ((down_dvel_bias * _gps_vel_innov(2) < 0.0f && _control_status.flags.gps)
					 || (down_dvel_bias * _ev_vel_innov(2) < 0.0f && _control_status.flags.ev_vel))
				     && ((down_dvel_bias * _gps_pos_innov(2) < 0.0f && _control_status.flags.gps_hgt)
					 || (down_dvel_bias * _baro_hgt_innov(2) < 0.0f && _control_status.flags.baro_hgt)
					 || (down_dvel_bias * _rng_hgt_innov(2) < 0.0f && _control_status.flags.rng_hgt)
					 || (down_dvel_bias * _ev_pos_innov(2) < 0.0f && _control_status.flags.ev_hgt)));

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
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[5]);
		}

		for (int i = 19; i <= 21; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[6]);
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
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[7]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<2>(22);
		}
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP)
{
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

void Ekf::resetQuatCov()
{
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

		P(22, 22) = R_TAS * sq(cos_yaw) + R_yaw * sq(-Wx * sin_yaw - Wy * cos_yaw) + initial_wind_var_body_y * sq(sin_yaw);
		P(22, 23) = R_TAS * sin_yaw * cos_yaw + R_yaw * (-Wx * sin_yaw - Wy * cos_yaw) * (Wx * cos_yaw - Wy * sin_yaw) -
			    initial_wind_var_body_y * sin_yaw * cos_yaw;
		P(23, 22) = P(22, 23);
		P(23, 23) = R_TAS * sq(sin_yaw) + R_yaw * sq(Wx * cos_yaw - Wy * sin_yaw) + initial_wind_var_body_y * sq(cos_yaw);

		// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
		P(22, 22) += P(4, 4);
		P(23, 23) += P(5, 5);

	} else {
		// without airspeed, start with a small initial uncertainty to improve the initial estimate
		P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
	}
}
