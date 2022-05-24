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
 * @file heading_fusion.cpp
 * Magnetometer fusion methods.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::fuseMag(const Vector3f &mag)
{
	// assign intermediate variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float magN = _state.mag_I(0);
	const float magE = _state.mag_I(1);
	const float magD = _state.mag_I(2);

	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	const float R_MAG = sq(fmaxf(_params.mag_noise, 0.0f));

	// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
	const char* numerical_error_covariance_reset_string = "numerical error - covariance reset";
	const float HKX0 = -magD*q2 + magE*q3 + magN*q0;
	const float HKX1 = magD*q3 + magE*q2 + magN*q1;
	const float HKX2 = magE*q1;
	const float HKX3 = magD*q0;
	const float HKX4 = magN*q2;
	const float HKX5 = magD*q1 + magE*q0 - magN*q3;
	const float HKX6 = ecl::powf(q0, 2) + ecl::powf(q1, 2) - ecl::powf(q2, 2) - ecl::powf(q3, 2);
	const float HKX7 = q0*q3 + q1*q2;
	const float HKX8 = q1*q3;
	const float HKX9 = q0*q2;
	const float HKX10 = 2*HKX7;
	const float HKX11 = -2*HKX8 + 2*HKX9;
	const float HKX12 = 2*HKX1;
	const float HKX13 = 2*HKX0;
	const float HKX14 = -2*HKX2 + 2*HKX3 + 2*HKX4;
	const float HKX15 = 2*HKX5;
	const float HKX16 = HKX10*P(0,17) - HKX11*P(0,18) + HKX12*P(0,1) + HKX13*P(0,0) - HKX14*P(0,2) + HKX15*P(0,3) + HKX6*P(0,16) + P(0,19);
	const float HKX17 = HKX10*P(16,17) - HKX11*P(16,18) + HKX12*P(1,16) + HKX13*P(0,16) - HKX14*P(2,16) + HKX15*P(3,16) + HKX6*P(16,16) + P(16,19);
	const float HKX18 = HKX10*P(17,18) - HKX11*P(18,18) + HKX12*P(1,18) + HKX13*P(0,18) - HKX14*P(2,18) + HKX15*P(3,18) + HKX6*P(16,18) + P(18,19);
	const float HKX19 = HKX10*P(2,17) - HKX11*P(2,18) + HKX12*P(1,2) + HKX13*P(0,2) - HKX14*P(2,2) + HKX15*P(2,3) + HKX6*P(2,16) + P(2,19);
	const float HKX20 = HKX10*P(17,17) - HKX11*P(17,18) + HKX12*P(1,17) + HKX13*P(0,17) - HKX14*P(2,17) + HKX15*P(3,17) + HKX6*P(16,17) + P(17,19);
	const float HKX21 = HKX10*P(3,17) - HKX11*P(3,18) + HKX12*P(1,3) + HKX13*P(0,3) - HKX14*P(2,3) + HKX15*P(3,3) + HKX6*P(3,16) + P(3,19);
	const float HKX22 = HKX10*P(1,17) - HKX11*P(1,18) + HKX12*P(1,1) + HKX13*P(0,1) - HKX14*P(1,2) + HKX15*P(1,3) + HKX6*P(1,16) + P(1,19);
	const float HKX23 = HKX10*P(17,19) - HKX11*P(18,19) + HKX12*P(1,19) + HKX13*P(0,19) - HKX14*P(2,19) + HKX15*P(3,19) + HKX6*P(16,19) + P(19,19);

	_mag_innov_var(0) = HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG;

	if (_mag_innov_var(0) < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_x = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magX %s", numerical_error_covariance_reset_string);
		return;
	}

	_fault_status.flags.bad_mag_x = false;

	const float HKX24 = 1.0F/_mag_innov_var(0);

	// intermediate variables for calculation of innovations variances for Y and Z axes
	// don't calculate all terms needed for observation jacobians and Kalman gains because
	// these will have to be recalculated when the X and Y axes are fused
	const float IV0 = q0*q1;
	const float IV1 = q2*q3;
	const float IV2 = 2*IV0 + 2*IV1;
	const float IV3 = 2*q0*q3 - 2*q1*q2;
	const float IV4 = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
	const float IV5 = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
	const float IV6 = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
	const float IV7 = -2*magD*q2 + 2*magE*q3 + 2*magN*q0;
	const float IV8 = ecl::powf(q2, 2);
	const float IV9 = ecl::powf(q3, 2);
	const float IV10 = ecl::powf(q0, 2) - ecl::powf(q1, 2);
	const float IV11 = IV10 + IV8 - IV9;
	const float IV12 = IV7*P(2,3);
	const float IV13 = IV5*P(0,1);
	const float IV14 = IV6*P(0,1);
	const float IV15 = IV4*P(2,3);
	const float IV16 = 2*q0*q2 + 2*q1*q3;
	const float IV17 = 2*IV0 - 2*IV1;
	const float IV18 = IV10 - IV8 + IV9;

	_mag_innov_var(1) = IV11*P(17,20) + IV11*(IV11*P(17,17) + IV2*P(17,18) - IV3*P(16,17) + IV4*P(2,17) + IV5*P(0,17) + IV6*P(1,17) - IV7*P(3,17) + P(17,20)) + IV2*P(18,20) + IV2*(IV11*P(17,18) + IV2*P(18,18) - IV3*P(16,18) + IV4*P(2,18) + IV5*P(0,18) + IV6*P(1,18) - IV7*P(3,18) + P(18,20)) - IV3*P(16,20) - IV3*(IV11*P(16,17) + IV2*P(16,18) - IV3*P(16,16) + IV4*P(2,16) + IV5*P(0,16) + IV6*P(1,16) - IV7*P(3,16) + P(16,20)) + IV4*P(2,20) + IV4*(IV11*P(2,17) - IV12 + IV2*P(2,18) - IV3*P(2,16) + IV4*P(2,2) + IV5*P(0,2) + IV6*P(1,2) + P(2,20)) + IV5*P(0,20) + IV5*(IV11*P(0,17) + IV14 + IV2*P(0,18) - IV3*P(0,16) + IV4*P(0,2) + IV5*P(0,0) - IV7*P(0,3) + P(0,20)) + IV6*P(1,20) + IV6*(IV11*P(1,17) + IV13 + IV2*P(1,18) - IV3*P(1,16) + IV4*P(1,2) + IV6*P(1,1) - IV7*P(1,3) + P(1,20)) - IV7*P(3,20) - IV7*(IV11*P(3,17) + IV15 + IV2*P(3,18) - IV3*P(3,16) + IV5*P(0,3) + IV6*P(1,3) - IV7*P(3,3) + P(3,20)) + P(20,20) + R_MAG;
	_mag_innov_var(2) = IV16*P(16,21) + IV16*(IV16*P(16,16) - IV17*P(16,17) + IV18*P(16,18) + IV4*P(3,16) - IV5*P(1,16) + IV6*P(0,16) + IV7*P(2,16) + P(16,21)) - IV17*P(17,21) - IV17*(IV16*P(16,17) - IV17*P(17,17) + IV18*P(17,18) + IV4*P(3,17) - IV5*P(1,17) + IV6*P(0,17) + IV7*P(2,17) + P(17,21)) + IV18*P(18,21) + IV18*(IV16*P(16,18) - IV17*P(17,18) + IV18*P(18,18) + IV4*P(3,18) - IV5*P(1,18) + IV6*P(0,18) + IV7*P(2,18) + P(18,21)) + IV4*P(3,21) + IV4*(IV12 + IV16*P(3,16) - IV17*P(3,17) + IV18*P(3,18) + IV4*P(3,3) - IV5*P(1,3) + IV6*P(0,3) + P(3,21)) - IV5*P(1,21) - IV5*(IV14 + IV16*P(1,16) - IV17*P(1,17) + IV18*P(1,18) + IV4*P(1,3) - IV5*P(1,1) + IV7*P(1,2) + P(1,21)) + IV6*P(0,21) + IV6*(-IV13 + IV16*P(0,16) - IV17*P(0,17) + IV18*P(0,18) + IV4*P(0,3) + IV6*P(0,0) + IV7*P(0,2) + P(0,21)) + IV7*P(2,21) + IV7*(IV15 + IV16*P(2,16) - IV17*P(2,17) + IV18*P(2,18) - IV5*P(1,2) + IV6*P(0,2) + IV7*P(2,2) + P(2,21)) + P(21,21) + R_MAG;

	// chedk innovation variances for being badly conditioned

	if (_mag_innov_var(1) < R_MAG) {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_y = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magY %s", numerical_error_covariance_reset_string);
		return;
	}

	_fault_status.flags.bad_mag_y = false;

	if (_mag_innov_var(2) < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_z = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
		return;
	}

	_fault_status.flags.bad_mag_z = false;

	// rotate magnetometer earth field state into body frame
	const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);

	const Vector3f mag_I_rot = R_to_body * _state.mag_I;

	// compute magnetometer innovations
	_mag_innov = mag_I_rot + _state.mag_B - mag;

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		_mag_innov(2) = 0.0f;
	}

	// Perform an innovation consistency check and report the result
	bool all_innovation_checks_passed = true;

	for (uint8_t index = 0; index <= 2; index++) {
		_mag_test_ratio(index) = sq(_mag_innov(index)) / (sq(math::max(_params.mag_innov_gate, 1.0f)) * _mag_innov_var(index));

		const bool innov_check_fail = (_mag_test_ratio(index) > 1.0f);

		if (innov_check_fail) {
			all_innovation_checks_passed = false;
		}

		if (index == 0) {
			_innov_check_fail_status.flags.reject_mag_x = innov_check_fail;

		} else if (index == 1) {
			_innov_check_fail_status.flags.reject_mag_y = innov_check_fail;

		} else {
			_innov_check_fail_status.flags.reject_mag_z = innov_check_fail;
		}
	}

	// we are no longer using heading fusion so set the reported test level to zero
	_yaw_test_ratio = 0.0f;

	// if any axis fails, abort the mag fusion
	if (!all_innovation_checks_passed) {
		return;
	}

	// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
	// before they are used to constrain heading drift
	const bool update_all_states = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) > (uint64_t)5e6);

	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3,16,17,18,19,20,21> Hfusion;
	Vector24f Kfusion;

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {

		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// Calculate X axis observation jacobians
			Hfusion.at<0>() = 2*HKX0;
			Hfusion.at<1>() = 2*HKX1;
			Hfusion.at<2>() = 2*HKX2 - 2*HKX3 - 2*HKX4;
			Hfusion.at<3>() = 2*HKX5;
			Hfusion.at<16>() = HKX6;
			Hfusion.at<17>() = 2*HKX7;
			Hfusion.at<18>() = 2*HKX8 - 2*HKX9;
			Hfusion.at<19>() = 1;

			// Calculate X axis Kalman gains
			if (update_all_states) {
				Kfusion(0) = HKX16*HKX24;
				Kfusion(1) = HKX22*HKX24;
				Kfusion(2) = HKX19*HKX24;
				Kfusion(3) = HKX21*HKX24;

				for (unsigned row = 4; row <= 15; row++) {
					Kfusion(row) = HKX24*(HKX10*P(row,17) - HKX11*P(row,18) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(row,16) + P(row,19));
				}

				for (unsigned row = 22; row <= 23; row++) {
					Kfusion(row) = HKX24*(HKX10*P(17,row) - HKX11*P(18,row) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(16,row) + P(19,row));
				}
			}

			Kfusion(16) = HKX17*HKX24;
			Kfusion(17) = HKX20*HKX24;
			Kfusion(18) = HKX18*HKX24;
			Kfusion(19) = HKX23*HKX24;

			for (unsigned row = 20; row <= 21; row++) {
				Kfusion(row) = HKX24*(HKX10*P(17,row) - HKX11*P(18,row) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(16,row) + P(19,row));
			}

		} else if (index == 1) {

			// recalculate innovation variance becasue states and covariances have changed due to previous fusion
			const float HKY0 = magD*q1 + magE*q0 - magN*q3;
			const float HKY1 = magD*q0 - magE*q1 + magN*q2;
			const float HKY2 = magD*q3 + magE*q2 + magN*q1;
			const float HKY3 = magD*q2;
			const float HKY4 = magE*q3;
			const float HKY5 = magN*q0;
			const float HKY6 = q1*q2;
			const float HKY7 = q0*q3;
			const float HKY8 = ecl::powf(q0, 2) - ecl::powf(q1, 2) + ecl::powf(q2, 2) - ecl::powf(q3, 2);
			const float HKY9 = q0*q1 + q2*q3;
			const float HKY10 = 2*HKY9;
			const float HKY11 = -2*HKY6 + 2*HKY7;
			const float HKY12 = 2*HKY2;
			const float HKY13 = 2*HKY0;
			const float HKY14 = 2*HKY1;
			const float HKY15 = -2*HKY3 + 2*HKY4 + 2*HKY5;
			const float HKY16 = HKY10*P(0,18) - HKY11*P(0,16) + HKY12*P(0,2) + HKY13*P(0,0) + HKY14*P(0,1) - HKY15*P(0,3) + HKY8*P(0,17) + P(0,20);
			const float HKY17 = HKY10*P(17,18) - HKY11*P(16,17) + HKY12*P(2,17) + HKY13*P(0,17) + HKY14*P(1,17) - HKY15*P(3,17) + HKY8*P(17,17) + P(17,20);
			const float HKY18 = HKY10*P(16,18) - HKY11*P(16,16) + HKY12*P(2,16) + HKY13*P(0,16) + HKY14*P(1,16) - HKY15*P(3,16) + HKY8*P(16,17) + P(16,20);
			const float HKY19 = HKY10*P(3,18) - HKY11*P(3,16) + HKY12*P(2,3) + HKY13*P(0,3) + HKY14*P(1,3) - HKY15*P(3,3) + HKY8*P(3,17) + P(3,20);
			const float HKY20 = HKY10*P(18,18) - HKY11*P(16,18) + HKY12*P(2,18) + HKY13*P(0,18) + HKY14*P(1,18) - HKY15*P(3,18) + HKY8*P(17,18) + P(18,20);
			const float HKY21 = HKY10*P(1,18) - HKY11*P(1,16) + HKY12*P(1,2) + HKY13*P(0,1) + HKY14*P(1,1) - HKY15*P(1,3) + HKY8*P(1,17) + P(1,20);
			const float HKY22 = HKY10*P(2,18) - HKY11*P(2,16) + HKY12*P(2,2) + HKY13*P(0,2) + HKY14*P(1,2) - HKY15*P(2,3) + HKY8*P(2,17) + P(2,20);
			const float HKY23 = HKY10*P(18,20) - HKY11*P(16,20) + HKY12*P(2,20) + HKY13*P(0,20) + HKY14*P(1,20) - HKY15*P(3,20) + HKY8*P(17,20) + P(20,20);

			_mag_innov_var(1) = (HKY10*HKY20 - HKY11*HKY18 + HKY12*HKY22 + HKY13*HKY16 + HKY14*HKY21 - HKY15*HKY19 + HKY17*HKY8 + HKY23 + R_MAG);

			if (_mag_innov_var(1) < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_y = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magY %s", numerical_error_covariance_reset_string);
				return;
			}
			const float HKY24 = 1.0F/_mag_innov_var(1);

			// Calculate Y axis observation jacobians
			Hfusion.setZero();
			Hfusion.at<0>() = 2*HKY0;
			Hfusion.at<1>() = 2*HKY1;
			Hfusion.at<2>() = 2*HKY2;
			Hfusion.at<3>() = 2*HKY3 - 2*HKY4 - 2*HKY5;
			Hfusion.at<16>() = 2*HKY6 - 2*HKY7;
			Hfusion.at<17>() = HKY8;
			Hfusion.at<18>() = 2*HKY9;
			Hfusion.at<20>() = 1;

			// Calculate Y axis Kalman gains
			if (update_all_states) {
				Kfusion(0) = HKY16*HKY24;
				Kfusion(1) = HKY21*HKY24;
				Kfusion(2) = HKY22*HKY24;
				Kfusion(3) = HKY19*HKY24;

				for (unsigned row = 4; row <= 15; row++) {
					Kfusion(row) = HKY24*(HKY10*P(row,18) - HKY11*P(row,16) + HKY12*P(2,row) + HKY13*P(0,row) + HKY14*P(1,row) - HKY15*P(3,row) + HKY8*P(row,17) + P(row,20));
				}

				for (unsigned row = 22; row <= 23; row++) {
					Kfusion(row) = HKY24*(HKY10*P(18,row) - HKY11*P(16,row) + HKY12*P(2,row) + HKY13*P(0,row) + HKY14*P(1,row) - HKY15*P(3,row) + HKY8*P(17,row) + P(20,row));
				}
			}

			Kfusion(16) = HKY18*HKY24;
			Kfusion(17) = HKY17*HKY24;
			Kfusion(18) = HKY20*HKY24;
			Kfusion(19) = HKY24*(HKY10*P(18,19) - HKY11*P(16,19) + HKY12*P(2,19) + HKY13*P(0,19) + HKY14*P(1,19) - HKY15*P(3,19) + HKY8*P(17,19) + P(19,20));
			Kfusion(20) = HKY23*HKY24;
			Kfusion(21) = HKY24*(HKY10*P(18,21) - HKY11*P(16,21) + HKY12*P(2,21) + HKY13*P(0,21) + HKY14*P(1,21) - HKY15*P(3,21) + HKY8*P(17,21) + P(20,21));

		} else if (index == 2) {

			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				continue;
			}

			// recalculate innovation variance becasue states and covariances have changed due to previous fusion
			const float HKZ0 = magD*q0 - magE*q1 + magN*q2;
			const float HKZ1 = magN*q3;
			const float HKZ2 = magD*q1;
			const float HKZ3 = magE*q0;
			const float HKZ4 = -magD*q2 + magE*q3 + magN*q0;
			const float HKZ5 = magD*q3 + magE*q2 + magN*q1;
			const float HKZ6 = q0*q2 + q1*q3;
			const float HKZ7 = q2*q3;
			const float HKZ8 = q0*q1;
			const float HKZ9 = ecl::powf(q0, 2) - ecl::powf(q1, 2) - ecl::powf(q2, 2) + ecl::powf(q3, 2);
			const float HKZ10 = 2*HKZ6;
			const float HKZ11 = -2*HKZ7 + 2*HKZ8;
			const float HKZ12 = 2*HKZ5;
			const float HKZ13 = 2*HKZ0;
			const float HKZ14 = -2*HKZ1 + 2*HKZ2 + 2*HKZ3;
			const float HKZ15 = 2*HKZ4;
			const float HKZ16 = HKZ10*P(0,16) - HKZ11*P(0,17) + HKZ12*P(0,3) + HKZ13*P(0,0) - HKZ14*P(0,1) + HKZ15*P(0,2) + HKZ9*P(0,18) + P(0,21);
			const float HKZ17 = HKZ10*P(16,18) - HKZ11*P(17,18) + HKZ12*P(3,18) + HKZ13*P(0,18) - HKZ14*P(1,18) + HKZ15*P(2,18) + HKZ9*P(18,18) + P(18,21);
			const float HKZ18 = HKZ10*P(16,17) - HKZ11*P(17,17) + HKZ12*P(3,17) + HKZ13*P(0,17) - HKZ14*P(1,17) + HKZ15*P(2,17) + HKZ9*P(17,18) + P(17,21);
			const float HKZ19 = HKZ10*P(1,16) - HKZ11*P(1,17) + HKZ12*P(1,3) + HKZ13*P(0,1) - HKZ14*P(1,1) + HKZ15*P(1,2) + HKZ9*P(1,18) + P(1,21);
			const float HKZ20 = HKZ10*P(16,16) - HKZ11*P(16,17) + HKZ12*P(3,16) + HKZ13*P(0,16) - HKZ14*P(1,16) + HKZ15*P(2,16) + HKZ9*P(16,18) + P(16,21);
			const float HKZ21 = HKZ10*P(3,16) - HKZ11*P(3,17) + HKZ12*P(3,3) + HKZ13*P(0,3) - HKZ14*P(1,3) + HKZ15*P(2,3) + HKZ9*P(3,18) + P(3,21);
			const float HKZ22 = HKZ10*P(2,16) - HKZ11*P(2,17) + HKZ12*P(2,3) + HKZ13*P(0,2) - HKZ14*P(1,2) + HKZ15*P(2,2) + HKZ9*P(2,18) + P(2,21);
			const float HKZ23 = HKZ10*P(16,21) - HKZ11*P(17,21) + HKZ12*P(3,21) + HKZ13*P(0,21) - HKZ14*P(1,21) + HKZ15*P(2,21) + HKZ9*P(18,21) + P(21,21);

			_mag_innov_var(2) = (HKZ10*HKZ20 - HKZ11*HKZ18 + HKZ12*HKZ21 + HKZ13*HKZ16 - HKZ14*HKZ19 + HKZ15*HKZ22 + HKZ17*HKZ9 + HKZ23 + R_MAG);

			if (_mag_innov_var(2) < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_z = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
				return;
			}

			const float HKZ24 = 1.0F/_mag_innov_var(2);

			// calculate Z axis observation jacobians
			Hfusion.setZero();
			Hfusion.at<0>() = 2*HKZ0;
			Hfusion.at<1>() = 2*HKZ1 - 2*HKZ2 - 2*HKZ3;
			Hfusion.at<2>() = 2*HKZ4;
			Hfusion.at<3>() = 2*HKZ5;
			Hfusion.at<16>() = 2*HKZ6;
			Hfusion.at<17>() = 2*HKZ7 - 2*HKZ8;
			Hfusion.at<18>() = HKZ9;
			Hfusion.at<21>() = 1;

			// Calculate Z axis Kalman gains
			if (update_all_states) {
				Kfusion(0) = HKZ16*HKZ24;
				Kfusion(1) = HKZ19*HKZ24;
				Kfusion(2) = HKZ22*HKZ24;
				Kfusion(3) = HKZ21*HKZ24;

				for (unsigned row = 4; row <= 15; row++) {
					Kfusion(row) = HKZ24*(HKZ10*P(row,16) - HKZ11*P(row,17) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(row,18) + P(row,21));
				}

				for (unsigned row = 22; row <= 23; row++) {
					Kfusion(row) = HKZ24*(HKZ10*P(16,row) - HKZ11*P(17,row) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(18,row) + P(21,row));
				}
			}

			Kfusion(16) = HKZ20*HKZ24;
			Kfusion(17) = HKZ18*HKZ24;
			Kfusion(18) = HKZ17*HKZ24;

			for (unsigned row = 19; row <= 20; row++) {
				Kfusion(row) = HKZ24*(HKZ10*P(16,row) - HKZ11*P(17,row) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(18,row) + P(row,21));
			}

			Kfusion(21) = HKZ23*HKZ24;
		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _mag_innov(index));

		switch (index) {
		case 0:
			_fault_status.flags.bad_mag_x = !is_fused;
			break;

		case 1:
			_fault_status.flags.bad_mag_y = !is_fused;
			break;

		case 2:
			_fault_status.flags.bad_mag_z = !is_fused;
			break;
		}

		if (is_fused) {
			limitDeclination();
		}
	}
}

bool Ekf::fuseYaw321(float yaw, float yaw_variance, bool zero_innovation)
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float R_YAW = fmaxf(yaw_variance, 1.0e-4f);
	const float measurement = wrap_pi(yaw);

	// calculate 321 yaw observation matrix
	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	const float SA0 = 2*q3;
	const float SA1 = 2*q2;
	const float SA2 = SA0*q0 + SA1*q1;
	const float SA3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	float SA4, SA5_inv;

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.0F/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	bool canUseB = false;
	const float SB0 = 2*q0;
	const float SB1 = 2*q1;
	const float SB2 = SB0*q3 + SB1*q2;
	const float SB4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	float SB3, SB5_inv;

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.0F/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.0F/SA5_inv;
		const float SA6 = 1.0F/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
		H_YAW(1) = SA5*(SA1*SA6 - SA8*q1);
		H_YAW(2) = SA5*(SA1*SA7 + SA9*q1);
		H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.0F/SB5_inv;
		const float SB6 = 1.0F/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		H_YAW(0) = -SB5*(SB0*SB6 - SB8*q3);
		H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
		H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
		H_YAW(3) = -SB5*(-SB0*SB7 - SB9*q3);
	} else {
		return false;
	}

	// calculate the yaw innovation and wrap to the interval between +-pi
	float innovation;

	if (zero_innovation) {
		innovation = 0.0f;

	} else {
		innovation = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - measurement);
	}

	// define the innovation gate size
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);

	// Update the quaternion states and covariance matrix
	return updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

bool Ekf::fuseYaw312(float yaw, float yaw_variance, bool zero_innovation)
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float R_YAW = fmaxf(yaw_variance, 1.0e-4f);
	const float measurement = wrap_pi(yaw);

	// calculate 312 yaw observation matrix
	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	const float SA0 = 2*q3;
	const float SA1 = 2*q2;
	const float SA2 = SA0*q0 - SA1*q1;
	const float SA3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
	float SA4, SA5_inv;

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.0F/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	bool canUseB = false;
	const float SB0 = 2*q0;
	const float SB1 = 2*q1;
	const float SB2 = -SB0*q3 + SB1*q2;
	const float SB4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
	float SB3, SB5_inv;

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.0F/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.0F/SA5_inv;
		const float SA6 = 1.0F/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
		H_YAW(1) = SA5*(-SA1*SA6 + SA8*q1);
		H_YAW(2) = SA5*(-SA1*SA7 - SA9*q1);
		H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.0F/SB5_inv;
		const float SB6 = 1.0F/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		H_YAW(0) = -SB5*(-SB0*SB6 + SB8*q3);
		H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
		H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
		H_YAW(3) = -SB5*(SB0*SB7 + SB9*q3);
	} else {
		return false;
	}

	float innovation;

	if (zero_innovation) {
		innovation = 0.0f;

	} else {
		// calculate the the innovation and wrap to the interval between +-pi
		innovation = wrap_pi(atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1)) - measurement);
	}

	// define the innovation gate size
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);

	// Update the quaternion states and covariance matrix
	return updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

// update quaternion states and covariances using the yaw innovation, yaw observation variance and yaw Jacobian
bool Ekf::updateQuaternion(const float innovation, const float variance, const float gate_sigma,
			   const Vector4f &yaw_jacobian)
{
	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
	// calculate the innovation variance
	_heading_innov_var = variance;

	for (unsigned row = 0; row <= 3; row++) {
		float tmp = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			tmp += P(row, col) * yaw_jacobian(col);
		}

		_heading_innov_var += yaw_jacobian(row) * tmp;
	}

	float heading_innov_var_inv;

	// check if the innovation variance calculation is badly conditioned
	if (_heading_innov_var >= variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.0f / _heading_innov_var;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("mag yaw fusion numerical error - covariance reset");
		return false;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;

	for (uint8_t row = 0; row <= 15; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * yaw_jacobian(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row, col) * yaw_jacobian(col);
			}

			Kfusion(row) *= heading_innov_var_inv;
		}
	}

	// innovation test ratio
	_yaw_test_ratio = sq(innovation) / (sq(gate_sigma) * _heading_innov_var);

	// we are no longer using 3-axis fusion so set the reported test levels to zero
	_mag_test_ratio.setZero();

	// set the magnetometer unhealthy if the test fails
	if (_yaw_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air && isTimedOut(_time_last_in_air, (uint64_t)5e6)) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			float gate_limit = sqrtf((sq(gate_sigma) * _heading_innov_var));
			_heading_innov = math::constrain(innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			resetZDeltaAngBiasCov();

		} else {
			return false;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
		_heading_innov = innovation;
	}

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	SquareMatrix24f KHP;
	float KH[4];

	for (unsigned row = 0; row < _k_num_states; row++) {

		KH[0] = Kfusion(row) * yaw_jacobian(0);
		KH[1] = Kfusion(row) * yaw_jacobian(1);
		KH[2] = Kfusion(row) * yaw_jacobian(2);
		KH[3] = Kfusion(row) * yaw_jacobian(3);

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(0, column);
			tmp += KH[1] * P(1, column);
			tmp += KH[2] * P(2, column);
			tmp += KH[3] * P(3, column);
			KHP(row, column) = tmp;
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	_fault_status.flags.bad_hdg = !healthy;

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, _heading_innov);

		return true;
	}

	return false;
}

void Ekf::fuseHeading(float measured_hdg, float obs_var)
{
	// observation variance
	float R_YAW = PX4_ISFINITE(obs_var) ? obs_var : 0.01f;

	// update transformation matrix from body to world frame using the current state estimate
	const float predicted_hdg = getEulerYaw(_R_to_earth);

	if (!PX4_ISFINITE(measured_hdg)) {
		measured_hdg = predicted_hdg;
	}

	// handle special case where yaw measurement is unavailable
	bool fuse_zero_innov = false;

	if (_is_yaw_fusion_inhibited) {
		// The yaw measurement cannot be trusted but we need to fuse something to prevent a badly
		// conditioned covariance matrix developing over time.
		if (!_control_status.flags.vehicle_at_rest) {
			// Vehicle is not at rest so fuse a zero innovation if necessary to prevent
			// unconstrained quaternion variance growth and record the predicted heading
			// to use as an observation when movement ceases.
			// TODO a better way of determining when this is necessary
			const float sumQuatVar = P(0, 0) + P(1, 1) + P(2, 2) + P(3, 3);

			if (sumQuatVar > _params.quat_max_variance) {
				fuse_zero_innov = true;
				R_YAW = 0.25f;
			}

			_last_static_yaw = predicted_hdg;

		} else {
			// Vehicle is at rest so use the last moving prediction as an observation
			// to prevent the heading from drifting and to enable yaw gyro bias learning
			// before takeoff.
			if (!PX4_ISFINITE(_last_static_yaw)) {
				_last_static_yaw = predicted_hdg;
			}

			measured_hdg = _last_static_yaw;
		}

	} else {
		_last_static_yaw = predicted_hdg;
	}

	if (shouldUse321RotationSequence(_R_to_earth)) {
		fuseYaw321(measured_hdg, R_YAW, fuse_zero_innov);

	} else {
		fuseYaw312(measured_hdg, R_YAW, fuse_zero_innov);
	}
}

void Ekf::fuseDeclination(float decl_sigma)
{
	// assign intermediate state variables
	const float magN = _state.mag_I(0);
	const float magE = _state.mag_I(1);

	// minimum North field strength before calculation becomes badly conditioned (T)
	constexpr float N_field_min = 0.001f;

	// observation variance (rad**2)
	const float R_DECL = sq(decl_sigma);

	// Calculate intermediate variables
	if (fabsf(magN) < sq(N_field_min)) {
		// calculation is badly conditioned close to +-90 deg declination
		return;
	}

	const float HK0 = ecl::powf(magN, -2);
	const float HK1 = HK0*ecl::powf(magE, 2) + 1.0F;
	const float HK2 = 1.0F/HK1;
	const float HK3 = 1.0F/magN;
	const float HK4 = HK2*HK3;
	const float HK5 = HK3*magE;
	const float HK6 = HK5*P(16,17) - P(17,17);
	const float HK7 = ecl::powf(HK1, -2);
	const float HK8 = HK5*P(16,16) - P(16,17);
	const float innovation_variance = -HK0*HK6*HK7 + HK7*HK8*magE/ecl::powf(magN, 3) + R_DECL;
	float HK9;

	if (innovation_variance > R_DECL) {
		HK9 = HK4/innovation_variance;
	} else {
		// variance calculation is badly conditioned
		return;
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	// Note Hfusion indices do not match state indices
	SparseVector24f<16,17> Hfusion;
	Hfusion.at<16>() = -HK0*HK2*magE;
	Hfusion.at<17>() = HK4;

	// Calculate the Kalman gains
	Vector24f Kfusion;

	for (unsigned row = 0; row <= 15; row++) {
		Kfusion(row) = -HK9*(HK5*P(row,16) - P(row,17));
	}

	Kfusion(16) = -HK8*HK9;
	Kfusion(17) = -HK6*HK9;

	for (unsigned row = 18; row <= 23; row++) {
		Kfusion(row) = -HK9*(HK5*P(16,row) - P(17,row));
	}

	const float innovation = math::constrain(atan2f(magE, magN) - getMagDeclination(), -0.5f, 0.5f);

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, innovation);

	_fault_status.flags.bad_mag_decl = !is_fused;

	if (is_fused) {
		limitDeclination();
	}
}

void Ekf::limitDeclination()
{
	// get a reference value for the earth field declinaton and minimum plausible horizontal field strength
	// set to 50% of the horizontal strength from geo tables if location is known
	float decl_reference;
	float h_field_min = 0.001f;

	if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			decl_reference = _mag_declination_gps;
			h_field_min = fmaxf(h_field_min, 0.5f * _mag_strength_gps * cosf(_mag_inclination_gps));

		} else {
			decl_reference = math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		decl_reference = math::radians(_params.mag_declination_deg);
	}

	// do not allow the horizontal field length to collapse - this will make the declination fusion badly conditioned
	// and can result in a reversal of the NE field states which the filter cannot recover from
	// apply a circular limit
	float h_field = sqrtf(_state.mag_I(0) * _state.mag_I(0) + _state.mag_I(1) * _state.mag_I(1));

	if (h_field < h_field_min) {
		if (h_field > 0.001f * h_field_min) {
			const float h_scaler = h_field_min / h_field;
			_state.mag_I(0) *= h_scaler;
			_state.mag_I(1) *= h_scaler;

		} else {
			// too small to scale radially so set to expected value
			const float mag_declination = getMagDeclination();
			_state.mag_I(0) = 2.0f * h_field_min * cosf(mag_declination);
			_state.mag_I(1) = 2.0f * h_field_min * sinf(mag_declination);
		}

		h_field = h_field_min;
	}

	// do not allow the declination estimate to vary too much relative to the reference value
	constexpr float decl_tolerance = 0.5f;
	const float decl_max = decl_reference + decl_tolerance;
	const float decl_min = decl_reference - decl_tolerance;
	const float decl_estimate = atan2f(_state.mag_I(1), _state.mag_I(0));

	if (decl_estimate > decl_max)  {
		_state.mag_I(0) = h_field * cosf(decl_max);
		_state.mag_I(1) = h_field * sinf(decl_max);

	} else if (decl_estimate < decl_min)  {
		_state.mag_I(0) = h_field * cosf(decl_min);
		_state.mag_I(1) = h_field * sinf(decl_min);
	}
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetomer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}
