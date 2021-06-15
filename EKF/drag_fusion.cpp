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
 * Body frame drag fusion methods used for multi-rotor wind estimation.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

void Ekf::fuseDrag()
{
	SparseVector24f<0,1,2,3,4,5,6,22,23> Hfusion;  // Observation Jacobians
	Vector24f Kfusion; // Kalman gain vector

	const float R_ACC = fmaxf(_params.drag_noise, 0.5f); // observation noise variance in specific force drag (m/sec**2)**2
	const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

	// correct rotor momentum drag for increase in required rotor mass flow with altitude
	// obtained from momentum disc theory
	const float mcoef_corrrected = _params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

	// drag model parameters
	const bool using_bcoef_x = _params.bcoef_x > 1.0f;
	const bool using_bcoef_y = _params.bcoef_y > 1.0f;
	const bool using_mcoef   = _params.mcoef   > 0.001f;

	if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
		return;
	}

	// get latest estimated orientation
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float &vn = _state.vel(0);
	const float &ve = _state.vel(1);
	const float &vd = _state.vel(2);

	// get latest wind velocity in earth frame
	const float &vwn = _state.wind_vel(0);
	const float &vwe = _state.wind_vel(1);

	// predicted specific forces
	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(vn - vwn, ve - vwe, vd);
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);
	const Vector3f rel_wind_body = earth_to_body * rel_wind_earth;

	// perform sequential fusion of XY specific forces
	for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
		// measured drag acceleration corrected for sensor bias
		const float mea_acc = _drag_sample_delayed.accelXY(axis_index)  - _state.delta_vel_bias(axis_index) / _dt_ekf_avg;

		// predicted drag force sign is opposite to predicted wind relative velocity
		const float drag_sign = (rel_wind_body(axis_index) >= 0.f) ? -1.f : 1.f;

		// Drag is modelled as an arbitrary combination of bluff body drag that proportional to
		// equivalent airspeed squared, and rotor momentum drag that is proportional to true airspeed
		// parallel to the rotor disc and mass flow through the rotor disc.
		float pred_acc; // predicted drag acceleration
		if (axis_index == 0) {
			float Kacc; // Derivative of specific force wrt airspeed
			if (using_mcoef && using_bcoef_x) {
				// Use a combination of bluff body and propeller momentum drag
				const float bcoef_inv = 1.0f / _params.bcoef_x;
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = 0.5 * rho * bcoef_inv * airspeed**2 + mcoef_corrrected * airspeed
				const float airspeed = (_params.bcoef_x  / rho) * (- mcoef_corrrected  + sqrtf(sq(mcoef_corrrected) + 2.0f * rho * bcoef_inv * fabsf(mea_acc)));
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed + mcoef_corrrected);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(0)) * drag_sign - rel_wind_body(0) * mcoef_corrrected;
			} else if (using_mcoef) {
				// Use propeller momentum drag only
				Kacc = fmaxf(1e-1f, mcoef_corrrected);
				pred_acc = - rel_wind_body(0) * mcoef_corrrected;
			} else if (using_bcoef_x) {
				// Use bluff body drag only
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = (0.5 * rho / _params.bcoef_x) * airspeed**2
				const float airspeed = sqrtf((2.0f * _params.bcoef_x  * fabsf(mea_acc)) / rho);
				const float bcoef_inv = 1.0f / _params.bcoef_x;
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(0)) * drag_sign;
			} else {
				// skip this axis
				continue;
			}

			// intermediate variables
			const float HK0 = vn - vwn;
			const float HK1 = ve - vwe;
			const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
			const float HK3 = 2*Kacc;
			const float HK4 = HK0*q1 + HK1*q2 + q3*vd;
			const float HK5 = HK0*q2 - HK1*q1 + q0*vd;
			const float HK6 = -HK0*q3 + HK1*q0 + q1*vd;
			const float HK7 = ecl::powf(q0, 2) + ecl::powf(q1, 2) - ecl::powf(q2, 2) - ecl::powf(q3, 2);
			const float HK8 = HK7*Kacc;
			const float HK9 = q0*q3 + q1*q2;
			const float HK10 = HK3*HK9;
			const float HK11 = q0*q2 - q1*q3;
			const float HK12 = 2*HK9;
			const float HK13 = 2*HK11;
			const float HK14 = 2*HK4;
			const float HK15 = 2*HK2;
			const float HK16 = 2*HK5;
			const float HK17 = 2*HK6;
			const float HK18 = -HK12*P(0,23) + HK12*P(0,5) - HK13*P(0,6) + HK14*P(0,1) + HK15*P(0,0) - HK16*P(0,2) + HK17*P(0,3) - HK7*P(0,22) + HK7*P(0,4);
			const float HK19 = HK12*P(5,23);
			const float HK20 = -HK12*P(23,23) - HK13*P(6,23) + HK14*P(1,23) + HK15*P(0,23) - HK16*P(2,23) + HK17*P(3,23) + HK19 - HK7*P(22,23) + HK7*P(4,23);
			const float HK21 = ecl::powf(Kacc, 2);
			const float HK22 = HK12*HK21;
			const float HK23 = HK12*P(5,5) - HK13*P(5,6) + HK14*P(1,5) + HK15*P(0,5) - HK16*P(2,5) + HK17*P(3,5) - HK19 + HK7*P(4,5) - HK7*P(5,22);
			const float HK24 = HK12*P(5,6) - HK12*P(6,23) - HK13*P(6,6) + HK14*P(1,6) + HK15*P(0,6) - HK16*P(2,6) + HK17*P(3,6) + HK7*P(4,6) - HK7*P(6,22);
			const float HK25 = HK7*P(4,22);
			const float HK26 = -HK12*P(4,23) + HK12*P(4,5) - HK13*P(4,6) + HK14*P(1,4) + HK15*P(0,4) - HK16*P(2,4) + HK17*P(3,4) - HK25 + HK7*P(4,4);
			const float HK27 = HK21*HK7;
			const float HK28 = -HK12*P(22,23) + HK12*P(5,22) - HK13*P(6,22) + HK14*P(1,22) + HK15*P(0,22) - HK16*P(2,22) + HK17*P(3,22) + HK25 - HK7*P(22,22);
			const float HK29 = -HK12*P(1,23) + HK12*P(1,5) - HK13*P(1,6) + HK14*P(1,1) + HK15*P(0,1) - HK16*P(1,2) + HK17*P(1,3) - HK7*P(1,22) + HK7*P(1,4);
			const float HK30 = -HK12*P(2,23) + HK12*P(2,5) - HK13*P(2,6) + HK14*P(1,2) + HK15*P(0,2) - HK16*P(2,2) + HK17*P(2,3) - HK7*P(2,22) + HK7*P(2,4);
			const float HK31 = -HK12*P(3,23) + HK12*P(3,5) - HK13*P(3,6) + HK14*P(1,3) + HK15*P(0,3) - HK16*P(2,3) + HK17*P(3,3) - HK7*P(3,22) + HK7*P(3,4);
			//const float HK32 = Kacc/(-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);

			// calculate innovation variance and exit if badly conditioned
			_drag_innov_var(0) = (-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);
			if (_drag_innov_var(0) < R_ACC) {
				return;
			}
			const float HK32 = Kacc / _drag_innov_var(0);

			// Observation Jacobians
			Hfusion.at<0>() = -HK2*HK3;
			Hfusion.at<1>() = -HK3*HK4;
			Hfusion.at<2>() = HK3*HK5;
			Hfusion.at<3>() = -HK3*HK6;
			Hfusion.at<4>() = -HK8;
			Hfusion.at<5>() = -HK10;
			Hfusion.at<6>() = HK11*HK3;
			Hfusion.at<22>() = HK8;
			Hfusion.at<23>() = HK10;

			// Kalman gains
			// Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
			// Kfusion(0) = -HK18*HK32;
			// Kfusion(1) = -HK29*HK32;
			// Kfusion(2) = -HK30*HK32;
			// Kfusion(3) = -HK31*HK32;
			// Kfusion(4) = -HK26*HK32;
			// Kfusion(5) = -HK23*HK32;
			// Kfusion(6) = -HK24*HK32;
			// Kfusion(7) = -HK32*(HK12*P(5,7) - HK12*P(7,23) - HK13*P(6,7) + HK14*P(1,7) + HK15*P(0,7) - HK16*P(2,7) + HK17*P(3,7) + HK7*P(4,7) - HK7*P(7,22));
			// Kfusion(8) = -HK32*(HK12*P(5,8) - HK12*P(8,23) - HK13*P(6,8) + HK14*P(1,8) + HK15*P(0,8) - HK16*P(2,8) + HK17*P(3,8) + HK7*P(4,8) - HK7*P(8,22));
			// Kfusion(9) = -HK32*(HK12*P(5,9) - HK12*P(9,23) - HK13*P(6,9) + HK14*P(1,9) + HK15*P(0,9) - HK16*P(2,9) + HK17*P(3,9) + HK7*P(4,9) - HK7*P(9,22));
			// Kfusion(10) = -HK32*(-HK12*P(10,23) + HK12*P(5,10) - HK13*P(6,10) + HK14*P(1,10) + HK15*P(0,10) - HK16*P(2,10) + HK17*P(3,10) - HK7*P(10,22) + HK7*P(4,10));
			// Kfusion(11) = -HK32*(-HK12*P(11,23) + HK12*P(5,11) - HK13*P(6,11) + HK14*P(1,11) + HK15*P(0,11) - HK16*P(2,11) + HK17*P(3,11) - HK7*P(11,22) + HK7*P(4,11));
			// Kfusion(12) = -HK32*(-HK12*P(12,23) + HK12*P(5,12) - HK13*P(6,12) + HK14*P(1,12) + HK15*P(0,12) - HK16*P(2,12) + HK17*P(3,12) - HK7*P(12,22) + HK7*P(4,12));
			// Kfusion(13) = -HK32*(-HK12*P(13,23) + HK12*P(5,13) - HK13*P(6,13) + HK14*P(1,13) + HK15*P(0,13) - HK16*P(2,13) + HK17*P(3,13) - HK7*P(13,22) + HK7*P(4,13));
			// Kfusion(14) = -HK32*(-HK12*P(14,23) + HK12*P(5,14) - HK13*P(6,14) + HK14*P(1,14) + HK15*P(0,14) - HK16*P(2,14) + HK17*P(3,14) - HK7*P(14,22) + HK7*P(4,14));
			// Kfusion(15) = -HK32*(-HK12*P(15,23) + HK12*P(5,15) - HK13*P(6,15) + HK14*P(1,15) + HK15*P(0,15) - HK16*P(2,15) + HK17*P(3,15) - HK7*P(15,22) + HK7*P(4,15));
			// Kfusion(16) = -HK32*(-HK12*P(16,23) + HK12*P(5,16) - HK13*P(6,16) + HK14*P(1,16) + HK15*P(0,16) - HK16*P(2,16) + HK17*P(3,16) - HK7*P(16,22) + HK7*P(4,16));
			// Kfusion(17) = -HK32*(-HK12*P(17,23) + HK12*P(5,17) - HK13*P(6,17) + HK14*P(1,17) + HK15*P(0,17) - HK16*P(2,17) + HK17*P(3,17) - HK7*P(17,22) + HK7*P(4,17));
			// Kfusion(18) = -HK32*(-HK12*P(18,23) + HK12*P(5,18) - HK13*P(6,18) + HK14*P(1,18) + HK15*P(0,18) - HK16*P(2,18) + HK17*P(3,18) - HK7*P(18,22) + HK7*P(4,18));
			// Kfusion(19) = -HK32*(-HK12*P(19,23) + HK12*P(5,19) - HK13*P(6,19) + HK14*P(1,19) + HK15*P(0,19) - HK16*P(2,19) + HK17*P(3,19) - HK7*P(19,22) + HK7*P(4,19));
			// Kfusion(20) = -HK32*(-HK12*P(20,23) + HK12*P(5,20) - HK13*P(6,20) + HK14*P(1,20) + HK15*P(0,20) - HK16*P(2,20) + HK17*P(3,20) - HK7*P(20,22) + HK7*P(4,20));
			// Kfusion(21) = -HK32*(-HK12*P(21,23) + HK12*P(5,21) - HK13*P(6,21) + HK14*P(1,21) + HK15*P(0,21) - HK16*P(2,21) + HK17*P(3,21) - HK7*P(21,22) + HK7*P(4,21));
			Kfusion(22) = -HK28*HK32;
			Kfusion(23) = -HK20*HK32;


		} else if (axis_index == 1) {
			float Kacc; // Derivative of specific force wrt airspeed
			if (using_mcoef && using_bcoef_y) {
				// Use a combination of bluff body and propeller momentum drag
				const float bcoef_inv = 1.0f / _params.bcoef_y;
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = 0.5 * rho * bcoef_inv * airspeed**2 + mcoef_corrrected * airspeed
				const float airspeed = (_params.bcoef_y  / rho) * (- mcoef_corrrected  + sqrtf(sq(mcoef_corrrected) + 2.0f * rho * bcoef_inv * fabsf(mea_acc)));
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed + mcoef_corrrected);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(1)) * drag_sign - rel_wind_body(1) * mcoef_corrrected;
			} else if (using_mcoef) {
				// Use propeller momentum drag only
				Kacc = fmaxf(1e-1f, mcoef_corrrected);
				pred_acc = - rel_wind_body(1) * mcoef_corrrected;
			} else if (using_bcoef_y) {
				// Use bluff body drag only
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = (0.5 * rho / _params.bcoef_y) * airspeed**2
				const float airspeed = sqrtf((2.0f * _params.bcoef_y * fabsf(mea_acc)) / rho);
				const float bcoef_inv = 1.0f / _params.bcoef_y;
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(1)) * drag_sign;
			} else {
				// nothing more to do
				return;
			}

			// intermediate variables
			const float HK0 = ve - vwe;
			const float HK1 = vn - vwn;
			const float HK2 = HK0*q0 - HK1*q3 + q1*vd;
			const float HK3 = 2*Kacc;
			const float HK4 = -HK0*q1 + HK1*q2 + q0*vd;
			const float HK5 = HK0*q2 + HK1*q1 + q3*vd;
			const float HK6 = HK0*q3 + HK1*q0 - q2*vd;
			const float HK7 = q0*q3 - q1*q2;
			const float HK8 = HK3*HK7;
			const float HK9 = ecl::powf(q0, 2) - ecl::powf(q1, 2) + ecl::powf(q2, 2) - ecl::powf(q3, 2);
			const float HK10 = HK9*Kacc;
			const float HK11 = q0*q1 + q2*q3;
			const float HK12 = 2*HK11;
			const float HK13 = 2*HK7;
			const float HK14 = 2*HK5;
			const float HK15 = 2*HK2;
			const float HK16 = 2*HK4;
			const float HK17 = 2*HK6;
			const float HK18 = HK12*P(0,6) + HK13*P(0,22) - HK13*P(0,4) + HK14*P(0,2) + HK15*P(0,0) + HK16*P(0,1) - HK17*P(0,3) - HK9*P(0,23) + HK9*P(0,5);
			const float HK19 = ecl::powf(Kacc, 2);
			const float HK20 = HK12*P(6,6) - HK13*P(4,6) + HK13*P(6,22) + HK14*P(2,6) + HK15*P(0,6) + HK16*P(1,6) - HK17*P(3,6) + HK9*P(5,6) - HK9*P(6,23);
			const float HK21 = HK13*P(4,22);
			const float HK22 = HK12*P(6,22) + HK13*P(22,22) + HK14*P(2,22) + HK15*P(0,22) + HK16*P(1,22) - HK17*P(3,22) - HK21 - HK9*P(22,23) + HK9*P(5,22);
			const float HK23 = HK13*HK19;
			const float HK24 = HK12*P(4,6) - HK13*P(4,4) + HK14*P(2,4) + HK15*P(0,4) + HK16*P(1,4) - HK17*P(3,4) + HK21 - HK9*P(4,23) + HK9*P(4,5);
			const float HK25 = HK9*P(5,23);
			const float HK26 = HK12*P(5,6) - HK13*P(4,5) + HK13*P(5,22) + HK14*P(2,5) + HK15*P(0,5) + HK16*P(1,5) - HK17*P(3,5) - HK25 + HK9*P(5,5);
			const float HK27 = HK19*HK9;
			const float HK28 = HK12*P(6,23) + HK13*P(22,23) - HK13*P(4,23) + HK14*P(2,23) + HK15*P(0,23) + HK16*P(1,23) - HK17*P(3,23) + HK25 - HK9*P(23,23);
			const float HK29 = HK12*P(2,6) + HK13*P(2,22) - HK13*P(2,4) + HK14*P(2,2) + HK15*P(0,2) + HK16*P(1,2) - HK17*P(2,3) - HK9*P(2,23) + HK9*P(2,5);
			const float HK30 = HK12*P(1,6) + HK13*P(1,22) - HK13*P(1,4) + HK14*P(1,2) + HK15*P(0,1) + HK16*P(1,1) - HK17*P(1,3) - HK9*P(1,23) + HK9*P(1,5);
			const float HK31 = HK12*P(3,6) + HK13*P(3,22) - HK13*P(3,4) + HK14*P(2,3) + HK15*P(0,3) + HK16*P(1,3) - HK17*P(3,3) - HK9*P(3,23) + HK9*P(3,5);
			// const float HK32 = Kacc/(HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);

			_drag_innov_var(1) = (HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);
			if (_drag_innov_var(1) < R_ACC) {
				// calculation is badly conditioned
				return;
			}
			const float HK32 = Kacc / _drag_innov_var(1);

			// Observation Jacobians
			Hfusion.at<0>() = -HK2*HK3;
			Hfusion.at<1>() = -HK3*HK4;
			Hfusion.at<2>() = -HK3*HK5;
			Hfusion.at<3>() = HK3*HK6;
			Hfusion.at<4>() = HK8;
			Hfusion.at<5>() = -HK10;
			Hfusion.at<6>() = -HK11*HK3;
			Hfusion.at<22>() = -HK8;
			Hfusion.at<23>() = HK10;

			// Kalman gains
			// Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
			// Kfusion(0) = -HK18*HK32;
			// Kfusion(1) = -HK30*HK32;
			// Kfusion(2) = -HK29*HK32;
			// Kfusion(3) = -HK31*HK32;
			// Kfusion(4) = -HK24*HK32;
			// Kfusion(5) = -HK26*HK32;
			// Kfusion(6) = -HK20*HK32;
			// Kfusion(7) = -HK32*(HK12*P(6,7) - HK13*P(4,7) + HK13*P(7,22) + HK14*P(2,7) + HK15*P(0,7) + HK16*P(1,7) - HK17*P(3,7) + HK9*P(5,7) - HK9*P(7,23));
			// Kfusion(8) = -HK32*(HK12*P(6,8) - HK13*P(4,8) + HK13*P(8,22) + HK14*P(2,8) + HK15*P(0,8) + HK16*P(1,8) - HK17*P(3,8) + HK9*P(5,8) - HK9*P(8,23));
			// Kfusion(9) = -HK32*(HK12*P(6,9) - HK13*P(4,9) + HK13*P(9,22) + HK14*P(2,9) + HK15*P(0,9) + HK16*P(1,9) - HK17*P(3,9) + HK9*P(5,9) - HK9*P(9,23));
			// Kfusion(10) = -HK32*(HK12*P(6,10) + HK13*P(10,22) - HK13*P(4,10) + HK14*P(2,10) + HK15*P(0,10) + HK16*P(1,10) - HK17*P(3,10) - HK9*P(10,23) + HK9*P(5,10));
			// Kfusion(11) = -HK32*(HK12*P(6,11) + HK13*P(11,22) - HK13*P(4,11) + HK14*P(2,11) + HK15*P(0,11) + HK16*P(1,11) - HK17*P(3,11) - HK9*P(11,23) + HK9*P(5,11));
			// Kfusion(12) = -HK32*(HK12*P(6,12) + HK13*P(12,22) - HK13*P(4,12) + HK14*P(2,12) + HK15*P(0,12) + HK16*P(1,12) - HK17*P(3,12) - HK9*P(12,23) + HK9*P(5,12));
			// Kfusion(13) = -HK32*(HK12*P(6,13) + HK13*P(13,22) - HK13*P(4,13) + HK14*P(2,13) + HK15*P(0,13) + HK16*P(1,13) - HK17*P(3,13) - HK9*P(13,23) + HK9*P(5,13));
			// Kfusion(14) = -HK32*(HK12*P(6,14) + HK13*P(14,22) - HK13*P(4,14) + HK14*P(2,14) + HK15*P(0,14) + HK16*P(1,14) - HK17*P(3,14) - HK9*P(14,23) + HK9*P(5,14));
			// Kfusion(15) = -HK32*(HK12*P(6,15) + HK13*P(15,22) - HK13*P(4,15) + HK14*P(2,15) + HK15*P(0,15) + HK16*P(1,15) - HK17*P(3,15) - HK9*P(15,23) + HK9*P(5,15));
			// Kfusion(16) = -HK32*(HK12*P(6,16) + HK13*P(16,22) - HK13*P(4,16) + HK14*P(2,16) + HK15*P(0,16) + HK16*P(1,16) - HK17*P(3,16) - HK9*P(16,23) + HK9*P(5,16));
			// Kfusion(17) = -HK32*(HK12*P(6,17) + HK13*P(17,22) - HK13*P(4,17) + HK14*P(2,17) + HK15*P(0,17) + HK16*P(1,17) - HK17*P(3,17) - HK9*P(17,23) + HK9*P(5,17));
			// Kfusion(18) = -HK32*(HK12*P(6,18) + HK13*P(18,22) - HK13*P(4,18) + HK14*P(2,18) + HK15*P(0,18) + HK16*P(1,18) - HK17*P(3,18) - HK9*P(18,23) + HK9*P(5,18));
			// Kfusion(19) = -HK32*(HK12*P(6,19) + HK13*P(19,22) - HK13*P(4,19) + HK14*P(2,19) + HK15*P(0,19) + HK16*P(1,19) - HK17*P(3,19) - HK9*P(19,23) + HK9*P(5,19));
			// Kfusion(20) = -HK32*(HK12*P(6,20) + HK13*P(20,22) - HK13*P(4,20) + HK14*P(2,20) + HK15*P(0,20) + HK16*P(1,20) - HK17*P(3,20) - HK9*P(20,23) + HK9*P(5,20));
			// Kfusion(21) = -HK32*(HK12*P(6,21) + HK13*P(21,22) - HK13*P(4,21) + HK14*P(2,21) + HK15*P(0,21) + HK16*P(1,21) - HK17*P(3,21) - HK9*P(21,23) + HK9*P(5,21));
			Kfusion(22) = -HK22*HK32;
			Kfusion(23) = -HK28*HK32;

		}

		// Apply an innovation consistency check with a 5 Sigma threshold
		_drag_innov(axis_index) = pred_acc - mea_acc;
		_drag_test_ratio(axis_index) = sq(_drag_innov(axis_index)) / (sq(5.0f) * _drag_innov_var(axis_index));

		// if the innovation consistency check fails then don't fuse the sample
		if (_drag_test_ratio(axis_index) <= 1.0f) {
			measurementUpdate(Kfusion, Hfusion, _drag_innov(axis_index));
		}
	}
}
