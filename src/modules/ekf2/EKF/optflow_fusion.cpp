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
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include "utils.hpp"

void Ekf::fuseOptFlow()
{
	float gndclearance = fmaxf(_params.rng_gnd_clearance, 0.1f);

	// get latest estimated orientation
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float vn = _state.vel(0);
	const float ve = _state.vel(1);
	const float vd = _state.vel(2);

	// get latest vehicle and terrain vertical position
	const float pd = _state.pos(2);
	const float ptd = _state.posd_terrain;

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar();

	// get rotation matrix from earth to body
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	const Vector3f vel_body = earth_to_body * vel_rel_earth;

	// height above ground of the IMU
	float heightAboveGndEst = _terrain_vpos - _state.pos(2);

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	heightAboveGndEst -= pos_offset_earth(2);

	// constrain minimum height above ground
	heightAboveGndEst = math::max(heightAboveGndEst, gndclearance);

	// calculate range from focal point to centre of image
	const float range = heightAboveGndEst / earth_to_body(2, 2); // absolute distance to the frame region in view

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

	// compute the velocities in body and local frames from corrected optical flow measurement
	// for logging only
	_flow_vel_body(0) = -opt_flow_rate(1) * range;
	_flow_vel_body(1) = opt_flow_rate(0) * range;
	_flow_vel_ne = Vector2f(_R_to_earth * Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));

	_flow_innov(0) =  vel_body(1) / range - opt_flow_rate(0); // flow around the X axis
	_flow_innov(1) = -vel_body(0) / range - opt_flow_rate(1); // flow around the Y axis

	// The derivation allows for an arbitrary body to flow sensor frame rotation which is
	// currently not supported by the EKF, so assume sensor frame is aligned with the
	// body frame
	const Dcmf Tbs = matrix::eye<float, 3>();

	// Sub Expressions
	const float HK0 = -Tbs(1,0)*q2 + Tbs(1,1)*q1 + Tbs(1,2)*q0;
	const float HK1 = Tbs(1,0)*q3 + Tbs(1,1)*q0 - Tbs(1,2)*q1;
	const float HK2 = Tbs(1,0)*q0 - Tbs(1,1)*q3 + Tbs(1,2)*q2;
	const float HK3 = q0*q1;
	const float HK4 = q2*q3;
	const float HK5 = HK3 + HK4;
	const float HK6 = 2*HK5;
	const float HK7 = q0*q2;
	const float HK8 = q1*q3;
	const float HK9 = HK7 - HK8;
	const float HK10 = 2*HK9;
	const float HK11 = powf(q3, 2);
	const float HK12 = powf(q2, 2);
	const float HK13 = -HK12;
	const float HK14 = powf(q0, 2);
	const float HK15 = powf(q1, 2);
	const float HK16 = HK14 - HK15;
	const float HK17 = HK11 + HK13 + HK16;
	const float HK18 = -HK10*Tbs(2,0) + HK17*Tbs(2,2) + HK6*Tbs(2,1);
	const float HK19 = -Tbs(2,0)*q2 + Tbs(2,1)*q1 + Tbs(2,2)*q0;
	const float HK20 = 2*Tbs(1,1);
	const float HK21 = 2*Tbs(1,0);
	const float HK22 = HK17*Tbs(1,2) + HK20*HK5 - HK21*HK9;
	const float HK23 = q0*q3;
	const float HK24 = q1*q2;
	const float HK25 = HK23 + HK24;
	const float HK26 = HK3 - HK4;
	const float HK27 = 2*Tbs(1,2);
	const float HK28 = -HK11;
	const float HK29 = HK12 + HK16 + HK28;
	const float HK30 = HK21*HK25 - HK26*HK27 + HK29*Tbs(1,1);
	const float HK31 = HK7 + HK8;
	const float HK32 = HK23 - HK24;
	const float HK33 = HK13 + HK14 + HK15 + HK28;
	const float HK34 = -HK20*HK32 + HK27*HK31 + HK33*Tbs(1,0);
	const float HK35 = HK22*vd + HK30*ve + HK34*vn;
	const float HK36 = HK18*(HK0*vd + HK1*ve + HK2*vn) + HK19*HK35;
	const float HK37 = pd - ptd;
	const float HK38 = 1.0F/HK37;
	const float HK39 = 2*HK38;
	const float HK40 = Tbs(1,0)*q1 + Tbs(1,1)*q2 + Tbs(1,2)*q3;
	const float HK41 = Tbs(2,0)*q3 + Tbs(2,1)*q0 - Tbs(2,2)*q1;
	const float HK42 = HK18*(-HK0*ve + HK1*vd + HK40*vn) + HK35*HK41;
	const float HK43 = Tbs(2,0)*q0 - Tbs(2,1)*q3 + Tbs(2,2)*q2;
	const float HK44 = -HK18*(HK0*vn - HK2*vd + HK40*ve) + HK35*HK43;
	const float HK45 = Tbs(2,0)*q1 + Tbs(2,1)*q2 + Tbs(2,2)*q3;
	const float HK46 = HK18*(-HK1*vn + HK2*ve + HK40*vd) + HK35*HK45;
	const float HK47 = HK18*HK38;
	const float HK48 = powf(HK37, -2);
	const float HK49 = HK18*HK48;
	const float HK50 = HK35*HK49;
	const float HK51 = HK18*HK34;
	const float HK52 = HK51*P(0,4);
	const float HK53 = HK18*HK30;
	const float HK54 = HK53*P(0,5);
	const float HK55 = HK18*HK22;
	const float HK56 = HK55*P(0,6);
	const float HK57 = HK35*HK47;
	const float HK58 = HK57*P(0,24);
	const float HK59 = HK57*P(0,9);
	const float HK60 = 2*HK46;
	const float HK61 = HK60*P(0,3);
	const float HK62 = 2*HK36;
	const float HK63 = HK62*P(0,0);
	const float HK64 = 2*HK42;
	const float HK65 = HK64*P(0,1);
	const float HK66 = 2*HK44;
	const float HK67 = HK66*P(0,2);
	const float HK68 = HK51*P(4,6);
	const float HK69 = HK53*P(5,6);
	const float HK70 = HK55*P(6,6);
	const float HK71 = HK57*P(6,9);
	const float HK72 = HK57*P(6,24);
	const float HK73 = HK60*P(3,6);
	const float HK74 = HK62*P(0,6);
	const float HK75 = HK64*P(1,6);
	const float HK76 = HK66*P(2,6);
	const float HK77 = HK51*P(4,5);
	const float HK78 = HK53*P(5,5);
	const float HK79 = HK55*P(5,6);
	const float HK80 = HK57*P(5,9);
	const float HK81 = HK57*P(5,24);
	const float HK82 = HK60*P(3,5);
	const float HK83 = HK62*P(0,5);
	const float HK84 = HK64*P(1,5);
	const float HK85 = HK66*P(2,5);
	const float HK86 = HK51*P(4,4);
	const float HK87 = HK53*P(4,5);
	const float HK88 = HK55*P(4,6);
	const float HK89 = HK57*P(4,9);
	const float HK90 = HK57*P(4,24);
	const float HK91 = HK60*P(3,4);
	const float HK92 = HK62*P(0,4);
	const float HK93 = HK64*P(1,4);
	const float HK94 = HK66*P(2,4);
	const float HK95 = HK51*P(4,24);
	const float HK96 = HK53*P(5,24);
	const float HK97 = HK55*P(6,24);
	const float HK98 = HK57*P(9,24);
	const float HK99 = HK57*P(24,24);
	const float HK100 = HK60*P(3,24);
	const float HK101 = HK62*P(0,24);
	const float HK102 = HK64*P(1,24);
	const float HK103 = HK66*P(2,24);
	const float HK104 = HK18/powf(HK37, 3);
	const float HK105 = HK104*HK35;
	const float HK106 = HK51*P(4,9);
	const float HK107 = HK53*P(5,9);
	const float HK108 = HK55*P(6,9);
	const float HK109 = HK57*P(9,9);
	const float HK110 = -HK98;
	const float HK111 = HK60*P(3,9);
	const float HK112 = HK62*P(0,9);
	const float HK113 = HK64*P(1,9);
	const float HK114 = HK66*P(2,9);
	const float HK115 = HK51*P(3,4);
	const float HK116 = HK53*P(3,5);
	const float HK117 = HK55*P(3,6);
	const float HK118 = HK57*P(3,9);
	const float HK119 = HK57*P(3,24);
	const float HK120 = HK60*P(3,3);
	const float HK121 = HK62*P(0,3);
	const float HK122 = HK64*P(1,3);
	const float HK123 = HK66*P(2,3);
	const float HK124 = HK51*P(1,4);
	const float HK125 = HK53*P(1,5);
	const float HK126 = HK55*P(1,6);
	const float HK127 = HK57*P(1,9);
	const float HK128 = HK57*P(1,24);
	const float HK129 = HK60*P(1,3);
	const float HK130 = HK62*P(0,1);
	const float HK131 = HK64*P(1,1);
	const float HK132 = HK66*P(1,2);
	const float HK133 = HK51*P(2,4);
	const float HK134 = HK53*P(2,5);
	const float HK135 = HK55*P(2,6);
	const float HK136 = HK57*P(2,9);
	const float HK137 = HK57*P(2,24);
	const float HK138 = HK60*P(2,3);
	const float HK139 = HK62*P(0,2);
	const float HK140 = HK64*P(1,2);
	const float HK141 = HK66*P(2,2);
	// const float HK142 = HK38/(HK105*(-HK100 - HK101 - HK102 + HK103 - HK95 - HK96 - HK97 + HK98 - HK99) - HK105*(-HK106 - HK107 - HK108 + HK109 + HK110 - HK111 - HK112 - HK113 + HK114) + HK22*HK49*(-HK68 - HK69 - HK70 + HK71 - HK72 - HK73 - HK74 - HK75 + HK76) + HK30*HK49*(-HK77 - HK78 - HK79 + HK80 - HK81 - HK82 - HK83 - HK84 + HK85) + HK34*HK49*(-HK86 - HK87 - HK88 + HK89 - HK90 - HK91 - HK92 - HK93 + HK94) + HK48*HK60*(-HK115 - HK116 - HK117 + HK118 - HK119 - HK120 - HK121 - HK122 + HK123) + HK48*HK62*(-HK52 - HK54 - HK56 - HK58 + HK59 - HK61 - HK63 - HK65 + HK67) + HK48*HK64*(-HK124 - HK125 - HK126 + HK127 - HK128 - HK129 - HK130 - HK131 + HK132) - HK48*HK66*(-HK133 - HK134 - HK135 + HK136 - HK137 - HK138 - HK139 - HK140 + HK141) - R_LOS);

	// calculate innovation variance for X axis observation and protect against a badly conditioned calculation
	_flow_innov_var(0) = -(HK105*(-HK100 - HK101 - HK102 + HK103 - HK95 - HK96 - HK97 + HK98 - HK99) - HK105*(-HK106 - HK107 - HK108 + HK109 + HK110 - HK111 - HK112 - HK113 + HK114) + HK22*HK49*(-HK68 - HK69 - HK70 + HK71 - HK72 - HK73 - HK74 - HK75 + HK76) + HK30*HK49*(-HK77 - HK78 - HK79 + HK80 - HK81 - HK82 - HK83 - HK84 + HK85) + HK34*HK49*(-HK86 - HK87 - HK88 + HK89 - HK90 - HK91 - HK92 - HK93 + HK94) + HK48*HK60*(-HK115 - HK116 - HK117 + HK118 - HK119 - HK120 - HK121 - HK122 + HK123) + HK48*HK62*(-HK52 - HK54 - HK56 - HK58 + HK59 - HK61 - HK63 - HK65 + HK67) + HK48*HK64*(-HK124 - HK125 - HK126 + HK127 - HK128 - HK129 - HK130 - HK131 + HK132) - HK48*HK66*(-HK133 - HK134 - HK135 + HK136 - HK137 - HK138 - HK139 - HK140 + HK141) - R_LOS);

	if (_flow_innov_var(0) < R_LOS) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}
	const float HK142 = HK38/(-_flow_innov_var(0));

	const float HK143 = -Tbs(0,0)*q2 + Tbs(0,1)*q1 + Tbs(0,2)*q0;
	const float HK144 = Tbs(0,0)*q3 + Tbs(0,1)*q0 - Tbs(0,2)*q1;
	const float HK145 = Tbs(0,0)*q0;
	const float HK146 = Tbs(0,2)*q2;
	const float HK147 = Tbs(0,1)*q3;
	const float HK148 = HK145 + HK146 - HK147;
	const float HK149 = HK17*Tbs(0,2) + HK6*Tbs(0,1);
	const float HK150 = -HK10*Tbs(0,0) + HK149;
	const float HK151 = 2*Tbs(0,2);
	const float HK152 = 2*Tbs(0,0);
	const float HK153 = HK152*HK25 + HK29*Tbs(0,1);
	const float HK154 = -HK151*HK26 + HK153;
	const float HK155 = 2*Tbs(0,1);
	const float HK156 = HK151*HK31 + HK33*Tbs(0,0);
	const float HK157 = -HK155*HK32 + HK156;
	const float HK158 = HK150*vd + HK154*ve + HK157*vn;
	const float HK159 = HK158*HK19 + HK18*(HK143*vd + HK144*ve + HK148*vn);
	const float HK160 = Tbs(0,0)*q1 + Tbs(0,1)*q2 + Tbs(0,2)*q3;
	const float HK161 = HK158*HK41 + HK18*(-HK143*ve + HK144*vd + HK160*vn);
	const float HK162 = HK18*(HK143*vn + HK160*ve + vd*(-HK145 - HK146 + HK147));
	const float HK163 = HK43*(vd*(HK149 + HK152*(-HK7 + HK8)) + ve*(HK151*(-HK3 + HK4) + HK153) + vn*(HK155*(-HK23 + HK24) + HK156));
	const float HK164 = HK158*HK45 + HK18*(-HK144*vn + HK148*ve + HK160*vd);
	const float HK165 = HK158*HK49;
	const float HK166 = HK157*HK18;
	const float HK167 = HK154*HK18;
	const float HK168 = HK150*HK18;
	const float HK169 = HK158*HK47;
	const float HK170 = 2*HK164;
	const float HK171 = 2*HK159;
	const float HK172 = 2*HK161;
	const float HK173 = -2*HK162 + 2*HK163;
	const float HK174 = HK166*P(0,4) + HK167*P(0,5) + HK168*P(0,6) + HK169*P(0,24) - HK169*P(0,9) + HK170*P(0,3) + HK171*P(0,0) + HK172*P(0,1) - HK173*P(0,2);
	const float HK175 = HK166*P(4,6) + HK167*P(5,6) + HK168*P(6,6) + HK169*P(6,24) - HK169*P(6,9) + HK170*P(3,6) + HK171*P(0,6) + HK172*P(1,6) - HK173*P(2,6);
	const float HK176 = HK166*P(4,5) + HK167*P(5,5) + HK168*P(5,6) + HK169*P(5,24) - HK169*P(5,9) + HK170*P(3,5) + HK171*P(0,5) + HK172*P(1,5) - HK173*P(2,5);
	const float HK177 = HK166*P(4,4) + HK167*P(4,5) + HK168*P(4,6) + HK169*P(4,24) - HK169*P(4,9) + HK170*P(3,4) + HK171*P(0,4) + HK172*P(1,4) - HK173*P(2,4);
	const float HK178 = HK169*P(9,24);
	const float HK179 = HK166*P(4,24) + HK167*P(5,24) + HK168*P(6,24) + HK169*P(24,24) + HK170*P(3,24) + HK171*P(0,24) + HK172*P(1,24) - HK173*P(2,24) - HK178;
	const float HK180 = HK104*HK158;
	const float HK181 = HK166*P(4,9) + HK167*P(5,9) + HK168*P(6,9) - HK169*P(9,9) + HK170*P(3,9) + HK171*P(0,9) + HK172*P(1,9) - HK173*P(2,9) + HK178;
	const float HK182 = HK166*P(3,4) + HK167*P(3,5) + HK168*P(3,6) + HK169*P(3,24) - HK169*P(3,9) + HK170*P(3,3) + HK171*P(0,3) + HK172*P(1,3) - HK173*P(2,3);
	const float HK183 = HK166*P(1,4) + HK167*P(1,5) + HK168*P(1,6) + HK169*P(1,24) - HK169*P(1,9) + HK170*P(1,3) + HK171*P(0,1) + HK172*P(1,1) - HK173*P(1,2);
	const float HK184 = HK166*P(2,4) + HK167*P(2,5) + HK168*P(2,6) + HK169*P(2,24) - HK169*P(2,9) + HK170*P(2,3) + HK171*P(0,2) + HK172*P(1,2) - HK173*P(2,2);
	// const float HK185 = HK38/(HK150*HK175*HK49 + HK154*HK176*HK49 + HK157*HK177*HK49 + HK170*HK182*HK48 + HK171*HK174*HK48 + HK172*HK183*HK48 - HK173*HK184*HK48 + HK179*HK180 - HK180*HK181 + R_LOS);

	// calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
	_flow_innov_var(1) = (HK150*HK175*HK49 + HK154*HK176*HK49 + HK157*HK177*HK49 + HK170*HK182*HK48 + HK171*HK174*HK48 + HK172*HK183*HK48 - HK173*HK184*HK48 + HK179*HK180 - HK180*HK181 + R_LOS);
	if (_flow_innov_var(1) < R_LOS) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}
	const float HK185 = HK38/_flow_innov_var(1);


	// run the innovation consistency check and record result
	bool flow_fail = false;
	float test_ratio[2];
	test_ratio[0] = sq(_flow_innov(0)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(0));
	test_ratio[1] = sq(_flow_innov(1)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(1));
	_optflow_test_ratio = math::max(test_ratio[0], test_ratio[1]);

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {
		if (test_ratio[obs_index] > 1.0f) {
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

	// fuse observation axes sequentially
	SparseVector25f<0,1,2,3,4,5,6,9,24> Hfusion; // Optical flow observation Jacobians
	Vector25f Kfusion; // Optical flow Kalman gains

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {

		// calculate observation Jocobians and Kalman gains
		if (obs_index == 0) {
			// Observation Jacobians - axis 0
			Hfusion.at<0>() = -HK36*HK39;
			Hfusion.at<1>() = -HK39*HK42;
			Hfusion.at<2>() = HK39*HK44;
			Hfusion.at<3>() = -HK39*HK46;
			Hfusion.at<4>() = -HK34*HK47;
			Hfusion.at<5>() = -HK30*HK47;
			Hfusion.at<6>() = -HK22*HK47;
			Hfusion.at<9>() = HK50;
			Hfusion.at<24>() = -HK50;

			// Kalman gains - axis 0
			Kfusion(0) = HK142*(HK52 + HK54 + HK56 + HK58 - HK59 + HK61 + HK63 + HK65 - HK67);
			Kfusion(1) = HK142*(HK124 + HK125 + HK126 - HK127 + HK128 + HK129 + HK130 + HK131 - HK132);
			Kfusion(2) = HK142*(HK133 + HK134 + HK135 - HK136 + HK137 + HK138 + HK139 + HK140 - HK141);
			Kfusion(3) = HK142*(HK115 + HK116 + HK117 - HK118 + HK119 + HK120 + HK121 + HK122 - HK123);
			Kfusion(4) = HK142*(HK86 + HK87 + HK88 - HK89 + HK90 + HK91 + HK92 + HK93 - HK94);
			Kfusion(5) = HK142*(HK77 + HK78 + HK79 - HK80 + HK81 + HK82 + HK83 + HK84 - HK85);
			Kfusion(6) = HK142*(HK68 + HK69 + HK70 - HK71 + HK72 + HK73 + HK74 + HK75 - HK76);
			Kfusion(7) = HK142*(HK51*P(4,7) + HK53*P(5,7) + HK55*P(6,7) + HK57*P(7,24) - HK57*P(7,9) + HK60*P(3,7) + HK62*P(0,7) + HK64*P(1,7) - HK66*P(2,7));
			Kfusion(8) = HK142*(HK51*P(4,8) + HK53*P(5,8) + HK55*P(6,8) + HK57*P(8,24) - HK57*P(8,9) + HK60*P(3,8) + HK62*P(0,8) + HK64*P(1,8) - HK66*P(2,8));
			Kfusion(9) = HK142*(HK106 + HK107 + HK108 - HK109 + HK111 + HK112 + HK113 - HK114 + HK98);

			for (unsigned row = 10; row <= 23; row++) {
				Kfusion(row) = HK142*(HK51*P(4,row) + HK53*P(5,row) + HK55*P(6,row) + HK57*P(row,24) - HK57*P(9,row) + HK60*P(3,row) + HK62*P(0,row) + HK64*P(1,row) - HK66*P(2,row));
			}

			Kfusion(24) = HK142*(HK100 + HK101 + HK102 - HK103 + HK110 + HK95 + HK96 + HK97 + HK99);

		} else {
			// Observation Jacobians - axis 1
			Hfusion.at<0>() = HK159*HK39;
			Hfusion.at<1>() = HK161*HK39;
			Hfusion.at<2>() = HK39*(HK162 - HK163);
			Hfusion.at<3>() = HK164*HK39;
			Hfusion.at<4>() = HK157*HK47;
			Hfusion.at<5>() = HK154*HK47;
			Hfusion.at<6>() = HK150*HK47;
			Hfusion.at<9>() = -HK165;
			Hfusion.at<24>() = HK165;

			// Kalman gains - axis 1
			Kfusion(0) = HK174*HK185;
			Kfusion(1) = HK183*HK185;
			Kfusion(2) = HK184*HK185;
			Kfusion(3) = HK182*HK185;
			Kfusion(4) = HK177*HK185;
			Kfusion(5) = HK176*HK185;
			Kfusion(6) = HK175*HK185;
			Kfusion(7) = HK185*(HK166*P(4,7) + HK167*P(5,7) + HK168*P(6,7) + HK169*P(7,24) - HK169*P(7,9) + HK170*P(3,7) + HK171*P(0,7) + HK172*P(1,7) - HK173*P(2,7));
			Kfusion(8) = HK185*(HK166*P(4,8) + HK167*P(5,8) + HK168*P(6,8) + HK169*P(8,24) - HK169*P(8,9) + HK170*P(3,8) + HK171*P(0,8) + HK172*P(1,8) - HK173*P(2,8));
			Kfusion(9) = HK181*HK185;

			for (unsigned row = 10; row <= 23; row++) {
				Kfusion(row) = HK185*(HK166*P(4,row) + HK167*P(5,row) + HK168*P(6,row) + HK169*P(row,24) - HK169*P(9,row) + HK170*P(3,row) + HK171*P(0,row) + HK172*P(1,row) - HK173*P(2,row));
			}

			Kfusion(24) = HK179*HK185;

		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _flow_innov(obs_index));

		if (obs_index == 0) {
			_fault_status.flags.bad_optflow_X = !is_fused;

		} else if (obs_index == 1) {
			_fault_status.flags.bad_optflow_Y = !is_fused;
		}

		if (is_fused) {
			_time_last_of_fuse = _time_last_imu;
		}
	}
}

// calculate optical flow body angular rate compensation
// returns false if bias corrected body rate data is unavailable
bool Ekf::calcOptFlowBodyRateComp()
{
	// reset the accumulators if the time interval is too large
	if (_delta_time_of > 1.0f) {
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
		return false;
	}

	bool is_body_rate_comp_available = false;
	const bool use_flow_sensor_gyro = PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(0)) && PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(1)) && PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(2));

	if (use_flow_sensor_gyro) {

		// if accumulation time differences are not excessive and accumulation time is adequate
		// compare the optical flow and and navigation rate data and calculate a bias error
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)
		    && (fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.1f)) {

			const Vector3f reference_body_rate(_imu_del_ang_of * (1.0f / _delta_time_of));

			const Vector3f measured_body_rate(_flow_sample_delayed.gyro_xyz * (1.0f / _flow_sample_delayed.dt));

			// calculate the bias estimate using  a combined LPF and spike filter
			_flow_gyro_bias = _flow_gyro_bias * 0.99f + matrix::constrain(measured_body_rate - reference_body_rate, -0.1f, 0.1f) * 0.01f;

			is_body_rate_comp_available = true;
		}

	} else {
		// Use the EKF gyro data if optical flow sensor gyro data is not available
		// for clarification of the sign see definition of flowSample and imuSample in common.h
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)) {
			_flow_sample_delayed.gyro_xyz = -_imu_del_ang_of / _delta_time_of * _flow_sample_delayed.dt;
			_flow_gyro_bias.zero();

			is_body_rate_comp_available = true;
		}
	}

	// reset the accumulators
	_imu_del_ang_of.setZero();
	_delta_time_of = 0.0f;
	return is_body_rate_comp_available;
}

// calculate the measurement variance for the optical flow sensor (rad/sec)^2
float Ekf::calcOptFlowMeasVar()
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.0f - (float)_params.flow_qual_min);

	if (weighting >= 1.0f) {
		weighting = math::constrain(((float)_flow_sample_delayed.quality - (float)_params.flow_qual_min) / weighting, 0.0f,
					    1.0f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.0f - weighting));

	return R_LOS;
}
