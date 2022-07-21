/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file gps_fusion.cpp
 * Function for fusing gps measurements
 */

/* #include <mathlib/mathlib.h> */
#include "ekf.h"

void Ekf::updateGpsYaw(const gpsSample &gps_sample)
{
	auto &gps_yaw = _aid_src_gnss_yaw;
	resetEstimatorAidStatusFlags(gps_yaw);

	if (PX4_ISFINITE(gps_sample.yaw)) {
		// initially populate for estimator_aid_src_gnss_yaw logging

		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		const float measured_hdg = wrap_pi(_gps_sample_delayed.yaw + _gps_yaw_offset);

		gps_yaw.observation = measured_hdg;
		gps_yaw.observation_variance = sq(fmaxf(_params.gps_heading_noise, 1.0e-2f));

		// define the predicted antenna array vector and rotate into earth frame
		const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
		const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;
		const float predicted_hdg = atan2f(ant_vec_ef(1), ant_vec_ef(0));
		gps_yaw.innovation = wrap_pi(predicted_hdg - gps_yaw.observation);

		gps_yaw.fusion_enabled = _control_status.flags.gps_yaw;

		gps_yaw.timestamp_sample = gps_sample.time_us;
	}
}

void Ekf::updateGpsVel(const gpsSample &gps_sample)
{
	auto &gps_vel = _aid_src_gnss_vel;
	resetEstimatorAidStatus(gps_vel);

	const float vel_var = sq(gps_sample.sacc);
	const Vector3f obs_var{vel_var, vel_var, vel_var * sq(1.5f)};

	// innovation gate size
	const float innov_gate = fmaxf(_params.gps_vel_innov_gate, 1.f);

	for (int i = 0; i < 3; i++) {
		gps_vel.observation[i] = gps_sample.vel(i);
		gps_vel.observation_variance[i] = obs_var(i);

		gps_vel.innovation[i] = _state.vel(i) - gps_sample.vel(i);
		gps_vel.innovation_variance[i] = P(4 + i, 4 + i) + obs_var(i);
	}

	setEstimatorAidStatusTestRatio(gps_vel, innov_gate);

	// vz special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && gps_vel.innovation_rejected[2]) {
		const float innov_limit = innov_gate * sqrtf(gps_vel.innovation_variance[2]);
		gps_vel.innovation[2] = math::constrain(gps_vel.innovation[2], -innov_limit, innov_limit);
		gps_vel.innovation_rejected[2] = false;
	}

	gps_vel.timestamp_sample = gps_sample.time_us;
}

void Ekf::updateGpsPos(const gpsSample &gps_sample)
{
	auto &gps_pos = _aid_src_gnss_pos;
	resetEstimatorAidStatus(gps_pos);

	Vector3f position;
	position(0) = gps_sample.pos(0);
	position(1) = gps_sample.pos(1);

	// vertical position - gps measurement has opposite sign to earth z axis
	position(2) = -(gps_sample.hgt - getEkfGlobalOriginAltitude() - _gps_hgt_offset);

	const float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);

	Vector3f obs_var;

	if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// if we are using other sources of aiding, then relax the upper observation
		// noise limit which prevents bad GPS perturbing the position estimate
		obs_var(0) = obs_var(1) = sq(fmaxf(gps_sample.hacc, lower_limit));

	} else {
		// if we are not using another source of aiding, then we are reliant on the GPS
		// observations to constrain attitude errors and must limit the observation noise value.
		float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
		obs_var(0) = obs_var(1) = sq(math::constrain(gps_sample.hacc, lower_limit, upper_limit));
	}

	obs_var(2) = getGpsHeightVariance();

	// innovation gate size
	float innov_gate = fmaxf(_params.gps_pos_innov_gate, 1.f);

	for (int i = 0; i < 3; i++) {
		gps_pos.observation[i] = position(i);
		gps_pos.observation_variance[i] = obs_var(i);

		gps_pos.innovation[i] = _state.pos(i) - position(i);
		gps_pos.innovation_variance[i] = P(7 + i, 7 + i) + obs_var(i);
	}

	setEstimatorAidStatusTestRatio(gps_pos, innov_gate);

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && gps_pos.innovation_rejected[2]) {
		const float innov_limit = innov_gate * sqrtf(gps_pos.innovation_variance[2]);
		gps_pos.innovation[2] = math::constrain(gps_pos.innovation[2], -innov_limit, innov_limit);
		gps_pos.innovation_rejected[2] = false;
	}

	gps_pos.timestamp_sample = gps_sample.time_us;
}

void Ekf::fuseGpsVel()
{
	// velocity
	auto &gps_vel = _aid_src_gnss_vel;

	// vx & vy
	gps_vel.fusion_enabled[0] = true;
	gps_vel.fusion_enabled[1] = true;

	if (!gps_vel.innovation_rejected[0] && !gps_vel.innovation_rejected[1]) {
		for (int i = 0; i < 2; i++) {
			if (fuseVelPosHeight(gps_vel.innovation[i], gps_vel.innovation_variance[i], i)) {
				gps_vel.fused[i] = true;
				gps_vel.time_last_fuse[i] = _time_last_imu;
			}
		}
	}

	// vz
	gps_vel.fusion_enabled[2] = true;

	if (gps_vel.fusion_enabled[2] && !gps_vel.innovation_rejected[2]) {
		if (fuseVelPosHeight(gps_vel.innovation[2], gps_vel.innovation_variance[2], 2)) {
			gps_vel.fused[2] = true;
			gps_vel.time_last_fuse[2] = _time_last_imu;
		}
	}
}

void Ekf::fuseGpsPos()
{
	auto &gps_pos = _aid_src_gnss_pos;

	// x & y
	gps_pos.fusion_enabled[0] = true;
	gps_pos.fusion_enabled[1] = true;

	if (!gps_pos.innovation_rejected[0] && !gps_pos.innovation_rejected[1]) {
		for (int i = 0; i < 2; i++) {
			if (fuseVelPosHeight(gps_pos.innovation[i], gps_pos.innovation_variance[i], 3 + i)) {
				gps_pos.fused[i] = true;
				gps_pos.time_last_fuse[i] = _time_last_imu;
			}
		}
	}

	// z
	gps_pos.fusion_enabled[2] = _control_status.flags.gps_hgt;

	if (gps_pos.fusion_enabled[2] && !gps_pos.innovation_rejected[2]) {
		if (fuseVelPosHeight(gps_pos.innovation[2], gps_pos.innovation_variance[2], 5)) {
			gps_pos.fused[2] = true;
			gps_pos.time_last_fuse[2] = _time_last_imu;
		}
	}
}
