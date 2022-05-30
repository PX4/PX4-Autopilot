/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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
 * @file ev_fusion.cpp
 * Function for fusing external vision (EV) velocity and position measurements
 */

#include "ekf.h"

void Ekf::updateEvVel(const extVisionSample &ev_sample)
{
	// innovation gate size
	const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

	auto &ev_vel = _aid_src_ev_vel;

	for (int i = 0; i < 3; i++) {
		ev_vel.observation[i] = ev_sample.vel(i);
		ev_vel.observation_variance[i] = fmaxf(ev_sample.velCov(i, i), sq(0.05f));

		ev_vel.innovation[i] = _state.vel(i) - ev_sample.vel(i);
		ev_vel.innovation_variance[i] = P(4 + i, 4 + i) + ev_vel.observation_variance[i];
		ev_vel.test_ratio[i] = sq(ev_vel.innovation[i]) / (sq(innov_gate) * ev_vel.innovation_variance[i]);

		ev_vel.innovation_rejected[i] = (ev_vel.test_ratio[i] > 1.f);
	}

	// vz special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && ev_vel.innovation_rejected[2]) {
		const float innov_limit = innov_gate * sqrtf(ev_vel.innovation_variance[2]);
		ev_vel.innovation[2] = math::constrain(ev_vel.innovation[2], -innov_limit, innov_limit);
		ev_vel.innovation_rejected[2] = false;
	}

	ev_vel.timestamp_sample = ev_sample.time_us;
}

void Ekf::updateEvPos(const extVisionSample &ev_sample)
{
	bool updated = false;

	// innovation gate size
	float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	auto &ev_pos = _aid_src_ev_pos;

	if (!_fuse_hpos_as_odom) {
		// use the absolute position
		for (int i = 0; i < 3; i++) {
			ev_pos.observation[i] = _ev_sample_delayed.pos(i);
			ev_pos.observation_variance[i] = fmaxf(_ev_sample_delayed.posVar(i), sq(0.01f));

			ev_pos.innovation[i] = _state.pos(i) - _ev_sample_delayed.pos(i);
		}

		updated = true;

	} else if (_hpos_prev_available) {
		// calculate the change in position since the last measurement
		Vector3f delta_pos = _ev_sample_delayed.pos - _ev_sample_delayed_prev.pos;

		for (int i = 0; i < 3; i++) {
			ev_pos.observation[i] = delta_pos(i);

			// observation 1-STD error, incremental pos observation is expected to have more uncertainty
			ev_pos.observation_variance[i] = fmaxf(_ev_sample_delayed.posVar(i), sq(0.5f));

			ev_pos.innovation[i] = _state.pos(i) - _hpos_pred_prev(i) - delta_pos(i);
		}

		updated = true;
	}

	if (updated) {

		for (int i = 0; i < 3; i++) {
			ev_pos.innovation_variance[i] = P(7 + i, 7 + i) + ev_pos.observation_variance[i];
			ev_pos.test_ratio[i] = sq(ev_pos.innovation[i]) / (sq(innov_gate) * ev_pos.innovation_variance[i]);

			ev_pos.innovation_rejected[i] = (ev_pos.test_ratio[i] > 1.f);
		}

		// z special case if there is bad vertical acceleration data, then don't reject measurement,
		// but limit innovation to prevent spikes that could destabilise the filter
		if (_fault_status.flags.bad_acc_vertical && ev_pos.innovation_rejected[2]) {
			const float innov_limit = innov_gate * sqrtf(ev_pos.innovation_variance[2]);
			ev_pos.innovation[2] = math::constrain(ev_pos.innovation[2], -innov_limit, innov_limit);
			ev_pos.innovation_rejected[2] = false;
		}

		ev_pos.timestamp_sample = ev_sample.time_us;
	}
}

void Ekf::fuseEvVel()
{
	// velocity
	auto &ev_vel = _aid_src_ev_vel;

	// vx & vy
	ev_vel.fusion_enabled[0] = true;
	ev_vel.fusion_enabled[1] = true;

	if (!ev_vel.innovation_rejected[0] && !ev_vel.innovation_rejected[1]) {
		for (int i = 0; i < 2; i++) {
			if (fuseVelPosHeight(ev_vel.innovation[i], ev_vel.innovation_variance[i], i)) {
				ev_vel.fused[i] = true;
				ev_vel.time_last_fuse[i] = _time_last_imu;
			}
		}
	}

	// vz
	ev_vel.fusion_enabled[2] = true;

	if (ev_vel.fusion_enabled[2] && !ev_vel.innovation_rejected[2]) {
		if (fuseVelPosHeight(ev_vel.innovation[2], ev_vel.innovation_variance[2], 2)) {
			ev_vel.fused[2] = true;
			ev_vel.time_last_fuse[2] = _time_last_imu;
		}
	}
}

void Ekf::fuseEvPos()
{
	auto &ev_pos = _aid_src_ev_pos;

	// x & y
	ev_pos.fusion_enabled[0] = true;
	ev_pos.fusion_enabled[1] = true;

	if (!ev_pos.innovation_rejected[0] && !ev_pos.innovation_rejected[1]) {
		for (int i = 0; i < 2; i++) {
			if (fuseVelPosHeight(ev_pos.innovation[i], ev_pos.innovation_variance[i], 3 + i)) {
				ev_pos.fused[i] = true;
				ev_pos.time_last_fuse[i] = _time_last_imu;
			}
		}
	}

	// z
	ev_pos.fusion_enabled[2] = _control_status.flags.ev_hgt;

	if (ev_pos.fusion_enabled[2] && !ev_pos.innovation_rejected[2]) {
		if (fuseVelPosHeight(ev_pos.innovation[2], ev_pos.innovation_variance[2], 2)) {
			ev_pos.fused[2] = true;
			ev_pos.time_last_fuse[2] = _time_last_imu;
		}
	}
}
