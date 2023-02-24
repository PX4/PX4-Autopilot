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
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include <mathlib/mathlib.h>
#include "ekf.h"

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
				     const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 2; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		aid_src.innovation_variance[i] = P(4 + i, 4 + i) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var,
				     const float innov_gate, estimator_aid_source3d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 3; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		aid_src.innovation_variance[i] = P(4 + i, 4 + i) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	// vz special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var,
		const float innov_gate, estimator_aid_source1d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	aid_src.observation = obs;
	aid_src.innovation = _state.pos(2) - aid_src.observation;

	aid_src.observation_variance = math::max(sq(0.01f), obs_var);
	aid_src.innovation_variance = P(9, 9) + aid_src.observation_variance;

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
		const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 2; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.pos(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		aid_src.innovation_variance[i] = P(7 + i, 7 + i) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.timestamp_sample = time_us;
}

void Ekf::fuseVelocity(estimator_aid_source2d_s &aid_src)
{
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		// vx, vy
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], 0)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], 1)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		// vx, vy, vz
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], 0)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], 1)
		    && fuseVelPosHeight(aid_src.innovation[2], aid_src.innovation_variance[2], 2)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// x & y
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], 3)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], 4)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// z
	if (aid_src.fusion_enabled && !aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation, aid_src.innovation_variance, 5)) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;
		}
	}
}

// Helper function that fuses a single velocity or position measurement
bool Ekf::fuseVelPosHeight(const float innov, const float innov_var, const int obs_index)
{
	Vector24f Kfusion;  // Kalman gain vector for any single observation - sequential fusion is used.
	const unsigned state_index = obs_index + 4;  // we start with vx and this is the 4. state

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < _k_num_states; row++) {
		Kfusion(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(Kfusion);

	SquareMatrix24f KHP;

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			KHP(row, column) = Kfusion(row) * P(state_index, column);
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	setVelPosStatus(obs_index, healthy);

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, innov);

		return true;
	}

	return false;
}

void Ekf::setVelPosStatus(const int index, const bool healthy)
{
	switch (index) {
	case 0:
		if (healthy) {
			_fault_status.flags.bad_vel_N = false;
			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_N = true;
		}

		break;

	case 1:
		if (healthy) {
			_fault_status.flags.bad_vel_E = false;
			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_E = true;
		}

		break;

	case 2:
		if (healthy) {
			_fault_status.flags.bad_vel_D = false;
			_time_last_ver_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_D = true;
		}

		break;

	case 3:
		if (healthy) {
			_fault_status.flags.bad_pos_N = false;
			_time_last_hor_pos_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_N = true;
		}

		break;

	case 4:
		if (healthy) {
			_fault_status.flags.bad_pos_E = false;
			_time_last_hor_pos_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_E = true;
		}

		break;

	case 5:
		if (healthy) {
			_fault_status.flags.bad_pos_D = false;
			_time_last_hgt_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_D = true;
		}

		break;
	}
}
