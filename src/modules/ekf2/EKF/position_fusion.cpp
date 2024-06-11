/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

void Ekf::updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
		const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 2; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.pos(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		const int state_index = State::pos.idx + i;
		aid_src.innovation_variance[i] = P(state_index, state_index) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var,
		const float innov_gate, estimator_aid_source1d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	aid_src.observation = obs;
	aid_src.innovation = _state.pos(2) - aid_src.observation;

	aid_src.observation_variance = math::max(sq(0.01f), obs_var);
	aid_src.innovation_variance = P(State::pos.idx + 2, State::pos.idx + 2) + aid_src.observation_variance;

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

void Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// x & y
	if (!aid_src.innovation_rejected
	    && fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], aid_src.observation_variance[0], State::pos.idx + 0)
	    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], aid_src.observation_variance[1], State::pos.idx + 1)
	   ) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_pos_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}
}

void Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// z
	if (!aid_src.innovation_rejected
	    && fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance, State::pos.idx + 2)
	   ) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hgt_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	_output_predictor.resetHorizontalPositionTo(delta_horz_pos);

	// record the state change
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		_state_reset_status.posNE_change = delta_horz_pos;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posNE_change += delta_horz_pos;
	}

	_state_reset_status.reset_count.posNE++;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - _state_reset_status.posNE_change);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	//_gps_pos_b_est.setBias(_gps_pos_b_est.getBias() + _state_reset_status.posNE_change);

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;
}

void Ekf::resetVerticalPositionTo(const float new_vert_pos, float new_vert_pos_var)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = new_vert_pos - old_vert_pos;

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_predictor.resetVerticalPositionTo(new_vert_pos, delta_z);

	// record the state change
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		_state_reset_status.posD_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posD_change += delta_z;
	}

	_state_reset_status.reset_count.posD++;

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_b_est.setBias(_baro_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - delta_z);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_TERRAIN)
	terrainHandleVerticalPositionReset(delta_z);
#endif

	// Reset the timout timer
	_time_last_hgt_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalPositionToLastKnown()
{
	ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
	_information_events.flags.reset_pos_to_last_known = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalPositionTo(_last_known_pos.xy(), sq(_params.pos_noaid_noise));
}

void Ekf::resetHorizontalPositionToExternal(const Vector2f &new_horiz_pos, float horiz_accuracy)
{
	ECL_INFO("reset position to external observation");
	_information_events.flags.reset_pos_to_ext_obs = true;

	resetHorizontalPositionTo(new_horiz_pos, sq(horiz_accuracy));
}
