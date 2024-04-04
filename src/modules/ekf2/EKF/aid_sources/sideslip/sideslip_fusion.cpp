/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file sideslip_fusion.cpp
 * sideslip fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_sideslip_innov_and_innov_var.h>
#include <ekf_derivation/generated/compute_sideslip_h_and_k.h>

#include <mathlib/mathlib.h>

void Ekf::controlBetaFusion(const imuSample &imu_delayed)
{
	_control_status.flags.fuse_beta = _params.beta_fusion_enabled && _control_status.flags.fixed_wing
		&& _control_status.flags.in_air && !_control_status.flags.fake_pos;

	if (_control_status.flags.fuse_beta) {

		// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fusion had been enabled externally:
		const bool beta_fusion_time_triggered = isTimedOut(_aid_src_sideslip.time_last_fuse, _params.beta_avg_ft_us);

		if (beta_fusion_time_triggered) {

			updateSideslip(_aid_src_sideslip);
			_innov_check_fail_status.flags.reject_sideslip = _aid_src_sideslip.innovation_rejected;

			// If starting wind state estimation, reset the wind states and covariances before fusing any data
			if (!_control_status.flags.wind) {
				// activate the wind states
				_control_status.flags.wind = true;
				// reset the timeout timers to prevent repeated resets
				_aid_src_sideslip.time_last_fuse = imu_delayed.time_us;
				resetWindToZero();
			}

			fuseSideslip(_aid_src_sideslip);
		}
	}
}

void Ekf::updateSideslip(estimator_aid_source1d_s &sideslip) const
{
	// reset flags
	resetEstimatorAidStatus(sideslip);

	const float R = sq(_params.beta_noise); // observation noise variance

	float innov = 0.f;
	float innov_var = 0.f;
	sym::ComputeSideslipInnovAndInnovVar(_state.vector(), P, R, FLT_EPSILON, &innov, &innov_var);

	sideslip.observation = 0.f;
	sideslip.observation_variance = R;
	sideslip.innovation = innov;
	sideslip.innovation_variance = innov_var;

	sideslip.timestamp_sample = _time_delayed_us;

	const float innov_gate = fmaxf(_params.beta_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(sideslip, innov_gate);
}

void Ekf::fuseSideslip(estimator_aid_source1d_s &sideslip)
{
	if (sideslip.innovation_rejected) {
		return;
	}
	// determine if we need the sideslip fusion to correct states other than wind
	bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	// Reset covariance and states if the calculation is badly conditioned
	if ((sideslip.innovation_variance < sideslip.observation_variance)
	    || (sideslip.innovation_variance < FLT_EPSILON)) {
		_fault_status.flags.bad_sideslip = true;

		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWind();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

		return;
	}

	_fault_status.flags.bad_sideslip = false;

	VectorState H; // Observation jacobian
	VectorState K; // Kalman gain vector

	sym::ComputeSideslipHAndK(_state.vector(), P, sideslip.innovation_variance, FLT_EPSILON, &H, &K);

	if (update_wind_only) {
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0);
		K.setZero();
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind;
	}

	const bool is_fused = measurementUpdate(K, H, sideslip.observation_variance, sideslip.innovation);

	sideslip.fused = is_fused;
	_fault_status.flags.bad_sideslip = !is_fused;

	if (is_fused) {
		sideslip.time_last_fuse = _time_delayed_us;
	}
}
