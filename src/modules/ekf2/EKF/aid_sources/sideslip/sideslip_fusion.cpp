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
#include <ekf_derivation/generated/compute_sideslip_h.h>

#include <mathlib/mathlib.h>

void Ekf::controlBetaFusion(const imuSample &imu_delayed)
{
	_control_status.flags.fuse_beta = _params.beta_fusion_enabled
					  && (_control_status.flags.fixed_wing || _control_status.flags.fuse_aspd)
					  && _control_status.flags.in_air
					  && !_control_status.flags.fake_pos;

	if (_control_status.flags.fuse_beta) {

		// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fusion had been enabled externally:
		const bool beta_fusion_time_triggered = isTimedOut(_aid_src_sideslip.time_last_fuse, _params.beta_avg_ft_us);

		if (beta_fusion_time_triggered) {

			updateSideslip(_aid_src_sideslip);
			_innov_check_fail_status.flags.reject_sideslip = _aid_src_sideslip.innovation_rejected;

			if (fuseSideslip(_aid_src_sideslip)) {
				_control_status.flags.wind = true;

			} else if (!_external_wind_init && !_control_status.flags.wind) {
				resetWindCov();
			}
		}
	}
}

void Ekf::updateSideslip(estimator_aid_source1d_s &aid_src) const
{
	float observation = 0.f;
	const float R = math::max(sq(_params.beta_noise), sq(0.01f)); // observation noise variance
	const float epsilon = 1e-3f;
	float innov;
	float innov_var;
	sym::ComputeSideslipInnovAndInnovVar(_state.vector(), P, R, epsilon, &innov, &innov_var);

	updateAidSourceStatus(aid_src,
			      _time_delayed_us,                         // sample timestamp
			      observation,                              // observation
			      R,                                        // observation variance
			      innov,                                    // innovation
			      innov_var,                                // innovation variance
			      math::max(_params.beta_innov_gate, 1.f)); // innovation gate
}

bool Ekf::fuseSideslip(estimator_aid_source1d_s &sideslip)
{
	if (sideslip.innovation_rejected) {
		return false;
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
			resetWindCov();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

		return false;
	}

	_fault_status.flags.bad_sideslip = false;

	const float epsilon = 1e-3f;

	const VectorState H = sym::ComputeSideslipH(_state.vector(), epsilon);
	VectorState K = P * H / sideslip.innovation_variance;

	if (update_wind_only) {
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0);
		K.setZero();
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind;
	}

	measurementUpdate(K, H, sideslip.observation_variance, sideslip.innovation);

	sideslip.fused = true;
	sideslip.time_last_fuse = _time_delayed_us;

	return true;
}
