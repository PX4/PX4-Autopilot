/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file wind.cpp
 * Helper functions for wind states
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_wind_init_and_cov_from_wind_speed_and_direction.h>

void Ekf::resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy,
		float wind_direction_accuracy)
{
	if (!_control_status.flags.in_air) {

		const float wind_speed_constrained = math::max(wind_speed, 0.0f);
		const float wind_direction_var = sq(wind_direction_accuracy);
		const float wind_speed_var = sq(wind_speed_accuracy);

		Vector2f wind;
		Vector2f wind_var;

		sym::ComputeWindInitAndCovFromWindSpeedAndDirection(wind_speed_constrained, wind_direction, wind_speed_var,
				wind_direction_var, &wind, &wind_var);

		ECL_INFO("reset wind states to external observation");
		_information_events.flags.reset_wind_to_ext_obs = true;
		_external_wind_init = true;

		resetWindTo(wind, wind_var);

	}
}

void Ekf::resetWindTo(const Vector2f &wind, const Vector2f &wind_var)
{
	_state.wind_vel = wind;

	if (PX4_ISFINITE(wind_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx,
						      math::min(sq(_params.initial_wind_uncertainty), wind_var(0)));
	}

	if (PX4_ISFINITE(wind_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx + 1,
						      math::min(sq(_params.initial_wind_uncertainty), wind_var(1)));
	}
}

void Ekf::resetWindCov()
{
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, sq(_params.initial_wind_uncertainty));
}

void Ekf::resetWindToZero()
{
	ECL_INFO("reset wind to zero");
	_state.wind_vel.setZero();
	resetWindCov();
}
