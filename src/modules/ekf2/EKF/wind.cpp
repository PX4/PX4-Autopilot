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

void Ekf::resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy, float wind_direction_accuracy)
{
	const float wind_speed_constrained = math::max(wind_speed, 0.0f);

	// wind direction is given as azimuth where wind blows FROM, we need direction where wind blows TO
	const float wind_direction_rad = wrap_pi(math::radians(wind_direction) + M_PI_F);
	Vector2f wind = wind_speed_constrained * Vector2f(cosf(wind_direction_rad), sinf(wind_direction_rad));

	ECL_INFO("reset wind states to external observation");
	_information_events.flags.reset_wind_to_ext_obs = true;

	Vector2f innov = _state.wind_vel - wind;
	float R = sq(0.1f);
	Vector2f innov_var{P(State::wind_vel.idx, State::wind_vel.idx) + R, P(State::wind_vel.idx + 1, State::wind_vel.idx + 1) + R};
	const bool control_status_wind_prev = _control_status.flags.wind;
	_control_status.flags.wind = true;
	fuseDirectStateMeasurement(innov(0), innov_var(0), R, State::wind_vel.idx);
	fuseDirectStateMeasurement(innov(1), innov_var(1), R, State::wind_vel.idx + 1);
	_control_status.flags.wind = control_status_wind_prev;

	// reset the horizontal velocity variances to allow the velocity states to be pulled towards
	// a solution that is aligned with the newly set wind estimates
	static constexpr float hor_vel_var = 25.0f;
	P.uncorrelateCovarianceSetVariance<2>(State::vel.idx, hor_vel_var);
}

void Ekf::resetWindTo(const Vector2f &wind, const Vector2f &wind_var)
{
	_state.wind_vel = wind;

	if (PX4_ISFINITE(wind_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx, math::max(sq(_params.initial_wind_uncertainty), wind_var(0)));
	}

	if (PX4_ISFINITE(wind_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx + 1, math::max(sq(_params.initial_wind_uncertainty), wind_var(1)));
	}
}

void Ekf::resetWindCov()
{
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, sq(_params.initial_wind_uncertainty));
}

void Ekf::resetWind()
{
#if defined(CONFIG_EKF2_AIRSPEED)
	if (_control_status.flags.fuse_aspd && isRecent(_airspeed_sample_delayed.time_us, 1e6)) {
		resetWindUsingAirspeed(_airspeed_sample_delayed);
		return;
	}
#endif // CONFIG_EKF2_AIRSPEED

	resetWindToZero();
}

void Ekf::resetWindToZero()
{
	ECL_INFO("reset wind to zero");

	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();

	resetWindCov();
}
