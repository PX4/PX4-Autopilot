/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file PreFlightCheckHelper.cpp
 * Class handling the EKF2 innovation pre flight checks
 */

#include "PreFlightChecker.hpp"

void PreFlightChecker::update(const float dt, const ekf2_innovations_s &innov)
{
	const float alpha = InnovationLpf::computeAlphaFromDtAndTauInv(dt, _innov_lpf_tau_inv);

	_has_heading_failed = preFlightCheckHeadingFailed(innov, alpha);
	_has_horiz_vel_failed = preFlightCheckHorizVelFailed(innov, alpha);
	_has_vert_vel_failed = preFlightCheckVertVelFailed(innov, alpha);
	_has_height_failed = preFlightCheckHeightFailed(innov, alpha);
}

bool PreFlightChecker::preFlightCheckHeadingFailed(const ekf2_innovations_s &innov, const float alpha)
{
	const float heading_test_limit = selectHeadingTestLimit();
	const float heading_innov_spike_lim = 2.0f * heading_test_limit;

	const float heading_innov_lpf = _filter_heading_innov.update(innov.heading_innov, alpha, heading_innov_spike_lim);

	return checkInnovFailed(innov.heading_innov, heading_innov_lpf, heading_test_limit);
}

float PreFlightChecker::selectHeadingTestLimit()
{
	// Select the max allowed heading innovaton depending on whether we are not aiding navigation using
	// observations in the NE reference frame and if the vehicle can use GPS course to realign in flight (fixedwing sideslip fusion).
	const bool is_ne_aiding = _is_using_gps_aiding || _is_using_ev_pos_aiding;

	return (is_ne_aiding && !_can_observe_heading_in_flight)
	       ? _nav_heading_innov_test_lim // more restrictive test limit
	       : _heading_innov_test_lim; // less restrictive test limit
}

bool PreFlightChecker::preFlightCheckHorizVelFailed(const ekf2_innovations_s &innov, const float alpha)
{
	bool has_failed = false;

	if (_is_using_gps_aiding || _is_using_ev_pos_aiding) {
		const Vector2f vel_ne_innov = Vector2f(innov.vel_pos_innov);
		Vector2f vel_ne_innov_lpf;
		vel_ne_innov_lpf(0) = _filter_vel_n_innov.update(vel_ne_innov(0), alpha, _vel_innov_spike_lim);
		vel_ne_innov_lpf(1) = _filter_vel_n_innov.update(vel_ne_innov(1), alpha, _vel_innov_spike_lim);
		has_failed |= checkInnov2DFailed(vel_ne_innov, vel_ne_innov_lpf, _vel_innov_test_lim);
	}

	if (_is_using_flow_aiding) {
		const Vector2f flow_innov = Vector2f(innov.flow_innov);
		Vector2f flow_innov_lpf;
		flow_innov_lpf(0) = _filter_flow_x_innov.update(flow_innov(0), alpha, _flow_innov_spike_lim);
		flow_innov_lpf(1) = _filter_flow_x_innov.update(flow_innov(1), alpha, _flow_innov_spike_lim);
		has_failed |= checkInnov2DFailed(flow_innov, flow_innov_lpf, _flow_innov_test_lim);
	}

	return has_failed;
}

bool PreFlightChecker::preFlightCheckVertVelFailed(const ekf2_innovations_s &innov, const float alpha)
{
	const float vel_d_innov = innov.vel_pos_innov[2];
	const float vel_d_innov_lpf = _filter_vel_d_innov.update(vel_d_innov, alpha, _vel_innov_spike_lim);
	return checkInnovFailed(vel_d_innov, vel_d_innov_lpf, _vel_innov_test_lim);
}

bool PreFlightChecker::preFlightCheckHeightFailed(const ekf2_innovations_s &innov, const float alpha)
{
	const float hgt_innov = innov.vel_pos_innov[5];
	const float hgt_innov_lpf = _filter_hgt_innov.update(hgt_innov, alpha, _hgt_innov_spike_lim);
	return checkInnovFailed(hgt_innov, hgt_innov_lpf, _hgt_innov_test_lim);
}

bool PreFlightChecker::checkInnovFailed(const float innov, const float innov_lpf, const float test_limit)
{
	return fabsf(innov_lpf) > test_limit || fabsf(innov) > 2.0f * test_limit;
}

bool PreFlightChecker::checkInnov2DFailed(const Vector2f &innov, const Vector2f &innov_lpf, const float test_limit)
{
	return innov_lpf.norm_squared() > sq(test_limit)
	       || innov.norm_squared() > sq(2.0f * test_limit);
}

void PreFlightChecker::reset()
{
	_is_using_gps_aiding = false;
	_is_using_flow_aiding = false;
	_is_using_ev_pos_aiding = false;
	_has_heading_failed = false;
	_has_horiz_vel_failed = false;
	_has_vert_vel_failed = false;
	_has_height_failed = false;
	_filter_vel_n_innov.reset();
	_filter_vel_e_innov.reset();
	_filter_vel_d_innov.reset();
	_filter_hgt_innov.reset();
	_filter_heading_innov.reset();
	_filter_flow_x_innov.reset();
	_filter_flow_y_innov.reset();
}
