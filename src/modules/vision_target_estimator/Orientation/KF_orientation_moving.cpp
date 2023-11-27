/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file KF_orientation_moving.cpp
 * @brief Filter to estimate the orientation of moving targets. State: [theta, theta_dot]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_orientation_moving.h"

namespace vision_target_estimator
{

void KF_orientation_moving::predictState(float dt)
{
	/*
	⎡dt⋅theta_dot + theta⎤
	⎣      theta_dot     ⎦
	*/

	_state(0, 0) = _state(0, 0) + _state(1, 0) * dt;
	_state(1, 0) = _state(1, 0);
}

void KF_orientation_moving::predictCov(float dt)
{
	/*
	⎡dt⋅p(0;1) + dt⋅(dt⋅p(1;1) + p(0;1)) + p(0;0)      dt⋅p(1;1) + p(0;1)⎤
	⎢                                                                    ⎥
	⎣             dt⋅p(1;1) + p(0;1)                         p(1;1)      ⎦
	*/

	const float off_diag = dt * _covariance(1, 1) + _covariance(0, 1);
	_covariance(0, 0) += dt * _covariance(0, 1) + dt * (dt * _covariance(1, 1) + _covariance(0, 1));
	_covariance(1, 0) = off_diag;
	_covariance(0, 1) = off_diag;
}


bool KF_orientation_moving::update()
{
	// Avoid zero-division
	if (fabsf(_innov_cov) < 1e-6f) {
		return false;
	}

	const float beta = _innov / _innov_cov * _innov;

	// Normalized innovation Squared threshold. Checks whether innovation is consistent with innovation covariance.
	if (beta > _nis_threshold) {
		return false;
	}

	const matrix::Matrix<float, 2, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;

	_state(0, 0) = matrix::wrap_pi(_state(0, 0));
	_state(1, 0) = matrix::wrap_pi(_state(1, 0));

	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KF_orientation_moving::setH(matrix::Vector<float, 2> h_meas)
{
	// h_meas = [theta, theta_dot]; For this filter: [theta, theta_dot]

	_meas_matrix(0, 0) = h_meas(0);
	_meas_matrix(0, 1) = h_meas(1);
}

void KF_orientation_moving::syncState(float dt)
{
	_sync_state(0, 0) = matrix::wrap_pi(_state(0, 0) - _state(1, 0) * dt);
	_sync_state(1, 0) = _state(1, 0);
}

float KF_orientation_moving::computeInnovCov(float meas_unc)
{
	/*
	[h(0)⋅(cov(0;0)⋅h(0) + cov(0;1)⋅h(1)) + h(1)⋅(cov(0;1)⋅h(0) + cov(1;1)⋅h(1)) + r]
	*/

	_innov_cov = _meas_matrix(0, 0) * (_covariance(0, 0) * _meas_matrix(0, 0) + _covariance(0,
					   1) * _meas_matrix(0, 1)) + _meas_matrix(0, 1) * (_covariance(0, 1) * _meas_matrix(0, 0) + _covariance(1,
							   1) * _meas_matrix(0, 1)) + meas_unc;
	return _innov_cov;
}

float KF_orientation_moving::computeInnov(float meas)
{
	/* z - H*x */
	_innov = matrix::wrap_pi(meas - (_meas_matrix * _sync_state)(0, 0));
	return _innov;
}

} // namespace vision_target_estimator
