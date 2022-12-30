/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file KF_orientation_static.h
 * Simple Kalman Filter for static target
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include "KF_orientation_static.h"

namespace landing_target_estimator
{

void KF_orientation_static::predictState(float dt)
{
	_state = _state;
}

void KF_orientation_static::predictCov(float dt)
{
	_covariance = _covariance;
}


bool KF_orientation_static::update()
{
	// Avoid zero-division
	if (_innov_cov  <= 0.000001f && _innov_cov  >= -0.000001f)  {
		return false;
	}

	float beta = _innov / _innov_cov * _innov;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	float kalmanGain = _covariance * _meas_matrix / _innov_cov;

	_state = matrix::wrap_pi(_state + kalmanGain * _innov);

	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KF_orientation_static::setH(matrix::Vector<float, 2> h_meas)
{
	// h_meas = [theta, theta_dot]

	// For this filter: [theta]

	_meas_matrix = h_meas(0);
}

void KF_orientation_static::syncState(float dt)
{
	_sync_state = _state;
}

float KF_orientation_static::computeInnovCov(float meas_unc)
{
	_innov_cov = _meas_matrix * _covariance * _meas_matrix + meas_unc;
	return _innov_cov;
}

float KF_orientation_static::computeInnov(float meas)
{
	/* z - H*x */
	_innov = matrix::wrap_pi(meas - (_meas_matrix * _sync_state));
	return _innov;
}

} // namespace landing_target_estimator