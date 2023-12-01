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
 * @file KF_orientation_static.cpp
 * @brief Filter to estimate the orientation of static targets. State: [yaw]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_orientation_static.h"

namespace vision_target_estimator
{

bool KF_orientation_static::update()
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

	const float kalmanGain = _state_covariance * _meas_matrix_row_vect / _innov_cov;

	_state = matrix::wrap_pi(_state + kalmanGain * _innov);

	_state_covariance = _state_covariance - kalmanGain * _meas_matrix_row_vect * _state_covariance;

	return true;
}

void KF_orientation_static::setH(const matrix::Vector<float, 2> &h_meas)
{
	_meas_matrix_row_vect = h_meas(AugmentedState::yaw);
}

void KF_orientation_static::syncState(float dt)
{
	_sync_state = _state;
}

float KF_orientation_static::computeInnovCov(float meas_unc)
{
	_innov_cov = _meas_matrix_row_vect * _state_covariance * _meas_matrix_row_vect + meas_unc;
	return _innov_cov;
}

float KF_orientation_static::computeInnov(float meas)
{
	/* z - H*x */
	_innov = matrix::wrap_pi(meas - (_meas_matrix_row_vect * _sync_state));
	return _innov;
}

} // namespace vision_target_estimator
