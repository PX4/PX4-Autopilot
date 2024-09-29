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

// TODO: rename to KF_position and clean up comments

/**
 * @file KF_position_moving.cpp
 * @brief Filter to estimate the pose of moving targets. State: [pos_rel, vel_uav, bias, acc_target, vel_target]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_orientation_unified.h"
#include "python_derivation/generated/syncState.h"
#include "python_derivation/generated/predictState.h"
#include "python_derivation/generated/predictCov.h"
#include "python_derivation/generated/computeInnovCov.h"

namespace vision_target_estimator
{

void KF_orientation_unified::predictState(float dt)
{
	matrix::Vector<float, vtest::State::size> state_updated;
	sym::Predictstate(dt, _state, &state_updated);

	for (int i = 0; i < vtest::State::size; i++) {
		_state(i) = matrix::wrap_pi(state_updated(i));
	}
}

void KF_orientation_unified::predictCov(float dt)
{
	matrix::Matrix<float, vtest::State::size, vtest::State::size> cov_updated;
	sym::Predictcov(dt, _state_covariance, &cov_updated);
	_state_covariance = cov_updated;
}


bool KF_orientation_unified::update()
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

	const matrix::Matrix<float, vtest::State::size, 1> kalmanGain = _state_covariance * _meas_matrix_row_vect / _innov_cov;

	_state = _state + kalmanGain * _innov;

	for (int i = 0; i < vtest::State::size; i++) {
		_state(i) = matrix::wrap_pi(_state(i));
	}

	_state_covariance = _state_covariance - kalmanGain * _meas_matrix_row_vect.transpose() * _state_covariance;

	return true;
}

void KF_orientation_unified::syncState(float dt)
{

	matrix::Vector<float, vtest::State::size> synced_state;
	sym::Syncstate(dt, _state, &synced_state);

	for (int i = 0; i < vtest::State::size; i++) {
		_sync_state(i) = matrix::wrap_pi(synced_state(i));
	}
}

float KF_orientation_unified::computeInnovCov(float meas_unc)
{
	float innov_cov_updated;
	sym::Computeinnovcov(meas_unc, _state_covariance, _meas_matrix_row_vect.transpose(), &innov_cov_updated);
	_innov_cov = innov_cov_updated;

	return _innov_cov;
}

float KF_orientation_unified::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix_row_vect.transpose() * _sync_state)(0, 0);
	return _innov;
}

} // namespace vision_target_estimator
