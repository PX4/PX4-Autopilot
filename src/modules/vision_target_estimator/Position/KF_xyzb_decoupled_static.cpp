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
 * @file KF_xyzb_decoupled_static.cpp
 * @brief Filter to estimate the pose of static targets. State: [pos_rel, vel_rel, bias]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_xyzb_decoupled_static.h"
#include "python_derivation/generated/decoupled_static_xyzb/predictCov.h"
#include "python_derivation/generated/decoupled_static_xyzb/computeInnovCov.h"

namespace vision_target_estimator
{

void KF_xyzb_decoupled_static::predictState(float dt, float acc_uav)
{
	_state(State::pos_rel) = _state(State::pos_rel) + _state(State::vel_rel) * dt - 0.5f * acc_uav * dt * dt;
	_state(State::vel_rel) = _state(State::vel_rel) - acc_uav * dt;
}

void KF_xyzb_decoupled_static::predictCov(float dt)
{
	matrix::Matrix<float, 3, 3> cov_updated;
	sym::Predictcov(dt, _input_var, _bias_var, _state_covariance, &cov_updated);
	_state_covariance = cov_updated;
}


bool KF_xyzb_decoupled_static::update()
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

	const matrix::Matrix<float, 3, 1> kalmanGain = _state_covariance * _meas_matrix_row_vect / _innov_cov;

	_state = _state + kalmanGain * _innov;
	_state_covariance = _state_covariance - kalmanGain * _meas_matrix_row_vect.transpose() * _state_covariance;

	return true;
}

void KF_xyzb_decoupled_static::setH(const matrix::Vector<float, 15> &h_meas, int direction)
{

	// For a static target, the relative velocity = - uav velocity
	if (direction == Direction::x) {
		_meas_matrix_row_vect(State::pos_rel) = h_meas(ExtendedState::pos_rel_x);
		_meas_matrix_row_vect(State::vel_rel) = -h_meas(ExtendedState::vel_uav_x);
		_meas_matrix_row_vect(State::bias) = h_meas(ExtendedState::bias_x);

	} else if (direction == Direction::y) {
		_meas_matrix_row_vect(State::pos_rel) = h_meas(ExtendedState::pos_rel_y);
		_meas_matrix_row_vect(State::vel_rel) = -h_meas(ExtendedState::vel_uav_y);
		_meas_matrix_row_vect(State::bias) = h_meas(ExtendedState::bias_y);

	} else {
		_meas_matrix_row_vect(State::pos_rel) = h_meas(ExtendedState::pos_rel_z);
		_meas_matrix_row_vect(State::vel_rel) = -h_meas(ExtendedState::vel_uav_z);
		_meas_matrix_row_vect(State::bias) = h_meas(ExtendedState::bias_z);
	}
}

void KF_xyzb_decoupled_static::syncState(float dt, float acc_uav)
{
	// Prediction: x(t1) = Phi*x(t0) + G*u <--> Backwards prediction: x(t0) = Phi.inv()*[x(t1) - G*u]
	_sync_state(State::pos_rel) = _state(State::pos_rel) - _state(State::vel_rel) * dt - 0.5f * acc_uav * dt * dt;
	_sync_state(State::vel_rel) = _state(State::vel_rel) + acc_uav * dt;
	_sync_state(State::bias) = _state(State::bias);
}

float KF_xyzb_decoupled_static::computeInnovCov(float meas_unc)
{
	float innov_cov_updated;
	sym::Computeinnovcov(meas_unc, _state_covariance, _meas_matrix_row_vect.transpose(), &innov_cov_updated);
	_innov_cov = innov_cov_updated;

	return _innov_cov;
}

float KF_xyzb_decoupled_static::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix_row_vect.transpose() * _sync_state)(0, 0);
	return _innov;
}

} // namespace vision_target_estimator
