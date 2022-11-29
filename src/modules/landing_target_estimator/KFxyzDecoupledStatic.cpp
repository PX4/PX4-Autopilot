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
 * @file KFxyzDecoupledStatic.h
 * Simple Kalman Filter for static target
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include "KFxyzDecoupledStatic.h"
#include "python_derivation/generated/decoupled_static/predictCov.h"
#include "python_derivation/generated/decoupled_static/computeInnovCov.h"

namespace landing_target_estimator
{

void KFxyzDecoupledStatic::predictState(float dt, float acc)
{
	_state(0, 0) = _state(0, 0) + _state(1, 0) * dt - 0.5f * acc * dt * dt;
	_state(1, 0) = _state(1, 0) - acc * dt;
}

void KFxyzDecoupledStatic::predictCov(float dt)
{
	matrix::Matrix<float, 3, 3> cov_updated;
	sym::Predictcov(dt, _input_var, _bias_var, _covariance, &cov_updated);
	_covariance = cov_updated;
}


bool KFxyzDecoupledStatic::update()
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

	matrix::Matrix<float, 3, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;
	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KFxyzDecoupledStatic::setH(matrix::Vector<float, 12> h_meas)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]

	// For this filter: [rx, r_dotx, bx]

	_meas_matrix(0, 0) = h_meas(0);
	_meas_matrix(0, 1) = h_meas(3);
	_meas_matrix(0, 2) = h_meas(6);
}

void KFxyzDecoupledStatic::syncState(float dt, float acc)
{
	_sync_state(0, 0) = _state(0, 0) - _state(1, 0) * dt - 0.5f * acc * dt * dt;
	_sync_state(1, 0) = _state(1, 0) + acc * dt;
	_sync_state(2, 0) = _state(2, 0);
}

float KFxyzDecoupledStatic::computeInnovCov(float meas_unc)
{
	float innov_cov_updated;
	sym::Computeinnovcov(meas_unc, _covariance, _meas_matrix, &innov_cov_updated);
	_innov_cov = innov_cov_updated;

	return _innov_cov;
}

float KFxyzDecoupledStatic::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix * _sync_state)(0, 0);
	return _innov;
}

} // namespace landing_target_estimator
