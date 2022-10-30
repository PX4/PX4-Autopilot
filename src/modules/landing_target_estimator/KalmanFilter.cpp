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
 * @file KalmanFilter.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 *
 */

#include "KalmanFilter.h"

namespace landing_target_estimator
{

void KalmanFilter::predictState(float dt, float acc)
{
	/*
	⎡        2                   ⎤
	⎢- 0.5⋅Ts ⋅a + Ts⋅x(1) + x(0)⎥
	⎢                            ⎥
	⎣        -Ts⋅a + x(1)        ⎦
	*/

	_state(0) = _state(0) + _state(1) * dt - dt * dt / 2.f * acc;
	_state(1) = _state(1) - acc * dt;
}

void KalmanFilter::predictCov(float dt)
{
	/*
	⎡       4                                                             3                        ⎤
	⎢0.25⋅Ts ⋅σₐ + Ts⋅p(0;1) + Ts⋅(Ts⋅p(1;1) + p(0;1)) + p(0;0)     0.5⋅Ts ⋅σₐ + Ts⋅p(1;1) + p(0;1)⎥
	⎢                                                                                              ⎥
	⎢                   3                                                     2                    ⎥
	⎣             0.5⋅Ts ⋅σₐ + Ts⋅p(1;1) + p(0;1)                           Ts ⋅σₐ + p(1;1)        ⎦
	*/

	//Q = var* [1/4*T^4, 1/2*T^3; 1/2*T^3, T^2]
	float off_diag = _input_var * 0.5f * dt * dt * dt + dt * _covariance(1, 1) + _covariance(0, 1);
	_covariance(0, 0) += _input_var * 0.25f * dt * dt * dt * dt + dt * _covariance(0, 1) + dt * (dt * _covariance(1,
			     1) + _covariance(0, 1));
	_covariance(1, 0) = off_diag;
	_covariance(0, 1) = off_diag;
	_covariance(1, 1) += _input_var * dt * dt;
}


bool KalmanFilter::update()
{
	// outlier rejection
	if (_innov_cov <= 0.000001f) {
		return false;
	}

	float beta = _innov / _innov_cov * _innov;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	matrix::Vector<float, 2> kalmanGain = _covariance * _meas_matrix / _innov_cov;

	_state(0) += kalmanGain(0) * _innov;
	_state(1) += kalmanGain(1) * _innov;

	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KalmanFilter::setH(matrix::Vector<float, 12> h_meas)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]

	// For this filter: [rx, r_dotx]

	_meas_matrix(0) = h_meas(0);
	_meas_matrix(1) = h_meas(3);

}

void KalmanFilter::syncState(float dt, float acc)
{
	/*
	⎡        2                   ⎤
	⎢- 0.5⋅Ts ⋅a - Ts⋅x(1) + x(0)⎥
	⎢                            ⎥
	⎣        Ts⋅a + x(1)         ⎦
	*/

	_sync_state(0) = _state(0) - _state(1) * dt - dt * dt / 2.f * acc;
	_sync_state(1) = _state(1) + acc * dt;
}

float KalmanFilter::computeInnovCov(float measUnc)
{
	/*
	[h(0)⋅(cov(0;0)⋅h(0) + cov(0;1)⋅h(1)) + h(1)⋅(cov(0;1)⋅h(0) + cov(1;1)⋅h(1)) + r]
	*/

	_innov_cov = _meas_matrix(0) * (_covariance(0, 0) * _meas_matrix(0) + _covariance(0,
					1) * _meas_matrix(1)) + _meas_matrix(1) * (_covariance(0, 1) * _meas_matrix(0) + _covariance(1,
							1) * _meas_matrix(1)) + measUnc;
	return _innov_cov;
}

float KalmanFilter::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - _meas_matrix * _sync_state;
	return _innov;
}

} // namespace landing_target_estimator
