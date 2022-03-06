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
 * @file WrenchEstimator.cpp
 */

#include <WrenchEstimator.hpp>

void WrenchEstimator::initialize(const float &tau_f,const float &tau_m, const Vector3f &inertia_diag)
{
	_K_force = (1.f / tau_f);
	_K_moment = (1.f / tau_m);

	_omega.setAll(0.f);
	_Fe.setAll(0.f);
	_Fe_inertial.setAll(0.f);
	_Me.setAll(0.f);

	_I.setIdentity();
	_I(0,0) = inertia_diag(0);
	_I(1,1) = inertia_diag(1);
	_I(2,2) = inertia_diag(2);

}

void WrenchEstimator::updateForce(const Vector3f &prediction_error_force, const float &dt, const bool &interaction_flag)
{
	_dt = dt;

	if (interaction_flag) {
		_integrateForce(prediction_error_force - _Fe);

		//Compute external force in FRD body frame
		_Fe_inertial = (_Fe);
	} else {
		_Fe.setAll(0.f);
		_Fe_inertial.setAll(0.f);
	}
}

void WrenchEstimator::updateMoment(const Vector3f &prediction_error_moment, const Vector3f &omega, const float &dt, const bool &interaction_flag)
{
	_dt = dt;

	Vector3f Iw = (_I*_omega);

	if (interaction_flag) {
		_integrateMoment(- prediction_error_moment + Iw.cross(_omega) - _Me);
		_Me -= (_K_moment * Iw);
	} else {
		_Me.setAll(0.f);
	}
}

inline void WrenchEstimator::_integrateForce(const Vector3f &u)
{
	_Fe += _K_force * u * _dt;
}

inline void WrenchEstimator::_integrateMoment(const Vector3f &u)
{
	_Me+= _K_moment * u * _dt;
}
