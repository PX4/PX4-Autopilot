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
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
	return;
}
void RateControl::setGeoPIDGains(const float &P, const float &I, const float &D){
	kP = 0.01f*P;
	kP = 0.01f*I;
	kD = 0.01f*D;
	return;
}

void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const matrix::Matrix3fSO3 &att, const matrix::Matrix3fSO3 &att_sp, const float dt, const bool landed)
{
	// angular rates error
	//Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	//const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
	//Geo PD
	angular_v = rate;
	Q_ = att_sp.transpose() * att;
	angular_vdn = rate_sp;
	d_angular_vd = (angular_vdn - angular_vd) / dt;
	omega_ = angular_v - Q_.transpose() * angular_vdn;
	//omega_ = _lp_filters_d.apply(omega_);

	//angular_v_cross = angular_v.skew();
	//omega_cross = omega_.skew();

	//F_omega_ = I_ + dt * omega_cross;
	//Q_n_ = Q_ * F_omega_;

	//F_angular_ = I_ + dt * angular_v_cross;
	s_ = Q_.sKgenerator();
	//const Vector3f torque = -kP * s_ - kD * omega_ + Inertia * (Q_.transpose() * d_angular_vd - omega_cross * Q_.transpose() * angular_vdn)
	// - Vector3f (Inertia * angular_v).skew() * Q_.transpose() * angular_vdn;


	if (!landed) {
		FI_ = integretor(FI_prev, angular_v, omega_, s_, dt);
	}

	for (int i = 0; i <= 2; i++) {
		if (FI_(i) >= sup) {
			FI_(i) = sup;
		}
		else if (FI_(i) <= -sup) {
			FI_(i) = -sup;
		}
	}


	Vector3f torque = -kP * s_ - kI * FI_ - kD * omega_ +
	Inertia * (Q_.transpose() * d_angular_vd - omega_.skew() * Q_.transpose() * angular_vdn) - Vector3f(Inertia*angular_v) .skew() * Q_.transpose() * angular_vdn;
	angular_vd = angular_vdn;
	FI_prev = FI_;
	// update integral only if we are not landed
	//if (!landed) {
	//	updateIntegral(rate_error, dt);
	//}

	return torque;
}

Vector3f
RateControl::integretor(Vector3f FI_p, Vector3f Om, Vector3f om, Vector3f s, float dt) {
	Vector3f FI;
	FI =dt * inv(Inertia) * (kP * s + kD * om + Vector3f(Inertia*Om).skew() * FI_p) + FI_p;
	return FI;
}
void RateControl::setIntegralSup(const float &bound){
	sup=bound;
	return;
}



void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
