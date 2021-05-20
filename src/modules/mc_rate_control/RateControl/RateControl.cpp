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
}

void RateControl::setSaturationStatus(const MultirotorMixer::saturation_status &status)
{
	_mixer_saturation_positive[0] = status.flags.roll_pos;
	_mixer_saturation_positive[1] = status.flags.pitch_pos;
	_mixer_saturation_positive[2] = status.flags.yaw_pos;
	_mixer_saturation_negative[0] = status.flags.roll_neg;
	_mixer_saturation_negative[1] = status.flags.pitch_neg;
	_mixer_saturation_negative[2] = status.flags.yaw_neg;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	//const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
	Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	//PX4_INFO("Rate Controller:\t%8.4f", (double)dt);
	if (landed)
	{
		ii_AC_R = 0;
		// ii_Pq_R = 0;
	}
	z_k_rate = rate_error;
	u_k_rate.setZero();
	if (!landed && RCAC_Aw_ON)
	{
		// TODO: DERIV IS NOT IMPLMENTED PROPERLY. 0 IS PLACED AS PLACEHOLDER
		u_k_rate(0) = _rcac_rate_x.compute_uk(z_k_rate(0), _rate_int(0), 0, _rcac_rate_x.get_rcac_uk());
		u_k_rate(1) = _rcac_rate_y.compute_uk(z_k_rate(1), _rate_int(1), 0, _rcac_rate_y.get_rcac_uk());
		u_k_rate(2) = _rcac_rate_z.compute_uk(z_k_rate(2), _rate_int(2), 0, _rcac_rate_z.get_rcac_uk());
		// ii_AC_R = ii_AC_R + 1;
		// if (ii_AC_R == 1)
		// {
		// 	init_RCAC_rate();
		// 	/*theta_k_Ac_PID(0,0) = _gain_p(0);
		// 	theta_k_Ac_PID(1,0) = _gain_i(0);
		// 	theta_k_Ac_PID(2,0) = _gain_d(0);
		// 	theta_k_Ac_PID(3,0) = _gain_ff(0);
		// 	theta_k_Ac_PID(4,0) = _gain_p(1);
		// 	theta_k_Ac_PID(5,0) = _gain_i(1);
		// 	theta_k_Ac_PID(6,0) = _gain_d(1);
		// 	theta_k_Ac_PID(7,0) = _gain_ff(1);
		// 	theta_k_Ac_PID(8,0) = _gain_p(2);
		// 	theta_k_Ac_PID(9,0) = _gain_i(2);
		// 	theta_k_Ac_PID(10,0) = _gain_d(2);
		// 	theta_k_Ac_PID(11,0) = _gain_ff(2);*/

		// }

		// // Ankit 01 30 2020:New SISO implementation
		// z_k_rate = rate_error;

		// phi_k_rate_x(0,0) = rate_error(0)*0; //Ankit: Disable P rate
		// phi_k_rate_x(0,1) = _rate_int(0);
		// phi_k_rate_x(0,2) = angular_accel(0) * 0;
		// phi_k_rate_x(0,3) = rate_sp(0) * 0;

		// phi_k_rate_y(0,0) = rate_error(1);
		// phi_k_rate_y(0,1) = _rate_int(1);
		// phi_k_rate_y(0,2) = angular_accel(1) * 0;
		// phi_k_rate_y(0,3) = rate_sp(1) * 0;

		// phi_k_rate_z(0,0) = rate_error(2);
		// phi_k_rate_z(0,1) = _rate_int(2);
		// phi_k_rate_z(0,2) = angular_accel(2) * 0;
		// phi_k_rate_z(0,3) = rate_sp(2) * 0;

		// dummy1 = phi_km1_rate_x * P_rate_x * phi_km1_rate_x.T() + 1.0f;
		// dummy2 = phi_km1_rate_y * P_rate_y * phi_km1_rate_y.T() + 1.0f;
		// dummy3 = phi_km1_rate_z * P_rate_z * phi_km1_rate_z.T() + 1.0f;
		// Gamma_rate(0) 	= dummy1(0,0);
		// Gamma_rate(1) 	= dummy2(0,0);
		// Gamma_rate(2) 	= dummy3(0,0);

		// P_rate_x = P_rate_x - (P_rate_x * phi_km1_rate_x.T()) * (phi_km1_rate_x * P_rate_x) / Gamma_rate(0);
		// P_rate_y = P_rate_y - (P_rate_y * phi_km1_rate_y.T()) * (phi_km1_rate_y * P_rate_y) / Gamma_rate(1);
		// P_rate_z = P_rate_z - (P_rate_z * phi_km1_rate_z.T()) * (phi_km1_rate_z * P_rate_z) / Gamma_rate(2);

		// dummy1 = N1_rate(0)*(phi_km1_rate_x * theta_k_rate_x - u_km1_rate(0));
		// dummy2 = N1_rate(1)*(phi_km1_rate_y * theta_k_rate_y - u_km1_rate(1));
		// dummy3 = N1_rate(2)*(phi_km1_rate_z * theta_k_rate_z - u_km1_rate(2));
		// theta_k_rate_x 	= theta_k_rate_x + (P_rate_x * phi_km1_rate_x.T()) * N1_rate(0) *(z_k_rate(0) + dummy1(0,0));
		// theta_k_rate_y 	= theta_k_rate_y + (P_rate_y * phi_km1_rate_y.T()) * N1_rate(1) *(z_k_rate(1) + dummy2(0,0));
		// theta_k_rate_z 	= theta_k_rate_z + (P_rate_z * phi_km1_rate_z.T()) * N1_rate(2) *(z_k_rate(2) + dummy3(0,0));

		// dummy1 = phi_k_rate_x * theta_k_rate_x;
		// dummy2 = phi_k_rate_y * theta_k_rate_y;
		// dummy3 = phi_k_rate_z * theta_k_rate_z;
		// u_k_rate(0) = dummy1(0,0);
		// u_k_rate(1) = dummy2(0,0);
		// u_k_rate(2) = dummy3(0,0);

		// u_km1_rate = u_k_rate;

		// phi_km1_rate_x = phi_k_rate_x;
		// phi_km1_rate_y = phi_k_rate_y;
		// phi_km1_rate_z = phi_k_rate_z;
	}
	//torque = alpha_PID*torque+u_k_rate;
	torque = alpha_PID_rate*torque+u_k_rate;

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_mixer_saturation_positive[i]) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_mixer_saturation_negative[i]) {
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
