/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tailsitter_recovery.cpp
*/

#include "tailsitter_recovery.h"
#include <math.h>

TailsitterRecovery::TailsitterRecovery():
	_in_recovery_mode(false)
{
	_att_p(0) = _att_p(1) = _att_p(2) = 4.0f;
}

TailsitterRecovery::~TailsitterRecovery()
{

}

void TailsitterRecovery::setAttGains(math::Vector<3> &att_p, float yaw_ff)
{
	_att_p = att_p;
	_yaw_ff = yaw_ff;
}

void TailsitterRecovery::calcOptimalRates(math::Quaternion &q, math::Quaternion &q_sp, float yaw_move_rate,
		math::Vector<3> &rates_opt)
{
	math::Matrix<3, 3> R = q.to_dcm();

	// compute error quaternion
	math::Quaternion q_sp_inv = {q_sp(0), -q_sp(1), -q_sp(2), -q_sp(3)};
	math::Quaternion q_error = q * q_sp_inv;

	// compute tilt angle and corresponding tilt axis
	math::Vector<3> zB = {0, 0, -1.0f};

	math::Vector<3> zI = q_error.conjugate(zB);

	float tilt_angle;
	float inner_prod = zI * zB;

	if (inner_prod >= 1.0f) {
		tilt_angle = 0.0f;

	} else if (inner_prod <= -1.0f) {
		tilt_angle = M_PI_F;

	} else {
		tilt_angle = acosf(inner_prod);
	}

	math::Vector<3> tilt_axis = {0, 0, -1};

	if (math::min(fabsf(tilt_angle), fabsf(tilt_angle - M_PI_F)) > 0.00001f) {
		tilt_axis = zI % zB;
	}

	tilt_axis = R.transposed() * tilt_axis;

	// compute desired rates based on tilt angle and tilt axis
	float tilt_dir = atan2f(tilt_axis(0), tilt_axis(1));
	float sign_x;
	float sign_y;
	float sign_z;

	if (tilt_dir < -M_PI_2_F) {
		tilt_dir += M_PI_F;
		sign_x = -1;
		sign_y = -1;
		sign_z = 1;

	} else if (tilt_dir < 0.0f) {
		tilt_dir = 0.0f - tilt_dir;
		sign_x = -1;
		sign_y = 1;
		sign_z = -1;

	} else if (tilt_dir < M_PI_2_F) {
		tilt_dir += 0;
		sign_x = 1;
		sign_y = 1;
		sign_z = 1;

	} else {
		tilt_dir = M_PI_F - tilt_dir;
		sign_x = 1;
		sign_y = -1;
		sign_z = -1;
	}

	// optimal coefficients
	float pwx_kx1 = -1.676f;
	float pwx_ky1 = 1.38f;
	float pwx_ky2 = 0.8725f;
	float pwx_sx0 = 0.3586f;
	float pwx_sx1 = 2.642f;

	float pwy_kx1 = -3.997f;
	float pwy_sx0 = 2.133f;
	float pwy_sx1 = 4.013f;

	float pwz_kx1 = 2.726f;
	float pwz_ky1 = 3.168f;
	float pwz_ky2 = -0.3913f;
	float pwz_sx0 = 1.75f;
	float pwz_sx1 = 2.298f;

	float x = tilt_angle;
	float y = tilt_dir;

	rates_opt(1) = (pwx_ky1 * (M_PI_2_F - y) + pwx_ky2 * powf(M_PI_2_F - y, 2)) *
		       (pwx_kx1 * x * SigmoidFunction(pwx_sx1 * (pwx_sx0 - x)) - pwx_kx1 * x * expf(pwx_sx1 *
				       (pwx_sx0 - M_PI_F)) * SigmoidFunction(pwx_sx1 * (M_PI_F - pwx_sx0)));
	rates_opt(0) = pwy_kx1 * x + powf((M_PI_2_F - y) / (M_PI_2_F),
					  2) * (-pwy_kx1 * x * SigmoidFunction(pwy_sx1 * (pwy_sx0 - x)));
	rates_opt(2) = (pwz_ky1 * (M_PI_2_F - y) + pwz_ky2 * powf(M_PI_2_F - y, 2)) *
		       (pwz_kx1 * (x - M_PI_F) * SigmoidFunction(pwz_sx1 * (x - pwz_sx0)) - pwz_kx1 * (x - M_PI_F) * SigmoidFunction(
				-pwz_sx0 * pwz_sx1));
	rates_opt(0) *= -sign_x;
	rates_opt(1) *= -sign_y * 1.5f;
	rates_opt(2) *= -sign_z;

	float yaw_w = R(2, 2) > 0.0f ? R(2, 2) : 0.0f;
	yaw_w = math::constrain(yaw_w, 0.0f, 1.0f);

	// we activate recovery mode if our tilt error is more that 30 degrees or
	if (!_in_recovery_mode) {
		if (fabsf(tilt_angle) > math::radians(30.0f)) {
			_in_recovery_mode = true;
		}

	} else {
		if (fabsf(tilt_angle) < math::radians(5.0f)) {
			_in_recovery_mode = false;
		}
	}

	// do normal attitude control, the other case has already been handled
	if (!_in_recovery_mode) {
		q_error = q_sp_inv * q;
		rates_opt = q_error(0) > 0.0f ? _att_p.emult(q_error.imag()) * (-2.0f) : _att_p.emult(q_error.imag()) * (2.0f);
		// don't want too strong yaw control. after recovery vehicle might have
		// very large yaw error and shouldn't turn too fast
		rates_opt(2) = math::constrain(rates_opt(2), -1.0f, 1.0f);
	}

	rates_opt(2) += yaw_move_rate * yaw_w * _yaw_ff;

}
