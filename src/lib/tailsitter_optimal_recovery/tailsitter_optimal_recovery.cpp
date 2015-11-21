/****************************************************************************
 *
 *	Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file tailsitter_optimal_recovery.c
 *
 * Tailsitter optimal rate controller (underactuated pitch axis)
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 */

#include "tailsitter_optimal_recovery.h"

#define SigmoidFunction(val) 1/(1 + expf(-val))

void computeOptimalRates(const math::Quaternion &q, const math::Quaternion &q_sp, float *_rates_sp)
{
	// compute error quaterion
	math::Quaternion q_sp_inv = {q_sp(0), -q_sp(1), -q_sp(2), -q_sp(3)};
	math::Quaternion q_error = q * q_sp_inv;

	// compute tilt angle and corresponding tilt axis
	math::Vector<3> zB = {0, 0, -1.0f};
	math::Vector<3> zI = q_error.conjugate(zB);
	zI.normalize();

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

	tilt_axis = q.conjugate_inversed(tilt_axis);

	// comute sign of desired rates based on tilt angle and tilt axis
	// we need to find out in which quadrant we are and correct the
	// sign of the desired rates accordingly
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

	float x = tilt_angle;
	float y = tilt_dir;

	// compute optimal rates based on fitted function
	_rates_sp[0] = (pwx_ky1 * (M_PI_2_F - y) + pwx_ky2 * powf(M_PI_2_F - y, 2)) *
		       (pwx_kx1 * x * SigmoidFunction(pwx_sx1 * (pwx_sx0 - x)) - pwx_kx1 * x * expf(pwx_sx1 *
				       (pwx_sx0 - M_PI_F)) * SigmoidFunction(pwx_sx1 * (M_PI_F - pwx_sx0)));
	_rates_sp[1] = pwy_kx1 * x + powf((M_PI_2_F - y) / (M_PI_2_F),
					  2) * (-pwy_kx1 * x * SigmoidFunction(pwy_sx1 * (pwy_sx0 - x)));
	_rates_sp[2] = (pwz_ky1 * (M_PI_2_F - y) + pwz_ky2 * powf(M_PI_2_F - y, 2)) *
		       (pwz_kx1 * (x - M_PI_F) * SigmoidFunction(pwz_sx1 * (x - pwz_sx0)) - pwz_kx1 * (x - M_PI_F) * SigmoidFunction(
				-pwz_sx0 * pwz_sx1));

	_rates_sp[0] *= -sign_x;
	_rates_sp[1] *= -sign_y;
	_rates_sp[2] *= -sign_z;
}