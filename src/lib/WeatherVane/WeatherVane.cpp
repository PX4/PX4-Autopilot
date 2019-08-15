/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file WeatherVane.cpp
 * Weathervane controller.
 *
 */

#include "WeatherVane.hpp"
#include <mathlib/mathlib.h>


WeatherVane::WeatherVane() :
	ModuleParams(nullptr)
{
	_R_sp_prev = matrix::Dcmf();
}

void WeatherVane::update(const matrix::Quatf &q_sp_prev, float yaw)
{
	_R_sp_prev = matrix::Dcmf(q_sp_prev);
	_yaw = yaw;
}

float WeatherVane::get_weathervane_yawrate()
{
	// direction of desired body z axis represented in earth frame
	matrix::Vector3f body_z_sp(_R_sp_prev(0, 2), _R_sp_prev(1, 2), _R_sp_prev(2, 2));

	// rotate desired body z axis into new frame which is rotated in z by the current
	// heading of the vehicle. we refer to this as the heading frame.
	matrix::Dcmf R_yaw = matrix::Eulerf(0.0f, 0.0f, -_yaw);
	body_z_sp = R_yaw * body_z_sp;
	body_z_sp.normalize();

	float roll_sp = -asinf(body_z_sp(1));

	float roll_exceeding_treshold = 0.0f;
	float min_roll_rad = math::radians(_param_wv_roll_min.get());

	if (roll_sp > min_roll_rad) {
		roll_exceeding_treshold = roll_sp - min_roll_rad;

	} else if (roll_sp < -min_roll_rad) {
		roll_exceeding_treshold = roll_sp + min_roll_rad;

	}

	return math::constrain(roll_exceeding_treshold * _param_wv_gain.get(), -math::radians(_param_wv_yrate_max.get()),
			       math::radians(_param_wv_yrate_max.get()));
}
