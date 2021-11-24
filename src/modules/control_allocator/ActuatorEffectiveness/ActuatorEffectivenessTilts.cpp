/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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


#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessTilts::ActuatorEffectivenessTilts(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_CT", i);
		_param_handles[i].control = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MINA", i);
		_param_handles[i].min_angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MAXA", i);
		_param_handles[i].max_angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_TD", i);
		_param_handles[i].tilt_direction = param_find(buffer);
	}

	_count_handle = param_find("CA_SV_TL_COUNT");
	updateParams();
}

void ActuatorEffectivenessTilts::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_count = count;

	for (int i = 0; i < count; i++) {
		param_get(_param_handles[i].control, (int32_t *)&_params[i].control);
		param_get(_param_handles[i].tilt_direction, (int32_t *)&_params[i].tilt_direction);
		param_get(_param_handles[i].min_angle, &_params[i].min_angle);
		param_get(_param_handles[i].max_angle, &_params[i].max_angle);

		_params[i].min_angle = math::radians(_params[i].min_angle);
		_params[i].max_angle = math::radians(_params[i].max_angle);

		_yaw_torque[i] = 0.f;
	}
}

bool ActuatorEffectivenessTilts::getEffectivenessMatrix(Configuration &configuration, bool force)
{
	if (!force) {
		return false;
	}

	for (int i = 0; i < _count; i++) {
		Vector3f torque{};

		if (_params[i].control == Control::Yaw) {
			torque(2) = _yaw_torque[i];
		}

		configuration.addActuator(ActuatorType::SERVOS, torque, Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessTilts::updateYawSign(const ActuatorEffectivenessRotors::Geometry &geometry)
{
	for (int i = 0; i < geometry.num_rotors; ++i) {
		int tilt_index = geometry.rotors[i].tilt_index;

		if (tilt_index == -1 || tilt_index >= _count) {
			continue;
		}

		if (_params[tilt_index].control != Control::Yaw) {
			continue;
		}

		// Find the yaw torque sign by checking the motor position and tilt direction.
		// Rotate position by -tilt_direction around z, then check the sign of y pos
		float tilt_direction = math::radians((float)_params[tilt_index].tilt_direction);
		Vector3f rotated_pos = Dcmf{Eulerf{0.f, 0.f, -tilt_direction}} * geometry.rotors[i].position;

		if (rotated_pos(1) < -0.01f) { // add minimal margin
			_yaw_torque[tilt_index] = 1.f;

		} else if (rotated_pos(1) > 0.01f) {
			_yaw_torque[tilt_index] = -1.f;
		}
	}
}

bool ActuatorEffectivenessTilts::hasYawControl() const
{
	for (int i = 0; i < _count; i++) {
		if (_params[i].control == Control::Yaw) {
			return true;
		}
	}

	return false;
}
