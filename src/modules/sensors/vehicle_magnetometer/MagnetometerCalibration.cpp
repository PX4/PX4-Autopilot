/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "MagnetometerCalibration.hpp"

#include <lib/parameters/param.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

namespace sensors
{

void MagnetometerCalibration::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		ParametersUpdate();
	}
}

Vector3f MagnetometerCalibration::Correct(const Vector3f &data)
{
	return _rotation * (_scale * ((data + _power * _power_compensation) - _offset));
}

int MagnetometerCalibration::FindCalibrationIndex(uint32_t device_id) const
{
	if (device_id == 0) {
		return -1;
	}

	for (unsigned i = 0; i < MAX_SENSOR_COUNT; ++i) {
		char str[16] {};
		sprintf(str, "CAL_%s%u_ID", SensorString(), i);

		int32_t device_id_val = 0;

		if (param_get(param_find(str), &device_id_val) != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if ((uint32_t)device_id_val == device_id) {
			return i;
		}
	}

	return -1;
}

void MagnetometerCalibration::ParametersUpdate()
{
	if (_device_id == 0) {
		return;
	}

	const int calibration_index = FindCalibrationIndex(_device_id);

	if (calibration_index >= 0) {

		char str[30] {};

		if (!_external) {
			// fine tune the rotation
			float x_offset = 0.f;
			float y_offset = 0.f;
			float z_offset = 0.f;
			param_get(param_find("SENS_BOARD_X_OFF"), &x_offset);
			param_get(param_find("SENS_BOARD_Y_OFF"), &y_offset);
			param_get(param_find("SENS_BOARD_Z_OFF"), &z_offset);

			const Dcmf board_rotation_offset(Eulerf(radians(x_offset), radians(y_offset), radians(z_offset)));

			// get transformation matrix from sensor/board to body frame
			int32_t board_rot = 0;
			param_get(param_find("SENS_BOARD_ROT"), &board_rot);
			_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)board_rot);

		} else {
			sprintf(str, "CAL_%s%u_ROT", SensorString(), calibration_index);
			int32_t rotation = 0;
			param_get(param_find(str), &rotation);

			_rotation = get_rot_matrix((enum Rotation)rotation);
		}

		sprintf(str, "CAL_%s%u_EN", SensorString(), calibration_index);
		int32_t enabled_val = 0;
		param_get(param_find(str), &enabled_val);

		_enabled = (enabled_val == 1);

		for (int axis = 0; axis < 3; axis++) {
			char axis_char = 'X' + axis;

			// offsets
			sprintf(str, "CAL_%s%u_%cOFF", SensorString(), calibration_index, axis_char);
			param_get(param_find(str), &_offset(axis));

			// scale
			sprintf(str, "CAL_%s%u_%cSCALE", SensorString(), calibration_index, axis_char);
			param_get(param_find(str), &_scale(axis, axis));

			// power compensation
			sprintf(str, "CAL_%s%u_%cCOMP", SensorString(), calibration_index, axis_char);
			param_get(param_find(str), &_power_compensation(axis));
		}

	} else {
		_enabled = true;
		_offset.zero();
		_scale.setIdentity();
		_power_compensation.zero();

		_rotation.setIdentity();
	}
}

void MagnetometerCalibration::PrintStatus()
{
	PX4_INFO("%s %d EN: %d, offset: [%.4f %.4f %.4f] scale: [%.4f %.4f %.4f]", SensorString(), _device_id, _enabled,
		 (double)_offset(0), (double)_offset(1), (double)_offset(2),
		 (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2));
}

} // namespace sensors
