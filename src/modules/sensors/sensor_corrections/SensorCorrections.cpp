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

#include "SensorCorrections.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

SensorCorrections::SensorCorrections(ModuleParams *parent, SensorType type) :
	ModuleParams(parent),
	_type(type)
{
}

void SensorCorrections::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;

		_temperature_calibration = false;
		_corrections_selected_instance = -1;

		_offset = Vector3f{0.f, 0.f, 0.f};
		_scale = Vector3f{1.f, 1.f, 1.f};

		SensorCorrectionsUpdate(true);

		// use calibration parameters (CAL_*) if temperature_calibration disabled
		if (!_temperature_calibration) {
			if (device_id > 0) {
				int cal_index = FindCalibrationIndex(device_id);

				if (cal_index >= 0) {
					_calibration_index = cal_index;

					SetCalibrationOffset();
					SetCalibrationScale();
				}
			}
		}


	}
}

const char *SensorCorrections::SensorString() const
{
	switch (_type) {
	case SensorType::Accelerometer:
		return "ACC";

	case SensorType::Gyroscope:
		return "GYRO";

	case SensorType::Magnetometer:
		return "MAG";
	}

	return nullptr;
}

int SensorCorrections::FindCalibrationIndex(uint32_t device_id) const
{
	if (device_id == 0) {
		return -1;
	}

	for (unsigned i = 0; i < MAX_SENSOR_COUNT; ++i) {
		char str[30] {};
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

void SensorCorrections::SetCalibrationOffset()
{
	// offsets (x, y, z)
	float offset[3] {0.f, 0.f, 0.f};
	char str[30] {};

	sprintf(str, "CAL_%s%u_XOFF", SensorString(), _calibration_index);
	param_get(param_find(str), &offset[0]);

	sprintf(str, "CAL_%s%u_YOFF", SensorString(), _calibration_index);
	param_get(param_find(str), &offset[1]);

	sprintf(str, "CAL_%s%u_ZOFF", SensorString(), _calibration_index);
	param_get(param_find(str), &offset[2]);

	if (PX4_ISFINITE(offset[0]) && PX4_ISFINITE(offset[1]) && PX4_ISFINITE(offset[2])) {
		_offset = matrix::Vector3f{offset};

	} else {
		_offset.zero();
	}
}

void SensorCorrections::SetCalibrationScale()
{
	// scale factors (x, y, z)
	float scale[3] {1.f, 1.f, 1.f};
	char str[30] {};

	sprintf(str, "CAL_%s%u_XSCALE", SensorString(), _calibration_index);
	param_get(param_find(str), &scale[0]);

	sprintf(str, "CAL_%s%u_YSCALE", SensorString(), _calibration_index);
	param_get(param_find(str), &scale[1]);

	sprintf(str, "CAL_%s%u_ZSCALE", SensorString(), _calibration_index);
	param_get(param_find(str), &scale[2]);

	if (PX4_ISFINITE(scale[0]) && PX4_ISFINITE(scale[1]) && PX4_ISFINITE(scale[2])) {
		_scale = matrix::Vector3f{scale};

	} else {
		_scale = matrix::Vector3f{1.f, 1.f, 1.f};
	}
}

void SensorCorrections::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// selected sensor has changed, find updated index
		if ((_corrections_selected_instance < 0) || force) {
			_corrections_selected_instance = -1;

			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {

				switch (_type) {
				case SensorType::Accelerometer:
					if (corrections.accel_device_ids[i] == _device_id) {
						_corrections_selected_instance = i;
						_temperature_calibration = true;
					}

					break;

				case SensorType::Gyroscope:
					if (corrections.gyro_device_ids[i] == _device_id) {
						_corrections_selected_instance = i;
						_temperature_calibration = true;
					}

					break;

				case SensorType::Magnetometer:
					// not currently handled
					_temperature_calibration = false;
					return;
				}
			}
		}

		switch (_type) {
		case SensorType::Accelerometer:
			switch (_corrections_selected_instance) {
			case 0:
				_offset = Vector3f{corrections.accel_offset_0};
				_scale = Vector3f{corrections.accel_scale_0};
				return;
			case 1:
				_offset = Vector3f{corrections.accel_offset_1};
				_scale = Vector3f{corrections.accel_scale_1};
				return;
			case 2:
				_offset = Vector3f{corrections.accel_offset_2};
				_scale = Vector3f{corrections.accel_scale_2};
				return;
			}

			break;

		case SensorType::Gyroscope:
			switch (_corrections_selected_instance) {
			case 0:
				_offset = Vector3f{corrections.gyro_offset_0};
				_scale = Vector3f{corrections.gyro_scale_0};
				return;
			case 1:
				_offset = Vector3f{corrections.gyro_offset_1};
				_scale = Vector3f{corrections.gyro_scale_1};
				return;
			case 2:
				_offset = Vector3f{corrections.gyro_offset_2};
				_scale = Vector3f{corrections.gyro_scale_2};
				return;
			}

			break;

		case SensorType::Magnetometer:
			// not currently handled
			break;
		}
	}
}

void SensorCorrections::ParametersUpdate()
{
	// fine tune the rotation
	const Dcmf board_rotation_offset(Eulerf(
			radians(_param_sens_board_x_off.get()),
			radians(_param_sens_board_y_off.get()),
			radians(_param_sens_board_z_off.get())));

	// get transformation matrix from sensor/board to body frame
	_board_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

	if (!_temperature_calibration) {
		SetCalibrationOffset();
		SetCalibrationScale();
	}
}

Vector3f SensorCorrections::Correct(const Vector3f &data)
{
	// TODO: determine if temp cal is enabled
	//
	//  temp cal enabled
	//    - use sensor_correction message for scale and offset
	//  otherwise
	//    - set scale and offset from regular calibration parameters
	//    - update scale and offset if parameter update

	if (_temperature_calibration) {
		SensorCorrectionsUpdate();
	}

	// apply offsets and scale
	// rotate corrected measurements from sensor to body frame
	return _board_rotation * Vector3f{(data - _offset).emult(_scale)};
}

void SensorCorrections::PrintStatus()
{
	PX4_INFO("sensor: %d", _device_id);
	PX4_INFO("offset: [%.3f %.3f %.3f]", (double)_offset(0), (double)_offset(1), (double)_offset(2));
	PX4_INFO("scale: [%.3f %.3f %.3f]", (double)_scale(0), (double)_scale(1), (double)_scale(2));

	if (_temperature_calibration) {
		PX4_INFO("temperature calibration enabled");
	}
}
