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

#include "Magnetometer.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>

using namespace matrix;
using namespace time_literals;

namespace calibration
{

Magnetometer::Magnetometer()
{
	Reset();
}

Magnetometer::Magnetometer(uint32_t device_id)
{
	Reset();
	set_device_id(device_id);
}

void Magnetometer::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		ParametersUpdate();
	}
}

void Magnetometer::set_external(bool external)
{
	_external = external;
}

void Magnetometer::set_scale(const Vector3f &scale)
{
	_scale(0, 0) = scale(0);
	_scale(1, 1) = scale(1);
	_scale(2, 2) = scale(2);
}

void Magnetometer::set_offdiagonal(const Vector3f &offdiagonal)
{
	_scale(0, 1) = offdiagonal(0);
	_scale(1, 0) = offdiagonal(0);

	_scale(0, 2) = offdiagonal(1);
	_scale(2, 0) = offdiagonal(1);

	_scale(1, 2) = offdiagonal(2);
	_scale(2, 1) = offdiagonal(2);
}

void Magnetometer::set_rotation(Rotation rotation)
{
	_rotation_enum = rotation;
	_rotation = get_rot_matrix(rotation);
}

void Magnetometer::ParametersUpdate()
{
	if (_device_id == 0) {
		Reset();
		return;
	}

	_calibration_index = FindCalibrationIndex(SensorString(), _device_id);

	if (_calibration_index >= 0) {

		if (!_external) {
			_rotation = GetBoardRotation();
			_rotation_enum = ROTATION_NONE;

		} else {
			int32_t rotation = GetCalibrationParam(SensorString(), "ROT", _calibration_index);
			_rotation_enum = static_cast<Rotation>(rotation);
			_rotation = get_rot_matrix((enum Rotation)rotation);
		}

		// CAL_MAGx_EN
		int32_t enabled = GetCalibrationParam(SensorString(), "EN", _calibration_index);
		_enabled = (enabled == 1);

		// CAL_MAGx_OFF{X,Y,Z}
		_offset = GetCalibrationParamsVector3f(SensorString(), "OFF", _calibration_index);

		// CAL_MAGx_SCALE{X,Y,Z}
		const Vector3f diag = GetCalibrationParamsVector3f(SensorString(), "SCALE", _calibration_index);

		// CAL_MAGx_ODIAG{X,Y,Z}
		const Vector3f offdiag = GetCalibrationParamsVector3f(SensorString(), "ODIAG", _calibration_index);

		float scale[9] {
			diag(0),    offdiag(0), offdiag(1),
			offdiag(0),    diag(1), offdiag(2),
			offdiag(1), offdiag(2),    diag(2)
		};
		_scale = Matrix3f{scale};

		// CAL_MAGx_COMP{X,Y,Z}
		_power_compensation = GetCalibrationParamsVector3f(SensorString(), "COMP", _calibration_index);

	} else {
		Reset();
	}
}

void Magnetometer::Reset()
{
	_rotation.setIdentity();
	_rotation_enum = ROTATION_NONE;
	_offset.zero();
	_scale.setIdentity();

	_power_compensation.zero();
	_power = 0.f;

	_enabled = true;

	_calibration_index = -1;
}

bool Magnetometer::ParametersSave()
{
	if (_calibration_index >= 0) {
		// save calibration
		SetCalibrationParam(SensorString(), "ID", _calibration_index, _device_id);
		SetCalibrationParam(SensorString(), "EN", _calibration_index, _enabled ? 1 : 0);
		SetCalibrationParamsVector3f(SensorString(), "OFF", _calibration_index, _offset);

		Vector3f scale{_scale.diag()};
		SetCalibrationParamsVector3f(SensorString(), "SCALE", _calibration_index, scale);

		Vector3f off_diag{_scale(0, 1), _scale(0, 2), _scale(1, 2)};
		SetCalibrationParamsVector3f(SensorString(), "ODIAG", _calibration_index, off_diag);

		SetCalibrationParamsVector3f(SensorString(), "COMP", _calibration_index, _power_compensation);

		if (_external) {
			SetCalibrationParam(SensorString(), "ROT", _calibration_index, (int32_t)_rotation_enum);

		} else {
			SetCalibrationParam(SensorString(), "ROT", _calibration_index, -1);
		}

		return true;
	}

	return false;
}

void Magnetometer::PrintStatus()
{
	PX4_INFO("%s %d EN: %d, offset: [%.4f %.4f %.4f] scale: [%.4f %.4f %.4f]", SensorString(), device_id(), enabled(),
		 (double)_offset(0), (double)_offset(1), (double)_offset(2),
		 (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2));

#if defined(DEBUG_BUILD)
	_scale.print()
#endif // DEBUG_BUILD
}

} // namespace calibration
