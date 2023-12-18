/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
#include <lib/sensor/Utilities.hpp>

using namespace matrix;
using namespace time_literals;
using namespace sensor::utilities;

namespace sensor
{
namespace calibration
{

Magnetometer::Magnetometer()
{
	Reset();
}

Magnetometer::Magnetometer(uint32_t device_id) :
	_configuration(device_id)
{
	set_device_id(device_id);
}

void Magnetometer::set_device_id(uint32_t device_id)
{
	bool external = DeviceExternal(device_id);

	if (_configuration.device_id() != device_id || _configuration.external() != external) {

		_configuration.set_device_id(device_id);

		Reset();

		ParametersUpdate();
		SensorCorrectionsUpdate(true);
	}
}

void Magnetometer::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		// valid device id required
		if (device_id() == 0) {
			return;
		}

		sensor_correction_s corrections;

		if (_sensor_correction_sub.copy(&corrections)) {
			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (corrections.mag_device_ids[i] == device_id()) {
					switch (i) {
					case 0:
						_thermal_offset = Vector3f{corrections.mag_offset_0};
						return;

					case 1:
						_thermal_offset = Vector3f{corrections.mag_offset_1};
						return;

					case 2:
						_thermal_offset = Vector3f{corrections.mag_offset_2};
						return;

					case 3:
						_thermal_offset = Vector3f{corrections.mag_offset_3};
						return;
					}
				}
			}
		}

		// zero thermal offset if not found
		_thermal_offset.zero();
	}
}

bool Magnetometer::set_offset(const Vector3f &offset)
{
	static constexpr float MIN_OFFSET_CHANGE = 0.005f;

	if (offset.isAllFinite()
	    && (Vector3f(_offset - offset).longerThan(MIN_OFFSET_CHANGE) || (_calibration_count == 0))
	   ) {
		_offset = offset;

		_calibration_count++;
		return true;
	}

	return false;
}

bool Magnetometer::set_scale(const Vector3f &scale)
{
	static constexpr float MIN_SCALE_CHANGE = 0.01f;

	if (scale.isAllFinite() && scale.longerThan(0.f)
	    && (Vector3f(_scale.diag() - scale).longerThan(MIN_SCALE_CHANGE) || (_calibration_count == 0))
	   ) {
		_scale(0, 0) = scale(0);
		_scale(1, 1) = scale(1);
		_scale(2, 2) = scale(2);

		_calibration_count++;
		return true;
	}

	return false;
}

bool Magnetometer::set_offdiagonal(const Vector3f &offdiagonal)
{
	if (offdiagonal.isAllFinite()
	    && (Vector3f(Vector3f{_scale(0, 1), _scale(0, 2), _scale(1, 2)} - offdiagonal).longerThan(0.01f)
		|| (_calibration_count == 0))
	   ) {

		_scale(0, 1) = offdiagonal(0);
		_scale(1, 0) = offdiagonal(0);

		_scale(0, 2) = offdiagonal(1);
		_scale(2, 0) = offdiagonal(1);

		_scale(1, 2) = offdiagonal(2);
		_scale(2, 1) = offdiagonal(2);

		_calibration_count++;
		return true;
	}

	return false;
}

bool Magnetometer::set_calibration_index(int calibration_index)
{
	if ((calibration_index >= 0) && (calibration_index < MAX_SENSOR_COUNT)) {
		_calibration_index = calibration_index;
		return true;
	}

	return false;
}

void Magnetometer::ParametersUpdate()
{
	if (device_id() == 0) {
		return;
	}

	_calibration_index = FindCurrentCalibrationIndex(SensorString(), device_id());

	if (_calibration_index == -1) {
		// no saved calibration available
		Reset();

	} else {
		ParametersLoad();

		if (calibrated() && !_configuration.configured()) {
			_configuration.ParametersSave(calibration_index());
		}
	}
}

bool Magnetometer::ParametersLoad()
{
	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {

		// CAL_{}n_{X,Y,Z}OFF
		Vector3f offset{
			GetCalibrationParamFloat(SensorString(), "XOFF", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "YOFF", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "ZOFF", _calibration_index)
		};

		// CAL_{}n_{X,Y,Z}SCALE
		Vector3f scale{
			GetCalibrationParamFloat(SensorString(), "XSCALE", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "YSCALE", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "ZSCALE", _calibration_index)
		};

		// CAL_{}n_{X,Y,Z}ODIAG
		Vector3f odiag{
			GetCalibrationParamFloat(SensorString(), "XODIAG", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "YODIAG", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "ZODIAG", _calibration_index)
		};

		// CAL_{}n_{X,Y,Z}COMP
		Vector3f comp{
			GetCalibrationParamFloat(SensorString(), "XCOMP", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "YCOMP", _calibration_index),
			GetCalibrationParamFloat(SensorString(), "ZCOMP", _calibration_index)
		};
		_power_compensation = comp;

		return set_offset(offset) && set_scale(scale) && set_offdiagonal(odiag);
	}

	return false;
}

void Magnetometer::Reset()
{
	_offset.zero();
	_scale.setIdentity();

	_power_compensation.zero();
	_power = 0.f;

	_thermal_offset.zero();

	_calibration_index = -1;

	_calibration_count = 0;
}

bool Magnetometer::ParametersSave(int desired_calibration_index, bool force)
{
	_configuration.ParametersSave();

	if (force && desired_calibration_index >= 0 && desired_calibration_index < MAX_SENSOR_COUNT) {
		_calibration_index = desired_calibration_index;

	} else if (!force || (_calibration_index < 0)
		   || (desired_calibration_index != -1 && desired_calibration_index != _calibration_index)) {

		// ensure we have a valid calibration slot (matching existing or first available slot)
		int8_t calibration_index_prev = _calibration_index;
		_calibration_index = FindAvailableCalibrationIndex(SensorString(), device_id(), desired_calibration_index);

		if (calibration_index_prev >= 0 && (calibration_index_prev != _calibration_index)) {
			PX4_WARN("%s %" PRIu32 " calibration index changed %" PRIi8 " -> %" PRIi8, SensorString(), device_id(),
				 calibration_index_prev, _calibration_index);
		}
	}

	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {
		// save calibration
		bool success = true;

		success &= SetCalibrationParam(SensorString(), "ID", _calibration_index, device_id());

		// CAL_{}n_{X,Y,Z}OFF
		success &= SetCalibrationParam(SensorString(), "XOFF", _calibration_index, _offset(0));
		success &= SetCalibrationParam(SensorString(), "YOFF", _calibration_index, _offset(1));
		success &= SetCalibrationParam(SensorString(), "ZOFF", _calibration_index, _offset(2));

		// CAL_{}n_{X,Y,Z}SCALE
		success &= SetCalibrationParam(SensorString(), "XSCALE", _calibration_index, _scale(0, 0));
		success &= SetCalibrationParam(SensorString(), "YSCALE", _calibration_index, _scale(1, 1));
		success &= SetCalibrationParam(SensorString(), "ZSCALE", _calibration_index, _scale(2, 2));

		// CAL_{}n_{X,Y,Z}ODIAG
		success &= SetCalibrationParam(SensorString(), "XODIAG", _calibration_index, _scale(0, 1));
		success &= SetCalibrationParam(SensorString(), "YODIAG", _calibration_index, _scale(0, 2));
		success &= SetCalibrationParam(SensorString(), "ZODIAG", _calibration_index, _scale(1, 2));

		// CAL_{}n_{X,Y,Z}COMP
		success &= SetCalibrationParam(SensorString(), "XCOMP", _calibration_index, _power_compensation(0));
		success &= SetCalibrationParam(SensorString(), "YCOMP", _calibration_index, _power_compensation(1));
		success &= SetCalibrationParam(SensorString(), "ZCOMP", _calibration_index, _power_compensation(2));

		return success;
	}

	return false;
}

void Magnetometer::PrintStatus()
{
	if (external()) {
		PX4_INFO_RAW("%s %" PRIu32
			     " EN: %d, offset: [%05.3f %05.3f %05.3f], scale: [%05.3f %05.3f %05.3f], Ext ROT: %d\n",
			     SensorString(), device_id(), enabled(),
			     (double)_offset(0), (double)_offset(1), (double)_offset(2),
			     (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2),
			     rotation_enum());

	} else {
		PX4_INFO_RAW("%s %" PRIu32
			     " EN: %d, offset: [%05.3f %05.3f %05.3f], scale: [%05.3f %05.3f %05.3f], Internal\n",
			     SensorString(), device_id(), enabled(),
			     (double)_offset(0), (double)_offset(1), (double)_offset(2),
			     (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2));
	}

	if (_thermal_offset.norm() > 0.f) {
		PX4_INFO_RAW("%s %" PRIu32 " temperature offset: [%.4f %.4f %.4f]\n", SensorString(), device_id(),
			     (double)_thermal_offset(0), (double)_thermal_offset(1), (double)_thermal_offset(2));
	}
}

} // namespace calibration
} // namespace sensor
