/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "DifferentialPressure.hpp"

#include "Utilities.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

#include <float.h>

namespace calibration
{

DifferentialPressure::DifferentialPressure()
{
	Reset();
}

DifferentialPressure::DifferentialPressure(uint32_t device_id)
{
	set_device_id(device_id);
}

void DifferentialPressure::set_device_id(uint32_t device_id)
{
	const bool external = device::device_is_external(device_id);

	if (_device_id != device_id || _external != external) {

		_device_id = device_id;
		_external = external;

		Reset();

		ParametersUpdate();
	}
}

bool DifferentialPressure::set_offset(const float &offset)
{
	if (fabsf(_offset - offset) > 0.01f) {
		if (PX4_ISFINITE(offset)) {
			_offset = offset;
			_calibration_count++;
			return true;
		}
	}

	return false;
}

bool DifferentialPressure::set_calibration_index(int calibration_index)
{
	if ((calibration_index >= 0) && (calibration_index < MAX_SENSOR_COUNT)) {
		_calibration_index = calibration_index;
		return true;
	}

	return false;
}

void DifferentialPressure::ParametersUpdate()
{
	if (_device_id == 0) {
		return;
	}

	_calibration_index = FindCurrentCalibrationIndex(SensorString(), _device_id);

	if (_calibration_index == -1) {
		// no saved calibration available
		Reset();

	} else {
		ParametersLoad();
	}
}

bool DifferentialPressure::ParametersLoad()
{
	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {
		// CAL_DPRESx_OFF
		set_offset(GetCalibrationParamFloat(SensorString(), "OFF", _calibration_index));

		return true;
	}

	return false;
}

bool DifferentialPressure::AdoptUnclaimedCalibration()
{
	if ((_device_id == 0) || calibrated()) {
		return false;
	}

	// only slot 0 can hold a legacy offset, and only while no sensor has claimed it
	static constexpr int kLegacyIndex = 0;

	int32_t slot_device_id = GetCalibrationParamInt32(SensorString(), "ID", kLegacyIndex);

	if (slot_device_id != 0) {
		return false;
	}

	const float slot_offset = GetCalibrationParamFloat(SensorString(), "OFF", kLegacyIndex);

	if (!PX4_ISFINITE(slot_offset) || (fabsf(slot_offset) < FLT_EPSILON)) {
		return false;
	}

	_offset = slot_offset;
	_calibration_index = kLegacyIndex;

	PX4_INFO("%s %" PRIu32 " adopted migrated offset %.3f Pa", SensorString(), _device_id, (double)_offset);

	return true;
}

void DifferentialPressure::Reset()
{
	_offset = 0;

	_calibration_index = -1;

	_calibration_count = 0;
}

bool DifferentialPressure::ParametersSave(int desired_calibration_index, bool force)
{
	if (force && desired_calibration_index >= 0 && desired_calibration_index < MAX_SENSOR_COUNT) {
		_calibration_index = desired_calibration_index;

	} else if (!force || (_calibration_index < 0)
		   || (desired_calibration_index != -1 && desired_calibration_index != _calibration_index)) {

		// ensure we have a valid calibration slot (matching existing or first available slot)
		int8_t calibration_index_prev = _calibration_index;
		_calibration_index = FindAvailableCalibrationIndex(SensorString(), _device_id, desired_calibration_index);

		if (calibration_index_prev >= 0 && (calibration_index_prev != _calibration_index)) {
			PX4_WARN("%s %" PRIu32 " calibration index changed %" PRIi8 " -> %" PRIi8, SensorString(), _device_id,
				 calibration_index_prev, _calibration_index);
		}
	}

	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {
		// save calibration
		bool success = true;
		success &= SetCalibrationParam(SensorString(), "ID", _calibration_index, _device_id);
		success &= SetCalibrationParam(SensorString(), "OFF", _calibration_index, _offset);

		return success;
	}

	// FindAvailableCalibrationIndex() searches a fixed four slots, but differential pressure only defines three
	PX4_ERR("%s %" PRIu32 " no free calibration slot (got %" PRIi8 ")", SensorString(), _device_id, _calibration_index);

	return false;
}

void DifferentialPressure::PrintStatus()
{
	PX4_INFO_RAW("%s %" PRIu32 " offset: %05.3f Pa, %s\n",
		     SensorString(), device_id(), (double)_offset,
		     external() ? "Ext" : "Internal");
}

} // namespace calibration
