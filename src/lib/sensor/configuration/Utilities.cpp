/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "Utilities.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

using math::radians;
using matrix::Eulerf;
using matrix::Dcmf;
using matrix::Vector3f;

namespace sensor
{
namespace configuration
{

static constexpr int MAX_SENSOR_COUNT = 4; // TODO: per sensor?

int8_t FindCurrentConfigurationIndex(const char *sensor_type, uint32_t device_id)
{
	if (device_id == 0) {
		return -1;
	}

	for (unsigned i = 0; i < MAX_SENSOR_COUNT; ++i) {
		char str[16 + 1] {};
		snprintf(str, sizeof(str), "SENS_%s%u_ID", sensor_type, i);

		int32_t device_id_val = 0;

		param_t param_handle = param_find_no_notification(str);

		if (param_handle == PARAM_INVALID) {
			continue;
		}

		// find again and get value, but this time mark it active
		if (param_get(param_find(str), &device_id_val) != OK) {
			continue;
		}

		if ((uint32_t)device_id_val == device_id) {
			return i;
		}
	}

	return -1;
}

int8_t FindAvailableConfigurationIndex(const char *sensor_type, uint32_t device_id, int8_t preferred_index)
{
	// if this device is already using a calibration slot then keep it
	int calibration_index = FindCurrentConfigurationIndex(sensor_type, device_id);

	if (calibration_index >= 0) {
		return calibration_index;
	}


	// device isn't currently using a calibration slot, select user preference (preferred_index)
	//  if available, otherwise use the first available slot
	uint32_t cal_device_ids[MAX_SENSOR_COUNT] {};

	for (unsigned i = 0; i < MAX_SENSOR_COUNT; ++i) {
		char str[16 + 1] {};
		snprintf(str, sizeof(str), "SENS_%s%u_ID", sensor_type, i);
		int32_t device_id_val = 0;

		if (param_get(param_find_no_notification(str), &device_id_val) == PX4_OK) {
			cal_device_ids[i] = device_id_val;
		}
	}

	// use preferred_index if it's available
	if ((preferred_index >= 0) && (preferred_index < MAX_SENSOR_COUNT)
	    && (cal_device_ids[preferred_index] == 0)) {

		calibration_index = preferred_index;

	} else {
		// otherwise select first available slot
		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			if (cal_device_ids[i] == 0) {
				calibration_index = i;
				break;
			}
		}
	}

	if (calibration_index == -1) {
		PX4_ERR("no %s calibration slots available", sensor_type);
	}

	return calibration_index;
}

int32_t GetConfigurationParamInt32(const char *sensor_type, const char *cal_type, uint8_t instance)
{
	// eg SENS_GNSSn_ID
	char str[16 + 1] {};
	snprintf(str, sizeof(str), "SENS_%s%" PRIu8 "_%s", sensor_type, instance, cal_type);

	int32_t value = 0;

	if (param_get(param_find(str), &value) != 0) {
		PX4_ERR("failed to get %s", str);
	}

	return value;
}

float GetConfigurationParamFloat(const char *sensor_type, const char *cal_type, uint8_t instance)
{
	// eg SENS_GNSSn_OFF
	char str[16 + 1] {};
	snprintf(str, sizeof(str), "SENS_%s%" PRIu8 "_%s", sensor_type, instance, cal_type);

	float value = NAN;

	if (param_get(param_find(str), &value) != 0) {
		PX4_ERR("failed to get %s", str);
	}

	return value;
}

Vector3f GetConfigurationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance)
{
	Vector3f values{0.f, 0.f, 0.f};

	char str[16 + 1] {};

	for (int axis = 0; axis < 3; axis++) {
		char axis_char = 'X' + axis;

		// eg SENS_GNSSn_{X,Y,Z}POS
		snprintf(str, sizeof(str), "SENS_%s%" PRIu8 "_%c%s", sensor_type, instance, axis_char, cal_type);

		if (param_get(param_find(str), &values(axis)) != 0) {
			PX4_ERR("failed to get %s", str);
		}
	}

	return values;
}

bool SetConfigurationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance, Vector3f values)
{
	int ret = PX4_OK;
	char str[16 + 1] {};

	for (int axis = 0; axis < 3; axis++) {
		char axis_char = 'X' + axis;

		// eg SENS_GNSSn_{X,Y,Z}POS
		snprintf(str, sizeof(str), "SENS_%s%" PRIu8 "_%c%s", sensor_type, instance, axis_char, cal_type);

		if (param_set_no_notification(param_find(str), &values(axis)) != 0) {
			PX4_ERR("failed to set %s = %.4f", str, (double)values(axis));
			ret = PX4_ERROR;
		}
	}

	return ret == PX4_OK;
}

} // namespace configuration
} // namespace sensor
