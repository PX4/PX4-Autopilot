/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#pragma once

#define TC_PRINT_DEBUG 0
#if TC_PRINT_DEBUG
#define TC_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define TC_DEBUG(fmt, ...)
#endif

#include <px4_log.h>

#define SENSOR_COUNT_MAX		3

/**
 * Base class for temperature calibration types (for all different sensor types)
 */
class TemperatureCalibrationBase
{
public:
	TemperatureCalibrationBase(float min_temperature_rise)
		: _min_temperature_rise(min_temperature_rise) {}

	virtual ~TemperatureCalibrationBase() {}

	/**
	 * check & update new sensor data.
	 * @return progress in range [0, 100], 110 when finished, <0 on error
	 */
	virtual int update() = 0;

	/**
	 * do final fitting & write the parameters. Call this exactly once after update() returned 110
	 * @return 0 on success, <0 otherwise
	 */
	virtual int finish() = 0;

	/** reset all driver-level calibration parameters */
	virtual void reset_calibration() = 0;

protected:

	/**
	 * set a system parameter (without system notification) and print an error if it fails
	 * @param format_str for example "CAL_GYRO%u_XOFF"
	 * @param index which index (will replace %u in format_str)
	 * @param value
	 * @return 0 on success
	 */
	inline int set_parameter(const char *format_str, unsigned index, const void *value);

	float _min_temperature_rise; ///< minimum difference in temperature before the process finishes
};



int TemperatureCalibrationBase::set_parameter(const char *format_str, unsigned index, const void *value)
{
	char param_str[30];
	(void)sprintf(param_str, format_str, index);
	int result = param_set_no_notification(param_find(param_str), value);

	if (result != 0) {
		PX4_ERR("unable to reset %s (%i)", param_str, result);
	}

	return result;
}

