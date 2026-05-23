/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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
 * @file ina_common.cpp
 *
 * Shared implementation for INA226/INA228/INA231 power monitor drivers.
 */

#include "ina_common.h"
#include <px4_platform_common/log.h>
#include <parameters/param.h>

INACommon::INACommon(transfer_func_t transfer_func, void *transfer_context, Battery &battery,
		     perf_counter_t sample_perf, perf_counter_t comms_errors) :
	_transfer_func(transfer_func),
	_transfer_context(transfer_context),
	_battery(battery),
	_sample_perf(sample_perf),
	_comms_errors(comms_errors)
{
}

void INACommon::loadParams(const char *current_param, const char *shunt_param, const char *config_param,
			   float default_max_current, float default_shunt, uint16_t default_config)
{
	float fvalue = default_max_current;
	_max_current = fvalue;
	param_t ph = param_find(current_param);

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = default_shunt;
	_rshunt = fvalue;
	ph = param_find(shunt_param);

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	ph = param_find(config_param);
	int32_t value = default_config;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_triggered = ((_config & INA_COMMON_MODE_MASK) >> INA_COMMON_MODE_SHIFTS) <=
			  ((INA_COMMON_MODE_SHUNT_BUS_TRIG & INA_COMMON_MODE_MASK) >>
			   INA_COMMON_MODE_SHIFTS);

	_current_lsb = _max_current / INA_COMMON_DN_MAX;
	_power_lsb = 25 * _current_lsb;
}

int INACommon::read(uint8_t address, int16_t &data)
{
	uint16_t received_bytes;
	int ret = PX4_ERROR;

	for (size_t i = 0; i < 3; i++) {
		ret = _transfer_func(_transfer_context, &address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

		if (ret == PX4_OK) {
			data = ina_common_swap16(received_bytes);
			break;

		} else {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int INACommon::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return _transfer_func(_transfer_context, data, sizeof(data), nullptr, 0);
}

int INACommon::init()
{
	write(INA_COMMON_REG_CONFIGURATION, INA_COMMON_RST);

	_cal = INA_COMMON_CONST / (_current_lsb * _rshunt);

	if (write(INA_COMMON_REG_CALIBRATION, _cal) < 0) {
		return -3;
	}

	int ret;

	if (!_mode_triggered) {
		ret = write(INA_COMMON_REG_CONFIGURATION, _config);

	} else {
		ret = PX4_OK;
	}

	_sensor_ok = true;
	_initialized = ret == PX4_OK;
	return ret;
}

int INACommon::measure()
{
	int ret = PX4_OK;

	if (_mode_triggered) {
		ret = write(INA_COMMON_REG_CONFIGURATION, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int INACommon::collect()
{
	perf_begin(_sample_perf);

	// Note: If the power module is connected backwards, then the values of _current will be negative but otherwise valid.
	bool success{true};
	success = success && (read(INA_COMMON_REG_BUSVOLTAGE, _bus_voltage) == PX4_OK);
	success = success && (read(INA_COMMON_REG_CURRENT, _current) == PX4_OK);

	if (setConnected(success)) {
		_battery.updateVoltage(static_cast<float>(_bus_voltage * INA_COMMON_VSCALE));
		_battery.updateCurrent(static_cast<float>(_current * _current_lsb));
	}

	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

bool INACommon::setConnected(bool state)
{
	if (state) {
		_connected = INA_COMMON_SAMPLE_FREQUENCY_HZ * 2;

	} else if (_connected > 0) {
		_connected--;
	}

	if (_connected > 0) {
		_battery.setConnected(true);

	} else {
		_battery.setConnected(false);
		_battery.updateVoltage(0);
		_battery.updateCurrent(0);
	}

	return state;
}
