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

/**
 * @file ina228.cpp
 * @author David Sidrane <david.sidrane@nscdg.com>
 *
 * Driver for the I2C attached INA228
 */

#include "ina228.h"


INA228::INA228(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina228_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina228_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "ina228_collection_err")),
	_measure_errors(perf_alloc(PC_COUNT, "ina228_measurement_err")),
	_battery(battery_index, this, INA228_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA228_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	_range = _max_current > (MAX_CURRENT - 1.0f) ? INA228_ADCRANGE_HIGH : INA228_ADCRANGE_LOW;

	fvalue = INA228_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA228_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	ph = param_find("INA228_CONFIG");
	int32_t value = INA228_ADCCONFIG;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_triggered = ((_config & INA228_MODE_MASK) >> INA228_MODE_SHIFTS) <=
			  ((INA228_MODE_TEMP_SHUNT_BUS_TRIG & INA228_MODE_MASK) >>
			   INA228_MODE_SHIFTS);

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 3.2f * _current_lsb;

	// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
	_battery.setConnected(false);
	_battery.updateVoltage(0.f);
	_battery.updateCurrent(0.f);
	_battery.updateTemperature(0.f);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

INA228::~INA228()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_measure_errors);
}

int INA228::read(uint8_t address, int16_t &data)
{
	// read desired little-endian value via I2C
	int16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228::read(uint8_t address, int32_t &data)
{
	// read desired 24 bit value via I2C
	int32_t received_bytes{0};
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes) - 1);

	if (ret == PX4_OK) {
		data = swap32(received_bytes) >> ((32 - 24) + 4); // Convert to 20bit value

		// Handle negative 20bit twos complement
		if (data & 0x80000) {
			data = -((0x000FFFFF & ~data) + 1);
		}

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228::read(uint8_t address, int64_t &data)
{
	// read desired 40 bit little-endian value via I2C
	int64_t received_bytes{0};
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes) - 3);

	if (ret == PX4_OK) {
		data = swap64(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228::read(uint8_t address, uint16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int INA228::write(uint8_t address, int16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int INA228::write(uint8_t address, int32_t value)
{
	uint8_t data[4] = {address, ((uint8_t)((value & 0xff0000) >> 16)), ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int INA228::write(uint8_t address, int64_t value)
{
	uint8_t data[6] = {address, ((uint8_t)((value & 0xff000000) >> 32)), ((uint8_t)((value & 0xff0000) >> 24)), ((uint8_t)((value & 0xff00) >> 16)), ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int
INA228::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	write(INA228_REG_CONFIG, (uint16_t)(INA228_RST_RESET | _range));

	_cal = INA228_CONST * _current_lsb * _rshunt;

	if (_range == INA228_ADCRANGE_LOW) {
		_cal *= 4;
	}

	if (write(INA228_REG_SHUNTCAL, _cal) < 0) {
		return -3;
	}

	// Set the CONFIG for max I

	write(INA228_REG_CONFIG, (uint16_t) _range);

	// If we run in continuous mode then start it here


	if (!_mode_triggered) {

		ret = write(INA228_REG_ADCCONFIG, _config);

	} else {
		ret = PX4_OK;
	}

	start();
	_sensor_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int
INA228::force_init()
{
	int ret = init();

	start();

	return ret;
}

int
INA228::probe()
{
	uint16_t value{0};

	if (read(INA228_MANUFACTURER_ID, value) != PX4_OK || value != INA228_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	if (read(INA228_DEVICE_ID, value) != PX4_OK || INA228_DEVICEID(value) != INA228_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return PX4_OK;
}

int
INA228::measure()
{
	int ret = PX4_OK;

	if (_mode_triggered) {
		ret = write(INA228_REG_ADCCONFIG, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int
INA228::collect()
{
	perf_begin(_sample_perf);

	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	// read from the sensor
	// Note: If the power module is connected backwards, then the values of _power, _current, and _shunt will be negative but otherwise valid.
	bool success{true};
	success = success && (read(INA228_REG_VSBUS, _bus_voltage) == PX4_OK);
	// success = success && (read(INA228_REG_POWER, _power) == PX4_OK);
	success = success && (read(INA228_REG_CURRENT, _current) == PX4_OK);
	//success = success && (read(INA228_REG_VSHUNT, _shunt) == PX4_OK);
	success = success && (read(INA228_REG_DIETEMP, _temperature) == PX4_OK);

	if (!success) {
		PX4_DEBUG("error reading from sensor");
		_bus_voltage = _power = _current = _shunt = 0;
	}

	_battery.setConnected(success);
	_battery.updateVoltage(static_cast<float>(_bus_voltage * INA228_VSCALE));
	_battery.updateCurrent(static_cast<float>(_current * _current_lsb));
	_battery.updateTemperature(static_cast<float>(_temperature * INA228_TSCALE));
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void
INA228::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA228_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
INA228::RunImpl()
{
	if (_initialized) {
		if (_collect_phase) {
			/* perform collection */
			if (collect() != PX4_OK) {
				perf_count(_collection_errors);
				/* if error restart the measurement state machine */
				start();
				return;
			}

			/* next phase is measurement */
			_collect_phase = !_mode_triggered;

			if (_measure_interval > INA228_CONVERSION_INTERVAL) {
				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(_measure_interval - INA228_CONVERSION_INTERVAL);
				return;
			}
		}

		/* Measurement  phase */

		/* Perform measurement */
		if (measure() != PX4_OK) {
			perf_count(_measure_errors);
		}

		/* next phase is collection */
		_collect_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(INA228_CONVERSION_INTERVAL);

	} else {
		_battery.setConnected(false);
		_battery.updateVoltage(0.f);
		_battery.updateCurrent(0.f);
		_battery.updateTemperature(0.f);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

		if (init() != PX4_OK) {
			ScheduleDelayed(INA228_INIT_RETRY_INTERVAL_US);
		}
	}
}

void
INA228::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 INA228_INIT_RETRY_INTERVAL_US / 1000);
	}
}
