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

/**
 * @file ina220.cpp
 * @author Ben Linne <benjinne@gmail.com>
 *
 * Driver for the I2C attached INA220
 * Based on the INA226 driver
 *
 */

#include "ina220.h"


INA220::INA220(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina220_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina220_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "ina220_collection_err")),
	_measure_errors(perf_alloc(PC_COUNT, "ina220_measurement_err")),
	_ch_type((PM_CH_TYPE)config.custom2),
	_battery(battery_index, this, INA220_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = (_ch_type == PM_CH_TYPE_VBATT) ? param_find("INA220_CUR_BAT") : param_find("INA220_CUR_REG");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = INA220_SHUNT;
	_rshunt = fvalue;
	ph = (_ch_type == PM_CH_TYPE_VBATT) ? param_find("INA220_SHUNT_BAT") : param_find("INA220_SHUNT_REG");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	ph = param_find("INA220_CONFIG");
	int32_t value = INA220_CONFIG;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_triggered = ((_config & INA220_MODE_MASK) >> INA220_MODE_SHIFTS) <=
			  ((INA220_MODE_SHUNT_BUS_TRIG & INA220_MODE_MASK) >>
			   INA220_MODE_SHIFTS);

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 25 * _current_lsb;

	if (_ch_type == PM_CH_TYPE_VBATT) {
		// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
		_battery.setConnected(false);
		_battery.updateVoltage(0.f);
		_battery.updateCurrent(0.f);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
	}

}

INA220::~INA220()
{
	// Reset config register on stop so restart probes correctly
	write(INA220_REG_CONFIGURATION, INA220_RST);

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_measure_errors);
}

int INA220::read(uint8_t address, int16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	int ret = PX4_ERROR;

	for (size_t i = 0; i < 3; i++) {
		ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

		if (ret == PX4_OK) {
			data = swap16(received_bytes);
			break;

		} else {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int INA220::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int
INA220::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	write(INA220_REG_CONFIGURATION, INA220_RST);

	_cal = INA220_CONST / (_current_lsb * _rshunt);

	if (write(INA220_REG_CALIBRATION, _cal) < 0) {
		return -3;
	}

	// If we run in continuous mode then start it here

	if (!_mode_triggered) {
		ret = write(INA220_REG_CONFIGURATION, _config);

	} else {
		ret = PX4_OK;
	}

	start();
	_sensor_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int
INA220::force_init()
{
	int ret = init();

	start();

	return ret;
}

int
INA220::probe()
{
	int16_t value{0};

	if (read(INA220_REG_CONFIGURATION, value) != PX4_OK || value != INA220_DEFAULT_CONFIG) {
		PX4_DEBUG("probe found wrong default_config %d", value);
		return -1;
	}

	PX4_DEBUG("probe found correct default_config %d", value);

	return PX4_OK;
}

int
INA220::measure()
{
	int ret = PX4_OK;

	if (_mode_triggered) {
		ret = write(INA220_REG_CONFIGURATION, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int
INA220::collect()
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
	success = success && (read(INA220_REG_BUSVOLTAGE, _bus_voltage) == PX4_OK);
	success = success && (read(INA220_REG_POWER, _bus_power) == PX4_OK);
	success = success && (read(INA220_REG_CURRENT, _bus_current) == PX4_OK);
	success = success && (read(INA220_REG_SHUNTVOLTAGE, _shunt) == PX4_OK);

	uint16_t unsigned_bus_voltage = _bus_voltage;
	_bus_voltage = unsigned_bus_voltage >> 3; // INA220 - the 3 LSB unused, so shift right

	if (!success) {
		PX4_DEBUG("error reading from sensor");
		_bus_voltage = _bus_power = _bus_current = _shunt = 0;
	}

	_voltage = static_cast<float>(_bus_voltage * INA220_VSCALE);
	_current = static_cast<float>(_bus_current * _current_lsb);
	_vshunt = static_cast<float>(_shunt * INA220_VSHUNTSCALE);

	switch (_ch_type) {
	case PM_CH_TYPE_VBATT: {
			_battery.setConnected(success);
			_battery.updateVoltage(_voltage);
			_battery.updateCurrent(_current);
			_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
		}
		break;

	case PM_CH_TYPE_VREG: {
			memset(&_pm_status, 0x00, sizeof(_pm_status));
			_pm_status.timestamp = hrt_absolute_time();
			_pm_status.voltage_v = _voltage;
			_pm_status.current_a = _current;
			_pm_pub_topic.publish(_pm_status);
		}
		break;

	}


	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void
INA220::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA220_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
INA220::RunImpl()
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

			if (_measure_interval > INA220_CONVERSION_INTERVAL) {
				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(_measure_interval - INA220_CONVERSION_INTERVAL);
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
		ScheduleDelayed(INA220_CONVERSION_INTERVAL);

	} else {

		if (_ch_type == PM_CH_TYPE_VBATT) {
			_battery.setConnected(false);
			_battery.updateVoltage(0.f);
			_battery.updateCurrent(0.f);
			_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
		}

		if (init() != PX4_OK) {
			ScheduleDelayed(INA220_INIT_RETRY_INTERVAL_US);
		}
	}
}

void
INA220::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		switch (_ch_type) {
		case PM_CH_TYPE_VBATT:
			printf("- type: BATT\n");
			break;

		case PM_CH_TYPE_VREG:
			printf("- type: VREG\n");
			break;

		default:
			printf("- type: UNKOWN\n");
			break;
		}

		printf("  - voltage: %9.4f VDC \n", (double) _voltage);
		printf("  - current: %9.4f ADC \n", (double) _current);
		printf("  - shunt: %9.4f mV \n", (double) _vshunt);

		printf("poll interval:  %u \n", _measure_interval);;

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 INA220_INIT_RETRY_INTERVAL_US / 1000);
	}
}
