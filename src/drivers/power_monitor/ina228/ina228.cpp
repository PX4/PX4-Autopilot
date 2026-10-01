/****************************************************************************
 *
 *   Copyright (C) 2021-2026 PX4 Development Team. All rights reserved.
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

#include "ina228.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace ina228;

INA228::INA228(const I2CSPIDriverConfig &config, int battery_index)
	: I2C(config),
	  ModuleParams(nullptr),
	  I2CSPIDriver(config),
	  _timing(computeTiming()),
	  _battery(battery_index, this, _timing.interval_us, battery_status_s::SOURCE_POWER_MODULE),
	  _sample_perf(perf_alloc(PC_ELAPSED, "ina228_read")),
	  _comms_errors(perf_alloc(PC_COUNT, "ina228_com_err")),
	  _collection_errors(perf_alloc(PC_COUNT, "ina228_collection_err")),
	  _bad_register_perf(perf_alloc(PC_COUNT, "ina228_bad_register")),
	  _reinit_perf(perf_alloc(PC_COUNT, "ina228_reinit")),
	  _not_ready_perf(perf_alloc(PC_COUNT, "ina228_not_ready"))
{
	const float max_current = _param_ina228_current.get();
	const float shunt_resistance = _param_ina228_shunt.get();

	// Pick the ADC range so the device doesn't clip at the configured max current.
	// Datasheet §8.2.2.1: R_SHUNT * I_MAX < V_SENSE_MAX.
	const float v_sense_max = shunt_resistance * max_current;
	const bool use_low_range = (v_sense_max <= ADCRANGE_LOW_V_SENSE);
	_config_value = use_low_range ? RANGE_LOW : RANGE_HIGH;

	_current_lsb = max_current / CURRENT_LSB_DIV;
	float shunt_calibration = SHUNT_CAL_K * _current_lsb * shunt_resistance;

	if (use_low_range) {
		shunt_calibration *= 4.f;
	}

	if (shunt_calibration > SHUNT_CAL_MAX) {
		PX4_ERR("SHUNT_CAL %.0f out of range, check INA228_CURRENT / INA228_SHUNT", (double)shunt_calibration);
		shunt_calibration = SHUNT_CAL_MAX;
		// keep the current scale consistent with the calibration actually written
		_current_lsb = shunt_calibration / (SHUNT_CAL_K * shunt_resistance * (use_low_range ? 4.f : 1.f));
	}

	_shunt_calibration = static_cast<uint16_t>(shunt_calibration);

	// Tolerate ~DISCONNECT_DEBOUNCE_US of consecutive failures before a full reinit.
	_max_consecutive_failures = math::max<uint32_t>(DISCONNECT_DEBOUNCE_US / _timing.interval_us, 1);

	// Publish an initial disconnected status so the first instance grabs uORB instance 0 immediately.
	_battery.setConnected(false);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	// Let the lower I2C layer absorb transient bus errors before we see them.
	I2C::_retries = 5;
}

INA228::~INA228()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_bad_register_perf);
	perf_free(_reinit_perf);
	perf_free(_not_ready_perf);
}

INA228::Timing INA228::computeTiming()
{
	int32_t adc_config = ADCCONFIG_DEFAULT;
	param_get(param_find("INA228_CONFIG"), &adc_config);

	uint16_t mode = (adc_config >> MODE_SHIFT) & 0xf;

	// Bus voltage and shunt (current) must both be converted, otherwise the
	// registers we publish from are never updated.
	if (adc_config < 0 || adc_config > UINT16_MAX || (mode & (MODE_BUS | MODE_SHUNT)) != (MODE_BUS | MODE_SHUNT)) {
		PX4_ERR("INA228_CONFIG 0x%04" PRIX32 " invalid (16 bit, bus and shunt enabled), using 0x%04X",
			static_cast<uint32_t>(adc_config), ADCCONFIG_DEFAULT);
		adc_config = ADCCONFIG_DEFAULT;
		mode = (adc_config >> MODE_SHIFT) & 0xf;
	}

	Timing t{};
	t.adc_config = static_cast<uint16_t>(adc_config);
	t.triggered = !(mode & MODE_CONTINUOUS);
	t.temperature_enabled = (mode & MODE_TEMP);

	// Enabled channels are converted in sequence and the sequence is repeated AVG times (datasheet §7.3.4).
	uint32_t sequence_us = CONVERSION_TIME_US[(adc_config >> VBUSCT_SHIFT) & 0x7]
			       + CONVERSION_TIME_US[(adc_config >> VSHCT_SHIFT) & 0x7];

	if (t.temperature_enabled) {
		sequence_us += CONVERSION_TIME_US[(adc_config >> VTCT_SHIFT) & 0x7];
	}

	t.conversion_us = AVERAGES[(adc_config >> AVG_SHIFT) & 0x7] * sequence_us;

	// Worst case with the internal oscillator running slow.
	t.conversion_max_us = static_cast<uint32_t>(ceilf(t.conversion_us * OSCILLATOR_TOLERANCE));

	if (t.triggered) {
		// Each tick triggers the next conversion and reads the previous one, so a conversion
		// has to finish before the next tick triggers again.
		t.min_interval_us = t.conversion_max_us + WAKEUP_TIME_US + TRIGGER_MARGIN_US;

	} else {
		// Poll no faster than the device produces samples so every read is a fresh one.
		t.min_interval_us = t.conversion_max_us;
	}

	t.interval_us = math::max(SAMPLE_INTERVAL_US, t.min_interval_us);

	return t;
}

int INA228::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	_state = State::RESET;
	return PX4_OK;
}

void INA228::enterReset()
{
	ScheduleClear();
	_state = State::RESET;
	_consecutive_failures = 0;
	ScheduleNow();
}

void INA228::enterConfigure()
{
	ScheduleClear();
	_state = State::CONFIGURE;
	_consecutive_failures = 0;
	ScheduleNow();
}

void INA228::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		// INA228_* are reboot-required; this is for the Battery (BATn_*) parameters.
		updateParams();
	}

	switch (_state) {
	case State::UNINITIALIZED: {
			if (init() != PX4_OK) {
				_battery.updateAndPublishBatteryStatus(now);
				ScheduleDelayed(INIT_RETRY_INTERVAL_US);
				return;
			}

			// init() advanced us to State::RESET
			ScheduleNow();
			return;
		}

	case State::RESET: {
			_battery.setConnected(false);
			_battery.updateVoltage(0.f);
			_battery.updateCurrent(-1.f);
			_battery.updateTemperature(NAN);
			_battery.updateAndPublishBatteryStatus(now);

			if (registerWrite(Register::CONFIG, RST) != PX4_OK) {
				ScheduleDelayed(INIT_RETRY_INTERVAL_US);
				return;
			}

			_state = State::CONFIGURE;
			ScheduleDelayed(RESET_DELAY_US);
			return;
		}

	case State::CONFIGURE: {
			// In triggered mode the ADCCONFIG write also starts the first conversion.
			const bool ok = (probe() == PX4_OK) &&
					(registerWrite(Register::SHUNT_CAL, _shunt_calibration) == PX4_OK) &&
					(registerWrite(Register::CONFIG, _config_value) == PX4_OK) &&
					(registerWrite(Register::ADCCONFIG, _timing.adc_config) == PX4_OK);

			if (!ok) {
				_state = State::RESET;
				ScheduleDelayed(INIT_RETRY_INTERVAL_US);
				return;
			}

			_trigger_time = hrt_absolute_time();
			_consecutive_failures = 0;
			_next_reg_to_check = 0;
			_last_config_check = now;
			_state = State::MEASURE;

			// The first tick comes one interval later, when the first conversion is complete
			// (interval >= worst-case conversion time in both modes). Continuous mode gets
			// extra margin since the device is not synchronised to us.
			ScheduleOnInterval(_timing.interval_us, _timing.triggered ? _timing.interval_us : _timing.interval_us + 5_ms);
			return;
		}

	case State::MEASURE: {
			const int ret = collect();

			if (ret == -EAGAIN) {
				// The previous conversion can't be done yet (this tick came early): read it next tick.
				return;
			}

			if (ret != PX4_OK) {
				perf_count(_collection_errors);

				if (++_consecutive_failures >= _max_consecutive_failures) {
					perf_count(_reinit_perf);
					PX4_WARN("consecutive failures, resetting");
					enterReset();
				}

				return;
			}

			_consecutive_failures = 0;

			// Checked after publishing so it doesn't add to the sample latency. A failed
			// read here is only counted as a comms error; the sample reads above decide
			// whether the device is gone.
			if (now - _last_config_check >= CONFIG_CHECK_INTERVAL_US) {
				_last_config_check = now;

				if (checkConfigurationRotating() == -EBADMSG) {
					// The device lost its configuration (e.g. brown-out while plugging the battery)
					// but is still responding: write it again without reporting the battery as
					// disconnected, which the commander would treat as a battery failure in flight.
					perf_count(_bad_register_perf);
					perf_count(_reinit_perf);
					PX4_WARN("configuration lost, reconfiguring");
					enterConfigure();
				}
			}

			return;
		}
	}
}

int INA228::collect()
{
	if (_timing.triggered) {
		// Ticks are on a fixed hrt grid, so a tick after a late one comes early. Triggering
		// before the previous conversion is done would restart it and we would read (and
		// publish) the one before it again.
		if (hrt_elapsed_time(&_trigger_time) < _timing.conversion_max_us + WAKEUP_TIME_US) {
			perf_count(_not_ready_perf);
			return -EAGAIN;
		}
	}

	perf_begin(_sample_perf);

	// Triggered mode: start the next conversion first. The result registers keep the previous,
	// completed conversion until the new one finishes (datasheet §7.3.4), so reading them right
	// after the trigger returns the sample triggered one tick ago. This gives a fixed sample
	// latency and the full tick for the conversion.
	if (_timing.triggered) {
		if (registerWrite(Register::ADCCONFIG, _timing.adc_config) != PX4_OK) {
			perf_end(_sample_perf);
			return PX4_ERROR;
		}

		_trigger_time = hrt_absolute_time();
	}

	int32_t bus_voltage = 0;
	int32_t current = 0;
	uint16_t temperature = 0;

	const bool reads_ok = (registerRead24(Register::VS_BUS, bus_voltage) == PX4_OK)
			      && (registerRead24(Register::CURRENT, current) == PX4_OK)
			      && (!_timing.temperature_enabled || registerRead(Register::DIETEMP, temperature) == PX4_OK);

	if (reads_ok) {
		_battery.setConnected(true);
		_battery.updateVoltage(static_cast<float>(bus_voltage) * V_LSB);
		_battery.updateCurrent(static_cast<float>(current) * _current_lsb);
		_battery.updateTemperature(_timing.temperature_enabled ?
					   static_cast<float>(static_cast<int16_t>(temperature)) * T_LSB : NAN);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
	}

	perf_end(_sample_perf);
	return reads_ok ? PX4_OK : PX4_ERROR;
}

int INA228::probe()
{
	uint16_t value = 0;

	if (registerRead(Register::MANUFACTURER_ID, value) != PX4_OK || value != MANFID) {
		PX4_DEBUG("probe mfgid %d", value);
		return PX4_ERROR;
	}

	if (registerRead(Register::DEVICE_ID, value) != PX4_OK || deviceId(value) != DIEID) {
		PX4_DEBUG("probe die id %d", value);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int INA228::checkConfigurationRotating()
{
	const struct {
		Register reg;
		uint16_t expected;
	} checks[] = {
		{ Register::CONFIG, _config_value },
		{ Register::SHUNT_CAL, _shunt_calibration },
		{ Register::ADCCONFIG, _timing.adc_config },
	};

	// In triggered mode ADCCONFIG is rewritten every tick anyway.
	const uint8_t num_checks = _timing.triggered ? 2 : 3;

	const auto &check = checks[_next_reg_to_check];
	uint16_t actual = 0;

	if (registerRead(check.reg, actual) != PX4_OK) {
		return -EIO;
	}

	if (actual != check.expected) {
		// I2C has no CRC: confirm the mismatch with a second read before acting on it.
		if (registerRead(check.reg, actual) != PX4_OK) {
			return -EIO;
		}

		if (actual != check.expected) {
			_last_readback[_next_reg_to_check] = actual;
			return -EBADMSG;
		}
	}

	_last_readback[_next_reg_to_check] = actual;
	_next_reg_to_check = (_next_reg_to_check + 1) % num_checks;
	return PX4_OK;
}

int INA228::registerRead(Register reg, uint16_t &value)
{
	uint8_t address = static_cast<uint8_t>(reg);
	uint8_t buf[2] {};

	const int ret = transfer(&address, 1, buf, sizeof(buf));

	if (ret == PX4_OK) {
		value = (buf[0] << 8) | buf[1];

	} else {
		perf_count(_comms_errors);
	}

	return ret;
}

int INA228::registerRead24(Register reg, int32_t &value)
{
	uint8_t address = static_cast<uint8_t>(reg);
	uint8_t buf[3] {};

	const int ret = transfer(&address, 1, buf, sizeof(buf));

	if (ret == PX4_OK) {
		// 20 bit two's complement result in bits 23..4
		const uint32_t raw = (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16)
				     | (static_cast<uint32_t>(buf[2]) << 8);
		value = static_cast<int32_t>(raw) >> 12;

	} else {
		perf_count(_comms_errors);
	}

	return ret;
}

int INA228::registerWrite(Register reg, uint16_t value)
{
	const uint8_t buf[3] = {
		static_cast<uint8_t>(reg),
		static_cast<uint8_t>((value >> 8) & 0xff),
		static_cast<uint8_t>(value & 0xff),
	};

	const int ret = transfer(buf, sizeof(buf), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void INA228::print_status()
{
	I2CSPIDriverBase::print_status();

	const char *state_str = "?";

	switch (_state) {
	case State::UNINITIALIZED:
		state_str = "UNINITIALIZED";
		break;

	case State::RESET:
		state_str = "RESET";
		break;

	case State::CONFIGURE:
		state_str = "CONFIGURE";
		break;

	case State::MEASURE:
		state_str = "MEASURE";
		break;
	}

	PX4_INFO("state: %s", state_str);
	PX4_INFO("ADC_CONFIG: 0x%04X (%s), %" PRIu32 " us per sample",
		 _timing.adc_config, _timing.triggered ? "triggered" : "continuous", _timing.conversion_us);
	PX4_INFO("sample interval: %" PRIu32 " us (min %" PRIu32 ")", _timing.interval_us, _timing.min_interval_us);

	if (_timing.triggered) {
		PX4_INFO("readback CONFIG: 0x%04X, SHUNT_CAL: 0x%04X", _last_readback[0], _last_readback[1]);

	} else {
		PX4_INFO("readback CONFIG: 0x%04X, SHUNT_CAL: 0x%04X, ADC_CONFIG: 0x%04X",
			 _last_readback[0], _last_readback[1], _last_readback[2]);
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_collection_errors);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_reinit_perf);
	perf_print_counter(_not_ready_perf);
}

I2CSPIDriverBase *INA228::instantiate(const I2CSPIDriverConfig &config, int /*runtime_instance*/)
{
	INA228 *instance = new INA228(config, config.custom1);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() == PX4_OK) {
		instance->ScheduleNow();

	} else if (config.keep_running) {
		// Driver stays alive even if the device isn't powered yet; RunImpl will retry.
		PX4_INFO("ina228 init failed on bus %d, will retry every %u ms.", config.bus,
			 static_cast<unsigned>(INIT_RETRY_INTERVAL_US / 1000));
		instance->ScheduleDelayed(INIT_RETRY_INTERVAL_US);

	} else {
		delete instance;
		return nullptr;
	}

	return instance;
}

void INA228::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the Texas Instruments INA228 power monitor.

Multiple instances can run simultaneously on separate buses or different I2C addresses.

If the device is not powered at startup, pass `-k` (keep_running) and the driver
will retry initialization every 500 ms so the battery can be plugged in later.

The ADC setup comes from `INA228_CONFIG` (written to the ADC_CONFIG register). The device
is read every 100 ms, or slower if one averaged sample takes longer, so every read returns
a new sample. The configuration registers are read back periodically and rewritten if the
device was reset (e.g. by a brown-out while plugging the battery).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ina228", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x45);
	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 3, "battery index for calibration values (1-3)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int ina228_main(int argc, char *argv[])
{
	using ThisDriver = INA228;
	BusCLIArguments cli{true, false};
	cli.i2c_address = 0x45;
	cli.default_i2c_frequency = BUS_CLOCK_HZ;
	cli.support_keep_running = true;
	cli.custom1 = 1;

	int ch;

	while ((ch = cli.getOpt(argc, argv, "t:")) != EOF) {
		switch (ch) {
		case 't':
			cli.custom1 = static_cast<int>(strtol(cli.optArg(), nullptr, 0));

			if (cli.custom1 < 1 || cli.custom1 > 3) {
				PX4_ERR("index must be 1-3");
				return -1;
			}

			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_POWER_DEVTYPE_INA228);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
