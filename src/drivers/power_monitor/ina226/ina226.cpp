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

#include "ina226.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace ina226;

INA226::INA226(const I2CSPIDriverConfig &config, int battery_index)
	: I2C(config),
	  ModuleParams(nullptr),
	  I2CSPIDriver(config),
	  _battery(battery_index, this, SAMPLE_INTERVAL_US, battery_status_s::SOURCE_POWER_MODULE),
	  _sample_perf(perf_alloc(PC_ELAPSED, "ina226_read")),
	  _comms_errors(perf_alloc(PC_COUNT, "ina226_com_err")),
	  _collection_errors(perf_alloc(PC_COUNT, "ina226_collection_err")),
	  _bad_register_perf(perf_alloc(PC_COUNT, "ina226_bad_register")),
	  _reinit_perf(perf_alloc(PC_COUNT, "ina226_reinit"))
{
	const float max_current = _param_ina226_current.get();
	const float shunt_resistance = _param_ina226_shunt.get();

	_current_lsb = max_current / 32768.f; // From datasheet: current_lsb = max_current / 2^15
	_calibration = static_cast<uint16_t>(CAL_K / (_current_lsb * shunt_resistance));

	_config_value = MODE_SHUNT_BUS_CONT | VSHCT_588US | VBUSCT_588US | AVERAGES_64;

	// Publish an initial disconnected status so the first instance grabs uORB instance 0 immediately.
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	// Let the lower I2C layer absorb transient bus errors before we see them.
	I2C::_retries = 5;
}

INA226::~INA226()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_bad_register_perf);
	perf_free(_reinit_perf);
}

int INA226::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	_state = State::RESET;
	return PX4_OK;
}

void INA226::RunImpl()
{
	const hrt_abstime start_time = hrt_absolute_time();

	switch (_state) {
	case State::UNINITIALIZED: {
			if (init() != PX4_OK) {
				_battery.updateAndPublishBatteryStatus(start_time);
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
			_battery.updateAndPublishBatteryStatus(start_time);

			if (registerWrite(Register::CONFIGURATION, RST) != PX4_OK) {
				ScheduleDelayed(INIT_RETRY_INTERVAL_US);
				return;
			}

			_state = State::CONFIGURE;
			ScheduleDelayed(RESET_DELAY_US);
			return;
		}

	case State::CONFIGURE: {
			const bool ok = (probe() == PX4_OK) &&
					(registerWrite(Register::CALIBRATION, _calibration) == PX4_OK) &&
					(registerWrite(Register::CONFIGURATION, _config_value) == PX4_OK);

			if (!ok) {
				_state = State::RESET;
				ScheduleDelayed(INIT_RETRY_INTERVAL_US);
				return;
			}

			// Communication success
			_consecutive_failures = 0;
			_next_reg_to_check = 0;
			_state = State::MEASURE;

			// Wait one full sample period + some margin
			ScheduleDelayed(SAMPLE_INTERVAL_US + 5_ms);
			return;
		}

	case State::MEASURE: {
			if (collect() == PX4_OK) {
				_consecutive_failures = 0;

			} else {
				perf_count(_collection_errors);

				if (++_consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
					perf_count(_reinit_perf);
					_state = State::RESET;
					_consecutive_failures = 0;
					PX4_WARN("consecutive failures, resetting");
					ScheduleNow();
					return;
				}
			}

			const hrt_abstime elapsed = hrt_elapsed_time(&start_time);
			const hrt_abstime scheduled_time = elapsed < SAMPLE_INTERVAL_US ? SAMPLE_INTERVAL_US - elapsed : 0;

			ScheduleDelayed(scheduled_time);
			return;
		}
	}
}

int INA226::collect()
{
	perf_begin(_sample_perf);

	// Verify one config register per cycle
	const bool config_ok = checkConfigurationRotating();

	if (!config_ok) {
		perf_count(_bad_register_perf);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	int16_t bus_voltage = 0;
	int16_t current = 0;

	const bool reads_ok = (registerRead(Register::BUSVOLTAGE, (uint16_t &)bus_voltage) == PX4_OK)
			      && (registerRead(Register::CURRENT, (uint16_t &)current) == PX4_OK);

	if (reads_ok) {
		_battery.setConnected(true);
		_battery.updateVoltage(static_cast<float>(bus_voltage) * V_LSB);
		_battery.updateCurrent(static_cast<float>(current) * _current_lsb);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
	}

	perf_end(_sample_perf);
	return reads_ok ? PX4_OK : PX4_ERROR;
}

int INA226::probe()
{
	uint16_t value = 0;

	if (registerRead(Register::MFG_ID, value) != PX4_OK || value != MANFID) {
		PX4_DEBUG("probe mfgid %d", value);
		return PX4_ERROR;
	}

	if (registerRead(Register::DIE_ID, value) != PX4_OK || value != DIEID) {
		PX4_DEBUG("probe die id %d", value);
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool INA226::checkConfigurationRotating()
{
	const struct {
		Register reg;
		uint16_t expected;
		uint16_t mask;
	} checks[] = {
		// CONFIGURATION D15 (RST) self-clears; D14:D12 are reserved and D14 reads back as 1.
		{ Register::CONFIGURATION, _config_value, 0x0FFF },
		{ Register::CALIBRATION,   _calibration,  0xFFFF },
	};

	const auto &check = checks[_next_reg_to_check];
	uint16_t actual = 0;

	if (registerRead(check.reg, actual) != PX4_OK) {
		return false;
	}

	if ((actual & check.mask) != (check.expected & check.mask)) {
		return false;
	}

	_next_reg_to_check = (_next_reg_to_check + 1) % (sizeof(checks) / sizeof(checks[0]));
	return true;
}

int INA226::registerRead(Register reg, uint16_t &value)
{
	uint8_t address = static_cast<uint8_t>(reg);
	uint16_t raw = 0;

	const int ret = transfer(&address, 1, (uint8_t *)&raw, sizeof(raw));

	if (ret == PX4_OK) {
		value = __builtin_bswap16(raw);

	} else {
		perf_count(_comms_errors);
	}

	return ret;
}

int INA226::registerWrite(Register reg, uint16_t value)
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

void INA226::print_status()
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
	PX4_INFO("sample interval: %llu us", SAMPLE_INTERVAL_US);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_collection_errors);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_reinit_perf);
}

I2CSPIDriverBase *INA226::instantiate(const I2CSPIDriverConfig &config, int /*runtime_instance*/)
{
	INA226 *instance = new INA226(config, config.custom1);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() == PX4_OK) {
		instance->ScheduleNow();

	} else if (config.keep_running) {
		// Driver stays alive even if the device isn't powered yet; RunImpl will retry.
		PX4_INFO("ina226 init failed on bus %d, will retry every %llu ms.", config.bus, INIT_RETRY_INTERVAL_US / 1000);
		instance->ScheduleDelayed(INIT_RETRY_INTERVAL_US);

	} else {
		delete instance;
		return nullptr;
	}

	return instance;
}

void INA226::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the Texas Instruments INA226 power monitor.

Multiple instances can run simultaneously on separate buses or different I2C addresses.

If the device is not powered at startup, pass `-k` (keep_running) and the driver
will retry initialization every 500 ms so the battery can be plugged in later.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ina226", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x41);
	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 3, "battery index for calibration values (1-3)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int ina226_main(int argc, char *argv[])
{
	using ThisDriver = INA226;
	BusCLIArguments cli{true, false};
	cli.i2c_address = 0x41;
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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_POWER_DEVTYPE_INA226);

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
