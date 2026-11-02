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
	  // On NuttX, I2C::init() rejects a bus running faster than the frequency the driver was started with
	  _trigger_write_min_us(config.bus_frequency > 0 ? TRIGGER_WRITE_BITS * 1'000'000 / config.bus_frequency : 0)
{
	const float max_current = _param_ina226_current.get();
	const float shunt_resistance = _param_ina226_shunt.get();

	_current_lsb = max_current / 32768.f; // From datasheet: current_lsb = max_current / 2^15

	// CAL is a 15-bit field (D15 is reserved). Legal INA226_CURRENT × INA226_SHUNT combos can
	// overflow it, and a value that doesn't fit would never read back as written.
	const float denom = _current_lsb * shunt_resistance;
	const float cal = (denom > 0.f) ? (CAL_K / denom) : 0.f;

	if ((cal < 1.f) || (cal > CALIBRATION_MAX)) {
		PX4_ERR("INA226 calibration %.1f out of range (INA226_CURRENT=%.4f INA226_SHUNT=%.9f)",
			(double)cal, (double)max_current, (double)shunt_resistance);
		_current_lsb = 0.f;
		_calibration = 0;

	} else {
		_calibration = static_cast<uint16_t>(cal);
	}

	// Triggered: writing this value starts one conversion. The bus voltage LSB (1.25 mV) is coarse,
	// so the shunt gets the longer conversion time.
	_config_value = MODE_SHUNT_BUS_TRIG | VSHCT_1100US | VBUSCT_588US | AVERAGES_4;

	if (_calibration != 0) {
		// Publish an initial disconnected status so the first instance grabs uORB instance 0 immediately.
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
	}

	// Let the lower I2C layer absorb transient bus errors before we see them.
	I2C::_retries = 5;
}

INA226::~INA226()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_reinit_perf);
}

int INA226::init()
{
	if (_calibration == 0) {
		return PX4_ERROR;
	}

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
			_failing_since = 0;
			ScheduleDelayed(RESET_DELAY_US);
			return;
		}

	case State::CONFIGURE: {
			// The CONFIGURATION write also triggers the first conversion.
			const bool ok = (probe() == PX4_OK) &&
					(registerWrite(Register::CALIBRATION, _calibration) == PX4_OK) &&
					(registerWrite(Register::CONFIGURATION, _config_value) == PX4_OK);

			if (!ok) {
				// Keep retrying every tick and only report the battery as disconnected (RESET)
				// after DISCONNECT_DEBOUNCE_US without a published sample.
				if (_failing_since == 0) {
					_failing_since = start_time;
				}

				if (start_time - _failing_since >= DISCONNECT_DEBOUNCE_US) {
					perf_count(_reinit_perf);
					PX4_WARN("consecutive failures, resetting");
					_state = State::RESET;
					ScheduleNow();

				} else {
					ScheduleDelayed(SAMPLE_INTERVAL_US);
				}

				return;
			}

			_trigger_time = hrt_absolute_time();
			_last_config_check = start_time;
			_state = State::MEASURE;

			// The first tick comes one interval later, when the first conversion is complete.
			ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
			return;
		}

	case State::MEASURE: {
			const int ret = collect();

			if (ret == -EAGAIN) {
				// The previous conversion can't be done yet (this tick came early): read it next tick.
				return;
			}

			if (ret != PX4_OK) {
				// The device may have lost power and come back with its reset configuration, and
				// its result registers read zero until the first conversion after that. Write the
				// configuration again before the next read instead of publishing such a sample.
				if (_failing_since == 0) {
					_failing_since = start_time;
				}

				ScheduleClear();

				if (start_time - _failing_since >= DISCONNECT_DEBOUNCE_US) {
					perf_count(_reinit_perf);
					PX4_WARN("consecutive failures, resetting");
					_state = State::RESET;
					ScheduleNow();

				} else {
					_state = State::CONFIGURE;
					ScheduleDelayed(SAMPLE_INTERVAL_US);
				}

				return;
			}

			_failing_since = 0;

			// Checked after publishing so it doesn't add to the sample latency. A failed
			// read here is only counted as a comms error; the sample reads above decide
			// whether the device is gone.
			if (start_time - _last_config_check >= CONFIG_CHECK_INTERVAL_US) {
				_last_config_check = start_time;

				if (checkCalibration() == -EBADMSG) {
					// The device lost its configuration (e.g. brown-out while plugging the battery)
					// but is still responding: write it again without reporting the battery as
					// disconnected, which the commander would treat as a battery failure in flight.
					perf_count(_reinit_perf);
					PX4_WARN("configuration lost, reconfiguring");
					ScheduleClear();
					_state = State::CONFIGURE;
					ScheduleNow();
				}
			}

			return;
		}
	}
}

int INA226::collect()
{
	// Ticks are on a fixed hrt grid, so a tick after a late one comes early. Triggering
	// before the previous conversion is done would restart it and we would read (and
	// publish) the one before it again. _trigger_time is taken after the previous trigger
	// write, so that conversion started before it, and the next one starts when the write
	// below ends, at least _trigger_write_min_us from now.
	if (hrt_elapsed_time(&_trigger_time) + _trigger_write_min_us < CONVERSION_TIME_MAX_US) {
		return -EAGAIN;
	}

	perf_begin(_sample_perf);

	// Start the next conversion first. The result registers keep the previous, completed
	// conversion until the new one finishes, so reading them right after the trigger returns
	// the sample triggered one tick ago. This gives a fixed sample latency and the full tick
	// for the conversion.
	if (registerWrite(Register::CONFIGURATION, _config_value) != PX4_OK) {
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	_trigger_time = hrt_absolute_time();

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

int INA226::checkCalibration()
{
	// CONFIGURATION is not checked: it is rewritten every tick to trigger the next conversion.
	uint16_t actual = 0;

	if (registerRead(Register::CALIBRATION, actual) != PX4_OK) {
		return -EIO;
	}

	if (actual != _calibration) {
		// I2C has no CRC: confirm the mismatch with a second read before acting on it.
		if (registerRead(Register::CALIBRATION, actual) != PX4_OK) {
			return -EIO;
		}

		if (actual != _calibration) {
			return -EBADMSG;
		}
	}

	return PX4_OK;
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
	perf_print_counter(_reinit_perf);
}

I2CSPIDriverBase *INA226::instantiate(const I2CSPIDriverConfig &config, int /*runtime_instance*/)
{
	INA226 *instance = new INA226(config, config.custom1);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->_calibration == 0) {
		delete instance;
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

The device is sampled at 100 Hz. Every sample is triggered by the driver, so the latency
from the measurement to the publication is the same for every sample.
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
