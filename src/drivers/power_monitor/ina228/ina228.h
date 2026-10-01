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

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/battery/battery.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

namespace ina228
{

static constexpr uint32_t BUS_CLOCK_HZ = 100'000;

static constexpr uint16_t MANFID = 0x5449;
static constexpr uint16_t DIEID = 0x228;

// Measurement scaling (from datasheet SLYS021A)
static constexpr float V_LSB = 195.3125e-6f; // V per LSB (VBUS, 20 bit)
static constexpr float T_LSB = 7.8125e-3f; // °C per LSB (DIETEMP)
static constexpr float CURRENT_LSB_DIV = 524288.f; // current_lsb = max_current / 2^19
static constexpr float SHUNT_CAL_K = 13107.2e6f; // shunt-cal scaling constant
static constexpr float ADCRANGE_LOW_V_SENSE = 0.04096f; // ±40.96 mV
static constexpr uint16_t SHUNT_CAL_MAX = 0x7fff; // SHUNT_CAL is a 15 bit field

// Default ADC_CONFIG: continuous, 540us conversion per channel x 3 channels x 64-sample average
// = 103.68 ms per output sample.
static constexpr uint16_t ADCCONFIG_DEFAULT = 0xF923;

// ADC timing (datasheet §7.3.4, §6.5)
static constexpr uint16_t CONVERSION_TIME_US[8] = {50, 84, 150, 280, 540, 1052, 2074, 4120};
static constexpr uint16_t AVERAGES[8] = {1, 4, 16, 64, 128, 256, 512, 1024};
static constexpr float OSCILLATOR_TOLERANCE = 1.01f; // ±1 % over temperature
static constexpr hrt_abstime WAKEUP_TIME_US = 60; // from shutdown, triggered mode only
// Triggered mode: the next trigger (the start of the next tick) must not land before the
// previous conversion is done, or it restarts that conversion and the output freezes.
// Covers tick-to-tick scheduling jitter and the duration of the trigger write itself.
static constexpr hrt_abstime TRIGGER_MARGIN_US = 500;
// Nominal sample interval, stretched if the ADC setup needs longer per sample.
static constexpr uint32_t SAMPLE_INTERVAL_US = 100'000;

// Recovery / robustness timing
static constexpr hrt_abstime INIT_RETRY_INTERVAL_US = 500_ms;
static constexpr hrt_abstime RESET_DELAY_US = 1_ms; // datasheet specifies 300us. Give some margin
static constexpr hrt_abstime DISCONNECT_DEBOUNCE_US = 2_s;
static constexpr hrt_abstime CONFIG_CHECK_INTERVAL_US = 100_ms;

// Register map (subset used by this driver)
enum class Register : uint8_t {
	CONFIG = 0x00,
	ADCCONFIG = 0x01,
	SHUNT_CAL = 0x02,
	VS_BUS = 0x05,
	DIETEMP = 0x06,
	CURRENT = 0x07,
	MANUFACTURER_ID = 0x3e,
	DEVICE_ID = 0x3f,
};

// CONFIG register bits
enum CONFIG_BIT : uint16_t {
	RST = (1u << 15),
	RANGE_HIGH = (0u << 4), // ±163.84 mV — used when R_SHUNT * I_MAX > 40.96 mV
	RANGE_LOW = (1u << 4), // ±40.96 mV
};

// ADC_CONFIG register fields
static constexpr uint16_t MODE_SHIFT = 12;
static constexpr uint16_t MODE_CONTINUOUS = 0x8; // MODE bit 3: 1 = continuous, 0 = triggered
static constexpr uint16_t MODE_BUS = 0x1; // MODE bit 0: bus voltage enabled
static constexpr uint16_t MODE_SHUNT = 0x2; // MODE bit 1: shunt voltage enabled
static constexpr uint16_t MODE_TEMP = 0x4; // MODE bit 2: temperature enabled
static constexpr uint16_t VBUSCT_SHIFT = 9;
static constexpr uint16_t VSHCT_SHIFT = 6;
static constexpr uint16_t VTCT_SHIFT = 3;
static constexpr uint16_t AVG_SHIFT = 0;

// DEVICE_ID register field accessor
static constexpr uint16_t DEVICE_ID_MASK = 0xfff0u;
static inline constexpr uint16_t deviceId(uint16_t v) { return (v & DEVICE_ID_MASK) >> 4; }

} // namespace ina228


class INA228 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA228>
{
public:
	INA228(const I2CSPIDriverConfig &config, int battery_index);
	~INA228() override;

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int init() override;
	void RunImpl();

	void print_status() override;

protected:
	int probe() override;

private:
	enum class State : uint8_t {
		UNINITIALIZED, // I2C::init() not yet called successfully — retry until it does
		RESET, // soft-reset the device, then transition to CONFIGURE
		CONFIGURE, // write SHUNT_CAL / CONFIG / ADCCONFIG, then transition to MEASURE
		MEASURE, // steady-state: (trigger,) read VS_BUS / CURRENT / DIETEMP, publish, repeat
	};

	// Sampling setup derived from INA228_CONFIG. Computed before the Battery member is
	// constructed, because Battery needs the sample interval.
	struct Timing {
		uint16_t adc_config;
		bool triggered;
		bool temperature_enabled;
		uint32_t conversion_us; // nominal time for one averaged output sample
		uint32_t conversion_max_us; // conversion_us with the oscillator running slow
		uint32_t min_interval_us; // shortest interval this ADC_CONFIG supports
		uint32_t interval_us; // effective sample interval
	};

	static Timing computeTiming();

	int collect();
	void enterReset();
	void enterConfigure();

	// Rotates through the configuration registers, one per call. Returns PX4_OK,
	// -EIO if the read fails, or -EBADMSG if the value doesn't match what we wrote
	// (the device has been reset behind our back).
	int checkConfigurationRotating();

	int registerRead(ina228::Register reg, uint16_t &value);
	int registerRead24(ina228::Register reg, int32_t &value);
	int registerWrite(ina228::Register reg, uint16_t value);

	// --- State -------------------------------------------------------------
	const Timing _timing;
	Battery _battery;

	State _state{State::UNINITIALIZED};
	uint16_t _consecutive_failures{0};
	uint16_t _max_consecutive_failures{1};

	uint8_t _next_reg_to_check{0};
	hrt_abstime _last_config_check{0};
	uint16_t _last_readback[3] {}; // CONFIG, SHUNT_CAL, ADCCONFIG
	hrt_abstime _trigger_time{0}; // when the last triggered conversion was started

	// Configuration computed from params
	float _current_lsb{0.f};
	uint16_t _shunt_calibration{0};
	uint16_t _config_value{0}; // CONFIG register value we wrote

	// Perf counters
	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _collection_errors;
	perf_counter_t _bad_register_perf;
	perf_counter_t _reinit_perf;
	perf_counter_t _not_ready_perf;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INA228_CURRENT>) _param_ina228_current,
		(ParamFloat<px4::params::INA228_SHUNT>) _param_ina228_shunt
	);
};
