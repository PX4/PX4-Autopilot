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

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/battery/battery.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>

using namespace time_literals;

namespace ina238
{

static constexpr uint32_t BUS_CLOCK_HZ = 100'000;

static constexpr uint16_t MANFID = 0x5449;
static constexpr uint16_t DIEID = 0x238;

// Measurement scaling (from datasheet)
static constexpr float V_LSB = 3.125e-3f; // V per LSB
static constexpr float T_LSB = 7.8125e-3f; // °C per LSB
static constexpr float SHUNT_CAL_K = 819.2e6f; // shunt-cal scaling constant
static constexpr float ADCRANGE_LOW_V_SENSE = 0.04096f; // ±40.96 mV

// Sample timing
// Each tick triggers one conversion, (1052us bus + 1052us shunt + 50us temperature) x 4 averages
// = 8.616 ms, read on the next tick. The rest of the tick is margin for scheduling latency.
static constexpr uint32_t CONVERSION_TIME_US = (1052 + 1052 + 50) * 4;
static constexpr hrt_abstime SAMPLE_INTERVAL_US = 10_ms;
static constexpr hrt_abstime WAKEUP_TIME_US = 60; // from shutdown, the state between triggered conversions
// Worst case from the trigger to a completed conversion: the internal oscillator running 1 % slow,
// plus the wake-up from shutdown.
static constexpr hrt_abstime CONVERSION_TIME_MAX_US = (CONVERSION_TIME_US * 100 + 98) / 99 + WAKEUP_TIME_US;
// The conversion starts when the trigger write ends, at least this many bit times after the write
// starts: address, register pointer and two data bytes, not counting the ACKs.
static constexpr uint32_t TRIGGER_WRITE_BITS = 32;

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

// DEVICE_ID register field accessor
static constexpr uint16_t DEVICE_ID_MASK = 0xfff0u;
static inline constexpr uint16_t deviceId(uint16_t v) { return (v & DEVICE_ID_MASK) >> 4; }

// ADCCONFIG register bits
enum ADCCONFIG_BIT : uint16_t {
	AVERAGES_1 = (0u << 0),
	AVERAGES_4 = (1u << 0),
	AVERAGES_16 = (2u << 0),
	AVERAGES_64 = (3u << 0),
	AVERAGES_128 = (4u << 0),
	AVERAGES_256 = (5u << 0),
	AVERAGES_512 = (6u << 0),
	AVERAGES_1024 = (7u << 0),

	VTCT_50US = (0u << 3),
	VSHCT_1052US = (5u << 6),
	VBUSCT_1052US = (5u << 9),

	MODE_TEMP_SHUNT_BUS_TRIG = (7u << 12), // triggered (single-shot) temperature, shunt and bus voltage
};

} // namespace ina238


class INA238 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA238>
{
public:
	INA238(const I2CSPIDriverConfig &config, int battery_index);
	~INA238() override;

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
		MEASURE, // steady-state: trigger, read VS_BUS / CURRENT / DIETEMP, publish, repeat
	};

	int collect();

	// Rotates through the configuration registers, one per call. Returns PX4_OK,
	// -EIO if the read fails, or -EBADMSG if the value doesn't match what we wrote
	// (the device has been reset behind our back).
	int checkConfigurationRotating();

	int registerRead(ina238::Register reg, uint16_t &value);
	int registerWrite(ina238::Register reg, uint16_t value);

	// --- State -------------------------------------------------------------
	Battery _battery;

	State _state{State::UNINITIALIZED};
	hrt_abstime _failing_since{0}; // first failure of the current run of failures, 0 if none

	uint8_t _next_reg_to_check{0};
	hrt_abstime _last_config_check{0};
	hrt_abstime _trigger_time{0}; // when the last conversion was triggered
	const hrt_abstime _trigger_write_min_us; // shortest time the trigger write takes on the bus

	// Configuration computed from params
	float _current_lsb{0.f};
	uint16_t _shunt_calibration{0};
	uint16_t _config_value{0}; // CONFIG register value we wrote
	uint16_t _adc_config_value{0}; // ADCCONFIG register value we wrote

	// Perf counters
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "ina238_read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "ina238_com_err")};
	perf_counter_t _reinit_perf{perf_alloc(PC_COUNT, "ina238_reinit")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INA238_CURRENT>) _param_ina238_current,
		(ParamFloat<px4::params::INA238_SHUNT>) _param_ina238_shunt
	);
};
