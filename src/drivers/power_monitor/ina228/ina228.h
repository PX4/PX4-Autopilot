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
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

namespace ina228
{

static constexpr uint16_t MANFID = 0x5449;
static constexpr uint16_t DIEID = 0x228;

// Measurement scaling (from datasheet SLYS021A)
static constexpr float V_LSB = 195.3125e-6f; // V per LSB (VBUS, 20 bit)
static constexpr float T_LSB = 7.8125e-3f; // °C per LSB (DIETEMP)
static constexpr float CURRENT_LSB_DIV = 524288.f; // current_lsb = max_current / 2^19
static constexpr float SHUNT_CAL_K = 13107.2e6f; // shunt-cal scaling constant
static constexpr float ADCRANGE_LOW_V_SENSE = 0.04096f; // ±40.96 mV
static constexpr uint16_t SHUNT_CAL_MAX = 0x7fff; // SHUNT_CAL is a 15 bit field

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

// ADCCONFIG register bits (subset used by this driver)
enum ADCCONFIG_BIT : uint16_t {
	AVERAGES_4 = (1u << 0),

	VTCT_50US = (0u << 3),
	VSHCT_1052US = (5u << 6),
	VBUSCT_1052US = (5u << 9),

	MODE_TEMP_SHUNT_BUS_TRIG = (7u << 12), // triggered (single-shot) temperature, shunt and bus voltage
};

// Wake-up time from shutdown, the state the device is in between triggered conversions
static constexpr uint32_t WAKEUP_TIME_US = 60;

// DEVICE_ID register: die ID in bits 15..4
static constexpr uint16_t DEVICE_ID_MASK = 0xfff0u;
static constexpr uint16_t DEVICE_ID_SHIFT = 4;

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
		MEASURE, // steady-state: trigger, read VS_BUS / CURRENT / DIETEMP, publish, repeat
	};

	int collect();

	// Rotates through the configuration registers, one per call. Returns PX4_OK,
	// -EIO if the read fails, or -EBADMSG if the value doesn't match what we wrote
	// (the device has been reset behind our back).
	int checkConfigurationRotating();

	int registerRead(ina228::Register reg, uint16_t &value);
	int registerRead24(ina228::Register reg, int32_t &value);
	int registerWrite(ina228::Register reg, uint16_t value);

	// --- State -------------------------------------------------------------
	Battery _battery;

	State _state{State::UNINITIALIZED};
	hrt_abstime _failing_since{0}; // first failure of the current run of failures, 0 if none

	uint8_t _next_reg_to_check{0};
	hrt_abstime _last_config_check{0};
	uint16_t _last_readback[2] {}; // CONFIG, SHUNT_CAL
	hrt_abstime _trigger_time{0}; // when the last conversion was triggered
	const hrt_abstime _trigger_write_min_us; // shortest time the trigger write takes on the bus

	// Configuration computed from params
	float _current_lsb{0.f};
	uint16_t _shunt_calibration{0};
	uint16_t _config_value{0}; // CONFIG register value we wrote

	// Perf counters
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "ina228_read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "ina228_com_err")};
	perf_counter_t _reinit_perf{perf_alloc(PC_COUNT, "ina228_reinit")};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INA228_CURRENT>) _param_ina228_current,
		(ParamFloat<px4::params::INA228_SHUNT>) _param_ina228_shunt
	);
};
