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

namespace ina226
{

static constexpr uint32_t BUS_CLOCK_HZ = 100'000;

static constexpr uint16_t MANFID = 0x5449; // TI
static constexpr uint16_t DIEID = 0x2260; // INA226 die + revision 0

// Measurement scaling (from datasheet)
static constexpr float V_LSB = 1.25e-3f; // bus voltage: V per LSB
static constexpr float CAL_K = 5.12e-3f; // CALIBRATION = CAL_K / (current_lsb * R_SHUNT)

// Sample timing
// Default ADC config produces one averaged sample every (588us + 588us) * 64 = 75.264 ms. Poll a hair slower.
static constexpr hrt_abstime SAMPLE_INTERVAL_US = 100_ms;

// Recovery / robustness timing
static constexpr hrt_abstime INIT_RETRY_INTERVAL_US = 500_ms;
static constexpr hrt_abstime RESET_DELAY_US = 1_ms;
static constexpr hrt_abstime DISCONNECT_DEBOUNCE_US = 2_s;
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = DISCONNECT_DEBOUNCE_US / SAMPLE_INTERVAL_US;

// Register map (subset used by this driver)
enum class Register : uint8_t {
	CONFIGURATION = 0x00,
	SHUNTVOLTAGE = 0x01,
	BUSVOLTAGE = 0x02,
	POWER = 0x03,
	CURRENT = 0x04,
	CALIBRATION = 0x05,
	MASKENABLE = 0x06,
	ALERTLIMIT = 0x07,
	MFG_ID = 0xfe,
	DIE_ID = 0xff,
};

// CONFIGURATION register bits
enum CONFIG_BIT : uint16_t {
	RST = (1u << 15),

	// Averaging count (bits 9-11)
	AVERAGES_1 = (0u << 9),
	AVERAGES_4 = (1u << 9),
	AVERAGES_16 = (2u << 9),
	AVERAGES_64 = (3u << 9),
	AVERAGES_128 = (4u << 9),
	AVERAGES_256 = (5u << 9),
	AVERAGES_512 = (6u << 9),
	AVERAGES_1024 = (7u << 9),

	// Bus voltage conversion time (bits 6-8)
	VBUSCT_140US = (0u << 6),
	VBUSCT_204US = (1u << 6),
	VBUSCT_332US = (2u << 6),
	VBUSCT_588US = (3u << 6),
	VBUSCT_1100US = (4u << 6),
	VBUSCT_2116US = (5u << 6),
	VBUSCT_4156US = (6u << 6),
	VBUSCT_8244US = (7u << 6),

	// Shunt voltage conversion time (bits 3-5)
	VSHCT_140US = (0u << 3),
	VSHCT_204US = (1u << 3),
	VSHCT_332US = (2u << 3),
	VSHCT_588US = (3u << 3),
	VSHCT_1100US = (4u << 3),
	VSHCT_2116US = (5u << 3),
	VSHCT_4156US = (6u << 3),
	VSHCT_8244US = (7u << 3),

	// Mode (bits 0-2)
	MODE_SHUTDOWN = (0u << 0),
	MODE_SHUNT_TRIG = (1u << 0),
	MODE_BUS_TRIG = (2u << 0),
	MODE_SHUNT_BUS_TRIG = (3u << 0),
	MODE_ADC_OFF = (4u << 0),
	MODE_SHUNT_CONT = (5u << 0),
	MODE_BUS_CONT = (6u << 0),
	MODE_SHUNT_BUS_CONT = (7u << 0),
};

} // namespace ina226


class INA226 : public device::I2C, public ModuleParams, public I2CSPIDriver<INA226>
{
public:
	INA226(const I2CSPIDriverConfig &config, int battery_index);
	~INA226() override;

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
		CONFIGURE, // write CALIBRATION / CONFIGURATION, then transition to MEASURE
		MEASURE, // steady-state: read BUSVOLTAGE / CURRENT, publish, repeat
	};

	int collect();

	// Rotates through the configuration registers one per call. Returns false
	// if a read fails or the value doesn't match what we wrote.
	bool checkConfigurationRotating();

	int registerRead(ina226::Register reg, uint16_t &value);
	int registerWrite(ina226::Register reg, uint16_t value);

	// --- State -------------------------------------------------------------
	Battery _battery;

	State _state{State::UNINITIALIZED};
	uint8_t _consecutive_failures{0};

	uint8_t _next_reg_to_check{0};

	// Configuration computed from params
	float _current_lsb{0.f};
	uint16_t _calibration{0};
	uint16_t _config_value{0}; // CONFIGURATION register value we wrote

	// Perf counters
	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _collection_errors;
	perf_counter_t _bad_register_perf;
	perf_counter_t _reinit_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INA226_CURRENT>) _param_ina226_current,
		(ParamFloat<px4::params::INA226_SHUNT>) _param_ina226_shunt
	);
};
