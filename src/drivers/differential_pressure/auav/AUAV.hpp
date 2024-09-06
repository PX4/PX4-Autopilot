/****************************************************************************
 *
 *   Copyright (c) 2017-2024 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>

using namespace time_literals;

static constexpr uint8_t I2C_ADDRESS_DIFFERENTIAL = 0x26;
static constexpr uint8_t I2C_ADDRESS_ABSOLUTE = 0x27;
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

class AUAV : public device::I2C, public I2CSPIDriver<AUAV>
{
public:
	explicit AUAV(const I2CSPIDriverConfig &config);
	virtual ~AUAV();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, const int runtime_instance);
	static void print_usage();

	virtual void RunImpl();
	void print_status() override;
	int init() override;

protected:
	enum class STATE : uint8_t {
		READ_CALIBDATA,
		REQUEST_MEASUREMENT,
		GATHER_MEASUREMENT
	};

	struct calib_eeprom_addr_t {
		uint8_t addr_a_hw;
		uint8_t addr_a_lw;
		uint8_t addr_b_hw;
		uint8_t addr_b_lw;
		uint8_t addr_c_hw;
		uint8_t addr_c_lw;
		uint8_t addr_d_hw;
		uint8_t addr_d_lw;
		uint8_t addr_tc50;
		uint8_t addr_es;
	};

	struct calib_data_raw_t {
		uint16_t a_hw;
		uint16_t a_lw;
		uint16_t b_hw;
		uint16_t b_lw;
		uint16_t c_hw;
		uint16_t c_lw;
		uint16_t d_hw;
		uint16_t d_lw;
		uint16_t tc50;
		uint16_t es;
	};

	struct calib_data_t {
		float a;
		float b;
		float c;
		float d;
		float tc50h;
		float tc50l;
		float es;
	};

	virtual void publish_pressure(const float pressure_p, const float temperature_c,
				      const hrt_abstime timestamp_sample) = 0;
	virtual int64_t get_conversion_interval() const = 0;
	virtual calib_eeprom_addr_t get_calib_eeprom_addr() const = 0;
	virtual float process_pressure_dig(const float pressure_dig) const = 0;

	void handle_state_read_calibdata();
	void handle_state_request_measurement();
	void handle_state_gather_measurement();

	int read_calibration_eeprom(const uint8_t eeprom_address, uint16_t &data);
	void process_calib_data_raw(const calib_data_raw_t calib_data_raw);
	float correct_pressure(const uint32_t pressure_raw, const uint32_t temperature_raw) const;
	float process_temperature_raw(const float temperature_raw) const;

	float _cal_range{10.0f};
	STATE _state{STATE::READ_CALIBDATA};
	calib_data_t _calib_data {};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

private:
	int probe() override;
};
