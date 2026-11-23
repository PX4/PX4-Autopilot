/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <stddef.h>
#include <stdint.h>

namespace Bosch_BMP388
{

static constexpr uint32_t I2C_SPEED = 100 * 1000;

static constexpr uint8_t BMP388_CHIP_ID = 0x50;
static constexpr uint8_t BMP390_CHIP_ID = 0x60;

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);

enum class Register : uint8_t {
	CHIP_ID = 0x00,
	REV_ID = 0x01,
	ERR_REG = 0x02,
	STATUS = 0x03,
	DATA_0 = 0x04, // pressure 0x04..0x06, temperature 0x07..0x09
	PWR_CTRL = 0x1B,
	OSR = 0x1C,
	TRIM_CRC = 0x30, // CRC-8 over NVM_PAR_T1..NVM_PAR_P11 (bmp3_selftest.c)
	NVM_PAR_T1 = 0x31, // calibration data 0x31..0x45
	CMD = 0x7E,
};

// ERR_REG
enum ERR_REG_BIT : uint8_t {
	CMD_ERR = Bit1,
};

// STATUS
enum STATUS_BIT : uint8_t {
	DRDY_TEMP = Bit6,
	DRDY_PRESS = Bit5,
	CMD_RDY = Bit4,
};

// PWR_CTRL
enum PWR_CTRL_BIT : uint8_t {
	MODE_FORCED = Bit4, // mode[5:4]: 00 sleep, 01/10 forced, 11 normal
	TEMP_EN = Bit1,
	PRESS_EN = Bit0,
};

// OSR
enum OSR_BIT : uint8_t {
	OSR_T_X2 = Bit3, // osr_t[5:3]: 2^n oversampling
	OSR_P_X16 = Bit2, // osr_p[2:0]: 2^n oversampling
};

// CMD
enum CMD_VALUE : uint8_t {
	SOFTRESET = 0xB6,
};

#pragma pack(push, 1)
// NVM_PAR_T1..NVM_PAR_P11 as laid out from 0x31, LSB first (BMP390 DS table 24)
struct calib_data {
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t par_t3;
	int16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int8_t par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t par_p7;
	int8_t par_p8;
	int16_t par_p9;
	int8_t par_p10;
	int8_t par_p11;
};

// DATA_0..DATA_5 (0x04..0x09). DS 3.10 asks for one burst from press_xlsb to temp_msb.
struct sensor_data {
	uint8_t press_7_0;
	uint8_t press_15_8;
	uint8_t press_23_16;
	uint8_t temp_7_0;
	uint8_t temp_15_8;
	uint8_t temp_23_16;
};
#pragma pack(pop)

static_assert(sizeof(calib_data) == 21);
static_assert(sizeof(sensor_data) == 6);

} // namespace Bosch_BMP388

using namespace Bosch_BMP388;

class BMP388 : public device::I2C, public I2CSPIDriver<BMP388>
{
public:
	BMP388(const I2CSPIDriverConfig &config);
	~BMP388() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:
	int probe() override;

	bool Configure();
	bool Measure();
	int Collect();
	void Failure();

	bool ReadCalibration();

	int64_t CompensateTemperature(uint32_t uncomp_temp, int64_t &t_lin) const;
	uint64_t CompensatePressure(uint32_t uncomp_press, int64_t t_lin) const;

	bool RegisterRead(Register reg, void *buf, size_t len);
	uint8_t RegisterRead(Register reg);
	bool RegisterWrite(Register reg, uint8_t value);

	PX4Barometer _px4_baro;

	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _conversion_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": conversion timeout")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		MEASURE,
		COLLECT,
	} _state{STATE::RESET};

	hrt_abstime _measure_timestamp{0};
	int _failure_count{0};
	int _configure_failures{0};
	int _cmd_rdy_polls{0};

	uint8_t _chip_id{0};
	uint8_t _rev_id{0};
	uint32_t _conversion_time_typ_us{0};
	uint32_t _conversion_time_max_us{0};

	calib_data _calib{};
};
