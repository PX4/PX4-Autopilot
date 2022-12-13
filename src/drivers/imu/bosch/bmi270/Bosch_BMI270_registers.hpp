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
 * @file Bosch_BMI270_registers.hpp
 *
 * Bosch BMI270 registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Bosch_BMI270
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t chip_id = 0x24;

enum class Register : uint8_t {
	CHIP_ID            = 0x00,

	ERR_REG            = 0x02,
	STATUS	           = 0x03,

	EVENT	           = 0x1B,

	INTERNAL_STATUS    = 0X21,
	TEMP_MSB           = 0x22,
	TEMP_LSB           = 0x23,
	FIFO_LENGTH_0      = 0x24,
	FIFO_LENGTH_1      = 0x25,
	FIFO_DATA          = 0x26,

	ACC_CONF           = 0x40,
	ACC_RANGE          = 0x41,
	GYR_CONF           = 0x42,

	FIFO_DOWNS         = 0x45,
	FIFO_WTM_0         = 0x46,
	FIFO_WTM_1         = 0x47,
	FIFO_CONFIG_0      = 0x48,
	FIFO_CONFIG_1      = 0x49,

	// controls interrupt pin behaviour
	INT1_IO_CTRL       = 0x53,

	INT_MAP_DATA       = 0x58,
	CONFIG1            = 0x59,

	CONFIG2            = 0x5E,
	PWR_CONF           = 0x7C,
	PWR_CTRL           = 0x7D,
	CMD                = 0x7E,



};

enum GYR_CONF_BIT : uint8_t {
	// 1.6 or 3.2kHz sample rate
	gyr_odr_1k6 = Bit3 | Bit2,
	gyr_odr_3k2 = Bit3 | Bit2 | Bit0,
	// set LPF to normal mode
	gyr_flt_mode_normal = Bit5,
	// set noise profile to high performance mode
	gyr_noise_hp = Bit6,
	// set filter profile to high performance mode
	gyr_flt_hp = Bit7
};

enum PWR_CTRL_BIT : uint8_t {
	gyr_en = Bit1,
	accel_en = Bit2,
	temp_en = Bit3,
};

// ACC_CONF
enum ACC_CONF_BIT : uint8_t {
	// [7:4] acc_bwp
	acc_bwp_Normal = Bit7 | Bit5,        // Filter setting normal

	// [3:0] acc_odr
	acc_odr_1600   = Bit3 | Bit2,        // ODR 1600 Hz
};

// ACC_RANGE
enum ACC_RANGE_BIT : uint8_t {
	acc_range_2g  = 0,           // ±2g
	acc_range_4g  = Bit0,        // ±4g
	acc_range_8g = Bit1,        // ±8g
	acc_range_16g = Bit1 | Bit0, // ±16g
};

// FIFO_CONFIG_0
enum FIFO_CONFIG_0_BIT : uint8_t {
	BIT1_ALWAYS = Bit1, // This bit must always be ‘1’.
	FIFO_mode   = Bit0,
};

// FIFO_CONFIG_1
enum FIFO_CONFIG_1_BIT : uint8_t {
	Acc_en      = Bit6,
	Gyr_en      = Bit7,
	BIT4_ALWAYS = Bit4, // This bit must always be ‘1’.
};

// INT1_IO_CONF
enum INT1_IO_CONF_BIT : uint8_t {
	int1_in  = Bit4,
	int1_out = Bit3,

	int1_lvl = Bit1,
};

// INT1_INT2_MAP_DATA
enum INT1_INT2_MAP_DATA_BIT : uint8_t {
	int2_drdy  = Bit6,
	int2_fwm   = Bit5,
	int2_ffull = Bit4,

	int1_drdy  = Bit2,
	int1_fwm   = Bit1,
	int1_ffull = Bit0,
};

// ACC_PWR_CONF
enum ACC_PWR_CONF_BIT : uint8_t {
	acc_pwr_save = 0x03
};

// ACC_PWR_CTRL
enum ACC_PWR_CTRL_BIT : uint8_t {
	acc_enable = 0x04
};


namespace FIFO
{
static constexpr size_t SIZE = 1024;

struct Data {
	uint8_t x_lsb;
	uint8_t x_msb;
	uint8_t y_lsb;
	uint8_t y_msb;
	uint8_t z_lsb;
	uint8_t z_msb;
};


enum class Header : uint8_t {
	sensor_accel_frame          = 0b10000100,
	sensor_gyro_frame           = 0b10001000,
	skip_frame                  = 0b01000000,
	sensor_time_frame           = 0b01000100,
	FIFO_input_config_frame     = 0b01001000,
	sample_drop_frame           = 0b01010000,
};

} // namespace FIFO

} // namespace Bosch_BMI270
