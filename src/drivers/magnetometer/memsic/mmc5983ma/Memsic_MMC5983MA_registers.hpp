/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file Memsic_MMC5983MA_registers.hpp
 *
 * Memsic MMC5983MA registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Memsic_MMC5983MA
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x30;

static constexpr uint8_t chip_identification_number = 0x30;

enum class Register : uint8_t {
	CHIP_ID       = 0x2F, // magnetometer chip identification number

	DATAX_MSB     = 0x00, // 8-bit LSB of x-axis magnetic field data
	DATAX_LSB     = 0x01, // 8-bit MSB of x-axis magnetic field data
	DATAY_MSB     = 0x02, // 8-bit LSB of y-axis magnetic field data
	DATAY_LSB     = 0x03, // 8-bit MSB of y-axis magnetic field data
	DATAZ_MSB     = 0x04, // 8-bit LSB of z-axis magnetic field data
	DATAZ_LSB     = 0x05, // 8-bit MSB of z-axis magnetic field data
	DATAXYZ_LSB   = 0x06, // 2-bit LSB of 3 axis magnetic field data
	TEMPERATURE   = 0x07, // temperature data

	STATUS        = 0x08, // Status register

	CONTROL0      = 0x09, // control register 0
	CONTROL1      = 0x0A, // control register 1
	CONTROL2      = 0x0B, // control register 2
	CONTROL3      = 0x0C, // control register 3
};

// CONTROL0
enum CONTROL0_BIT : uint8_t {
	MEASURE_FIELD = Bit0,
	MEASURE_TEMP = Bit1,

	INT_ENABLE = Bit2,

	SET        = Bit3,
	RESET      = Bit4,
	AUTOSR     = Bit5,

	OTP_READ   = Bit6,
};

// CONTROL1
enum CONTROL1_BIT : uint8_t {
	BW_SET = 0,
	BW_CLEAR = Bit1 | Bit0,

	INHIBIT = Bit4 | Bit3 | Bit2,
	SOFTRESET = Bit7,
};

// CONTROL2
enum CONTROL2_BIT : uint8_t {
	// 2:0 data rate control
	CMF_20HZ_SET   = Bit1 | Bit0,
	CMF_20HZ_CLEAR = Bit2,

	CM_ENABLE      = Bit3,
	PS_ENABLE      = Bit7,
};

// CONTROL3
enum CONTROL3_BIT : uint8_t {
	ST_ENABLE = Bit2 | Bit1,
	SPI_3W    = Bit6,
};

} // namespace Memsic_MMC5983MA
