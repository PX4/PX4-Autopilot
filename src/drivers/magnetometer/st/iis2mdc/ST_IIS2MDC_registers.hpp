/****************************************************************************
 *
 *   Copyright (c) 2024-2025 PX4 Development Team. All rights reserved.
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
 * @file ST_IIS2MDC_registers.hpp
 *
 * ST IIS2MDC registers.
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

namespace ST_IIS2MDC
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b001'1110;

static constexpr uint8_t Device_ID = 0b0100'0000;

enum class Register : uint8_t {
	WHO_AM_I   = 0x4F,

	CFG_REG_A  = 0x60,
	CFG_REG_B  = 0x61,
	CFG_REG_C  = 0x62,

	STATUS_REG = 0x67,
};

// CFG_REG_A
enum CFG_REG_A_BIT : uint8_t {
	COMP_TEMP_EN        = Bit7, // internal temperature sensor
	REBOOT              = Bit6,
	SOFT_RST            = Bit5,
	LP		    = Bit4, // Low-Power mode

	// 3:2 ODR: 10 50 Hz
	ODR_50_HZ_SET       = Bit3, // ODR1: 1
	ODR_50_HZ_CLEAR     = Bit2, // ODR0: 0

	// 1:0 MD: 00 Continuous mode
	MD_CONTINUOUS_CLEAR = Bit1 | Bit0, // MD1: 0, MD0: 0 Continuous mode
};

// CFG_REG_B
enum CFG_REG_B_BIT : uint8_t {
	OFF_CANC = Bit1, // offset cancellation
	LPF      = Bit0, // digital low-pass filter
};

// CFG_REG_C
enum CFG_REG_C_BIT : uint8_t {
	I2C_DIS   = Bit5,
	BDU       = Bit4,
	BLE       = Bit3, // 1: Big Endian
};

// STATUS_REG
enum STATUS_REG_BIT : uint8_t {
	Zyxor   = Bit7, // X, Y, Z axis data overrun

	Zyxda   = Bit3, // X, Y, Z new data available
};


} // namespace ST_IIS2MDC
