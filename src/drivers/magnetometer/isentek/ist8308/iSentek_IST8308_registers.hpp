/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file iSentek_IST8308_registers.hpp
 *
 * iSentek IST8308 registers.
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

namespace iSentek_IST8308
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x0C;

static constexpr uint8_t Device_ID = 0x08;

enum class Register : uint8_t {
	WAI     = 0x00, // Who Am I Register

	STR     = 0x0C, // Self-Test Register

	STAT    = 0x10, // Status Register
	DATAXL  = 0x11,
	DATAXH  = 0x12,
	DATAYL  = 0x13,
	DATAYH  = 0x14,
	DATAZL  = 0x15,
	DATAZH  = 0x16,

	ACTR    = 0x20, // Action Register

	CNTL1   = 0x30, // Control Setting Register 1
	CNTL2   = 0x31, // Control Setting Register 2
	CNTL3   = 0x32, // Control Setting Register 3
	CNTL4   = 0x34, // Control Setting Register 4

	OSRCNTL = 0x41, // Over-Sampling Ratio Control Register
};

// STAT
enum STAT_BIT : uint8_t {
	DOR  = Bit1, // Data overrun bit
	DRDY = Bit0, // Data ready
};

// ACTR
enum ACTR_BIT : uint8_t {
	SUSPEND_EN = Bit1,
};

// CNTL1
enum CNTL1_BIT : uint8_t {
	// 6:5 NSF[1:0]: Noise Suppression Filter setting
	NSF_DISABLE = Bit6 | Bit5, // Disable
};

// CNTL2
enum CNTL2_BIT : uint8_t {
	// 4:0 MODE [4:0]: Operation mode setting
	MODE_ODR_50HZ_SET   = Bit2 | Bit1, // 5’h06: Continuous Measurement Mode with ODR 50Hz
	MODE_ODR_50HZ_CLEAR = Bit3 | Bit0,
};

// CNTL3
enum CNTL3_BIT : uint8_t {
	SRST = Bit0, // Soft reset, perform the same routine as POR
};

// CNTL4
enum CNTL4_BIT : uint8_t {
	// DR [1:0]: Sensor Dynamic Range and Sensitivity setting
	DR_200UT_SET   = Bit0, // ±200 uT, Sensitivity=13.2 LSB/uT
	DR_200UT_CLEAR = Bit1
};

// OSRCNTL
enum OSRCNTL_BIT : uint8_t {
	// 5:3
	OSR_Y_32_SET    = Bit5 | Bit3, // 3’b101: OSR=32 (ODRmax=50)
	OSR_Y_32_CLEAR  = Bit4,

	// 2:0
	OSR_XZ_32_SET   = Bit2 | Bit0, // 3’b101: OSR=32 (ODRmax=50)
	OSR_XZ_32_CLEAR = Bit1,
};

} // namespace iSentek_IST8308
