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

/**
 * @file AKM_AK09940A_registers.hpp
 *
 * Asahi Kasei Microdevices (AKM) AK09940A registers.
 *
 */

#pragma once

#include <cstdint>

namespace AKM_AK09940A
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b0001100;

static constexpr uint8_t Company_ID = 0x48;

enum class Register : uint8_t {
	WIA1  = 0x00, // Company ID of AKM
	WIA2  = 0x01, // Device ID of AK09940A

	ST    = 0x0F, // Status (for Polling)
	ST1   = 0x10, // Status 1
	HXL   = 0x11, // Measurements for X-axis data
	HXM   = 0x12,
	HXH   = 0x13,
	HYL   = 0x14, // Measurements for Y-axis data
	HYM   = 0x15,
	HYH   = 0x16,
	HZL   = 0x17, // Measurements for Z-axis data
	HZM   = 0x18,
	HZH   = 0x19,
	TMPS  = 0x1A, // Measurement Temperature data
	ST2   = 0x1B, // Status 2

	SXL   = 0x20, // Self test data: X-axis data
	SXH   = 0x21,
	SYL   = 0x22, // Self test data: Y-axis data
	SYH   = 0x23,
	SZL   = 0x24, // Self test data: Z-axis data
	SZH   = 0x25,

	CNTL1 = 0x30, // Control 1
	CNTL2 = 0x31, // Control 2
	CNTL3 = 0x32, // Control 3
	CNTL4 = 0x33, // Control 4
};

// ST (for polling)
enum ST_BIT : uint8_t {
	DOR  = Bit1, // Data overrun
	DRDY = Bit0, // Data is ready
};

// ST1
enum class ST1_BIT : uint8_t {
	// FNUM[4:1] bits correspond to how many data sets are in the FIFO buffer, up to 0b1000 (8 sets)
	FNUM  = Bit4 | Bit3 | Bit2 | Bit1,
	DRDY  = Bit0, // Data is Ready
};

// ST2
enum class ST2_BIT : uint8_t {
	INV   = Bit1, // Data is invalid
	DOR   = Bit0, // Data overrun
};

// CNTL2
enum class CNTL2_BIT : uint8_t {
	TEM   = Bit6, // Temperature measurement, enable/disable
};

// CNTL3
enum class CNTL3_BIT : uint8_t {
	// MODE[4:0] bits
	MODE3_SET       = Bit2 | Bit1,          // “00110”: Continuous measurement mode 3 (50Hz)
	MODE_SELF_TEST  = Bit4,                 // "10000": Self-test mode
	MT 		= Bit6 | Bit5,		// Power Drive setting
	MODE3_CLR       = Bit4 | Bit3 | Bit0,   // bits to clear when setting MODE3
};

// CNTL4
enum class CNTL4_BIT : uint8_t {
	SRST = Bit0,
};

} // namespace AKM_AK09940A
