/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ST_L3GD20H_registers.hpp
 *
 * ST L3GD20H registers.
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

namespace ST_L3GD20H
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface

static constexpr uint8_t DIR_READ = Bit7;
static constexpr uint8_t AUTO_INCREMENT = Bit6;

static constexpr uint8_t WHOAMI = 0b11010111; // Who I am ID

static constexpr uint32_t ODR  = 758; // 757.6 Hz Angular rate output data rate

enum class Register : uint8_t {
	WHO_AM_I      = 0x0F,

	CTRL1         = 0x20,
	CTRL2         = 0x21,
	CTRL3         = 0x22,
	CTRL4         = 0x23,
	CTRL5         = 0x24,

	OUT_TEMP      = 0x26,
	STATUS        = 0x27,
	OUT_X_L       = 0x28,
	OUT_X_H       = 0x29,
	OUT_Y_L       = 0x2A,
	OUT_Y_H       = 0x2B,
	OUT_Z_L       = 0x2C,
	OUT_Z_H       = 0x2D,
	FIFO_CTRL     = 0x2E,
	FIFO_SRC      = 0x2F,

	LOW_ODR       = 0x39,
};

// CTRL1
enum CTRL1_BIT : uint8_t {
	ODR_800HZ_CUTOFF_100HZ = Bit7 | Bit6 | Bit5 | Bit4,
	PD                     = Bit3,                       // Power-down mode enable
};

// CTRL3
enum CTRL3_BIT : uint8_t {
	H_Lactive = Bit5, // Interrupt active configuration on INT. Default value 0. (0: high; 1:low)

	INT2_DRDY = Bit3, // Date Ready on DRDY/INT2 pin.
	INT2_FTH  = Bit2, // FIFO Threshold interrupt on DRDY/INT2 pin.
	INT2_ORun = Bit1, // FIFO Overrun interrupt on DRDY/INT2 pin.
};

// CTRL4
enum CTRL4_BIT : uint8_t {
	BDU        = Bit7,        // Block data update
	FS_2000DPS = Bit5 | Bit4, // Full scale selection 2000 dps
};

// CTRL5
enum CTRL5_BIT : uint8_t {
	BOOT    = Bit7,
	FIFO_EN = Bit6,
};

// FIFO_CTRL
enum FIFO_CTRL_BIT : uint8_t {
	// FM2-FM0
	Bypass_mode         = Bit7 | Bit6 | Bit5, // 000 Bypass mode
	FIFO_mode           = Bit5,               // 001 FIFO mode
	Dynamic_stream_mode = Bit7 | Bit6,        // 110 Dynamic stream mode

	// FTH4-FTH0: FIFO threshold setting.
	FTH40               = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

// FIFO_SRC
enum FIFO_SRC_BIT : uint8_t {
	FTH   = Bit7,                             // FIFO threshold status.
	OVRN  = Bit6,                             // Overrun bit status.
	EMPTY = Bit5,                             // FIFO empty bit.
	FSS   = Bit4 | Bit3 | Bit2 | Bit1 | Bit0, // FSS4-FSS0: FIFO stored data level
};

// LOW_ODR
enum LOW_ODR_BIT : uint8_t {
	DRDY_HL = Bit5, // DRDY/INT2 pin active level. 1 = DRDY active low

	I2C_dis = Bit3, // 1 = SPI only
	SW_RES  = Bit2, // 1 = Reset Device
};

namespace FIFO
{
//  L3GD20H embeds 32 slots of 16-bit data FIFO for each of the three output channels
static constexpr size_t SIZE = 32 * 2 * 3;

struct DATA {
	uint8_t OUT_X_L;
	uint8_t OUT_X_H;
	uint8_t OUT_Y_L;
	uint8_t OUT_Y_H;
	uint8_t OUT_Z_L;
	uint8_t OUT_Z_H;
};

}

} // namespace ST_L3GD20H
