/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file VTT_VCM5883_registers.hpp
 *
 * QST VCM5883 registers.
 *
 */

#pragma once

#include <cstdint>

namespace VTT_VCM5883
{

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b0001100; // default I2C address

static constexpr uint8_t Chip_ID = 0x82;

enum class Register : uint8_t {
	X_LSB      = 0x00, // Data Output X LSB Register XOUT[7:0]
	X_MSB      = 0x01, // Data Output X MSB Register XOUT[15:8]
	Y_LSB      = 0x02, // Data Output Y LSB Register YOUT[7:0]
	Y_MSB      = 0x03, // Data Output Y MSB Register YOUT[15:8]
	Z_LSB      = 0x04, // Data Output Z LSB Register ZOUT[7:0]
	Z_MSB      = 0x05, // Data Output Z MSB Register ZOUT[15:8]

	CNTL2      = 0x0a, // Control Register 2
	CNTL1      = 0x0b, // Control Register 1
	REG_ID     = 0x0c, // Chip ID

	CHIP_ID    = 0x0c,
};


// CNTL2
enum CNTL2_BIT : uint8_t {
	//
	// 0AH[7:4] Note : During initialization , register 0AH(bit7~bit4) must be written 0100b
	CTRL2_INIT   = Bit7,

	// ODR[3:2]
	ODR2         = Bit2,
	ODR_50HZ     = Bit3,        // 10

	// MODE[0]
	Mode0        = Bit0,        //
	Mode_Normal  = Bit0,        // 1
};

// CNTL1

enum CNTL1_BIT : uint8_t {
	SOFT_RST        = Bit7, // Soft reset, restore default value of all registers.
	SET_RESET       = Bit1 | Bit0,  // Set Reset (Cleared)
};

} // namespace VCM5883
