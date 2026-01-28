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
 * @file QST_QMC5883P_registers.hpp
 *
 * QST QMC5883P registers.
 *
 * @author Amovlab Lv Guofei <service@amovauto.com>
 *
 */
#pragma once

#include <cstdint>

namespace QST_QMC5883P
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

// default value
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b0101100; // default I2C address

static constexpr uint8_t Chip_ID = 0x80;

//static constexpr uint8_t XYZ_SIGN_CONFIG = 0x29; // XYZ symbol configuration value
enum class Register : uint8_t {
//  Register          addr
	X_LSB           = 0x01, // Data Output X LSB Register XOUT[7:0]
	X_MSB           = 0x02, // Data Output X MSB Register XOUT[15:8]
	Y_LSB           = 0x03, // Data Output Y LSB Register YOUT[7:0]
	Y_MSB           = 0x04, // Data Output Y MSB Register YOUT[15:8]
	Z_LSB           = 0x05, // Data Output Z LSB Register ZOUT[7:0]
	Z_MSB           = 0x06, // Data Output Z MSB Register ZOUT[15:8]
	N_LSB           = 0X07, // No Use
	N_MSB           = 0X08, // No Use
	STATUS          = 0x09, // Status Register 1

	CNTL1           = 0x0A, // Control Register 1
	CNTL2           = 0x0B, // Control Register 2

	CHIP_ID         = 0x00,
};

// STATUS
enum STATUS_BIT : uint8_t {
	DRDY            = Bit0, // 0: no new data, 1: new data is ready
	OVL             = Bit1, // 0: no data overflow occurs, 1: data overflow occurs
};

// CNTL1
enum CNTL1_BIT : uint8_t {
	// OSR2[7:6]
	OSR2_8          =  Bit7 | Bit6, // 00
	// OSR1[5:4]
	OSR1_8          =  Bit5 | Bit4, // 11
	// ODR[3:2]
	ODR_50HZ        =  Bit2,        // 01
	// MODE[1:0]
	MODE_CONTINUOUS = Bit1 | Bit0,  // 11
};

// CNTL2
enum CNTL2_BIT : uint8_t {
	// RNG[3:2]
	RNG_2G          = Bit3 | Bit2,  // 11

	// SELF_TEST[6]
	SELF_TEST       = Bit6, //1: self_test enable, auto clear after the data is updated

	// SOFT_RST[7]
	SOFT_RST        = 0, //1: Soft reset, restore default value of all registers, 0: no reset
};

} // namespace QMC5883P
