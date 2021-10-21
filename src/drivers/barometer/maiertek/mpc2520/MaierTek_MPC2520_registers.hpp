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
 * @file MaierTek_MPC2520_registers.hpp
 *
 * MaierTek MPC2520 registers.
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

namespace MaierTek_MPC2520
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x76;

static constexpr uint8_t Product_ID = 0x0;
static constexpr uint8_t Revision_ID = 0x0;

enum class Register : uint8_t {
	PSR_B2     = 0x00, // PSR[23:16] (r)
	PSR_B1     = 0x01, // PSR[15:8]  (r)
	PSR_B0     = 0x02, // PSR[7:0]   (r)
	TMP_B2     = 0x03, // TMP[23:16] (r)
	TMP_B1     = 0x04, // TMP[15:8]  (r)
	TMP_B0     = 0x05, // TMP[7:0]   (r)
	PRS_CFG    = 0x06,
	TMP_CFG    = 0x07,
	MEAS_CFG   = 0x08,
	CFG_REG    = 0x09,

	RESET      = 0x0C,
	ID         = 0x0D, // PROD_ID [3:0] (r) REV_ID [3:0] (r)

	COEF       = 0x10,
};

// PRS_CFG
enum PRS_CFG_BIT : uint8_t {
	// PM_RATE[6:4]
	PM_RATE_32_SET   = Bit6 | Bit4, // 101 - 32 measurements pr. sec.
	PM_RATE_32_CLEAR = Bit5,

	// PM_PRC[3:0]
	PM_PRC_8_SET     = Bit1 | Bit0, // 0011 - 8 times.
	PM_PRC_8_CLEAR   = Bit3 | Bit2,
};

// TMP_CFG
enum TMP_CFG_BIT : uint8_t {
	TMP_EXT           = Bit7,

	// TMP_RATE[6:4]
	TMP_RATE_32_SET   = Bit6 | Bit4, // 101 - 32 measurements pr. sec.
	TMP_RATE_32_CLEAR = Bit5,

	// PM_PRC[2:0]
	TMP_PRC_8_SET     = Bit1 | Bit0, // 011 - 8 times.
	TMP_PRC_8_CLEAR   = Bit2,
};

// MEAS_CFG
enum MEAS_CFG_BIT : uint8_t {
	COEF_RDY = Bit7,
	SENSOR_RDY = Bit6,
	TMP_RDY    = Bit5,
	PRS_RDY    = Bit4,

	MEAS_CTRL_CONT_PRES_TEMP = Bit2 | Bit1 | Bit0, // 111 - Continuous pressure and temperature measurement
};

// RESET
enum RESET_BIT : uint8_t {
	SOFT_RST = Bit3 | Bit0, // Write '1001' to generate a soft reset.
};

// CFG_REG
enum CFG_REG_BIT : uint8_t {
	T_SHIFT = Bit3, // Temperature result bit-shift, Note: Must be set to '1' when the oversampling rate is >8 times.
	P_SHIFT = Bit2, // Pressure result bit-shift, Note: Must be set to '1' when the oversampling rate is >8 times.
};

} // namespace MaierTek_MPC2520
