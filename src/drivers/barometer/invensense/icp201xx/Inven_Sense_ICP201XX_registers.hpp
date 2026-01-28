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
 * @file Inven_Sense_ICP201XX_registers.hpp
 *
 * icp201xx registers.
 *
 */

#pragma once

#include <cstdint>

namespace Inven_Sense_ICP201XX
{
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x63;
static constexpr uint8_t EXPECTED_DEVICE_ID = 0x63;

enum class Register : uint8_t {
	EMPTY = 0x00,
	TRIM1_MSB = 0x05,
	TRIM2_LSB = 0x06,
	TRIM2_MSB = 0x07,
	DEVICE_ID = 0x0C,
	OTP_MTP_OTP_CFG1 = 0xAC,
	OTP_MTP_MR_LSB = 0xAD,
	OTP_MTP_MR_MSB = 0xAE,
	OTP_MTP_MRA_LSB = 0xAF,
	OTP_MTP_MRA_MSB = 0xB0,
	OTP_MTP_MRB_LSB = 0xB1,
	OTP_MTP_MRB_MSB = 0xB2,
	OTP_MTP_OTP_ADDR = 0xB5,
	OTP_MTP_OTP_CMD = 0xB6,
	OTP_MTP_RD_DATA = 0xB8,
	OTP_MTP_OTP_STATUS = 0xB9,
	OTP_DEBUG2 = 0xBC,
	MASTER_LOCK = 0xBE,
	OTP_MTP_OTP_STATUS2 = 0xBF,
	MODE_SELECT = 0xC0,
	INTERRUPT_STATUS = 0xC1,
	INTERRUPT_MASK = 0xC2,
	FIFO_CONFIG = 0xC3,
	FIFO_FILL = 0xC4,
	SPI_MODE = 0xC5,
	PRESS_ABS_LSB = 0xC7,
	PRESS_ABS_MSB = 0xC8,
	PRESS_DELTA_LSB = 0xC9,
	PRESS_DELTA_MSB = 0xCA,
	DEVICE_STATUS = 0xCD,
	I3C_INFO = 0xCE,
	VERSION = 0xD3,
	FIFO_BASE = 0xFA
};
} // namespace Inven_Sense_ICP201XX
