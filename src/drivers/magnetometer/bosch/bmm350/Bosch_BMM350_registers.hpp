/****************************************************************************
 *
 *   Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
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
 * @file Bosch_BMM350_registers.hpp
 *
 * Bosch BMM350 registers.
 *
 */

#pragma once

#include <cstdint>

namespace Bosch_BMM350
{
#define ENABLE_X_AXIS(axis_data) (axis_data = (axis_data & ~(0x01)) | (1 & 0x01))
#define ENABLE_Y_AXIS(axis_data) (axis_data = ((axis_data & ~(0x02)) | ((1 << 0x1) & 0x02)))
#define ENABLE_Z_AXIS(axis_data) (axis_data = ((axis_data & ~(0x04)) | ((1 << 0x2) & 0x04)))

static constexpr uint32_t I2C_SPEED = 400 * 1000;
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x14;
static constexpr uint8_t chip_identification_number = 0x33;

enum class Register : uint8_t {
	CHIP_ID = 0x00,

	PAD_CTRL = 0x03,
	PMU_CMD_AGGR_SET = 0x04,
	PMU_CMD_AXIS_EN = 0x05,
	PMU_CMD = 0x06,
	PMU_STATUS_0 = 0x07,
	PMU_STATUS_1 = 0x08,

	I2C_WDT_SET = 0x0a,

	DATAX_XLSB = 0x31,

	OTP_CMD = 0x50,
	OTP_DATA_MSB = 0x52,
	OTP_DATA_LSB = 0x53,
	OTP_STATUS = 0x55,

	TMR_SELF_TEST_USER = 0x60,
	CMD = 0x7e
};

enum CONTROL_CMD : uint8_t {
	SOFT_RESET = 0xb6,
	EN_XYZ = 0x7
};

enum PMU_CONTROL_CMD : uint8_t {
	PMU_CMD_SUSPEND = 0x00,
	PMU_CMD_NM = 0x01,
	PMU_CMD_UPDATE_OAE = 0x02,
	PMU_CMD_FM = 0x03,
	PMU_CMD_FAST_FM = 0x04,
	PMU_CMD_FGR = 0x05,
	PMU_CMD_FAST_FGR = 0x06,
	PMU_CMD_BR = 0x07,
	PMU_CMD_BR_FAST = 0x08,
	PMU_CMD_NM_TC = 0x09
};

static inline uint8_t PMU_CMD_STATUS_0_RES(uint8_t val)
{
	return (val >> 5) & 0x7;
};

enum PMU_STATUS_0_BIT : uint8_t {
	PMU_BUSY = (1 << 0),
	ODR_OVWR = (1 << 1),
	AVG_OVWR = (1 << 2),
	PWR_NORMAL = (1 << 3),
	ILLEGAL_CMD = (1 << 4)
};

enum PMU_STATUS_VALUE {
	PMU_STATUS_SUS = 0x0,
	PMU_STATUS_NM = 0x1,
	PMU_STATUS_UPDATE_OAE = 0x2,
	PMU_STATUS_FM = 0x3,
	PMU_STATUS_FM_FAST = 0x4,
	PMU_STATUS_FGR = 0x5,
	PMU_STATUS_FGR_FAST = 0x6,
	PMU_STATUS_BR = 0x7,
	PMU_STATUS_BR_FAST = 0x7,
};

enum ODR_CONTROL_CMD : uint8_t {
	ODR_400HZ = 0x2,
	ODR_200HZ = 0x3,
	ODR_100HZ = 0x4,
	ODR_50HZ = 0x5,
	ODR_25HZ = 0x6,
	ODR_12_5HZ = 0x7,
	ODR_6_25HZ = 0x8,
	ODR_3_125HZ = 0x9,
	ODR_1_5625HZ = 0xa
};

enum AVG_CONTROL_CMD : uint8_t {
	AVG_NO_AVG = 0x0,
	AVG_2 = 0x1,
	AVG_4 = 0x2,
	AVG_8 = 0x3
};

enum ENABLE_AXIS_BIT : uint8_t {
	EN_X = (1 << 0),
	EN_Y = (1 << 1),
	EN_Z = (1 << 2)
};

enum OTP_CONTROL_CMD : uint8_t {
	PWR_OFF_OTP = 0x80,
	OTP_DIR_READ = 0x20,
	OTP_WORD_MSK = 0x1F,
};

enum SELF_TEST_CMD : uint8_t {
	SELF_TEST_DISABLE = 0x00,
	SELF_TEST_POS_X = 0x0d,
	SELF_TEST_NEG_X = 0x0b,
	SELF_TEST_POS_Y = 0x15,
	SELF_TEST_NEG_Y = 0x13,
};
} // namespace Bosch_BMM350
