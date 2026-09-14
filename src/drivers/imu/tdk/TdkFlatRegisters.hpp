/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cstddef>
#include <cstdint>

namespace tdk_direct_registers
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint8_t DIR_READ                 = 0x80;
static constexpr size_t  FIFO_SIZE_MAX            = 1024;
static constexpr size_t  FIFO_PACKET_SIZE_CLASSIC = 12;
static constexpr size_t FIFO_PACKET_SIZE_ICM20602 = 14;
static constexpr size_t FIFO_PACKET_SIZE_MAX      = FIFO_PACKET_SIZE_ICM20602;

enum class Register : uint8_t {
	XG_OFFS_TC_H       = 0x04,
	XG_OFFS_TC_L       = 0x05,
	YG_OFFS_TC_H       = 0x07,
	YG_OFFS_TC_L       = 0x08,
	ZG_OFFS_TC_H       = 0x0A,
	ZG_OFFS_TC_L       = 0x0B,
	SMPLRT_DIV         = 0x19,
	CONFIG             = 0x1A,
	GYRO_CONFIG        = 0x1B,
	ACCEL_CONFIG       = 0x1C,
	ACCEL_CONFIG2      = 0x1D,
	FIFO_EN            = 0x23,
	I2C_MST_CTRL       = 0x24,
	I2C_SLV0_ADDR      = 0x25,
	I2C_SLV0_REG       = 0x26,
	I2C_SLV0_CTRL      = 0x27,
	I2C_SLV4_CTRL      = 0x34,
	INT_PIN_CFG        = 0x37,
	INT_ENABLE         = 0x38,
	TEMP_OUT_H         = 0x41,
	TEMP_OUT_L         = 0x42,
	EXT_SENS_DATA_00   = 0x49,
	EXT_SENS_DATA_23   = 0x60,
	FIFO_WM_TH1        = 0x60,
	FIFO_WM_TH2        = 0x61,
	I2C_SLV0_DO        = 0x63,
	I2C_MST_DELAY_CTRL = 0x67,
	SIGNAL_PATH_RESET  = 0x68,
	USER_CTRL          = 0x6A,
	PWR_MGMT_1         = 0x6B,
	I2C_IF             = 0x70,
	FIFO_COUNTH        = 0x72,
	FIFO_COUNTL        = 0x73,
	FIFO_R_W           = 0x74,
	WHO_AM_I           = 0x75,
	XA_OFFSET_H        = 0x77,
	XA_OFFSET_L        = 0x78,
	YA_OFFSET_H        = 0x7A,
	YA_OFFSET_L        = 0x7B,
	ZA_OFFSET_H        = 0x7D,
	ZA_OFFSET_L        = 0x7E,
};

enum class CONFIG_BIT : uint8_t {
	FIFO_MODE = Bit6,
	DLPF_CFG_BYPASS_DLPF_8KHZ = 7,
};

enum class GYRO_CONFIG_BIT : uint8_t {
	FS_SEL_250_DPS  = 0,
	FS_SEL_500_DPS  = Bit3,
	FS_SEL_1000_DPS = Bit4,
	FS_SEL_2000_DPS = Bit4 | Bit3,
	FCHOICE_B_8KHZ_BYPASS_DLPF = Bit1 | Bit0,
};

enum class ACCEL_CONFIG_BIT : uint8_t {
	ACCEL_FS_SEL_2G  = 0,
	ACCEL_FS_SEL_4G  = Bit3,
	ACCEL_FS_SEL_8G  = Bit4,
	ACCEL_FS_SEL_16G = Bit4 | Bit3,
};

enum class ACCEL_CONFIG2_BIT : uint8_t {
	FIFO_SIZE = Bit7 | Bit6, // IAM20680HP only; reserved on ICM20689.
	ACCEL_FCHOICE_B = Bit3,
};

enum class FIFO_EN_BIT : uint8_t {
	TEMP_FIFO_EN  = Bit7,
	XG_FIFO_EN    = Bit6,
	YG_FIFO_EN    = Bit5,
	ZG_FIFO_EN    = Bit4,
	GYRO_FIFO_EN  = Bit4,
	ACCEL_FIFO_EN = Bit3,
	SLAVE_FIFO_EN = Bit2 | Bit1 | Bit0, // MPU9250 auxiliary I2C slave FIFO sources.
};

enum class INT_PIN_CFG_BIT : uint8_t {
	INT_LEVEL    = Bit7,
	LATCH_INT_EN = Bit5,
	INT_RD_CLEAR = Bit4,
	BYPASS_EN    = Bit1,
};

enum class INT_ENABLE_BIT : uint8_t {
	DATA_RDY_INT_EN = Bit0,
};

enum class SIGNAL_PATH_RESET_BIT : uint8_t {
	GYRO_RESET  = Bit2,
	ACCEL_RESET = Bit1,
	TEMP_RESET  = Bit0,
};

enum class USER_CTRL_BIT : uint8_t {
	FIFO_EN      = Bit6,
	I2C_MST_EN   = Bit5,
	I2C_IF_DIS   = Bit4,
	FIFO_RST     = Bit2,
	I2C_MST_RST  = Bit1,
	SIG_COND_RST = Bit0,
};

enum class PWR_MGMT_1_BIT : uint8_t {
	DEVICE_RESET = Bit7,
	SLEEP        = Bit6,
	CLKSEL_0     = Bit0,
};

enum class I2C_MST_CTRL_BIT : uint8_t {
	I2C_MST_P_NSR = Bit4,
	I2C_MST_CLK_400_KHZ = 13,
};

enum class I2C_SLV0_ADDR_BIT : uint8_t {
	I2C_SLV0_RNW = Bit7,
};

enum class I2C_SLV0_CTRL_BIT : uint8_t {
	I2C_SLV0_EN      = Bit7,
	I2C_SLV0_BYTE_SW = Bit6,
	I2C_SLV0_REG_DIS = Bit5,
};

} // namespace tdk_direct_registers
