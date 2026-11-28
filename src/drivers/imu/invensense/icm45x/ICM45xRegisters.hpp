/****************************************************************************
 *
 *   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
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
 * @file ICM45xRegisters.hpp
 * @brief Shared Gen-4 IREG registers and per-chip BANK_0 maps for ICM-45686 and ICM-56686.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace icm45x_registers
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint8_t DIR_READ = Bit7;

static constexpr float TEMPERATURE_SENSITIVITY = 128.f; // LSB/degC
static constexpr float TEMPERATURE_OFFSET      = 25.f;  // degC

// Shared Gen-4 BANK_0 registers (identical addresses on ICM-45686 and ICM-56686).
namespace common
{
static constexpr uint8_t WHO_AM_I       = 0x72;
static constexpr uint8_t IREG_ADDR_15_8 = 0x7C;
static constexpr uint8_t IREG_ADDR_7_0  = 0x7D;
static constexpr uint8_t IREG_DATA      = 0x7E;
static constexpr uint8_t REG_MISC2      = 0x7F;

static constexpr uint8_t REG_MISC2_SOFT_RST  = Bit1;
static constexpr uint8_t REG_MISC2_IREG_DONE = Bit0;
static constexpr uint8_t PWR_MGMT0_LOW_NOISE = Bit3 | Bit2 | Bit1 | Bit0;
} // namespace common

namespace icm45686
{

static constexpr uint8_t WHOAMI            = 0xE9;
static constexpr size_t  FIFO_SIZE         = 8192;
static constexpr uint8_t FIFO_PACKET_BYTES = 16;

enum class BANK_0 : uint8_t {
	TEMP_DATA1_UI         = 0x0C,
	TEMP_DATA0_UI         = 0x0D,
	PWR_MGMT0             = 0x10,
	FIFO_COUNT_0          = 0x12,
	FIFO_COUNT_1          = 0x13,
	FIFO_DATA             = 0x14,
	INT1_CONFIG0          = 0x16,
	INT1_CONFIG2          = 0x18,
	INT1_STATUS0          = 0x19,
	ACCEL_CONFIG0         = 0x1B,
	GYRO_CONFIG0          = 0x1C,
	FIFO_CONFIG0          = 0x1D,
	FIFO_CONFIG1_0        = 0x1E,
	FIFO_CONFIG1_1        = 0x1F,
	FIFO_CONFIG2          = 0x20,
	FIFO_CONFIG3          = 0x21,
	FIFO_CONFIG4          = 0x22,
	RTC_CONFIG            = 0x26,
	IOC_PAD_SCENARIO_OVRD = 0x31,
	REG_MISC1             = 0x35,
	WHO_AM_I              = 0x72,
	IREG_ADDR_15_8        = 0x7C,
	IREG_ADDR_7_0         = 0x7D,
	IREG_DATA             = 0x7E,
	REG_MISC2             = 0x7F,
};

namespace IREG
{
static constexpr uint16_t SMC_CONTROL_0      = 0xA258; // ACCEL_LP_CLK_SEL
static constexpr uint16_t SREG_CTRL          = 0xA267; // SREG_DATA_ENDIAN_SEL
static constexpr uint16_t IPREG_SYS1_REG_166 = 0xA4A6; // GYRO_SRC_CTRL
static constexpr uint16_t IPREG_SYS2_REG_123 = 0xA57B; // ACCEL_SRC_CTRL
} // namespace IREG

enum REG_MISC1_BIT : uint8_t {
	OSC_ID_OVRD_RC_SET   = Bit1, // 0010: request internal relaxation oscillator for MCLK
	OSC_ID_OVRD_RC_CLEAR = Bit3 | Bit2 | Bit0,
	OSC_ID_OVRD_MASK     = Bit3 | Bit2 | Bit1 | Bit0,
};

enum SMC_CONTROL_0_BIT : uint8_t {
	ACCEL_LP_CLK_SEL = Bit4,
};

enum SREG_CTRL_BIT : uint8_t {
	SREG_DATA_ENDIAN_SEL = Bit1,
};

enum INT1_CONFIG0_BIT : uint8_t {
	INT1_STATUS_EN_FIFO_THS = Bit1,
};

enum INT1_CONFIG2_BIT : uint8_t {
	INT1_DRIVE    = Bit2, // 0: push-pull, 1: open-drain
	INT1_MODE     = Bit1, // 0: pulsed, 1: latched
	INT1_POLARITY = Bit0, // 0: active-low, 1: active-high
};

enum ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_UI_FS_SEL_32_G_SET   = 0,
	ACCEL_UI_FS_SEL_32_G_CLEAR = Bit6 | Bit5 | Bit4,
	ACCEL_ODR_6400_HZ_SET      = Bit1 | Bit0,
	ACCEL_ODR_6400_HZ_CLEAR    = Bit2,
};

enum GYRO_CONFIG0_BIT : uint8_t {
	GYRO_UI_FS_SEL_4000_DPS_SET   = 0,
	GYRO_UI_FS_SEL_4000_DPS_CLEAR = Bit7 | Bit6 | Bit5 | Bit4,
	GYRO_ODR_6400_HZ_SET          = Bit1 | Bit0,
	GYRO_ODR_6400_HZ_CLEAR        = Bit2,
};

enum FIFO_CONFIG0_BIT : uint8_t {
	FIFO_MODE_BYPASS_SET       = 0,
	FIFO_MODE_BYPASS_CLEAR     = Bit7 | Bit6,
	FIFO_MODE_STOP_ON_FULL_SET = Bit7,
	FIFO_MODE_STOP_ON_FULL_CLR = Bit6,
	FIFO_DEPTH_8K_SET          = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
	FIFO_DEPTH_8K_CLEAR        = Bit5,
};

enum FIFO_CONFIG2_BIT : uint8_t {
	FIFO_FLUSH             = Bit7,
	FIFO_WR_WM_EQ_OR_GT_TH = Bit3,
};

enum FIFO_CONFIG3_BIT : uint8_t {
	FIFO_ES1_EN   = Bit5,
	FIFO_ES0_EN   = Bit4,
	FIFO_HIRES_EN = Bit3,
	FIFO_GYRO_EN  = Bit2,
	FIFO_ACCEL_EN = Bit1,
	FIFO_IF_EN    = Bit0,
};

enum FIFO_CONFIG4_BIT : uint8_t {
	FIFO_COMP_EN       = Bit2,
	FIFO_TMST_FSYNC_EN = Bit1,
};

enum RTC_CONFIG_BIT : uint8_t {
	RTC_MODE = Bit5,
};

enum IOC_PAD_SCENARIO_OVRD_BIT : uint8_t {
	PADS_INT2_CFG_OVRD       = Bit2,
	PADS_INT2_CFG_OVRD_CLKIN = Bit1,
	PADS_INT2_CFG_OVRD_FSYNC = Bit0,
};

enum IPREG_SYS1_REG_166_BIT : uint8_t {
	GYRO_SRC_CTRL_INTERP_AAF_SET   = Bit6,
	GYRO_SRC_CTRL_INTERP_AAF_CLEAR = Bit5,
};

enum IPREG_SYS2_REG_123_BIT : uint8_t {
	ACCEL_SRC_CTRL_INTERP_AAF_SET   = Bit1,
	ACCEL_SRC_CTRL_INTERP_AAF_CLEAR = Bit0,
};

} // namespace icm45686

namespace icm56686
{

static constexpr uint8_t  WHOAMI            = 0x08;
static constexpr size_t   FIFO_SIZE         = 2048;
static constexpr uint8_t  FIFO_PACKET_BYTES = 20;
static constexpr uint16_t FIFO_MAX_FRAMES   = FIFO_SIZE / FIFO_PACKET_BYTES;

enum class BANK_0 : uint8_t {
	PWR_MGMT0         = 0x14,
	FIFO_COUNT_0      = 0x16,
	FIFO_COUNT_1      = 0x17,
	FIFO_DATA         = 0x18,
	INT1_CONFIG0      = 0x1A,
	INT1_CONFIG2      = 0x1C,
	INT1_STATUS0      = 0x1D,
	ACCEL_CONFIG0     = 0x1F,
	GYRO_CONFIG0      = 0x20,
	FIFO_CONFIG0      = 0x21,
	FIFO_CONFIG1_0    = 0x22,
	FIFO_CONFIG1_1    = 0x23,
	FIFO_CONFIG2      = 0x24,
	FIFO_CONFIG3      = 0x25,
	FIFO_CONFIG4      = 0x26,
	INTF_CONFIG1_OVRD = 0x31,
	DRIVE_CONFIG0     = 0x36,
	PWR_MGMT_AUX1     = 0x58,
	INT2_CONFIG0      = 0x5A,
	INT2_CONFIG2      = 0x5C,
	WHO_AM_I          = 0x72,
	IREG_ADDR_15_8    = 0x7C,
	IREG_ADDR_7_0     = 0x7D,
	IREG_DATA         = 0x7E,
	REG_MISC2         = 0x7F,
};

namespace IREG
{
static constexpr uint16_t SMC_CONTROL_0           = 0xA258;
static constexpr uint16_t SREG_CTRL               = 0xA260;
static constexpr uint16_t INT_PULSE_MIN_ON_INTF0  = 0xA262;
static constexpr uint16_t INT_PULSE_MIN_ON_INTF1  = 0xA263;
static constexpr uint16_t INT_PULSE_MIN_OFF_INTF0 = 0xA264;
static constexpr uint16_t INT_PULSE_MIN_OFF_INTF1 = 0xA265;
static constexpr uint16_t GYRO_SRC_CTRL           = 0xA49A;
static constexpr uint16_t GYRO_UI_LPF_CFG         = 0xA49E;
static constexpr uint16_t ACCEL_SRC_CTRL          = 0xA56D;
static constexpr uint16_t ACCEL_UI_LPF_CFG        = 0xA570;
} // namespace IREG

enum INT_CONFIG0_BIT : uint8_t {
	INT1_STATUS_EN_RESET_DONE = Bit7,
	STATUS_EN_FIFO_THS        = Bit1,
	STATUS_EN_FIFO_FULL       = Bit0,
};

enum INT_CONFIG2_BIT : uint8_t {
	ACTIVE_HIGH = Bit0,
	CONFIG_MASK = Bit2 | Bit1 | Bit0,
};

enum INT1_STATUS0_BIT : uint8_t {
	INT1_STATUS_RESET_DONE = Bit7,
};

enum ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_UI_FS_SEL_32_G = 0,
	ACCEL_ODR_6400_HZ    = Bit1 | Bit0,
	ACCEL_CONFIG0_MASK   = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

enum GYRO_CONFIG0_BIT : uint8_t {
	GYRO_UI_FS_SEL_4000_DPS = 0,
	GYRO_ODR_6400_HZ        = Bit1 | Bit0,
	GYRO_CONFIG0_MASK       = 0xFF,
};

enum FIFO_CONFIG0_BIT : uint8_t {
	FIFO_MODE_BYPASS       = 0,
	FIFO_MODE_STOP_ON_FULL = Bit7,
	FIFO_MODE_MASK         = Bit7 | Bit6,
	FIFO_DEPTH_2K          = Bit2 | Bit1 | Bit0,
	FIFO_DEPTH_MASK        = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

enum FIFO_CONFIG2_BIT : uint8_t {
	FIFO_FLUSH          = Bit7,
	FIFO_RESERVED_RESET = Bit5,
	FIFO_WR_WM_GT_TH    = Bit3,
	FIFO_CONFIG2_MASK   = Bit6 | Bit5 | Bit4 | Bit3,
};

enum FIFO_CONFIG3_BIT : uint8_t {
	FIFO_HIRES_EN     = Bit3,
	FIFO_GYRO_EN      = Bit2,
	FIFO_ACCEL_EN     = Bit1,
	FIFO_IF_EN        = Bit0,
	FIFO_CONFIG3_MASK = Bit3 | Bit2 | Bit1 | Bit0,
};

enum FIFO_CONFIG4_BIT : uint8_t {
	FIFO_TMST_FSYNC_EN = Bit0,
	FIFO_CONFIG4_MASK  = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

enum INTF0_PULSE_BIT : uint8_t {
	DURATION_8_US = Bit0,
	DURATION_MASK = Bit2 | Bit1 | Bit0,
};

enum SREG_CTRL_BIT : uint8_t {
	SREG_SIFS_20BITS_EN  = Bit3,
	SREG_DATA_ENDIAN_BIG = Bit1,
};

enum SMC_CONTROL_0_BIT : uint8_t {
	TMST_EN = Bit0,
};

enum GYRO_SRC_CTRL_BIT : uint8_t {
	GYRO_SRC_CTRL_MASK                = Bit3 | Bit2,
	GYRO_SRC_CTRL_SRC_ON_PREFILTER_ON = Bit3,
};

enum GYRO_UI_LPF_CFG_BIT : uint8_t {
	GYRO_UI_LPFBW_ODR_DIV_2 = 0,
	GYRO_UI_LPFBW_MASK      = Bit6 | Bit5 | Bit4,
};

enum ACCEL_SRC_CTRL_BIT : uint8_t {
	ACCEL_SRC_CTRL_MASK                = Bit1 | Bit0,
	ACCEL_SRC_CTRL_SRC_ON_PREFILTER_ON = Bit1,
};

enum ACCEL_UI_LPF_CFG_BIT : uint8_t {
	ACCEL_UI_LPFBW_ODR_DIV_2 = 0,
	ACCEL_UI_LPFBW_MASK      = Bit2 | Bit1 | Bit0,
};

} // namespace icm56686

} // namespace icm45x_registers
