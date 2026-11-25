/****************************************************************************
 *
 *   Copyright (c) 2020-2026 PX4 Development Team. All rights reserved.
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
 * @file ICM42xRegisters.hpp
 * @brief Shared banked ICM-4268x / IIM-4265x registers; model-specific behavior lives in profiles.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace icm42x_registers
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr float TEMPERATURE_SENSITIVITY = 132.48f; // LSB/C
static constexpr float TEMPERATURE_OFFSET      = 25.f;    // C

namespace Register
{

enum class BANK_0 : uint8_t {
	DEVICE_CONFIG      = 0x11,
	INT_CONFIG         = 0x14,
	FIFO_CONFIG        = 0x16,
	TEMP_DATA1         = 0x1D,
	TEMP_DATA0         = 0x1E,
	INT_STATUS         = 0x2D,
	FIFO_COUNTH        = 0x2E,
	FIFO_COUNTL        = 0x2F,
	FIFO_DATA          = 0x30,
	SIGNAL_PATH_RESET  = 0x4B,
	INTF_CONFIG0       = 0x4C,
	INTF_CONFIG1       = 0x4D,
	PWR_MGMT0          = 0x4E,
	GYRO_CONFIG0       = 0x4F,
	ACCEL_CONFIG0      = 0x50,
	GYRO_CONFIG1       = 0x51,
	GYRO_ACCEL_CONFIG0 = 0x52,
	ACCEL_CONFIG1      = 0x53,
	TMST_CONFIG        = 0x54,
	FIFO_CONFIG1       = 0x5F,
	FIFO_CONFIG2       = 0x60,
	FIFO_CONFIG3       = 0x61,
	INT_CONFIG0        = 0x63,
	INT_CONFIG1        = 0x64,
	INT_SOURCE0        = 0x65,
	SELF_TEST_CONFIG   = 0x70,
	WHO_AM_I           = 0x75,
	REG_BANK_SEL       = 0x76,
};

enum class BANK_1 : uint8_t {
	GYRO_CONFIG_STATIC2 = 0x0B,
	GYRO_CONFIG_STATIC3 = 0x0C,
	GYRO_CONFIG_STATIC4 = 0x0D,
	GYRO_CONFIG_STATIC5 = 0x0E,
	INTF_CONFIG5        = 0x7B,
};

enum class BANK_2 : uint8_t {
	ACCEL_CONFIG_STATIC2 = 0x03,
	ACCEL_CONFIG_STATIC3 = 0x04,
	ACCEL_CONFIG_STATIC4 = 0x05,
	AUX1_CONFIG1         = 0x44,
	AUX1_CONFIG2         = 0x45,
	AUX1_SPI_REG1        = 0x71,
	AUX1_SPI_REG2        = 0x72,
	AUX1_SPI_REG3        = 0x73,
};

} // namespace Register

enum class DEVICE_CONFIG_BIT : uint8_t {
	SOFT_RESET_CONFIG = Bit0,
};

enum class INT_CONFIG_BIT : uint8_t {
	INT1_MODE          = Bit2,
	INT1_DRIVE_CIRCUIT = Bit1,
	INT1_POLARITY      = Bit0,
};

enum class FIFO_CONFIG_BIT : uint8_t {
	FIFO_MODE_STOP_ON_FULL = Bit7 | Bit6,
};

enum class INT_STATUS_BIT : uint8_t {
	RESET_DONE_INT = Bit4,
	DATA_RDY_INT   = Bit3,
	FIFO_THS_INT   = Bit2,
	FIFO_FULL_INT  = Bit1,
};

enum class SIGNAL_PATH_RESET_BIT : uint8_t {
	ABORT_AND_RESET = Bit3,
	FIFO_FLUSH      = Bit1,
};

enum class INTF_CONFIG0_BIT : uint8_t {
	FIFO_HOLD_LAST_DATA_EN  = Bit7,
	FIFO_COUNT_REC          = Bit6,
	FIFO_COUNT_ENDIAN       = Bit5,
	SENSOR_DATA_ENDIAN      = Bit4,
	UI_SIFS_CFG_DISABLE_I2C = Bit1 | Bit0,
};

enum class INTF_CONFIG1_BIT : uint8_t {
	AFSR_CLEAR   = Bit7,
	AFSR_SET     = Bit6,
	RTC_MODE     = Bit2,
	CLKSEL_CLEAR = Bit1,
	CLKSEL       = Bit0,
};

enum class PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2,
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0,
};

enum class GYRO_CONFIG0_BIT : uint8_t {
	GYRO_FS_SEL_MAX     = 0,
	GYRO_ODR_8KHZ_SET   = Bit1 | Bit0,
	GYRO_ODR_8KHZ_CLEAR = Bit3 | Bit2,
};

enum class ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_FS_SEL_MAX     = 0,
	ACCEL_ODR_8KHZ_SET   = Bit1 | Bit0,
	ACCEL_ODR_8KHZ_CLEAR = Bit3 | Bit2,
};

enum class GYRO_CONFIG1_BIT : uint8_t {
	GYRO_UI_FILT_ORD = Bit3 | Bit2,
};

enum class GYRO_ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_UI_FILT_BW = Bit7 | Bit6 | Bit5 | Bit4,
	GYRO_UI_FILT_BW  = Bit3 | Bit2 | Bit1 | Bit0,
};

enum class ACCEL_CONFIG1_BIT : uint8_t {
	ACCEL_UI_FILT_ORD = Bit4 | Bit3,
};

enum class TMST_CONFIG_BIT : uint8_t {
	TMST_TO_REGS_EN = Bit4,
	TMST_RES        = Bit3,
	TMST_DELTA_EN   = Bit2,
	TMST_FSYNC_EN   = Bit1,
	TMST_EN         = Bit0,
};

enum class FIFO_CONFIG1_BIT : uint8_t {
	FIFO_RESUME_PARTIAL_RD = Bit6,
	FIFO_WM_GT_TH          = Bit5,
	FIFO_HIRES_EN          = Bit4,
	FIFO_TMST_FSYNC_EN     = Bit3,
	FIFO_TEMP_EN           = Bit2,
	FIFO_GYRO_EN           = Bit1,
	FIFO_ACCEL_EN          = Bit0,
};

enum class INT_CONFIG0_BIT : uint8_t {
	CLEAR_ON_FIFO_READ = Bit3,
};

enum class INT_CONFIG1_BIT : uint8_t {
	INT_TPULSE_DURATION   = Bit6,
	INT_TDEASSERT_DISABLE = Bit5,
	INT_ASYNC_RESET       = Bit4,
};

enum class INT_SOURCE0_BIT : uint8_t {
	FIFO_THS_INT1_EN = Bit2,
};

enum class GYRO_CONFIG_STATIC2_BIT : uint8_t {
	GYRO_AAF_DIS = Bit1,
	GYRO_NF_DIS  = Bit0,
};

enum class GYRO_CONFIG_STATIC3_BIT : uint8_t {
	GYRO_AAF_DELT_585HZ_SET   = Bit3 | Bit2 | Bit0,
	GYRO_AAF_DELT_585HZ_CLEAR = Bit5 | Bit4 | Bit1,
};

enum class GYRO_CONFIG_STATIC4_BIT : uint8_t {
	GYRO_AAF_DELTSQR_LSB_585HZ_SET   = Bit7 | Bit5 | Bit3 | Bit1,
	GYRO_AAF_DELTSQR_LSB_585HZ_CLEAR = Bit6 | Bit4 | Bit2 | Bit0,
};

enum class GYRO_CONFIG_STATIC5_BIT : uint8_t {
	GYRO_AAF_BITSHIFT_585HZ_SET      = Bit7,
	GYRO_AAF_BITSHIFT_585HZ_CLEAR    = Bit6 | Bit5 | Bit4,
	GYRO_AAF_DELTSQR_MSB_585HZ_SET   = 0,
	GYRO_AAF_DELTSQR_MSB_585HZ_CLEAR = Bit3 | Bit2 | Bit1 | Bit0,
};

enum class INTF_CONFIG5_BIT : uint8_t {
	PIN9_FUNCTION_CLKIN_SET   = Bit2,
	PIN9_FUNCTION_CLKIN_CLEAR = Bit1,
};

enum class ACCEL_CONFIG_STATIC2_BIT : uint8_t {
	ACCEL_AAF_DELT_585HZ_SET   = Bit4 | Bit3 | Bit1,
	ACCEL_AAF_DELT_585HZ_CLEAR = Bit6 | Bit5 | Bit2,
	ACCEL_AAF_DIS              = Bit0,
};

enum class ACCEL_CONFIG_STATIC3_BIT : uint8_t {
	ACCEL_AAF_DELTSQR_LSB_585HZ_SET   = Bit7 | Bit5 | Bit3 | Bit1,
	ACCEL_AAF_DELTSQR_LSB_585HZ_CLEAR = Bit6 | Bit4 | Bit2 | Bit0,
};

enum class ACCEL_CONFIG_STATIC4_BIT : uint8_t {
	ACCEL_AAF_BITSHIFT_585HZ_SET   = Bit7,
	ACCEL_AAF_BITSHIFT_585HZ_CLEAR = Bit6 | Bit5 | Bit4,
	ACCEL_AAF_DELTSQR_MSB_SET      = 0,
	ACCEL_AAF_DELTSQR_MSB_CLEAR    = Bit3 | Bit2 | Bit1 | Bit0,
};

enum class AUX1_CONFIG1_BIT : uint8_t {
	AUX1_ACCEL_LP_CLK_SEL = Bit5,
	GYRO_AUX1_EN          = Bit1,
	ACCEL_AUX1_EN         = Bit0,
};

enum class AUX1_CONFIG2_BIT : uint8_t {
	GYRO_AUX1_HPF_DIS = Bit6,
};

enum class AUX1_SPI_REG1_BIT : uint8_t {
	AUX1_SPI_REG1_CLEAR = Bit1,
	AUX1_SPI_REG1_SET   = Bit0,
};

enum class AUX1_SPI_REG2_BIT : uint8_t {
	AUX1_SPI_REG2_CLEAR = Bit1,
	AUX1_SPI_REG2_SET   = Bit0,
};

enum class AUX1_SPI_REG3_BIT : uint8_t {
	AUX1_SPI_REG3_CLEAR = Bit1,
	AUX1_SPI_REG3_SET   = Bit0,
};

namespace FIFO
{
static constexpr size_t SIZE                 = 2048;
static constexpr size_t STANDARD_PACKET_SIZE = 16;
static constexpr size_t PACKET_SIZE          = 20;
} // namespace FIFO

} // namespace icm42x_registers
