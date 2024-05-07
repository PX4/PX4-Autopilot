/****************************************************************************
 *
 *   Copyright (c) Technology Innovation Institute. All rights reserved.
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
 * @file BMM350_registers.hpp
 *
 * Bosch BMM350 registers.
 *
 */

#pragma once

#include <cstdint>

namespace Bosch_BMM350
{
static constexpr uint32_t I2C_SPEED          = 400000;
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x14;

static constexpr uint8_t DEFAULT_ID          = 0x33;

static constexpr int16_t OVERFLOW_XYAXES     = -4096;
static constexpr int16_t OVERFLOW_ZAXIS      = -16384;

enum class Register : uint8_t {
	CHIP_ID           = 0x00,
	ERR_REG           = 0x02,
	PAD_CTRL          = 0x03,
	PMU_CMD_AGGR_SET  = 0x04,
	PMU_CMD_AXIS_EN   = 0x05,
	PMU_CMD           = 0x06,
	PMU_CMD_STATUS_0  = 0x07,
	PMU_CMD_STATUS_1  = 0x08,
	I3C_ERR           = 0x09,
	I2C_WDT_SET       = 0x0A,
	INT_CTRL          = 0x2E,
	INT_CTRL_IBI      = 0x2F,
	INT_STATUS        = 0x30,
	MAG_X_XLSB        = 0x31,
	MAG_X_LSB         = 0x32,
	MAG_X_MSB         = 0x33,
	MAG_Y_XLSB        = 0x34,
	MAG_Y_LSB         = 0x35,
	MAG_Y_MSB         = 0x36,
	MAG_Z_XLSB        = 0x37,
	MAG_Z_LSB         = 0x38,
	MAG_Z_MSB         = 0x39,
	TEMP_XLSB         = 0x3A,
	TEMP_LSB          = 0x3B,
	TEMP_MSB          = 0x3C,
	SENSORTIME_XLSB   = 0x3D,
	SENSORTIME_LSB    = 0x3E,
	SENSORTIME_MSB    = 0x3F,
	OTP_CMD_REG       = 0x50,
	OTP_DATA_MSB_REG  = 0x52,
	OTP_DATA_LSB_REG  = 0x53,
	OTP_STATUS_REG    = 0x55,
	TMR_SELFTEST_USER = 0x60,
	CTRL_USER         = 0x61,
	CMD               = 0x7E
};

// Reports Sensor Error Flag. Will be cleared on read. If the user writes a 1 into any status bit, this will also clear that bit.
enum ERR_REG_BIT : uint8_t {
	pmu_cmd_error = (1 << 0),           // A new PMU_CMD is issued when previous command has not been finished
};

// Configure pad behavior
enum PAD_CTRL_DRV : uint8_t {
	DRV_0 = 0,                          // Drive: weakest
	DRV_2,
	DRV_3,
	DRV_4,
	DRV_5,
	DRV_6,
	DRV_7,                              // Drive: strongest
};

// Configuration of the ODR and AVG
enum PMU_CMD_AGGR_ODR : uint8_t {
	ODR_400HZ    = 0x02,                // 400 Hz odr
	ODR_200HZ    = 0x03,                // 200 Hz odr
	ODR_100HZ    = 0x04,                // 100 Hz odr
	ODR_50HZ     = 0x05,                // 50 Hz odr
	ODR_25HZ     = 0x06,                // 25 Hz odr
	ODR_12_5HZ   = 0x07,                // 12.5 Hz odr
	ODR_6_25HZ   = 0x08,                // 6.25 Hz odr
	ODR_3_125HZ  = 0x09,                // 3.125 Hz odr
	ODR_1_5625HZ = 0x0A,                // 1.5625 Hz odr
};

enum PMU_CMD_AGGR_AVG : uint8_t {
	NO_AVG = 0,                         // No average
	AVG_2 = 1,                          // Average between 2 samples
	AVG_4 = 2,                          // Average between 4 samples
	AVG_8 = 3,                          // Average between 8 samples
};

static inline uint8_t PMU_CMD_AGGR(enum PMU_CMD_AGGR_ODR odr, enum PMU_CMD_AGGR_AVG avg)
{
	return (odr & 0xf) | ((avg & 3) << 4);
}

// axis configuration
enum PMU_CMD_AXIS_EN_BIT : uint8_t {
	EN_X = (1 << 0),                    // Enable Axis X
	EN_Y = (1 << 1),                    // Enable Axis Y
	EN_Z = (1 << 2),                    // Enable Axis Z
};

// PMU cmd configuration
enum PMU_CMD : uint8_t {
	SUS       = 0x00,                   // Go to SUSPEND mode
	NM        = 0x01,                   // Go to NORMAL mode
	UPD_OAE   = 0x02,                   // Update odr and avg parameter
	FM        = 0x03,                   // Go to FORCED mode with full CRST recharge
	FM_FAST   = 0x04,                   // Go to FORCED mode with fast CRST recharge
	FGR       = 0x05,                   // Do flux-guide reset with full CRST recharge
	FGR_FAST  = 0x06,                   // Do flux-guide reset with fast CRST recharge
	BR        = 0x07,                   // Do bit reset with full CRST recharge
	BR_FAST   = 0x08,                   // Do bit reset with fast CRST recharge
};

// Sensor Status Flag
enum PMU_CMD_STATUS_0_BIT : uint8_t {
	PMU_CMD_BUSY           = (1 << 0),
	ODR_OVWR               = (1 << 1),
	AVG_OVWR               = (1 << 2),
	PWR_MODE_IS_NORMAL     = (1 << 3),
	CMD_IS_ILLEGAL         = (1 << 4)
};

static inline uint8_t PMU_CMD_STATUS_0_VAL(uint8_t x)
{
	return (x >> 5) & 0x7;
};

// PMU status flags

static inline uint8_t PMU_CMD_STATUS_1_ODR(uint8_t x)
{
	return x & 0xf;
};

static inline uint8_t PMU_CMD_STATUS_1_AVG(uint8_t x)
{
	return (x >> 4) & 0x3;
};

enum PMU_CMD_STATUS_0_VALUE {
	PMU_CMD_STATUS_0_SUS             = 0,
	PMU_CMD_STATUS_0_NM              = 1,
	PMU_CMD_STATUS_0_UPD_OAE         = 2,
	BMM350_PMU_CMD_STATUS_0_FM       = 3,
	BMM350_PMU_CMD_STATUS_0_FM_FAST  = 4,
	BMM350_PMU_CMD_STATUS_0_FGR      = 5,
	BMM350_PMU_CMD_STATUS_0_FGR_FAST = 6,
	BMM350_PMU_CMD_STATUS_0_BR       = 7,
	BMM350_PMU_CMD_STATUS_0_BR_FAST  = 7
};

// I3C Bus Error Statistics. Will be cleared on read. If the user writes a 1 into any status bit, this will also clear that bit
enum I3C_ERR_BIT : uint8_t {
	i3c_error_0 = (1 << 0),             // SDR parity error occured
	i3c_error_3 = (1 << 3),             // S0/S1 error occurred
};

// i2c watchdog configure registers

enum I2C_WDT_SET_PERIOD : uint8_t {
	WD_1_28MS  = 0,                     // I2C watch dog time out after 1.28ms
	WD_40_96MS = 1,                     // I2C watch dog time out after 40.96ms
};

static inline uint8_t I2C_WDT_SET(bool enable, enum I2C_WDT_SET_PERIOD period)
{
	return (enable ? 1 : 0) | ((uint8_t)period << 1);
};

enum INT_CTRL_BIT : uint8_t {
	int_mode         = (1 << 0),        // If set, output is in Latched Mode, else in Pulsed Mode
	int_pol          = (1 << 1),        // If set, output is Active High, else Active Low
	int_od           = (1 << 2),        // Configure output: 0: open-drain or 1: push-pull
	int_output_en    = (1 << 3),        // Enable mapping of int on INT pin
	drdy_data_reg_en = (1 << 7),        // Enable Mag Data Ready interrupt onto INT pin and INT_STATUS
};

enum INT_CTRL_IBI_BIT : uint8_t {
	drdy_int_map_to_ibi            = (1 << 0), // Map the drdy interrupt to I3C IBI
	clear_drdy_int_status_upon_ibi = (1 << 4), // Clear INT_STATUS.drdy upon I3C IBI
};

enum INT_STATUS_BIT : uint8_t {
	drdy_data_reg = (1 << 2),           // 0x0: no_new data, 0x1: new_data
};

enum OTP_CMD : uint8_t {
	ADDR_MASK   = 0x1F,
	DIR_READ    = 0x20,
	DIR_PRGM_1B = 0x40,
	DIR_PRGM    = 0x60,
	PWR_OFF_OTP = 0x80,
	EXT_READ    = 0xA0,
	EXT_PRGM    = 0xE0,
};

// OTP status register
enum OTP_STATUS_BIT : uint8_t {
	OTP_CMD_DONE       = (1 << 0),
	CUR_PAGE_ADDR_MASK = (0xf << 1),
	ERROR_MASK         = (0x7 << 5),
};

// Command Register
enum CMD : uint8_t {
	softreset = 0xb6,                   // Perform soft reset
};

enum TMR_SELFTEST_USER : uint8_t {
	DISABLE = 0x00,
	POS_X   = 0x0D,
	NEG_X   = 0x0B,
	POS_Y   = 0x15,
	NEG_Y   = 0x13,
};

} // namespace BMM350
