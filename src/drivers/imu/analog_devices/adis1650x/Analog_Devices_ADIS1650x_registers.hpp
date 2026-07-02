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

/**
 * @file Analog_Devices_ADIS1650x_registers.hpp
 *
 * Analog Devices ADIS16500/16501/16505/16507 registers.
 */

#pragma once

#include <cstdint>

namespace Analog_Devices_ADIS1650x
{


static constexpr uint32_t SPI_SPEED          = 1'100'000;
static constexpr uint32_t SPI_STALL_US       = 16;

// Timing constants
static constexpr uint32_t SW_RESET_MS      = 350;        // ms after software/hardware reset
static constexpr uint32_t RST_PULSE_US     = 100;        // hardware RST pin assert duration (us)
static constexpr uint32_t MSC_CTRL_UPDATE_US = 200;
static constexpr uint32_t DEC_RATE_UPDATE_US = 30;
static constexpr uint32_t FILT_UPDATE_US   = 30;

// Internal clock
static constexpr uint32_t INT_CLK_HZ       = 2000;

// Temperature scale: 0.1 °C / LSB
static constexpr float TEMP_SCALE          = 0.1f;

// Burst read command (0x6800)
static constexpr uint8_t BURST_CMD_MSB     = 0x68;
static constexpr uint8_t BURST_CMD_LSB     = 0x00;

// Product IDs
static constexpr uint16_t ADIS16500_PROD_ID = 0x4074;
static constexpr uint16_t ADIS16501_PROD_ID = 0x5FB5;
static constexpr uint16_t ADIS16505_PROD_ID = 0x4079;
static constexpr uint16_t ADIS16507_PROD_ID = 0x407B;

// Registers
enum class Register : uint8_t {
	DIAG_STAT = 0x02,
	RNG_MDL   = 0x5E,
	FILT_CTRL = 0x5C,
	MSC_CTRL  = 0x60,
	DEC_RATE  = 0x64,
	GLOB_CMD  = 0x68,
	PROD_ID   = 0x72,
};

// MSC_CTRL bits
static constexpr uint16_t MSC_CTRL_DR_POL   = (1 << 0);  // Data ready active high
static constexpr uint16_t MSC_CTRL_GCOMP    = (1 << 6);  // G-compensation enable
static constexpr uint16_t MSC_CTRL_PCOMP    = (1 << 7);  // Point of percussion enable
static constexpr uint16_t MSC_CTRL_OUT_SEL  = (1 << 8);  // Output select (0=gyro/accel)
static constexpr uint16_t MSC_CTRL_BURST_32 = (1 << 9);  // 32-bit burst mode

// GLOB_CMD bits
static constexpr uint16_t GLOB_CMD_SW_RESET        = (1 << 7);
static constexpr uint16_t GLOB_CMD_SENSOR_SELF_TEST = (1 << 2);

// Self-test duration (datasheet t_ST)
static constexpr uint32_t SELF_TEST_TIME_MS = 24;

// DIAG_STAT register bits
enum DIAG_STAT_BIT : uint16_t {
	DIAG_STAT_Z_ACCEL_FAILURE      = (1 << 13),
	DIAG_STAT_Y_ACCEL_FAILURE      = (1 << 12),
	DIAG_STAT_X_ACCEL_FAILURE      = (1 << 11),
	DIAG_STAT_Z_GYRO_FAILURE       = (1 << 10),
	DIAG_STAT_Y_GYRO_FAILURE       = (1 <<  9),
	DIAG_STAT_X_GYRO_FAILURE       = (1 <<  8),
	DIAG_STAT_SELF_TEST_ERR        = (1 <<  5),
	DIAG_STAT_POWER_SUPPLY         = (1 <<  4),
	DIAG_STAT_SPI_COMM_ERROR       = (1 <<  3),
	DIAG_STAT_FLASH_UPDATE_FAILURE = (1 <<  2),
	DIAG_STAT_DATAPATH_OVERRUN     = (1 <<  1),
	DIAG_STAT_SENSOR_INIT_FAILURE  = (1 <<  0),
};

// Filter flush delay: >= 1/(2000 Hz) to allow one output cycle between bursts
static constexpr uint32_t FILT_FLUSH_DELAY_US = 1100;

// Group delay base values (no filtering, no decimation) from datasheet Table 8
static constexpr float GROUP_DELAY_ACCEL_BASE_US = 1570.0f;
static constexpr float GROUP_DELAY_GYRO_BASE_US  = 1437.0f;  // average of X=1510, Y=1510, Z=1290
static constexpr float SAMPLE_PERIOD_US          = 500.0f;    // 1/2000 Hz = 500 us

// RNG_MDL gyro range bits [3:2]
static constexpr uint16_t RNG_MDL_GYRO_MASK  = 0x0C;
static constexpr uint16_t RNG_MDL_125DPS     = 0x00;  // bits[3:2] = 00 => ±125 deg/s
static constexpr uint16_t RNG_MDL_500DPS     = 0x04;  // bits[3:2] = 01 => ±500 deg/s
static constexpr uint16_t RNG_MDL_2000DPS    = 0x0C;  // bits[3:2] = 11 => ±2000 deg/s

} // namespace Analog_Devices_ADIS1650x
