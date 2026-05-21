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
 * @file Analog_Devices_ADIS1657x_registers.hpp
 *
 * Analog Devices ADIS16575/ADIS16576/ADIS16577 registers.
 *
 */

#pragma once

#include <cstdint>

static constexpr uint16_t Bit0  = (1 << 0);
static constexpr uint16_t Bit1  = (1 << 1);
static constexpr uint16_t Bit2  = (1 << 2);
static constexpr uint16_t Bit3  = (1 << 3);
static constexpr uint16_t Bit4  = (1 << 4);
static constexpr uint16_t Bit5  = (1 << 5);
static constexpr uint16_t Bit6  = (1 << 6);
static constexpr uint16_t Bit7  = (1 << 7);
static constexpr uint16_t Bit8  = (1 << 8);
static constexpr uint16_t Bit9  = (1 << 9);
static constexpr uint16_t Bit10 = (1 << 10);
static constexpr uint16_t Bit11 = (1 << 11);
static constexpr uint16_t Bit12 = (1 << 12);
static constexpr uint16_t Bit13 = (1 << 13);
static constexpr uint16_t Bit14 = (1 << 14);
static constexpr uint16_t Bit15 = (1 << 15);

namespace Analog_Devices_ADIS1657x
{


static constexpr uint32_t SPI_SPEED        = 2'000'000;
static constexpr uint32_t SPI_STALL_PERIOD = 5;

// Timing constants (suffix indicates unit: _MS = milliseconds, _US = microseconds)
static constexpr uint32_t SW_RESET_MS        = 350;
static constexpr uint32_t RST_PULSE_US       = 100;
static constexpr uint32_t DEC_RATE_UPDATE_US = 30;
static constexpr uint32_t FILT_UPDATE_US     = 30;
static constexpr uint32_t MSC_CTRL_UPDATE_US = 200;
static constexpr uint32_t RETRY_DELAY_US     = 100'000;

// Internal clock
static constexpr uint32_t INT_CLK_HZ = 2000;

// Temperature scale
static constexpr float TEMP_SCALE = 0.1f;  // 0.1 C/LSB

// Burst read command
static constexpr uint8_t BURST_CMD_MSB = 0x68;
static constexpr uint8_t BURST_CMD_LSB = 0x00;

// Product IDs
static constexpr uint16_t ADIS16575_PROD_ID = 0x40BF;
static constexpr uint16_t ADIS16576_PROD_ID = 0x40C0;
static constexpr uint16_t ADIS16577_PROD_ID = 0x40C1;

enum class Register : uint8_t {
	DIAG_STAT  = 0x02,
	FILT_CTRL  = 0x5C,
	RNG_MDL    = 0x5E,
	MSC_CTRL   = 0x60,
	DEC_RATE   = 0x64,
	GLOB_CMD   = 0x68,
	PROD_ID    = 0x72,
};

// MSC_CTRL register bits
enum MSC_CTRL_BIT : uint16_t {
	MSC_CTRL_DR_POL   = Bit0,   // Data ready polarity
	MSC_CTRL_POP_EN   = Bit6,   // Point of percussion alignment enable
	MSC_CTRL_GSEN_EN  = Bit7,   // G sensitivity enable
	MSC_CTRL_OUT_SEL  = Bit8,   // Output select (0=gyro/accel, 1=delta angle/velocity)
	MSC_CTRL_BURST_32 = Bit9,   // 32-bit burst mode enable
};

// POP_EN: factory-calibrated correction for IMU offset from vehicle CoM — does not conflict with EKF2.
// GSEN_EN: factory-calibrated gyro sensitivity to linear acceleration — does not conflict with EKF2.
static constexpr uint16_t MSC_CTRL_DEFAULT = MSC_CTRL_DR_POL | MSC_CTRL_POP_EN | MSC_CTRL_GSEN_EN;

// GLOB_CMD register bits
enum GLOB_CMD_BIT : uint16_t {
	GLOB_CMD_SENSOR_SELF_TEST = Bit2,
	GLOB_CMD_SW_RESET         = Bit7,
};

// DIAG_STAT register bits
enum DIAG_STAT_BIT : uint16_t {
	DIAG_STAT_MICROCONTROLLER_FAULT    = Bit15,
	DIAG_STAT_Z_ACCEL_FAILURE          = Bit13,
	DIAG_STAT_Y_ACCEL_FAILURE          = Bit12,
	DIAG_STAT_X_ACCEL_FAILURE          = Bit11,
	DIAG_STAT_Z_GYRO_FAILURE           = Bit10,
	DIAG_STAT_Y_GYRO_FAILURE           = Bit9,
	DIAG_STAT_X_GYRO_FAILURE           = Bit8,
	DIAG_STAT_SYNC_DPLL_UNLOCK         = Bit7,
	DIAG_STAT_MEMORY_ERROR             = Bit6,
	DIAG_STAT_SELF_TEST_ERR            = Bit5,
	DIAG_STAT_POWER_SUPPLY_MON         = Bit4,
	DIAG_STAT_SPI_COMM_ERROR           = Bit3,
	DIAG_STAT_FLASH_UPDATE_FAILURE     = Bit2,
	DIAG_STAT_DATAPATH_OVERRUN         = Bit1,
	DIAG_STAT_SENSOR_INIT_FAILURE      = Bit0,
};

// Self-test duration (datasheet t_ST)
static constexpr uint32_t SELF_TEST_TIME_MS = 19;

// Filter flush delay: >= 1/(2000 Hz) to allow one output cycle between bursts
static constexpr uint32_t FILT_FLUSH_DELAY_US = 1100;

// RNG_MDL gyro range bits [3:2]
static constexpr uint16_t RNG_MDL_GYRO_MASK = 0x0C;
static constexpr uint16_t RNG_MDL_450DPS    = 0x04;  // bits[3:2] = 01
static constexpr uint16_t RNG_MDL_2000DPS   = 0x0C;  // bits[3:2] = 11

} // namespace Analog_Devices_ADIS1657x
