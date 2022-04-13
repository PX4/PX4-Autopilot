/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file InvenSense_ICM40609D_registers.hpp
 *
 * Invensense ICM-40609-D registers.
 *
 */

#pragma once

#include <cstdint>

namespace InvenSense_ICM40609D
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

static constexpr uint32_t SPI_SPEED = 24 * 1000 * 1000; // 24 MHz SPI
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x3B;

static constexpr float TEMPERATURE_SENSITIVITY = 132.48f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 25.f; // C

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

	PWR_MGMT0          = 0x4E,
	GYRO_CONFIG0       = 0x4F,
	ACCEL_CONFIG0      = 0x50,
	GYRO_CONFIG1       = 0x51,
	ACCEL_CONFIG1      = 0x53,

	FIFO_CONFIG1       = 0x5F,
	FIFO_CONFIG2       = 0x60,
	FIFO_CONFIG3       = 0x61,

	INT_CONFIG0        = 0x63,
	INT_CONFIG1        = 0x64,
	INT_SOURCE0        = 0x65,

	WHO_AM_I           = 0x75,
	REG_BANK_SEL       = 0x76,
};

};

//---------------- BANK0 Register bits

// DEVICE_CONFIG
enum DEVICE_CONFIG_BIT : uint8_t {
	SOFT_RESET_CONFIG = Bit0,
};

// INT_CONFIG
enum INT_CONFIG_BIT : uint8_t {
	INT1_MODE           = Bit2,
	INT1_DRIVE_CIRCUIT  = Bit1,
	INT1_POLARITY       = Bit0,
};

// FIFO_CONFIG
enum FIFO_CONFIG_BIT : uint8_t {
	// 7:6 FIFO_MODE
	FIFO_MODE_STOP_ON_FULL = Bit7 | Bit6, // 11: STOP-on-FULL Mode
};

// INT_STATUS
enum INT_STATUS_BIT : uint8_t {
	RESET_DONE_INT = Bit4,

	FIFO_THS_INT   = Bit2,
	FIFO_FULL_INT  = Bit1,
};

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	FIFO_FLUSH = Bit1,
};

// PWR_MGMT0
enum PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2, // 11: Places gyroscope in Low Noise (LN) Mode
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0, // 11: Places accelerometer in Low Noise (LN) Mode
};

// GYRO_CONFIG0
enum GYRO_CONFIG0_BIT : uint8_t {
	// 7:5 GYRO_FS_SEL
	//  0b000: ±2000dps (default)
	GYRO_FS_SEL_2000_DPS_CLEAR = Bit7 | Bit6 | Bit5,

	// 3:0 GYRO_ODR
	//  0b0001: 32kHz (maximum)
	GYRO_ODR_32KHZ_SET         = Bit0,
	GYRO_ODR_32KHZ_CLEAR       = Bit3 | Bit2 | Bit0,
	//  0b0011: 8kHz
	GYRO_ODR_8KHZ_SET          = Bit1 | Bit0,
	GYRO_ODR_8KHZ_CLEAR        = Bit3 | Bit2,
	//  0b0110: 1kHz (default)
	GYRO_ODR_1KHZ_SET          = Bit2 | Bit1,
	GYRO_ODR_1KHZ_CLEAR        = Bit3 | Bit0,
};

// ACCEL_CONFIG0
enum ACCEL_CONFIG0_BIT : uint8_t {
	// 7:5 ACCEL_FS_SEL
	//  0b000: ±32g (default)
	ACCEL_FS_SEL_32G_CLEAR     = Bit7 | Bit6 | Bit5,

	// 3:0 ACCEL_ODR
	//  0b0001: 32kHz (maximum)
	ACCEL_ODR_32KHZ_SET        = Bit0,
	ACCEL_ODR_32KHZ_CLEAR      = Bit3 | Bit2 | Bit0,
	//  0b0011: 8kHz
	ACCEL_ODR_8KHZ_SET         = Bit1 | Bit0,
	ACCEL_ODR_8KHZ_CLEAR       = Bit3 | Bit2,
	//  0b0110: 1kHz (default)
	ACCEL_ODR_1KHZ_SET         = Bit2 | Bit1,
	ACCEL_ODR_1KHZ_CLEAR       = Bit3 | Bit0,
};

// GYRO_CONFIG1
enum GYRO_CONFIG1_BIT : uint8_t {
	GYRO_UI_FILT_ORD_1ST_CLEAR = Bit3 | Bit2, // 00: 1st Order UI filter
};

// ACCEL_CONFIG1
enum ACCEL_CONFIG1_BIT : uint8_t {
	ACCEL_UI_FILT_ORD_1ST_CLEAR = Bit4 | Bit3, // 00: 1st Order UI filter
};

// FIFO_CONFIG1
enum FIFO_CONFIG1_BIT : uint8_t {
	FIFO_RESUME_PARTIAL_RD = Bit6,

	FIFO_TEMP_EN           = Bit2,
	FIFO_GYRO_EN           = Bit1,
	FIFO_ACCEL_EN          = Bit0,
};

// INT_CONFIG0
enum INT_CONFIG0_BIT : uint8_t {
	// 3:2 FIFO_THS_INT_CLEAR
	FIFO_THS_INT_CLEAR = Bit3, // 10: Clear on FIFO data 1Byte Read
};

// INT_CONFIG1
enum INT_CONFIG1_BIT : uint8_t {
	INT_TPULSE_DURATION   = Bit6, // 1: Interrupt pulse duration is 8 µs. Required if ODR ≥ 4kHz, optional for ODR < 4kHz.
	INT_TDEASSERT_DISABLE = Bit5, // 1: Disables de-assert duration. Required if ODR ≥ 4kHz, optional for ODR < 4kHz.
	INT_ASYNC_RESET       = Bit4, // User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation
};

// INT_SOURCE0
enum INT_SOURCE0_BIT : uint8_t {
	RESET_DONE_INT1_EN = Bit4, // 1: Reset done interrupt routed to INT1 (enabled by default)

	FIFO_THS_INT1_EN   = Bit2, // FIFO threshold interrupt routed to INT1
};

// REG_BANK_SEL
enum REG_BANK_SEL_BIT : uint8_t {
	USER_BANK_0 = 0,           // 0: Select USER BANK 0.
	USER_BANK_1 = Bit4,        // 1: Select USER BANK 1.
	USER_BANK_2 = Bit5,        // 2: Select USER BANK 2.
	USER_BANK_3 = Bit5 | Bit4, // 3: Select USER BANK 3.
};

namespace FIFO
{
static constexpr size_t SIZE = 2048;

// FIFO_DATA layout when FIFO_CONFIG1 has FIFO_GYRO_EN and FIFO_ACCEL_EN set

// Packet 3
struct DATA {
	uint8_t FIFO_Header;
	uint8_t ACCEL_DATA_X1;
	uint8_t ACCEL_DATA_X0;
	uint8_t ACCEL_DATA_Y1;
	uint8_t ACCEL_DATA_Y0;
	uint8_t ACCEL_DATA_Z1;
	uint8_t ACCEL_DATA_Z0;
	uint8_t GYRO_DATA_X1;
	uint8_t GYRO_DATA_X0;
	uint8_t GYRO_DATA_Y1;
	uint8_t GYRO_DATA_Y0;
	uint8_t GYRO_DATA_Z1;
	uint8_t GYRO_DATA_Z0;
	uint8_t FIFO_TEMP_DATA; // Temperature[7:0]
	uint8_t TimeStamp_h;    // TimeStamp[15:8]
	uint8_t TimeStamp_l;    // TimeStamp[7:0]
};

// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
enum FIFO_HEADER_BIT : uint8_t {
	HEADER_MSG             = Bit7, // 1: FIFO is empty
	HEADER_ACCEL           = Bit6,
	HEADER_GYRO            = Bit5,

	HEADER_TIMESTAMP_FSYNC = Bit3 | Bit2,
	HEADER_ODR_ACCEL       = Bit1,
	HEADER_ODR_GYRO        = Bit0,
};

}
} // namespace InvenSense_ICM40609D
