/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file ST_LSM303D_registers.hpp
 *
 * ST LSM303D registers.
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

namespace ST_LSM303D
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface

static constexpr uint8_t DIR_READ       = Bit7;
static constexpr uint8_t AUTO_INCREMENT = Bit6;

static constexpr uint8_t WHOAMI = 0b01001001; // Who I am ID

static constexpr uint32_t LA_ODR  = 1600; // Accelerometer output data rate

static constexpr float TEMPERATURE_SENSITIVITY = 8.f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 25.f; // C
static constexpr float TEMPERATURE_SENSOR_MIN = -40.f; // °C
static constexpr float TEMPERATURE_SENSOR_MAX = 85.f; // °C

enum class Register : uint8_t {
	TEMP_OUT_L    = 0x05,
	TEMP_OUT_H    = 0x06,
	STATUS_M      = 0x07,
	OUT_X_L_M     = 0x08,

	WHO_AM_I      = 0x0F,

	CTRL0         = 0x1F,
	CTRL1         = 0x20,
	CTRL2         = 0x21,
	CTRL3         = 0x22,
	CTRL4         = 0x23,
	CTRL5         = 0x24,
	CTRL6         = 0x25,
	CTRL7         = 0x26,
	STATUS_A      = 0x27,
	OUT_X_L_A     = 0x28,

	FIFO_CTRL     = 0x2E,
	FIFO_SRC      = 0x2F,
};

// STATUS_M
enum STATUS_M_BIT : uint8_t {
	ZYXMOR = Bit7, // Magnetic X, Y and Z-axis and temperature data overrun

	ZYXMDA = Bit3, // X, Y and Z-axis and temperature new data available
};

// CTRL0
enum CTRL0_BIT : uint8_t {
	BOOT    = Bit7, // Reboot memory content
	FIFO_EN = Bit6,
	FTH_EN  = Bit5, // FIFO programmable threshold enable

	BIT4_MUST_BE_CLEARED = Bit4, // This bit must be set to ‘0’ for the correct working of the device.
	BIT3_MUST_BE_CLEARED = Bit3, // This bit must be set to ‘0’ for the correct working of the device.
};

// CTRL1
enum CTRL1_BIT : uint8_t {
	// AODR [7:4]
	AODR_1600HZ_SET   = Bit7 | Bit5, // 1010 1600 Hz
	AODR_1600HZ_CLEAR = Bit6 | Bit4,

	BDU               = Bit3, // Block data update
	AZEN              = Bit2, // Acceleration Z-axis enable
	AYEN              = Bit2, // Acceleration Y-axis enable
	AXEN              = Bit2, // Acceleration X-axis enable
};

// CTRL2
enum CTRL2_BIT : uint8_t {
	// ABW [7:6] - Accelerometer anti-alias filter bandwidth
	ABW_773_HZ           = Bit7 | Bit6, // Accelerometer anti-alias filter bandwidth 773 Hz (maximum)

	// AFS [5:3] - Acceleration full-scale selection
	AFS_16G_SET          = Bit5,        // 100 ± 16g
	AFS_16G_CLEAR        = Bit4 | Bit3,
	BIT2_MUST_BE_CLEARED = Bit2, // This bit must be set to ‘0’ for the correct working of the device.
};

// CTRL3
enum CTRL3_BIT : uint8_t {
	INT1_DRDY_A = Bit2, // Accelerometer data-ready signal on INT1
};

// CTRL4
enum CTRL4_BIT : uint8_t {
	INT2_DRDY_A  = Bit3, // Accelerometer data-ready signal on INT2.

	INT2_FTH     = Bit0, // FIFO threshold interrupt on INT2
};

// CTRL5
enum CTRL5_BIT : uint8_t {
	TEMP_EN           = Bit7,        // Temperature sensor enable

	// M_RES [6:5]
	M_RES_HIGH        = Bit6 | Bit5, // Magnetic resolution selection (11: high resolution)

	// M_ODR [4:2]
	M_ODR_50_HZ_SET   = Bit4,        // 100  50Hz
	M_ODR_50_HZ_CLEAR = Bit3 | Bit2,
};

// CTRL6
enum CTRL6_BIT : uint8_t {
	// MFS [6:5]
	MFS_12_GAUSS = Bit6 | Bit5, // Magnetic full-scale selection ± 12 gauss
};

// CTRL7
enum CTRL7_BIT : uint8_t {
	AFDS = Bit5,        // Filtered acceleration data selection (0: internal filter bypassed)

	MD   = Bit1 | Bit0, // Magnetic sensor mode selection (00: Continuous-conversion mode)
};

// FIFO_CTRL
enum FIFO_CTRL_BIT : uint8_t {
	// FM [7:5]
	FIFO_MODE_SET   = Bit5,
	FIFO_MODE_CLEAR = Bit7 | Bit6,

	Bypass_mode     = Bit7 | Bit6 | Bit5, // 000

	// FTH [4:0]: FIFO threshold level.
	FIFO_THRESHOLD  = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

// FIFO_SRC_REG
enum FIFO_SRC_BIT : uint8_t {
	FTH   = Bit7, // FIFO threshold status (set to 1 when FIFO content exceeds threshold level)
	OVRN  = Bit6, // FIFO overrun status
	EMPTY = Bit5, // Empty status.
	// FSS [4:0]: FIFO stored data level.
	FSS   = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};


namespace FIFO
{
//  LSM303D embeds 32 slots of 16-bit data FIFO for each of the three output channels
static constexpr size_t SIZE = 32 * 2 * 3;

struct DATA {
	uint8_t OUT_X_L_A;
	uint8_t OUT_X_H_A;
	uint8_t OUT_Y_L_A;
	uint8_t OUT_Y_H_A;
	uint8_t OUT_Z_L_A;
	uint8_t OUT_Z_H_A;
};

}

} // namespace ST_LSM303D
