/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file InvenSense_ICM42670P_registers.hpp
 *
 * Invensense ICM-42670-P registers.
 *
 */

#pragma once

#include <cstdint>

namespace InvenSense_ICM42670P
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

static constexpr uint32_t SPI_SPEED = 12 * 1000 * 1000; // 24 MHz SPI
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x67;

static constexpr float TEMPERATURE_SENSITIVITY = 128.0f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 25.f; // C

namespace Register
{

enum class BANK_0 : uint8_t {
	DEVICE_CONFIG     = 0x01,

	INT_CONFIG        = 0x06,


	TEMP_DATA1        = 0x09,
	TEMP_DATA0        = 0x0A,

	INT_STATUS        = 0x3A,
	FIFO_COUNTH       = 0x3D,
	FIFO_COUNTL       = 0x3E,
	FIFO_DATA         = 0x3F,

	SIGNAL_PATH_RESET = 0x02,

	PWR_MGMT0         = 0x1F,
	GYRO_CONFIG0      = 0x20,
	ACCEL_CONFIG0     = 0x21,
	GYRO_CONFIG1      = 0x23,
	ACCEL_CONFIG1     = 0x24,

	FIFO_CONFIG1      = 0x28,
	FIFO_CONFIG2      = 0x29,
	FIFO_CONFIG3      = 0x2A,

	INT_SOURCE0       = 0x2B,

	WHO_AM_I          = 0x75,
	// REG_BANK_SEL      = 0x76,

	BLK_SEL_W         = 0x79,
	MADDR_W           = 0x7A,
	M_W               = 0x7B,

	BLK_SEL_R         = 0x7C,
	MADDR_R           = 0x7D,
	M_R               = 0x7E,

};

enum class MREG_1 : uint8_t {
	FIFO_CONFIG5_MREG1      = 0x01,
	INT_CONFIG0_MREG1       = 0x04,
};

enum class MREG_2 : uint8_t {
	OTP_CTRL7_MREG2  = 0x06,
};

enum class MREG_3 : uint8_t {
	XA_ST_DATA_MREG3 = 0x00,
	YA_ST_DATA_MREG3 = 0x01,
	ZA_ST_DATA_MREG3 = 0x02,
	XG_ST_DATA_MREG3 = 0x03,
	YG_ST_DATA_MREG3 = 0x04,
	ZG_ST_DATA_MREG3 = 0x05,
};



};

//---------------- BANK0 Register bits

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	SOFT_RESET_DEVICE_CONFIG  = Bit4, //
	FIFO_FLUSH                = Bit2,
};

// INT_CONFIG
enum INT_CONFIG_BIT : uint8_t {
	INT1_MODE           = Bit2,
	INT1_DRIVE_CIRCUIT  = Bit1,
	INT1_POLARITY       = Bit0,
};
// GYRO_CONFIG1
enum GYRO_CONFIG1_BIT : uint8_t {

	// 2:0 GYRO_ODR
	GYRO_UI_FILT_BW_16Hz   = Bit2 | Bit1 | Bit0,   // 111: 16Hz
	GYRO_UI_FILT_BW_25Hz   = Bit2 | Bit1,          // 110: 25Hz
	GYRO_UI_FILT_BW_34Hz   = Bit2 | Bit0,  	// 101: 34Hz
	GYRO_UI_FILT_BW_53Hz   = Bit2,         	// 100: 53Hz
	GYRO_UI_FILT_BW_73Hz   = Bit1 | Bit0,  	// 011: 73Hz
	GYRO_UI_FILT_BW_121Hz  = Bit1,        		// 010: 121Hz
	GYRO_UI_FILT_BW_180Hz  = Bit0,        		// 001: 180Hz
};

// ACCEL_CONFIG1
enum ACCEL_CONFIG1_BIT : uint8_t {

	// 2:0 ACCEL_ODR
	ACCEL_UI_FILT_BW_16Hz   = Bit2 | Bit1 | Bit0,   // 111: 16Hz
	ACCEL_UI_FILT_BW_25Hz   = Bit2 | Bit1,          // 110: 25Hz
	ACCEL_UI_FILT_BW_34Hz   = Bit2 | Bit0,  	// 101: 34Hz
	ACCEL_UI_FILT_BW_53Hz   = Bit2,         	// 100: 53Hz
	ACCEL_UI_FILT_BW_73Hz   = Bit1 | Bit0,  	// 011: 73Hz
	ACCEL_UI_FILT_BW_121Hz  = Bit1,        		// 010: 121Hz
	ACCEL_UI_FILT_BW_180Hz  = Bit0,        		// 001: 180Hz
};
// FIFO_CONFIG1
enum FIFO_CONFIG1_BIT : uint8_t {
	// 1 FIFO_MODE
	FIFO_MODE_STOP_ON_FULL = Bit1, // 11: STOP-on-FULL Mode
};

// INT_STATUS
enum INT_STATUS_BIT : uint8_t {
	RESET_DONE_INT = Bit4,
	FIFO_THS_INT   = Bit2,
	FIFO_FULL_INT  = Bit1,
};

// PWR_MGMT0
enum PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2, // 11: Places gyroscope in Low Noise (LN) Mode
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0, // 11: Places accelerometer in Low Noise (LN) Mode
};

// GYRO_CONFIG0
enum GYRO_CONFIG0_BIT : uint8_t {
	// 6:5 GYRO_FS_SEL
	GYRO_FS_SEL_2000_DPS = 0,            // 0b000 = ±2000dps
	GYRO_FS_SEL_1000_DPS = Bit5,         // 0b001 = ±1000 dps
	GYRO_FS_SEL_500_DPS  = Bit6,         // 0b010 = ±500 dps
	GYRO_FS_SEL_250_DPS  = Bit6 | Bit5,  // 0b011 = ±250 dps

	// 3:0 GYRO_ODR
	GYRO_ODR_1600Hz      = Bit2 | Bit0,         // 0101: 1600Hz
	GYRO_ODR_800Hz       = Bit2 | Bit1,         // 0110: 800Hz
	GYRO_ODR_400Hz       = Bit2 | Bit1 | Bit0,  // 0111: 400Hz
	GYRO_ODR_200Hz       = Bit3,         // 1000: 200Hz
};

// ACCEL_CONFIG0
enum ACCEL_CONFIG0_BIT : uint8_t {
	// 6:5 ACCEL_FS_SEL
	ACCEL_FS_SEL_16G = 0,           // 000: ±16g
	ACCEL_FS_SEL_8G  = Bit5,        // 001: ±8g
	ACCEL_FS_SEL_4G  = Bit6,        // 010: ±4g
	ACCEL_FS_SEL_2G  = Bit6 | Bit5, // 011: ±2g

	// 3:0 ACCEL_ODR
	ACCEL_ODR_1600Hz  = Bit2 | Bit0,         // 0101: 1600Hz
	ACCEL_ODR_800Hz  = Bit2 | Bit1,         // 0110: 800Hz
	ACCEL_ODR_400Hz   = Bit2 | Bit1 | Bit0,  // 0111: 400Hz
	ACCEL_ODR_200Hz   = Bit3,         	// 1000: 200Hz
};

// FIFO_CONFIG5
enum FIFO_CONFIG5_BIT : uint8_t {
	FIFO_RESUME_PARTIAL_RD = Bit4,
	FIFO_WM_GT_TH          = Bit5,
	FIFO_HIRES_EN          = Bit3,
	FIFO_TMST_FSYNC_EN     = Bit2,
	FIFO_GYRO_EN           = Bit1,
	FIFO_ACCEL_EN          = Bit0,
};

// INT_CONFIG0
enum INT_CONFIG0_BIT : uint8_t {
	// 3:2 FIFO_THS_INT_CLEAR
	CLEAR_ON_FIFO_READ = Bit3,
};

// INT_SOURCE0
enum INT_SOURCE0_BIT : uint8_t {
	ST_INT1_END        = Bit7,
	FSYNC_INT1_EN      = Bit6,
	PLL_RDY_INT1_EN    = Bit5,
	RESET_DONE_INT1_EN = Bit4,
	DRDY_INT1_EN       = Bit3,
	FIFO_THS_INT1_EN   = Bit2, // FIFO threshold interrupt routed to INT1
	FIFO_FULL_INT1_EN  = Bit1,
	AGC_RDY_INT1_EN    = Bit0,
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
	uint8_t temperature;  // Temperature[7:0]
	uint8_t timestamp_l;
	uint8_t timestamp_h;
};

// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
enum FIFO_HEADER_BIT : uint8_t {
	HEADER_MSG             = Bit7, // 1: FIFO is empty
	HEADER_ACCEL           = Bit6,
	HEADER_GYRO            = Bit5,
	HEADER_20              = Bit4,
	HEADER_TIMESTAMP_FSYNC = Bit3 | Bit2,
	HEADER_ODR_ACCEL       = Bit1,
	HEADER_ODR_GYRO        = Bit0,
};

}
} // namespace InvenSense_ICM42670P
