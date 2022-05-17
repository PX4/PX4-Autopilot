/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

static constexpr uint32_t SPI_SPEED = 24 * 1000 * 1000; // 24 MHz SPI
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x67;

static constexpr float TEMPERATURE_SENSITIVITY = 128.0f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 25.f; // C

namespace Register
{

enum class BANK_0 : uint8_t {
	SIGNAL_PATH_RESET = 0x02,

	INT_CONFIG        = 0x06,

	TEMP_DATA1        = 0x09,
	TEMP_DATA0        = 0x0A,

	PWR_MGMT0         = 0x1F,
	GYRO_CONFIG0      = 0x20,
	ACCEL_CONFIG0     = 0x21,
	GYRO_CONFIG1      = 0x23,
	ACCEL_CONFIG1     = 0x24,

	FIFO_CONFIG1      = 0x28,
	FIFO_CONFIG2      = 0x29,
	FIFO_CONFIG3      = 0x2A,
	INT_SOURCE0       = 0x2B,

	INT_STATUS        = 0x3A,
	FIFO_COUNTH       = 0x3D,
	FIFO_COUNTL       = 0x3E,
	FIFO_DATA         = 0x3F,

	WHO_AM_I          = 0x75,

	BLK_SEL_W         = 0x79,
	MADDR_W           = 0x7A,
	M_W               = 0x7B,
	BLK_SEL_R         = 0x7C,
	MADDR_R           = 0x7D,
	M_R               = 0x7E,
};

enum class MREG1 : uint8_t {
	FIFO_CONFIG5 = 0x01,

	INT_CONFIG0  = 0x04,
};

};

//---------------- BANK0 Register bits

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	SOFT_RESET_DEVICE_CONFIG  = Bit4, // 1: Software reset enabled
	FIFO_FLUSH                = Bit2, // When set to 1, FIFO will get flushed
};

// INT_CONFIG
enum INT_CONFIG_BIT : uint8_t {
	INT1_MODE          = Bit2, // INT1 interrupt mode 1: Latched mode
	INT1_DRIVE_CIRCUIT = Bit1, // INT1 drive circuit 1: Push pull
	INT1_POLARITY      = Bit0, // INT1 interrupt polarity 0: Active low
};

// PWR_MGMT0
enum PWR_MGMT0_BIT : uint8_t {
	// 3:2 GYRO_MODE
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2, // 11: Places gyroscope in Low Noise (LN) Mode
	// 1:0 ACCEL_MODE
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0, // 11: Places accelerometer in Low Noise (LN) Mode
};

// GYRO_CONFIG0
enum GYRO_CONFIG0_BIT : uint8_t {
	// 6:5 GYRO_FS_SEL
	GYRO_FS_SEL_2000_DPS_SET   = 0,           // 0b000 = ±2000dps
	GYRO_FS_SEL_2000_DPS_CLEAR = Bit6 | Bit5,

	// 3:0 GYRO_ODR
	GYRO_ODR_1600HZ_SET        = Bit2 | Bit0, // 0101: 1600Hz
	GYRO_ODR_1600HZ_CLEAR      = Bit3 | Bit1,
};

// ACCEL_CONFIG0
enum ACCEL_CONFIG0_BIT : uint8_t {
	// 6:5 ACCEL_UI_FS_SEL
	ACCEL_UI_FS_SEL_16G_SET   = 0,           // 000: ±16g
	ACCEL_UI_FS_SEL_16G_CLEAR = Bit6 | Bit5,

	// 3:0 ACCEL_ODR
	ACCEL_ODR_1600HZ_SET      = Bit2 | Bit0, // 0101: 1600Hz
	ACCEL_ODR_1600HZ_CLEAR    = Bit3 | Bit1,
};

// GYRO_CONFIG1
enum GYRO_CONFIG1_BIT : uint8_t {
	// 2:0 GYRO_UI_FILT_BW
	GYRO_UI_FILT_BW_BYPASSED_CLEAR = Bit2 | Bit1 | Bit0, // 000: Low pass filter bypassed
};

// ACCEL_CONFIG1
enum ACCEL_CONFIG1_BIT : uint8_t {
	// 2:0 ACCEL_UI_FILT_BW
	ACCEL_UI_FILT_BW_BYPASSED_CLEAR = Bit2 | Bit1 | Bit0, // 000: Low pass filter bypassed
};

// FIFO_CONFIG1
enum FIFO_CONFIG1_BIT : uint8_t {
	// FIFO_MODE
	FIFO_MODE_STOP_ON_FULL = Bit1, // 1: STOP-on-FULL Mode
	FIFO_BYPASS            = Bit0, // 0: FIFO is not bypassed
};

// INT_STATUS
enum INT_STATUS_BIT : uint8_t {
	RESET_DONE_INT = Bit4,
	FIFO_THS_INT   = Bit2,
	FIFO_FULL_INT  = Bit1,
};

// INT_SOURCE0
enum INT_SOURCE0_BIT : uint8_t {
	RESET_DONE_INT1_EN = Bit4, // 1: Reset done interrupt routed to INT1
	FIFO_THS_INT1_EN   = Bit2, // FIFO threshold interrupt routed to INT1
};


//---------------- USER BANK MREG1 Register bits

// FIFO_CONFIG5
enum FIFO_CONFIG5_BIT : uint8_t {
	FIFO_WM_GT_TH = Bit5, // 0: Trigger FIFO Watermark interrupt when FIFO_COUNT = FIFO_WM
	FIFO_GYRO_EN  = Bit1, // 1: Enables Gyro packets to go to FIFO
	FIFO_ACCEL_EN = Bit0, // 1: Enables Accel packets to go to FIFO
};

// INT_CONFIG0
enum INT_CONFIG0_BIT : uint8_t {
	// 3:2 FIFO_THS_INT_CLEAR
	FIFO_THS_INT_CLEAR = Bit3, // 10: Clear on FIFO data 1Byte Read
};

namespace FIFO
{
static constexpr size_t SIZE = 1024;

// FIFO_DATA layout when FIFO_CONFIG1 has FIFO_GYRO_EN and FIFO_ACCEL_EN set

// Packet 3
struct DATA {
	uint8_t FIFO_Header;
	uint8_t ACCEL_DATA_X1; // Accel X [19:12]
	uint8_t ACCEL_DATA_X0; // Accel X [11:4]
	uint8_t ACCEL_DATA_Y1; // Accel Y [19:12]
	uint8_t ACCEL_DATA_Y0; // Accel Y [11:4]
	uint8_t ACCEL_DATA_Z1; // Accel Z [19:12]
	uint8_t ACCEL_DATA_Z0; // Accel Z [11:4]
	uint8_t GYRO_DATA_X1;  // Gyro X [19:12]
	uint8_t GYRO_DATA_X0;  // Gyro X [11:4]
	uint8_t GYRO_DATA_Y1;  // Gyro Y [19:12]
	uint8_t GYRO_DATA_Y0;  // Gyro Y [11:4]
	uint8_t GYRO_DATA_Z1;  // Gyro Z [19:12]
	uint8_t GYRO_DATA_Z0;  // Gyro Z [11:4]
	uint8_t temperature;   // Temperature[7:0]
	uint8_t TimeStamp_h;   // TimeStamp[15:8]
	uint8_t TimeStamp_l;   // TimeStamp[7:0]
};

// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
enum FIFO_HEADER_BIT : uint8_t {
	HEADER_MSG             = Bit7, // 1: FIFO is empty
	HEADER_ACCEL           = Bit6, // 1: Packet is sized so that accel data have location in the packet, FIFO_ACCEL_EN must be 1
	HEADER_GYRO            = Bit5, // 1: Packet is sized so that gyro data have location in the packet, FIFO_GYRO_EN must be1
	HEADER_20              = Bit4, // 1: Packet has a new and valid sample of extended 20-bit data for gyro and/or accel
	HEADER_TIMESTAMP_FSYNC = Bit3 | Bit2,
	HEADER_ODR_ACCEL       = Bit1, // 1: The ODR for accel is different for this accel data packet compared to the previous accel packet
	HEADER_ODR_GYRO        = Bit0, // 1: The ODR for gyro is different for this gyro data packet compared to the previous gyro packet
};

}
} // namespace InvenSense_ICM42670P
