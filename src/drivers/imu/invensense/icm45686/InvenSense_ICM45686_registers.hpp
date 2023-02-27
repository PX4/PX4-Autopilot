/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file InvenSense_ICM45686_registers.hpp
 *
 * Invensense ICM-45686 registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace InvenSense_ICM45686
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

static constexpr uint8_t WHOAMI = 0xE9;

static constexpr float TEMPERATURE_SENSITIVITY = 132.48f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 25.f; // C

namespace Register
{

enum class BANK_0 : uint8_t {
	PWR_MGMT0 = 0x10,
	FIFO_COUNT_0 = 0x12,
	FIFO_COUNT_1 = 0x13,
	FIFO_DATA = 0x14,

	INT1_CONFIG0 = 0x16,
	INT1_CONFIG1 = 0x17,
	INT1_CONFIG2 = 0x18,
	INT1_STATUS0 = 0x19,
	ACCEL_CONFIG0 = 0x1B,
	GYRO_CONFIG0 = 0x1C,
	FIFO_CONFIG0 = 0x1D,
	FIFO_CONFIG1_0 = 0x1E,
	FIFO_CONFIG1_1 = 0x1F,
	FIFO_CONFIG2 = 0x20,
	FIFO_CONFIG3 = 0x21,
	FIFO_CONFIG4 = 0x22,
	RTC_CONFIG = 0x26,
	DMP_EXT_SEN_ODR_CFG = 0x27,
	EDMP_APEX_EN0 = 0x29,
	EDMP_APEX_EN1 = 0x2A,
	APEX_BUFFER_MGMT = 0x2B,
	INTF_CONFIG0 = 0x2C,
	INTF_CONFIG1_OVRD = 0x2D,
	INTF_AUX_CONFIG = 0x2E,
	IOC_PAD_SCENARIO = 0x2F,
	IOC_PAD_SCENARIO_AUX_OVRD = 0x30,
	IOC_PAD_SCENARIO_OVRD = 0x31,
	DRIVE_CONFIG0 = 0x32,
	DRIVE_CONFIG1 = 0x33,
	DRIVE_CONFIG2 = 0x34,
	INT_APEX_CONFIG1 = 0x3a,
	INT_APEX_STATUS0 = 0x3b,
	INT_APEX_STATUS1 = 0x3c,

	INT2_CONFIG0 = 0x56,
	INT2_CONFIG1 = 0x57,
	INT2_CONFIG2 = 0x58,
	INT2_STATUS0 = 0x59,

	WHO_AM_I = 0x72,
	REG_MISC2 = 0x7F,
};

};

//---------------- BANK0 Register bits

// PWR_MGMT0
enum PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2, // 11: Places gyroscope in Low Noise (LN) Mode
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0, // 11: Places accelerometer in Low Noise (LN) Mode
};

enum INT1_STATUS0 : uint8_t {
	INT1_STATUS_RESET_DONE = Bit7,
	INT1_STATUS_AUX1_AGC = Bit6,
	INT1_STATUS_AP_AGC_RDY = Bit5,
	INT1_STATUS_AP_FSYNC = Bit4,
	INT1_STATUS_AP_AUX1_DRDY = Bit3,
	INT1_STATUS_AP_DRDY = Bit2,
	INT1_STATUS_FIFO_THS = Bit1,
	INT1_STATUS_FIFO_FULL = Bit0,
};

enum ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_UI_FS_SEL_32_G_SET = 0,
	ACCEL_UI_FS_SEL_32_G_CLEAR = Bit6 | Bit5 | Bit4,
	ACCEL_UI_FS_SEL_16_G_SET = Bit4,
	ACCEL_UI_FS_SEL_16_G_CLEAR = Bit6 | Bit5,
	ACCEL_UI_FS_SEL_8_G_SET = Bit5,
	ACCEL_UI_FS_SEL_8_G_CLEAR = Bit6 | Bit4,
	ACCEL_ODR_6400_HZ_SET = Bit0 | Bit1,
	ACCEL_ODR_6400_HZ_CLEAR = Bit2,
	ACCEL_ODR_3200_HZ_SET = Bit2,
	ACCEL_ODR_3200_HZ_CLEAR = Bit0 | Bit1,
	ACCEL_ODR_1600_HZ_SET = Bit2 | Bit0,
	ACCEL_ODR_1600_HZ_CLEAR = Bit1,
	ACCEL_ODR_800_HZ_SET = Bit2 | Bit1,
	ACCEL_ODR_800_HZ_CLEAR = Bit0,
};

enum GYRO_CONFIG0_BIT : uint8_t {
	GYRO_UI_FS_SEL_4000_DPS_SET = 0,
	GYRO_UI_FS_SEL_4000_DPS_CLEAR = Bit7 | Bit6 | Bit5 | Bit4,
	GYRO_UI_FS_SEL_2000_DPS_SET = Bit4,
	GYRO_UI_FS_SEL_2000_DPS_CLEAR = Bit7 | Bit6 | Bit5,
	GYRO_UI_FS_SEL_1000_DPS_SET = Bit5,
	GYRO_UI_FS_SEL_1000_DPS_CLEAR = Bit7 | Bit6 | Bit4,
	GYRO_ODR_6400_HZ_SET = Bit0 | Bit1,
	GYRO_ODR_6400_HZ_CLEAR = Bit2,
	GYRO_ODR_3200_HZ_SET = Bit2,
	GYRO_ODR_3200_HZ_CLEAR = Bit0 | Bit1,
	GYRO_ODR_1600_HZ_SET = Bit2 | Bit0,
	GYRO_ODR_1600_HZ_CLEAR = Bit1,
	GYRO_ODR_800_HZ_SET = Bit2 | Bit1,
	GYRO_ODR_800_HZ_CLEAR = Bit0,
};

enum FIFO_CONFIG0_BIT : uint8_t {
	FIFO_MODE_BYPASS_SET = 0,
	FIFO_MODE_BYPASS_CLEAR = Bit6 | Bit7,
	FIFO_MODE_STREAM_SET = Bit6,
	FIFO_MODE_STREAM_CLEAR = Bit7,
	FIFO_MODE_STOP_ON_FULL_SET = Bit7,
	FIFO_MODE_STOP_ON_FULL_CLEAR = Bit6,
	FIFO_DEPTH_2K_SET = Bit0 | Bit1 | Bit2,
	FIFO_DEPTH_2K_CLEAR = Bit3 | Bit4,
	FIFO_DEPTH_8K_SET = Bit0 | Bit1 | Bit2 | Bit3 | Bit4,
	FIFO_DEPTH_8K_CLEAR = 0,
};

enum FIFO_CONFIG2_BIT : uint8_t {
	FIFO_FLUSH = Bit7,
	FIFO_WR_WM_GT_TH_EQUAL = 0,
	FIFO_WR_WM_GT_TH_GREATER_THAN = Bit3,
};

enum FIFO_CONFIG3_BIT : uint8_t {
	FIFO_ES1_EN = Bit5, // External sensor 1 data insertion into FIFO frame
	FIFO_ES0_EN = Bit4, // External sensor 0 data insertion into FIFO frame
	FIFO_HIRES_EN = Bit3, // High resolution accel and gyro data insertion into FIFO frame
	FIFO_GYRO_EN = Bit2, // Gyro data insertion into FIFO frame
	FIFO_ACCEL_EN = Bit1, // Accel data insertion into FIFO frame
	FIFO_IF_EN = Bit0, // Enable FIFO
};

enum FIFO_CONFIG4_BIT : uint8_t {
	FIFO_COMP_EN = Bit2, // FIFO compression enabled
	FIFO_TMST_FSYNC_EN = Bit1, // Timestamp/FSYNC data inserted into FIFO frame
};

enum RTC_CONFIG_BIT : uint8_t {
	RTC_ALIGN = Bit6, // Re-align command is generated by writing 1 to this bit
	RTC_MODE = Bit5, // 0: RTC functionality not enabled, 1: RTC functionality enabled
};

enum IOC_PAD_SCENARIO_OVRD_BIT : uint8_t {
	PADS_INT2_CFG_OVRD = Bit2, // Override enable for PADS_INT2_CFG, 0: disable, 1: enable
	PADS_INT2_CFG_OVRD_INT2 = 0,
	PADS_INT2_CFG_OVRD_FSYNC = Bit0,
	PADS_INT2_CFG_OVRD_CLKIN = Bit1,
};

enum REG_MISC2_BIT : uint8_t {
	SOFT_RST = Bit1, // 1: Triggers soft reset operation
};


// IPREG_TOP1
//static constexpr uint8_t BANK_IPREG_TOP1 = 0xA2;
//static constexpr uint8_t SREG_CTRL = 0x67;
//enum SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIT : uint8_t {
//   SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIG = Bit1, // big endian as documented (instead of default little endian)
//};


namespace FIFO
{
static constexpr size_t SIZE = 8192;

struct DATA {
	uint8_t FIFO_Header;
	uint8_t ACCEL_DATA_XH; // Accel X [19:12]
	uint8_t ACCEL_DATA_XL; // Accel X [11:4]
	uint8_t ACCEL_DATA_YH; // Accel Y [19:12]
	uint8_t ACCEL_DATA_YL; // Accel Y [11:4]
	uint8_t ACCEL_DATA_ZH; // Accel Z [19:12]
	uint8_t ACCEL_DATA_ZL; // Accel Z [11:4]
	uint8_t GYRO_DATA_XH;  // Gyro X [19:12]
	uint8_t GYRO_DATA_XL;  // Gyro X [11:4]
	uint8_t GYRO_DATA_YH;  // Gyro Y [19:12]
	uint8_t GYRO_DATA_YL;  // Gyro Y [11:4]
	uint8_t GYRO_DATA_ZH;  // Gyro Z [19:12]
	uint8_t GYRO_DATA_ZL;  // Gyro Z [11:4]
	uint8_t TEMP_DATA_H;    // Temperature[15:8]
	uint8_t TEMP_DATA_L;    // Temperature[7:0]
	uint8_t Timestamp_H;   // Timestamp[15:8]
	uint8_t Timestamp_L;   // Timestamp[7:0]
	uint8_t HIGHRES_X_LSB; // Accel X LSB [3:0] Gyro X LSB [3:0]
	uint8_t HIGHRES_Y_LSB; // Accel Y LSB [3:0] Gyro Y LSB [3:0]
	uint8_t HIGHRES_Z_LSB; // Accel Z LSB [3:0] Gyro Z LSB [3:0]
};

// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
enum FIFO_HEADER_BIT : uint8_t {
	HEADER_MSG             = Bit7, // 1: FIFO is empty
	HEADER_ACCEL           = Bit6, // 1: Packet is sized so that accel data have location in the packet, FIFO_ACCEL_EN must be 1
	HEADER_GYRO            = Bit5, // 1: Packet is sized so that gyro data have location in the packet, FIFO_GYRO_EN must be1
	HEADER_20              = Bit4, // 1: Packet has a new and valid sample of extended 20-bit data for gyro and/or accel
	HEADER_TIMESTAMP_FSYNC = Bit3 | Bit2, // 10: Packet contains ODR Timestamp
	HEADER_ODR_ACCEL       = Bit1, // 1: The ODR for accel is different for this accel data packet compared to the previous accel packet
	HEADER_ODR_GYRO        = Bit0, // 1: The ODR for gyro is different for this gyro data packet compared to the previous gyro packet
};

}
} // namespace InvenSense_ICM42688P
