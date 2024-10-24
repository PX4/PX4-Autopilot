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
 * @file InvenSense_ICM42688P_registers.hpp
 *
 * Invensense ICM-42688-P registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace InvenSense_ICM42688P
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

static constexpr uint8_t WHOAMI = 0x47;
static constexpr uint8_t WHOAMI686 = 0x44;

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
	GYRO_CONFIG_STATIC2  = 0x0B,
	GYRO_CONFIG_STATIC3  = 0x0C,
	GYRO_CONFIG_STATIC4  = 0x0D,
	GYRO_CONFIG_STATIC5  = 0x0E,

	INTF_CONFIG5         = 0x7B,
};

enum class BANK_2 : uint8_t {
	ACCEL_CONFIG_STATIC2 = 0x03,
	ACCEL_CONFIG_STATIC3 = 0x04,
	ACCEL_CONFIG_STATIC4 = 0x05,
};

};

//---------------- BANK0 Register bits

// DEVICE_CONFIG
enum DEVICE_CONFIG_BIT : uint8_t {
	SOFT_RESET_CONFIG = Bit0, //
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
	DATA_RDY_INT   = Bit3,
	FIFO_THS_INT   = Bit2,
	FIFO_FULL_INT  = Bit1,
};

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	ABORT_AND_RESET = Bit3,
	FIFO_FLUSH      = Bit1,
};

// INTF_CONFIG0
enum INTF_CONFIG0_BIT : uint8_t {
	FIFO_HOLD_LAST_DATA_EN  = Bit7,
	FIFO_COUNT_REC          = Bit6,
	FIFO_COUNT_ENDIAN       = Bit5,
	SENSOR_DATA_ENDIAN      = Bit4,
	UI_SIFS_CFG_DISABLE_I2C = Bit1 | Bit0,
};

// INTF_CONFIG1
enum INTF_CONFIG1_BIT : uint8_t {
	RTC_MODE = Bit2, // 0: No input RTC clock is required, 1: RTC clock input is required
	CLKSEL = Bit0,
	CLKSEL_CLEAR = Bit1,
};

// PWR_MGMT0
enum PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE  = Bit3 | Bit2, // 11: Places gyroscope in Low Noise (LN) Mode
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0, // 11: Places accelerometer in Low Noise (LN) Mode
};

// GYRO_CONFIG0
enum GYRO_CONFIG0_BIT : uint8_t {
	// 7:5 GYRO_FS_SEL
	GYRO_FS_SEL_2000_DPS = 0, // 0b000 = ±2000dps (default)
	GYRO_FS_SEL_1000_DPS = Bit5,
	GYRO_FS_SEL_500_DPS  = Bit6,
	GYRO_FS_SEL_250_DPS  = Bit6 | Bit5,
	GYRO_FS_SEL_125_DPS  = Bit7,


	// 3:0 GYRO_ODR
	//  0001: 32kHz
	GYRO_ODR_32KHZ_SET   = Bit0,
	GYRO_ODR_32KHZ_CLEAR = Bit3 | Bit2 | Bit0,
	//  0010: 16kHz
	GYRO_ODR_16KHZ_SET   = Bit1,
	GYRO_ODR_16KHZ_CLEAR = Bit3 | Bit2 | Bit0,
	//  0011: 8kHz
	GYRO_ODR_8KHZ_SET    = Bit1 | Bit0,
	GYRO_ODR_8KHZ_CLEAR  = Bit3 | Bit2,
	//  0110: 1kHz (default)
	GYRO_ODR_1KHZ_SET    = Bit2 | Bit1,
	GYRO_ODR_1KHZ_CLEAR  = Bit3 | Bit0,
};

// ACCEL_CONFIG0
enum ACCEL_CONFIG0_BIT : uint8_t {
	// 7:5 ACCEL_FS_SEL
	ACCEL_FS_SEL_16G = 0, // 000: ±16g (default)
	ACCEL_FS_SEL_8G  = Bit5,
	ACCEL_FS_SEL_4G  = Bit6,
	ACCEL_FS_SEL_2G  = Bit6 | Bit5,


	// 3:0 ACCEL_ODR
	//  0001: 32kHz
	ACCEL_ODR_32KHZ_SET   = Bit0,
	ACCEL_ODR_32KHZ_CLEAR = Bit3 | Bit2 | Bit0,
	//  0010: 16kHz
	ACCEL_ODR_16KHZ_SET   = Bit1,
	ACCEL_ODR_16KHZ_CLEAR = Bit3 | Bit2 | Bit0,
	//  0011: 8kHz
	ACCEL_ODR_8KHZ_SET    = Bit1 | Bit0,
	ACCEL_ODR_8KHZ_CLEAR  = Bit3 | Bit2,
	//  0110: 1kHz (default)
	ACCEL_ODR_1KHZ_SET    = Bit2 | Bit1,
	ACCEL_ODR_1KHZ_CLEAR  = Bit3 | Bit0,
};

// GYRO_CONFIG1
enum GYRO_CONFIG1_BIT : uint8_t {
	GYRO_UI_FILT_ORD = Bit3 | Bit2, // 00: 1st Order
};

// GYRO_ACCEL_CONFIG0
enum GYRO_ACCEL_CONFIG0_BIT : uint8_t {
	// 7:4 ACCEL_UI_FILT_BW
	ACCEL_UI_FILT_BW = Bit7 | Bit6 | Bit5 | Bit4, // 0: BW=ODR/2

	// 3:0 GYRO_UI_FILT_BW
	GYRO_UI_FILT_BW  = Bit3 | Bit2 | Bit1 | Bit0, // 0: BW=ODR/2
};

// ACCEL_CONFIG1
enum ACCEL_CONFIG1_BIT : uint8_t {
	ACCEL_UI_FILT_ORD = Bit4 | Bit3, // 00: 1st Order
};

// TMST_CONFIG
enum TMST_CONFIG_BIT : uint8_t {
	TMST_TO_REGS_EN = Bit4, // 1: TMST_VALUE[19:0] read returns timestamp value
	TMST_RES	= Bit3, // 0: 1us resolution, 1: 16us resolution or 1 RTC period when clock is used
	TMST_DELTA_EN   = Bit2, // 1: Time Stamp delta enable
	TMST_FSYNC_EN   = Bit1, // 1: The contents of the Timestamp feature of FSYNC is enabled
	TMST_EN		= Bit0, // 1: Time Stamp register enable (default)
};

// FIFO_CONFIG1
enum FIFO_CONFIG1_BIT : uint8_t {
	FIFO_RESUME_PARTIAL_RD = Bit6,
	FIFO_WM_GT_TH          = Bit5,
	FIFO_HIRES_EN          = Bit4,
	FIFO_TMST_FSYNC_EN     = Bit3,
	FIFO_TEMP_EN           = Bit2,
	FIFO_GYRO_EN           = Bit1,
	FIFO_ACCEL_EN          = Bit0,
};

// INT_CONFIG0
enum INT_CONFIG0_BIT : uint8_t {
	// 3:2 FIFO_THS_INT_CLEAR
	CLEAR_ON_FIFO_READ = Bit3,
};

// INT_CONFIG1
enum INT_CONFIG1_BIT : uint8_t {
	INT_ASYNC_RESET = Bit4, // User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation
};

// INT_SOURCE0
enum INT_SOURCE0_BIT : uint8_t {
	UI_FSYNC_INT1_EN   = Bit6,
	PLL_RDY_INT1_EN    = Bit5,
	RESET_DONE_INT1_EN = Bit4,
	UI_DRDY_INT1_EN    = Bit3,
	FIFO_THS_INT1_EN   = Bit2, // FIFO threshold interrupt routed to INT1
	FIFO_FULL_INT1_EN  = Bit1,
	UI_AGC_RDY_INT1_EN = Bit0,
};

// REG_BANK_SEL
enum REG_BANK_SEL_BIT : uint8_t {
	// 2:0 BANK_SEL
	BANK_SEL_0 = 0,           // 000: Bank 0 (default)
	BANK_SEL_1 = Bit0,        // 001: Bank 1
	BANK_SEL_2 = Bit1,        // 010: Bank 2
	BANK_SEL_3 = Bit1 | Bit0, // 011: Bank 3
	BANK_SEL_4 = Bit2,        // 100: Bank 4
};


//---------------- BANK1 Register bits

// GYRO_CONFIG_STATIC2
enum GYRO_CONFIG_STATIC2_BIT : uint8_t {
	GYRO_AAF_DIS = Bit1, // 1: Disable gyroscope anti-aliasing filter
	GYRO_NF_DIS  = Bit0, // 1: Disable Notch Filter
};

// GYRO_CONFIG_STATIC3
enum GYRO_CONFIG_STATIC3_BIT : uint8_t {
	// 5:0 GYRO_AAF_DELT
	//  585 Hz = 13 (0b00'1101)
	GYRO_AAF_DELT_585HZ_SET   = Bit3 | Bit2 | Bit0,
	GYRO_AAF_DELT_585HZ_CLEAR = Bit5 | Bit4 | Bit1,
};

// GYRO_CONFIG_STATIC4
enum GYRO_CONFIG_STATIC4_BIT : uint8_t {
	// 7:0 GYRO_AAF_DELTSQR
	//  585 Hz = 170 (0b1010'1010)
	GYRO_AAF_DELTSQR_LSB_585HZ_SET   = Bit7 | Bit5 | Bit3 | Bit1,
	GYRO_AAF_DELTSQR_LSB_585HZ_CLEAR = Bit6 | Bit4 | Bit2 | Bit0,
};

// GYRO_CONFIG_STATIC5
enum GYRO_CONFIG_STATIC5_BIT : uint8_t {
	// 7:4 GYRO_AAF_BITSHIFT
	//  585 Hz = 8 (0b1000)
	GYRO_AAF_BITSHIFT_585HZ_SET      = Bit7,
	GYRO_AAF_BITSHIFT_585HZ_CLEAR    = Bit6 | Bit5 | Bit4,

	// 3:0 GYRO_AAF_DELTSQR[11:8]
	//  585 Hz = 170 (0b0000'1010'1010)
	GYRO_AAF_DELTSQR_MSB_585HZ_SET   = 0,
	GYRO_AAF_DELTSQR_MSB_585HZ_CLEAR = Bit3 | Bit2 | Bit1 | Bit0,
};

// INTF_CONFIG5
enum INTF_CONFIG5_BIT : uint8_t {
	// 2:1 PIN9_FUNCTION
	PIN9_FUNCTION_CLKIN_SET   = Bit2, // 0b10: CLKIN
	PIN9_FUNCTION_CLKIN_CLEAR = Bit1,

	PIN9_FUNCTION_RESET_SET   = 0,
	PIN9_FUNCTION_RESET_CLEAR = Bit2 | Bit1,
};

//---------------- BANK2 Register bits

// ACCEL_CONFIG_STATIC2
enum ACCEL_CONFIG_STATIC2_BIT : uint8_t {
	// 6:1 ACCEL_AAF_DELT
	//  585 Hz = 13 (0b00'1101)
	ACCEL_AAF_DELT_585HZ_SET   = Bit4 | Bit3 | Bit1,
	ACCEL_AAF_DELT_585HZ_CLEAR = Bit6 | Bit5 | Bit2,

	// 0 ACCEL_AAF_DIS
	ACCEL_AAF_DIS              = Bit0, // 0: Enable accelerometer anti-aliasing filter (default)
};

// ACCEL_CONFIG_STATIC3
enum ACCEL_CONFIG_STATIC3_BIT : uint8_t {
	// 7:0 ACCEL_AAF_DELTSQR[7:0]
	//  585 Hz = 170 (0b0000'1010'1010)
	ACCEL_AAF_DELTSQR_LSB_585HZ_SET   = Bit7 | Bit5 | Bit3 | Bit1,
	ACCEL_AAF_DELTSQR_LSB_585HZ_CLEAR = Bit6 | Bit4 | Bit2 | Bit0,
};

// ACCEL_CONFIG_STATIC4
enum ACCEL_CONFIG_STATIC4_BIT : uint8_t {
	// 7:4 ACCEL_AAF_BITSHIFT
	//  585 Hz = 8 (0b1000)
	ACCEL_AAF_BITSHIFT_585HZ_SET   = Bit7,
	ACCEL_AAF_BITSHIFT_585HZ_CLEAR = Bit6 | Bit5 | Bit4,

	// 3:0 ACCEL_AAF_DELTSQR[11:8]
	//  585 Hz = 170 (0b0000'1010'1010)
	ACCEL_AAF_DELTSQR_MSB_SET      = 0,
	ACCEL_AAF_DELTSQR_MSB_CLEAR    = Bit3 | Bit2 | Bit1 | Bit0,
};

namespace FIFO
{
static constexpr size_t SIZE = 2048;

// FIFO_DATA layout when FIFO_CONFIG1 has FIFO_GYRO_EN and FIFO_ACCEL_EN set

// Packet 4
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
	uint8_t TEMP_DATA1;    // Temperature[15:8]
	uint8_t TEMP_DATA0;    // Temperature[7:0]
	uint8_t TimeStamp_h;   // TimeStamp[15:8]
	uint8_t TimeStamp_l;   // TimeStamp[7:0]
	uint8_t Ext_Accel_X_Gyro_X; // Accel X [3:0] Gyro X [3:0]
	uint8_t Ext_Accel_Y_Gyro_Y; // Accel Y [3:0] Gyro Y [3:0]
	uint8_t Ext_Accel_Z_Gyro_Z; // Accel Z [3:0] Gyro Z [3:0]
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
