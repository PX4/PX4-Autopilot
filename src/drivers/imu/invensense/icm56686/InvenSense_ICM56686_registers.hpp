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
 * @file InvenSense_ICM56686_registers.hpp
 *
 * InvenSense ICM-56686 registers.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace InvenSense_ICM56686
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

// Staying below 10 MHz avoids the additional FIFO data-to-count tBUF requirement.
static constexpr uint32_t SPI_SPEED = 8 * 1000 * 1000;
static constexpr uint8_t DIR_READ = Bit7;

static constexpr uint8_t WHOAMI = 0x08;

// Sensitivity of the 16 bit temperature data, which is what the 20 byte high resolution FIFO packet
// carries. The 8 bit temperature of the 8/16 byte packets uses 2 LSB/degC instead.
static constexpr float TEMPERATURE_SENSITIVITY = 128.f; // LSB/degC
static constexpr float TEMPERATURE_OFFSET = 25.f; // degC

namespace Register
{

enum class BANK_0 : uint8_t {
	PWR_MGMT0 = 0x14,
	FIFO_COUNT_0 = 0x16,
	FIFO_COUNT_1 = 0x17,
	FIFO_DATA = 0x18,
	INT1_CONFIG0 = 0x1A,
	INT1_CONFIG2 = 0x1C,
	INT1_STATUS0 = 0x1D,
	ACCEL_CONFIG0 = 0x1F,
	GYRO_CONFIG0 = 0x20,
	FIFO_CONFIG0 = 0x21,
	FIFO_CONFIG1_0 = 0x22,
	FIFO_CONFIG1_1 = 0x23,
	FIFO_CONFIG2 = 0x24,
	FIFO_CONFIG3 = 0x25,
	FIFO_CONFIG4 = 0x26,
	INTF_CONFIG1_OVRD = 0x31,
	DRIVE_CONFIG0 = 0x36,
	PWR_MGMT_AUX1 = 0x58,
	INT2_CONFIG0 = 0x5A,
	INT2_CONFIG2 = 0x5C,
	WHO_AM_I = 0x72,
	IREG_ADDR_15_8 = 0x7C,
	IREG_ADDR_7_0 = 0x7D,
	IREG_DATA = 0x7E,
	REG_MISC2 = 0x7F,
};

} // namespace Register

namespace IREG
{

static constexpr uint16_t SMC_CONTROL_0 = 0xA258;
static constexpr uint16_t SREG_CTRL = 0xA260;
static constexpr uint16_t INT_PULSE_MIN_ON_INTF0 = 0xA262;
static constexpr uint16_t INT_PULSE_MIN_OFF_INTF0 = 0xA264;
static constexpr uint16_t GYRO_SRC_CTRL = 0xA49A;
static constexpr uint16_t GYRO_UI_LPF_CFG = 0xA49E;
static constexpr uint16_t ACCEL_SRC_CTRL = 0xA56D;
static constexpr uint16_t ACCEL_UI_LPF_CFG = 0xA570;

} // namespace IREG

enum PWR_MGMT0_BIT : uint8_t {
	GYRO_MODE_LOW_NOISE = Bit3 | Bit2,
	ACCEL_MODE_LOW_NOISE = Bit1 | Bit0,
};

enum INT_CONFIG0_BIT : uint8_t {
	INT1_STATUS_EN_RESET_DONE = Bit7,
	STATUS_EN_FIFO_THS = Bit1,
	STATUS_EN_FIFO_FULL = Bit0,
};

enum INT_CONFIG2_BIT : uint8_t {
	ACTIVE_HIGH = Bit0,
	CONFIG_MASK = Bit2 | Bit1 | Bit0,
};

enum INT1_STATUS0_BIT : uint8_t {
	INT1_STATUS_RESET_DONE = Bit7,
	INT1_STATUS_FIFO_THS = Bit1,
	INT1_STATUS_FIFO_FULL = Bit0,
};

enum ACCEL_CONFIG0_BIT : uint8_t {
	ACCEL_UI_FS_SEL_32_G = 0,
	ACCEL_ODR_6400_HZ = Bit1 | Bit0,
	ACCEL_CONFIG0_MASK = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

enum GYRO_CONFIG0_BIT : uint8_t {
	GYRO_UI_FS_SEL_4000_DPS = 0,
	GYRO_ODR_6400_HZ = Bit1 | Bit0,
	GYRO_CONFIG0_MASK = 0xFF,
};

enum FIFO_CONFIG0_BIT : uint8_t {
	FIFO_MODE_BYPASS = 0,
	FIFO_MODE_STOP_ON_FULL = Bit7,
	FIFO_MODE_MASK = Bit7 | Bit6,
	FIFO_DEPTH_2K = Bit2 | Bit1 | Bit0,
	FIFO_DEPTH_MASK = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

enum FIFO_CONFIG2_BIT : uint8_t {
	FIFO_FLUSH = Bit7,
	FIFO_RESERVED_RESET = Bit5, // reset 0x20, bits [6:5] reserved (DS-000563 v1.1 §23.34)
	FIFO_INT_OVFL = Bit4,
	FIFO_WR_WM_GT_TH = Bit3,
	FIFO_CONFIG2_MASK = Bit6 | Bit5 | Bit4 | Bit3,
};

enum FIFO_CONFIG3_BIT : uint8_t {
	FIFO_HIRES_EN = Bit3,
	FIFO_GYRO_EN = Bit2,
	FIFO_ACCEL_EN = Bit1,
	FIFO_IF_EN = Bit0,
	FIFO_CONFIG3_MASK = Bit3 | Bit2 | Bit1 | Bit0,
};

enum FIFO_CONFIG4_BIT : uint8_t {
	FIFO_COMP_NC_FLOW_CFG_MASK = Bit4 | Bit3 | Bit2,
	FIFO_COMP_EN = Bit1,
	FIFO_TMST_FSYNC_EN = Bit0,
	FIFO_CONFIG4_MASK = FIFO_COMP_NC_FLOW_CFG_MASK | FIFO_COMP_EN | FIFO_TMST_FSYNC_EN,
};

enum REG_MISC2_BIT : uint8_t {
	SOFT_RST = Bit1,
	IREG_DONE = Bit0,
};

enum INTF0_PULSE_BIT : uint8_t {
	DURATION_8_US = Bit0,
	DURATION_MASK = Bit2 | Bit1 | Bit0,
};

enum SREG_CTRL_BIT : uint8_t {
	SREG_SIFS_20BITS_EN = Bit3,
	SREG_DATA_ENDIAN_BIG = Bit1,
	SREG_CTRL_MASK = Bit3 | Bit1,
};

enum SMC_CONTROL_0_BIT : uint8_t {
	TMST_EN = Bit0,
};

enum GYRO_SRC_CTRL_BIT : uint8_t {
	GYRO_SRC_CTRL_MASK = Bit3 | Bit2,
	GYRO_SRC_CTRL_SRC_ON_PREFILTER_ON = Bit3,
};

enum GYRO_UI_LPF_CFG_BIT : uint8_t {
	GYRO_UI_LPFBW_ODR_DIV_2 = 0,
	GYRO_UI_LPFBW_MASK = Bit6 | Bit5 | Bit4,
};

enum ACCEL_SRC_CTRL_BIT : uint8_t {
	ACCEL_SRC_CTRL_MASK = Bit1 | Bit0,
	ACCEL_SRC_CTRL_SRC_ON_PREFILTER_ON = Bit1,
};

enum ACCEL_UI_LPF_CFG_BIT : uint8_t {
	ACCEL_UI_LPFBW_ODR_DIV_2 = 0,
	ACCEL_UI_LPFBW_MASK = Bit2 | Bit1 | Bit0,
};

namespace FIFO
{

static constexpr size_t SIZE = 2048;

struct DATA {
	uint8_t header;
	uint8_t accel_x_l;
	uint8_t accel_x_h;
	uint8_t accel_y_l;
	uint8_t accel_y_h;
	uint8_t accel_z_l;
	uint8_t accel_z_h;
	uint8_t gyro_x_l;
	uint8_t gyro_x_h;
	uint8_t gyro_y_l;
	uint8_t gyro_y_h;
	uint8_t gyro_z_l;
	uint8_t gyro_z_h;
	uint8_t temperature_l;
	uint8_t temperature_h;
	uint8_t timestamp_l;
	uint8_t timestamp_h;
	uint8_t highres_x;
	uint8_t highres_y;
	uint8_t highres_z;
} __attribute__((packed));

static_assert(sizeof(DATA) == 20);

// The FIFO count reports the number of unread frames, a full 2K FIFO holds this many.
static constexpr uint16_t MAX_FRAMES = SIZE / sizeof(DATA);

// A sample of exactly negative full scale marks invalid sensor data.
static constexpr int32_t INVALID_SAMPLE = -524288;
static constexpr int16_t INVALID_TEMPERATURE = INT16_MIN;

static constexpr uint8_t HEADER = Bit6 | Bit5 | Bit4 | Bit3;
static constexpr uint8_t HEADER_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2;

static constexpr uint8_t accelNibble(const uint8_t extension)
{
	return (extension >> 4) & 0x0F;
}

static constexpr uint8_t gyroNibble(const uint8_t extension)
{
	return extension & 0x0F;
}

static constexpr int32_t reassemble20Bit(const uint8_t high, const uint8_t low, const uint8_t lowest)
{
	uint32_t value = (static_cast<uint32_t>(high) << 12)
			 | (static_cast<uint32_t>(low) << 4)
			 | (lowest & 0x0F);

	if (value & (1u << 19)) {
		value |= 0xFFF00000u;
	}

	return static_cast<int32_t>(value);
}

static_assert(accelNibble(0xA5) == 0x0A);
static_assert(gyroNibble(0xA5) == 0x05);
static_assert(reassemble20Bit(0x00, 0x00, 0x00) == 0);
static_assert(reassemble20Bit(0x7F, 0xFF, 0x0F) == 524287);
static_assert(reassemble20Bit(0x80, 0x00, 0x00) == INVALID_SAMPLE);
static_assert(reassemble20Bit(0xFF, 0xFF, 0x0F) == -1);

// The sensor data is byte swapped, SREG_DATA_ENDIAN_SEL selects little endian.
static constexpr int32_t accelX(const DATA &sample)
{
	return reassemble20Bit(sample.accel_x_h, sample.accel_x_l, accelNibble(sample.highres_x));
}

static constexpr int32_t accelY(const DATA &sample)
{
	return reassemble20Bit(sample.accel_y_h, sample.accel_y_l, accelNibble(sample.highres_y));
}

static constexpr int32_t accelZ(const DATA &sample)
{
	return reassemble20Bit(sample.accel_z_h, sample.accel_z_l, accelNibble(sample.highres_z));
}

static constexpr int32_t gyroX(const DATA &sample)
{
	return reassemble20Bit(sample.gyro_x_h, sample.gyro_x_l, gyroNibble(sample.highres_x));
}

static constexpr int32_t gyroY(const DATA &sample)
{
	return reassemble20Bit(sample.gyro_y_h, sample.gyro_y_l, gyroNibble(sample.highres_y));
}

static constexpr int32_t gyroZ(const DATA &sample)
{
	return reassemble20Bit(sample.gyro_z_h, sample.gyro_z_l, gyroNibble(sample.highres_z));
}

} // namespace FIFO
} // namespace InvenSense_ICM56686
