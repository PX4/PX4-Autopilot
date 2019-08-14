/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

namespace Bosch_BMI088_Gyroscope
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

static constexpr uint8_t ID = 0x0F;

enum class
Register : uint8_t {

	GYRO_CHIP_ID		= 0x00,

	INT_STATUS_1		= 0x0A,

	FIFO_STATUS		= 0x0E,
	GYRO_RANGE		= 0x0F,
	GYRO_BANDWIDTH		= 0x10,
	GYRO_LPM1		= 0x11,

	GYRO_SOFTRESET		= 0x14,
	GYRO_INT_CTRL		= 0x15,

	INT3_INT4_IO_CONF	= 0x16,

	INT3_INT4_IO_MAP	= 0x18,

	FIFO_WM_ENABLE		= 0x1E,

	FIFO_EXT_INT_S		= 0x34,

	FIFO_CONFIG_0		= 0x3D,
	FIFO_CONFIG_1		= 0x3E,
	FIFO_DATA		= 0x3F,
};

// FIFO_STATUS
enum
FIFO_STATUS_BIT : uint8_t {
	Fifo_overrun		= Bit7,
	Fifo_frame_counter	= Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

// GYRO_INT_CTRL
enum
GYRO_INT_CTRL_BIT : uint8_t {
	data_en = Bit7,
	fifo_en = Bit6,
};

// INT3_INT4_IO_CONF
enum
INT3_INT4_IO_CONF_BIT : uint8_t {
	Int4_od		= Bit3,
	Int4_lvl	= Bit2,
	Int3_od		= Bit1,
	Int3_lvl	= Bit0,
};

// INT3_INT4_IO_MAP
enum
INT3_INT4_IO_MAP_BIT : uint8_t {
	Int4_data	= Bit7,
	Int4_fifo	= Bit5,
	Int3_fifo	= Bit2,
	Int3_data	= Bit0,
};

// FIFO_WM_ENABLE
enum
FIFO_WM_ENABLE_BIT : uint8_t {
	enabled		= Bit7 | Bit3,
	disabled	= Bit3,
};

// FIFO_EXT_INT_S
enum
FIFO_EXT_INT_S_BIT : uint8_t {
	ext_fifo_s_en	= Bit5,
	ext_fifo_s_sel	= Bit4,
};

// FIFO_CONFIG_1
enum
FIFO_CONFIG_1_BIT : uint8_t {
	STREAM_MODE	= Bit7,
	FIFO_MODE	= Bit6,
};


namespace FIFO
{

struct DATA {
	uint8_t RATE_X_LSB;
	uint8_t RATE_X_MSB;
	uint8_t RATE_Y_LSB;
	uint8_t RATE_Y_MSB;
	uint8_t RATE_Z_LSB;
	uint8_t RATE_Z_MSB;
};
static_assert(sizeof(DATA) == 6);

// 100 frames of data in FIFO mode
static constexpr size_t SIZE = sizeof(DATA) * 100;

} // namespace FIFO
} // namespace Bosch_BMI088_Gyroscope
