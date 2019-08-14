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

namespace Bosch_BMI088_Accelerometer
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

static constexpr uint8_t ID = 0x1E;

enum class
Register : uint8_t {

	ACC_CHIP_ID		= 0x00,

	ACC_ERR_REG		= 0x02,

	TEMP_MSB		= 0x22,
	TEMP_LSB		= 0x23,

	FIFO_LENGTH_0		= 0x24,
	FIFO_LENGTH_1		= 0x25,

	FIFO_DATA		= 0x26,

	ACC_CONF		= 0x40,
	ACC_RANGE		= 0x41,

	FIFO_WTM_0		= 0x46,
	FIFO_WTM_1		= 0x47,
	FIFO_CONFIG_0		= 0x48,
	FIFO_CONFIG_1		= 0x49,

	INT1_IO_CONF		= 0x53,
	INT2_IO_CONF		= 0x54,

	INT1_INT2_MAP_DATA	= 0x58,

	ACC_PWR_CONF		= 0x7C,
	ACC_PWR_CTRL		= 0x7D,
	ACC_SOFTRESET		= 0x7E,
};

// ACC_ERR_REG
enum
ACC_ERR_REG_BIT : uint8_t {
	error_code	= Bit2,

	fatal_err	= Bit0,
};

// FIFO_CONFIG_1
enum
FIFO_CONFIG_1_BIT : uint8_t {
	Acc_en	= Bit6,

	Int1_en	= Bit3,
	Int2_en	= Bit2,
};

// INT1_IO_CONF
enum
INT1_IO_CONF_BIT : uint8_t {
	int1_in		= Bit4,
	int1_out	= Bit3,

	int1_lvl	= Bit1,
};

// INT2_IO_CONF
enum
INT2_IO_CONF_BIT : uint8_t {
	int2_io		= Bit4,
	int2_out	= Bit3,
	int2_od		= Bit2,
	int2_lvl	= Bit1,
};

// INT1_INT2_MAP_DATA
enum
INT1_INT2_MAP_DATA_BIT : uint8_t {
	int2_drdy	= Bit6,
	int2_fwm	= Bit5,
	int2_ffull	= Bit4,

	int1_drdy	= Bit2,
	int1_fwm	= Bit1,
	int1_ffull	= Bit0,
};


namespace FIFO
{
static constexpr size_t SIZE = 1024;

// 1. Acceleration sensor data frame	- Frame length: 7 bytes (1 byte header + 6 bytes payload)
// Payload: the next bytes contain the sensor data in the same order as defined in the register map (addresses 0x12 – 0x17).
// 2. Skip Frame				- Frame length: 2 bytes (1 byte header + 1 byte payload)
// Payload: one byte containing the number of skipped frames. When more than 0xFF frames have been skipped, 0xFF is returned.
// 3. Sensortime Frame			- Frame length: 4 bytes (1 byte header + 3 bytes payload)
// Payload: Sensortime (content of registers 0x18 – 0x1A), taken when the last byte of the last frame is read.

struct DATA {
	uint8_t Header;
	uint8_t ACC_X_LSB;
	uint8_t ACC_X_MSB;
	uint8_t ACC_Y_LSB;
	uint8_t ACC_Y_MSB;
	uint8_t ACC_Z_LSB;
	uint8_t ACC_Z_MSB;
};
static_assert(sizeof(DATA) == 7);

enum header : uint8_t {
	sensor_data_frame	= 0b10000100,
	skip_frame		= 0b01000000,
	sensor_time_frame	= 0b01000100,
	FIFO_input_config_frame	= 0b01001000,
	sample_drop_frame	= 0b01010000,
};

} // namespace FIFO
} // namespace Bosch_BMI088_Accelerometer
