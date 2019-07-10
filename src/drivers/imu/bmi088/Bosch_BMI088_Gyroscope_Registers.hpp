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

static constexpr uint8_t ID = 0x0F;

enum class
Register : uint8_t {

	GYRO_CHIP_ID		= 0x00,

	FIFO_STATUS		= 0x0E,
	GYRO_RANGE		= 0x0F,
	GYRO_BANDWIDTH		= 0x10,

	GYRO_SOFTRESET		= 0x14,
	GYRO_INT_CTRL		= 0x15,

	INT3_INT4_IO_MAP	= 0x18,

	FIFO_WM_ENABLE		= 0x1E,

	FIFO_EXT_INT_S		= 0x34,

	FIFO_CONFIG_0		= 0x3D,
	FIFO_CONFIG_1		= 0x3E,
	FIFO_DATA		= 0x3F,
};

namespace FIFO
{
static constexpr size_t SIZE = 1024;

struct DATA {
	uint8_t Header;
	uint8_t RATE_X_LSB;
	uint8_t RATE_X_MSB;
	uint8_t RATE_Y_LSB;
	uint8_t RATE_Y_MSB;
	uint8_t RATE_Z_LSB;
	uint8_t RATE_Z_MSB;
};
static_assert(sizeof(DATA) == 7);

} // namespace FIFO
} // namespace Bosch_BMI088_Gyroscope
