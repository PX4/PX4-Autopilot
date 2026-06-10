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
 * @file InvenSense_IIM20670_registers.hpp
 *
 * Invensense IIM-20670 registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace InvenSense_IIM20670
{
static constexpr uint16_t Bit0  = (1 << 0);
static constexpr uint16_t Bit1  = (1 << 1);
static constexpr uint16_t Bit2  = (1 << 2);
static constexpr uint16_t Bit3  = (1 << 3);
static constexpr uint16_t Bit4  = (1 << 4);
static constexpr uint16_t Bit5  = (1 << 5);
static constexpr uint16_t Bit6  = (1 << 6);
static constexpr uint16_t Bit7  = (1 << 7);
static constexpr uint16_t Bit8  = (1 << 8);
static constexpr uint16_t Bit9  = (1 << 9);
static constexpr uint16_t Bit10 = (1 << 10);
static constexpr uint16_t Bit11 = (1 << 11);
static constexpr uint16_t Bit12 = (1 << 12);
static constexpr uint16_t Bit13 = (1 << 13);
static constexpr uint16_t Bit14 = (1 << 14);
static constexpr uint16_t Bit15 = (1 << 15);

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI

static constexpr uint8_t WHOAMI = 0xF3;            // whoami[7:0] (Bank 1, 0x0E)
static constexpr uint16_t FIXED_VALUE = 0xAA55;    // fixed_value[15:0] (Bank 0, 0x0B)

static constexpr float TEMPERATURE_SENSITIVITY = 20.f; // LSB/°C
static constexpr float TEMPERATURE_OFFSET = 25.f;      // °C

// 32-bit SPI protocol:
//  MOSI [RW | A4:A0 | 00       | D15:D0 | CRC7:CRC0]
//  MISO [RW | A4:A0 | RS1:RS0  | D15:D0 | CRC7:CRC0]
// Register reads are out-of-frame: the response to a command is returned
// during the following SPI transfer.
static constexpr uint32_t DIR_WRITE = (1u << 31);

// Return status bits (RS1:RS0)
enum RETURN_STATUS : uint8_t {
	RS_RESERVED    = 0b00,
	RS_SUCCESS     = 0b01,
	RS_IN_PROGRESS = 0b10, // data not prepared, or sensor data read during self-test
	RS_ERROR       = 0b11,
};

// Security feature unlock sequence (full-scale change and ODR pin routing),
// sent verbatim over SPI while in Bank 0 (see datasheet sections 4.11 and 6.17)
static constexpr uint32_t UNLOCK_SEQUENCE[] {
	0xE4000288,
	0xE400018B,
	0xE400048E,
	0xE40300AD,
	0xE4018017,
	0xE4028030,
};

namespace Register
{

enum class BANK_0 : uint8_t {
	GYRO_X_DATA     = 0x00,
	GYRO_Y_DATA     = 0x01,
	GYRO_Z_DATA     = 0x02,
	TEMP1_DATA      = 0x03,
	ACCEL_X_DATA    = 0x04,
	ACCEL_Y_DATA    = 0x05,
	ACCEL_Z_DATA    = 0x06,
	TEMP2_DATA      = 0x07,
	ACCEL_X_DATA_LR = 0x08,
	ACCEL_Y_DATA_LR = 0x09,
	ACCEL_Z_DATA_LR = 0x0A,
	FIXED_VALUE_REG = 0x0B,
	FLT_YZ          = 0x0C,
	FLT_X           = 0x0E,
	TEMP12_DELTA    = 0x0F,
	SELF_TEST       = 0x16,
	TEST            = 0x17,
	RESET_CONTROL   = 0x18,
	MODE            = 0x19,
	BANK_SELECT     = 0x1F,
};

enum class BANK_1 : uint8_t {
	WHOAMI = 0x0E,
};

enum class BANK_3 : uint8_t {
	ODR_CONFIG_1 = 0x11, // bits 13:8
	ODR_CONFIG_2 = 0x13, // bits 7:4
	ODR_CONFIG_3 = 0x14, // bits 5 and 9
	ODR_CONFIG_5 = 0x16, // bit 0
	ODR_CONFIG_6 = 0x17, // bit 12
};

enum class BANK_6 : uint8_t {
	SENSITIVITY_CONFIG = 0x14, // accel_fs_sel[2:0]
};

enum class BANK_7 : uint8_t {
	SENSITIVITY_CONFIG = 0x14, // gyro_fs_sel[3:0]
};

};

//---------------- BANK0 Register bits

// FLT_YZ (flt_y[5:0], flt_z[11:6]) and FLT_X (flt_x[13:8])
// filter setting 0b101010: gyro 60 Hz / accel 400 Hz cut-off (widest bandwidth)
static constexpr uint16_t FLT_GYRO_60HZ_ACCEL_400HZ = 0b101010;

// RESET_CONTROL
enum RESET_CONTROL_BIT : uint16_t {
	hard_reset = Bit2,
	soft_reset = Bit1,
};

//---------------- BANK6/BANK7 Register bits

// SENSITIVITY_CONFIG (Bank 6) accel_fs_sel[2:0]
enum ACCEL_FS_SEL_BIT : uint16_t {
	// accel_fs_sel[2:0] = 001: FS ±16.384 g, FS_LR ±65.536 g
	ACCEL_FS_SEL_16G_SET   = Bit0,
	ACCEL_FS_SEL_16G_CLEAR = Bit2 | Bit1,
	// accel_fs_sel[2:0] = 011: FS ±32.768 g, FS_LR ±65.536 g
	ACCEL_FS_SEL_32G_SET   = Bit1 | Bit0,
	ACCEL_FS_SEL_32G_CLEAR = Bit2,
};

// SENSITIVITY_CONFIG (Bank 7) gyro_fs_sel[3:0] = 0011: FS ±1966 dps
enum GYRO_FS_SEL_BIT : uint16_t {
	GYRO_FS_SEL_1966DPS_SET   = Bit1 | Bit0,
	GYRO_FS_SEL_1966DPS_CLEAR = Bit3 | Bit2,
};

} // namespace InvenSense_IIM20670
