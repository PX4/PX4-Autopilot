/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <cstdint>

namespace Murata_SCH16T
{
// General definitions
static constexpr uint16_t EOI = (1 << 1);               // End of Initialization
static constexpr uint16_t EN_SENSOR = (1 << 0);         // Enable RATE and ACC measurement
static constexpr uint16_t DRY_DRV_EN = (1 << 5);        // Enables Data ready function
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);

// Filter settings
static constexpr uint8_t FILTER_13_HZ = (0b010);
static constexpr uint8_t FILTER_30_HZ = (0b001);
static constexpr uint8_t FILTER_68_HZ = (0b000);
static constexpr uint8_t FILTER_280_HZ = (0b011);
static constexpr uint8_t FILTER_370_HZ = (0b100);
static constexpr uint8_t FILTER_235_HZ = (0b101);
static constexpr uint8_t FILTER_BYPASS = (0b111);

// Dynamic range settings
static constexpr uint8_t RATE_RANGE_300 = (0b001);
static constexpr uint8_t ACC12_RANGE_80 = (0b001);
static constexpr uint8_t ACC3_RANGE_260 = (0b000);

// Decimation ratio settings
static constexpr uint8_t DECIMATION_NONE =	(0b000);
static constexpr uint8_t DECIMATION_5900_HZ =	(0b001);
static constexpr uint8_t DECIMATION_2950_HZ =	(0b010);
static constexpr uint8_t DECIMATION_1475_HZ =	(0b011);
static constexpr uint8_t DECIMATION_738_HZ =	(0b100);

union CTRL_FILT_RATE_Register {
	struct {
		uint16_t FILT_SEL_RATE_X  : 3;
		uint16_t FILT_SEL_RATE_Y  : 3;
		uint16_t FILT_SEL_RATE_Z  : 3;
		uint16_t reserved         : 7;
	} bits;

	uint16_t value;
};

union CTRL_FILT_ACC12_Register {
	struct {
		uint16_t FILT_SEL_ACC_X12  : 3;
		uint16_t FILT_SEL_ACC_Y12  : 3;
		uint16_t FILT_SEL_ACC_Z12  : 3;
		uint16_t reserved         : 7;
	} bits;

	uint16_t value;
};

union CTRL_FILT_ACC3_Register {
	struct {
		uint16_t FILT_SEL_ACC_X3  : 3;
		uint16_t FILT_SEL_ACC_Y3  : 3;
		uint16_t FILT_SEL_ACC_Z3  : 3;
		uint16_t reserved         : 7;
	} bits;

	uint16_t value;
};

union RATE_CTRL_Register {
	struct {
		uint16_t DEC_RATE_X2  : 3;
		uint16_t DEC_RATE_Y2  : 3;
		uint16_t DEC_RATE_Z2  : 3;
		uint16_t DYN_RATE_XYZ2: 3;
		uint16_t DYN_RATE_XYZ1: 3;
		uint16_t reserved     : 1;
	} bits;

	uint16_t value;
};

union ACC12_CTRL_Register {
	struct {
		uint16_t DEC_ACC_X2  : 3;
		uint16_t DEC_ACC_Y2  : 3;
		uint16_t DEC_ACC_Z2  : 3;
		uint16_t DYN_ACC_XYZ2: 3;
		uint16_t DYN_ACC_XYZ1: 3;
		uint16_t reserved     : 1;
	} bits;

	uint16_t value;
};

union ACC3_CTRL_Register {
	struct {
		uint16_t DYN_ACC_XYZ3  : 3;
		uint16_t reserved     : 13;
	} bits;

	uint16_t value;
};

// Data registers
#define RATE_X1         0x01 // 20 bit
#define RATE_Y1         0x02 // 20 bit
#define RATE_Z1         0x03 // 20 bit
#define ACC_X1          0x04 // 20 bit
#define ACC_Y1          0x05 // 20 bit
#define ACC_Z1          0x06 // 20 bit
#define ACC_X3          0x07 // 20 bit
#define ACC_Y3          0x08 // 20 bit
#define ACC_Z3          0x09 // 20 bit
#define RATE_X2         0x0A // 20 bit
#define RATE_Y2         0x0B // 20 bit
#define RATE_Z2         0x0C // 20 bit
#define ACC_X2          0x0D // 20 bit
#define ACC_Y2          0x0E // 20 bit
#define ACC_Z2          0x0F // 20 bit
#define TEMP            0x10 // 16 bit
// Status registers
#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM        0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit
// Control registers
#define CTRL_FILT_RATE  0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3  0x27 // 9 bit
#define CTRL_RATE       0x28 // 15 bit
#define CTRL_ACC12      0x29 // 15 bit
#define CTRL_ACC3       0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST         0x34 // 13 bit
#define CTRL_MODE       0x35 // 4 bit
#define CTRL_RESET      0x36 // 4 bit
// Misc registers
#define ASIC_ID         0x3B // 12 bit
#define COMP_ID         0x3C // 16 bit
#define SN_ID1          0x3D // 16 bit
#define SN_ID2          0x3E // 16 bit
#define SN_ID3          0x3F // 16 bit

// STAT_SUM_SAT bits
#define STAT_SUM_SAT_RSVD    (1 << 15)
#define STAT_SUM_SAT_RATE_X1 (1 << 14)
#define STAT_SUM_SAT_RATE_Y1 (1 << 13)
#define STAT_SUM_SAT_RATE_Z1 (1 << 12)
#define STAT_SUM_SAT_ACC_X1  (1 << 11)
#define STAT_SUM_SAT_ACC_Y1  (1 << 10)
#define STAT_SUM_SAT_ACC_Z1  (1 << 9)
#define STAT_SUM_SAT_ACC_X3  (1 << 8)
#define STAT_SUM_SAT_ACC_Y3  (1 << 7)
#define STAT_SUM_SAT_ACC_Z3  (1 << 6)
#define STAT_SUM_SAT_RATE_X2 (1 << 5)
#define STAT_SUM_SAT_RATE_Y2 (1 << 4)
#define STAT_SUM_SAT_RATE_Z2 (1 << 3)
#define STAT_SUM_SAT_ACC_X2  (1 << 2)
#define STAT_SUM_SAT_ACC_Y2  (1 << 1)
#define STAT_SUM_SAT_ACC_Z2  (1 << 0)

} // namespace Murata_SCH16T
