/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file Analog_Devices_ADIS16507_registers.hpp
 *
 * Analog Devices ADIS16507 registers.
 *
 */

#pragma once

#include <cstdint>

namespace Murata_SCH16T
{
static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000;       // 2 MHz SPI serial interface
static constexpr uint32_t SPI_SPEED_BURST = 1 * 1000 * 1000; // 1 MHz SPI serial interface for burst read
static constexpr uint32_t SPI_STALL_PERIOD = 16; // 16 us Stall period between data

static constexpr uint32_t SAMPLE_INTERVAL_US = 678; // 1500 Hz -- decimation factor 8, F_PRIM/16, 1.475 kHz

#define CTRL_RATE_ADDR 0x28

#define CTRL_FILT_RATE 	0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3 	0x27 // 9 bit
#define CTRL_RATE 		0x28 // 15 bit
#define CTRL_ACC12 		0x29 // 15 bit
#define CTRL_ACC3 		0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST 		0x34 // 13 bit
#define CTRL_MODE 		0x35 // 4 bit
#define CTRL_RESET 		0x36 // 4 bit

#define ASIC_ID      	0x3B // 12 bit
#define COMP_ID      	0x3C // 16 bit

#define SN_ID1      	0x3D // 16 bit
#define SN_ID2      	0x3E // 16 bit
#define SN_ID3     		0x3F // 16 bit

#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM    	0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit




// Defines the Target Address in SCH16T.
// TA[9:8] bits are used as Chip Select information, and thus they are not part of the effective address.
// TA[7:0] are used as effective address within the chip.

enum class Register : uint64_t {
	REQ48_READ_SN_ID1 = 0x0F4800000065UL,
	REQ48_READ_SN_ID2 = 0x0F8800000053UL,
	REQ48_READ_SN_ID3 = 0x0FC8000000A4UL,

	// Register operations
	REQ48_WRITE_FILTER_13HZ_RATE   = 0x096800009205UL,  // Set 13 Hz filter for rate (LPF2)
	REQ48_WRITE_FILTER_13HZ_ACC12  = 0x09A800009233UL,  // Set 13 Hz filter for acceleration (LPF2)
	REQ48_WRITE_CTRL_RATE_DEC3     = 0x0A28002492B5UL,  // Set RATE range to 315 dps, decimation factor 4 (DYN2, DEC3)
	REQ48_WRITE_CTRL_ACC12_DEC3    = 0x0A6800129205UL,  // Set ACC range to 80 m/s2, decimation factor 4 (DYN1, DEC3)
	REQ48_WRITE_EN_SENSOR          = 0x0D68000001D3UL,  // Enable RATE and ACC measurement

};

} // namespace Analog_Devices_ADIS16507
