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

// TODO: move to a central header
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

namespace Analog_Devices_ADIS16507
{
static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000;       // 2 MHz SPI serial interface
static constexpr uint32_t SPI_SPEED_BURST = 1 * 1000 * 1000; // 1 MHz SPI serial interface for burst read
static constexpr uint32_t SPI_STALL_PERIOD = 16; // 16 us Stall period between data

static constexpr uint32_t SAMPLE_INTERVAL_US = 500; // 2000 Hz

static constexpr uint16_t DIR_WRITE = 0x80;

static constexpr uint16_t BURST_READ_CMD = 0x6800;


namespace Register
{
static constexpr uint16_t DIAG_STAT = 0x02;
static constexpr uint16_t FILT_CTRL = 0x5C;
static constexpr uint16_t RANG_MDL = 0x5E;
static constexpr uint16_t MSC_CTRL = 0x60;
static constexpr uint16_t GLOB_CMD = 0x68;
static constexpr uint16_t FIRM_REV = 0x6C; // Identification, firmware revision
static constexpr uint16_t FIRM_DM = 0x6E; // Identification, date code, day and month
static constexpr uint16_t FIRM_Y = 0x70; // Identification, date code, year
static constexpr uint16_t PROD_ID = 0x72; // Identification, part number
static constexpr uint16_t SERIAL_NUM = 0x74; // Identification, serial number
static constexpr uint16_t FLSHCNT_LOW = 0x7C; // Output, flash memory write cycle counter, lower word
static constexpr uint16_t FLSHCNT_HIGH = 0x7E; // Output, flash memory write cycle counter, upper word

static constexpr uint16_t PROD_ID_EXPECTED = 0x407B;

// DIAG_STAT
enum DIAG_STAT_BIT : uint16_t {
	Accelerometer_failure       = Bit10, // A 1 indicates failure of the accelerometer at the conclusion of the self test
	Gyro_2_failure              = Bit9,  // A 1 indicates failure of Gyro 2 at the conclusion of the self test
	Gyro_1_failure              = Bit8,  // A 1 indicates failure of Gyro 1 at the conclusion of the self test
	Clock_error                 = Bit7,  // A 1 indicates that the internal data sampling clock
	Memory_failure              = Bit6,  // A 1 indicates a failure in the flash memory test
	Sensor_failure              = Bit5,  // A 1 indicates failure of at least one sensor, at the conclusion of the self test
	Standby_mode                = Bit4,  // A 1 indicates that the voltage across VDD and GND is <2.8 V, which causes data processing to stop
	SPI_communication_error     = Bit3,  // A 1 indicates that the total number of SCLK cycles is not equal to an integer multiple of 16
	Flash_memory_update_failure = Bit2,  // A 1 indicates that the most recent flash memory update failed
	Data_path_overrun           = Bit1,  // A 1 indicates that one of the data paths have experienced an overrun condition
};

// FILT_CTRL
enum FILT_CTRL_BIT : uint16_t {

};

// MSC_CTRL
enum MSC_CTRL_BIT : uint16_t {
	BURST32     = Bit9, // 32-bit burst enable bit
	GYRO_COMP   = Bit7, // Linear acceleration compensation for gyroscopes
	DR_polarity = Bit0, // 1 = active high when data is valid
};

// GLOB_CMD
enum GLOB_CMD_BIT : uint16_t {
	Software_reset    = Bit7,
	Flash_memory_test = Bit4,
	Sensor_self_test  = Bit2,
};

} // namespace Register

} // namespace Analog_Devices_ADIS16507
