/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file Honeywell_HMC6343_registers.hpp
 *
 * Honeywell HMC6343 registers and commands.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Honeywell_HMC6343
{
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x19; // default I2C address (0x32 >> 1 = 0x19)

enum class Register : uint8_t {
	// HMC6343 Registers
	SLAVE_ADDR	 = 0x00, // I2C Slave Address
	SW_VERSION 	 = 0x02, // Software Version Number
	OP_MODE1	 = 0x04, // Operational Mode Register 1
	OP_MODE2	 = 0x05, // Operational Mode Register 2
	SN_LSB		 = 0x06, // Device Serial Number LSB
	SN_MSB		 = 0x07, // Device Serial Number MSB
	DATE_CODE_YY	 = 0x08, // Package Date Code: Last Two Digits of the Year
	DATE_CODE_WW	 = 0x09, // Package Date Code: Fiscal Week
	DEVIATION_LSB	 = 0x0A, // Deviation Angle (±1800) in tenths of a degree LSB
	DEVIATION_MSB	 = 0x0B, // Deviation Angle (±1800) in tenths of a degree MSB
	VARIATION_LSB	 = 0x0C, // Variation Angle (±1800) in tenths of a degree LSB
	VARIATION_MSB	 = 0x0D, // Variation Angle (±1800) in tenths of a degree MSB
	XOFFSET_LSB	 = 0x0E, // Hard-Iron Calibration Offset for the X-axis LSB
	XOFFSET_MSB	 = 0x0F, // Hard-Iron Calibration Offset for the X-axis MSB
	YOFFSET_LSB	 = 0x10, // Hard-Iron Calibration Offset for the Y-axis LSB
	YOFFSET_MSB	 = 0x11, // Hard-Iron Calibration Offset for the Y-axis MSB
	ZOFFSET_LSB	 = 0x12, // Hard-Iron Calibration Offset for the Z-axis LSB
	ZOFFSET_MSB	 = 0x13, // Hard-Iron Calibration Offset for the Z-axis MSB
	FILTER_LSB	 = 0x14, // Heading IIR Filter (0x00 to 0x0F typical) LSB
	FILTER_MSB	 = 0x15, // Heading IIR Filter (set at zero) MSB
};

enum class Command : uint8_t {
	// HMC6343 Commands
	POST_ACCEL	 = 0x40, // Post Accel Data (MSB/LSB (6 Bytes))
	POST_MAG 	 = 0x45, // Post Mag Data (MSB/LSB (6 Bytes))
	POST_HEADING	 = 0x50, // Post Heading Data ((MSB/LSB (6 Bytes)))
	POST_TILT	 = 0x55, // Post Tilt Data (MSB/LSB (6 Bytes))
	POST_OPMODE1	 = 0x65, // Read Current Value of OP Mode 1
	ENTER_CAL	 = 0x71, // Enter User Calibration Mode
	ORIENT_LEVEL	 = 0x72, // Level Orientation
	ORIENT_SIDEWAYS	 = 0x73, // Upright Sideways Orientation
	ORIENT_FLATFRONT = 0x74, // Upright Flat Front Orientation
	ENTER_RUN	 = 0x75, // Enter Run Mode
	ENTER_STANDBY	 = 0x76, // Enter Standby Mode
	EXIT_CAL	 = 0x7E, // Exit User Calibration Mode
	RESET		 = 0x82, // Reset the Processor
	ENTER_SLEEP	 = 0x83, // Enter Sleep Mode
	EXIT_SLEEP	 = 0x84, // Exit Sleep Mode
	READ_EEPROM	 = 0xE1, // Read from EEPROM
	WRITE_EEPROM	 = 0xF1, // Write to EEPROM
};

// OP_Mode1
enum OP_Mode1_BIT : uint8_t {
	Filter = Bit5, // IIR Heading Filter used if set
	Run    = Bit4, // Run Mode if set
	Stdby  = Bit3, // Standby Mode if set
	UF     = Bit2, // Upright Front Orientation if set
	UE     = Bit1, // Upright Edge Orientation if set
	LEVEL  = Bit0, // Level Orientation if set
};

// OP_Mode2
enum OP_Mode2_BIT : uint8_t {
	MR1 = Bit1, // Measurement Rate
	MR0 = Bit0, // 0,0 = 1 Hz 0,1 = 5 Hz (default) 1,0 = 10 Hz 1,1 = Not Assigned
};

} // namespace Honeywell_HMC6343
