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
 * @file Analog_Devices_ADIS16607_registers.hpp
 *
 * Analog Devices ADIS16607 registers.
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

namespace Analog_Devices_ADIS16607
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000;       // 10 MHz SPI serial interface

static constexpr uint16_t DIR_READ = 0x80;

static constexpr uint16_t device_identification = 0x6000;

static constexpr uint32_t SAMPLE_INTERVAL_US = 125; // 8000 Hz

enum class Register : uint16_t {
	DEV_ID = 0x00,

	DIAG_STAT = 0x05,

	USER_GPIO_CFG1 = 0x2F,

	SPI_HALFDUPLEX_KEY = 0x32,

	USER_DATA_CFG = 0x34,

	USER_FIFO_CFG = 0x35,

	SOFT_RESET = 0x36,

	MSC_CTRL = 0x39,
};

// DIAG_STAT
enum DIAG_STAT_BIT : uint16_t {
	Gyro_CST_fault       		= Bit15, // The gyro continuous self-test has failed.
	Accelerometer_CST_fault         = Bit14,  // An accelerometer continuous self-test has failed
	Gyro_fault              	= Bit13,  // The gyro sensor has been disabled
	Accelerometer_overload_or_fault = Bit12,  // The accelerometer has been disabled
	Power_supplies_fault            = Bit11,  // Fault in Any of the Power Supplies
	SDSP_fault              	= Bit10,  // SDSP Combined Error Status
	BL_fault                	= Bit9,  // Bootloader Fault
	FW_fault     			= Bit8,  // Firmware Faults.
	SYNC_LOST 			= Bit2,  // DPLL Has lost its sync input
	DPLL_UnLocked           	= Bit1,  // Status of the DPLL
	FIFO_Threshold_Met 		= Bit0, // 1 Indicates that the FIFO contains at least the desired number of samples, as set in the FIFO_SAMPLES register

};

// USER_GPIO_CFG1
enum USER_GPIO_CFG1_BIT : uint16_t {
	GPIO3_DR = Bit9, // Configure GPIO pins(GPIO3 as DR)
};

// USER_DATA_CFG
enum USER_DATA_CFG_BIT : uint16_t {
	WORD_SIZE_32 = Bit15, // 32-bit Output Word Length
	DATA_CNTR_EN   = Bit14, // Enable Output Data Counter
	TEMPERATURE_EN = Bit12, // Enable Temperature Sensor
	Z_DELTANG_EN = Bit11,
	Y_DELTANG_EN = Bit10,
	X_DELTANG_EN = Bit9,
	Z_DELTVEL_EN = Bit8,
	Y_DELTVEL_EN = Bit7,
	X_DELTVEL_EN = Bit6,
	Z_GYRO_EN = Bit5,
	Y_GYRO_EN = Bit4,
	X_GYRO_EN = Bit3,
	Z_ACCEL_EN = Bit2,
	Y_ACCEL_EN = Bit1,
	X_ACCEL_EN = Bit0,
};

// USER_FIFO_CFG
enum USER_FIFO_CFG_BIT : uint16_t {
	CLEAR_FIFOB = Bit15, // Clear the FIFO pointers to reset it
};

// MSC_CTRL
enum MSC_CTRL_BIT : uint16_t {
	FILT_BW_500Hz = Bit8, // fc= 500 Hz
	Self_test_1 = Bit6,  // enable Self-Test mode
};



} // namespace Analog_Devices_ADIS16607
