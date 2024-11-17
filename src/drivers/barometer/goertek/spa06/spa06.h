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

/**
 * @file spa06.h
 *
 * Shared defines for the spa06 driver.
 */
#pragma once

#include <px4_platform_common/i2c_spi_buses.h>

#define SPA06_ADDR_ID	0x0d
#define SPA06_ADDR_RESET 0x0c   // set to reset
#define SPA06_ADDR_CAL 0x10
#define SPA06_ADDR_PRS_CFG  0x06
#define SPA06_ADDR_TMP_CFG  0x07
#define SPA06_ADDR_MEAS_CFG	0x08
#define SPA06_ADDR_CFG_REG	0x09
#define SPA06_ADDR_DATA 0x00


#define SPA06_VALUE_RESET 9
#define SPA06_VALUE_ID    0x10

namespace spa06
{

#pragma pack(push,1)
struct calibration_s {
	int16_t c0, c1;
	int32_t c00, c10;
	int16_t c01, c11, c20, c21, c30;
}; //calibration data

struct data_s {
	uint8_t p_msb;
	uint8_t p_lsb;
	uint8_t p_xlsb;

	uint8_t t_msb;
	uint8_t t_lsb;
	uint8_t t_xlsb;
}; // data
#pragma pack(pop)

class ISPA06
{
public:
	virtual ~ISPA06() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of data into buffer, return same pointer
	virtual int read(uint8_t addr, uint8_t *buf, uint8_t len) = 0;
	// bulk read of calibration data into buffer, return same pointer

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;
};

} /* namespace */


/* interface factories */
#if defined(CONFIG_SPI)
extern spa06::ISPA06 *spa06_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
#endif // CONFIG_SPI
#if defined(CONFIG_I2C)
extern spa06::ISPA06 *spa06_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
#endif // CONFIG_I2C
