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
 * @file SPA06_SPI.cpp
 *
 * SPI interface for Goertek SPA06
 */

#include "spa06.h"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>

#if defined(CONFIG_SPI)

/* SPI protocol address bits */
#define DIR_READ			(1<<7)  //for set
#define DIR_WRITE			~(1<<7) //for clear

class SPA06_SPI: public device::SPI, public spa06::ISPA06
{
public:
	SPA06_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	virtual ~SPA06_SPI() override = default;

	int init() override { return SPI::init(); }

	uint8_t	get_reg(uint8_t addr) override;
	int	set_reg(uint8_t value, uint8_t addr) override;

	int read(uint8_t addr, uint8_t *buf, uint8_t len) override;

	uint32_t get_device_id() const override { return device::SPI::get_device_id(); }

	uint8_t get_device_address() const override { return device::SPI::get_device_address(); }
};

spa06::ISPA06 *
spa06_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode)
{
	return new SPA06_SPI(busnum, device, bus_frequency, spi_mode);
}

SPA06_SPI::SPA06_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_BARO_DEVTYPE_SPA06, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

uint8_t
SPA06_SPI::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr | DIR_READ), 0}; // set MSB bit
	transfer(&cmd[0], &cmd[0], 2);

	return cmd[1];
}

int
SPA06_SPI::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr & DIR_WRITE), value}; // clear MSB bit
	return transfer(&cmd[0], nullptr, 2);
}

int
SPA06_SPI::read(uint8_t addr, uint8_t *buf, uint8_t len)
{
	uint8_t tx_buf[len + 1] = {(uint8_t)(addr | DIR_READ)}; // GCC support VLA, let's use it

	return transfer(tx_buf, buf, len);
}

#endif // CONFIG_SPI
