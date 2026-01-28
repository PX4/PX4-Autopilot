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
 * @file SPA06_I2C.cpp
 *
 * SPI interface for Goertek SPA06
 */

#include "spa06.h"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>

#if defined(CONFIG_I2C)

class SPA06_I2C: public device::I2C, public spa06::ISPA06
{
public:
	SPA06_I2C(uint8_t bus, uint32_t device, int bus_frequency);
	virtual ~SPA06_I2C() override = default;

	int init() override { return I2C::init(); }

	uint8_t	get_reg(uint8_t addr) override;
	int	set_reg(uint8_t value, uint8_t addr) override;

	int read(uint8_t addr, uint8_t *buf, uint8_t len) override;
	//spa06::data_s		*get_data(uint8_t addr) override;
	//spa06::calibration_s	*get_calibration(uint8_t addr) override;

	uint32_t get_device_id() const override { return device::I2C::get_device_id(); }

	uint8_t get_device_address() const override { return device::I2C::get_device_address(); }
private:
	spa06::calibration_s	_cal{};
	spa06::data_s		_data{};
};

spa06::ISPA06 *spa06_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency)
{
	return new SPA06_I2C(busnum, device, bus_frequency);
}

SPA06_I2C::SPA06_I2C(uint8_t bus, uint32_t device, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_SPA06, MODULE_NAME, bus, device, bus_frequency)
{
}

uint8_t
SPA06_I2C::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), 0};
	transfer(&cmd[0], 1, &cmd[1], 1);

	return cmd[1];
}

int
SPA06_I2C::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

int
SPA06_I2C::read(uint8_t addr, uint8_t *buf, uint8_t len)
{
	return transfer(&addr, 1, buf, len);
}

#endif // CONFIG_I2C
