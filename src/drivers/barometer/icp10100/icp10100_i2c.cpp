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
 * @file icp10100_i2c.cpp
 *
 * I2C interface for ICP10100
 */

#include <drivers/device/i2c.h>

#include "icp10100.h"

class ICP10100_I2C: public device::I2C, public IICP10100
{
public:
	ICP10100_I2C(uint8_t bus, uint32_t device, int bus_frequency);
	virtual ~ICP10100_I2C() = default;

	int init();

	int read_response(uint16_t cmd, uint8_t *buf, uint8_t len);
	int send_command(uint16_t cmd);
	int send_command(uint16_t cmd, uint8_t *data, uint8_t len);
	int read_measurement_results(uint8_t *buf, uint8_t len);

	uint32_t get_device_id() const override { return device::I2C::get_device_id(); }

	uint8_t get_device_address() const override { return device::I2C::get_device_address(); }
};

IICP10100 *icp10100_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency)
{
	return new ICP10100_I2C(busnum, device, bus_frequency);
}

ICP10100_I2C::ICP10100_I2C(uint8_t bus, uint32_t device, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_ICP10100, MODULE_NAME, bus, device, bus_frequency)
{
}

int ICP10100_I2C::init()
{
	return I2C::init();
}

int ICP10100_I2C::read_measurement_results(uint8_t *buf, uint8_t len)
{
	return transfer(nullptr, 0, buf, len);
}

int ICP10100_I2C::read_response(uint16_t cmd, uint8_t *buf, uint8_t len)
{
	uint8_t buff[2];
	buff[0] = (cmd >> 8) & 0xff;
	buff[1] = cmd & 0xff;
	return transfer(&buff[0], 2, buf, len);
}

int ICP10100_I2C::send_command(uint16_t cmd)
{
	uint8_t buf[2];
	buf[0] = (cmd >> 8) & 0xff;
	buf[1] = cmd & 0xff;
	return transfer(buf, sizeof(buf), nullptr, 0);
}

int ICP10100_I2C::send_command(uint16_t cmd, uint8_t *data, uint8_t len)
{
	uint8_t buf[5];
	buf[0] = (cmd >> 8) & 0xff;
	buf[1] = cmd & 0xff;
	memcpy(&buf[2], data, len);
	return transfer(&buf[0], len + 2, nullptr, 0);
}
