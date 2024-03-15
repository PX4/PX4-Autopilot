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

#include "mmc5983ma.h"
#include <drivers/device/i2c.h>

class MMC5983MA_I2C : public device::I2C
{
public:
	MMC5983MA_I2C(const I2CSPIDriverConfig &config);
	virtual ~MMC5983MA_I2C() = default;

	virtual int read(unsigned address, void *data, unsigned count) override;
	virtual int write(unsigned address, void *data, unsigned count) override;

protected:
	virtual int probe();
};

MMC5983MA_I2C::MMC5983MA_I2C(const I2CSPIDriverConfig &config) :
	I2C(config)
{
}

int MMC5983MA_I2C::probe()
{
	uint8_t data = 0;

	if (read(MMC5983MA_ADDR_PRODUCT_ID, &data, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	if (data != MMC5983MA_PRODUCT_ID) {
		DEVICE_DEBUG("MMC5983MA bad ID: %02x", data);
		return -EIO;
	}

	_retries = 1;

	return OK;
}

int MMC5983MA_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int MMC5983MA_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

device::Device *MMC5983MA_I2C_interface(const I2CSPIDriverConfig &config)
{
	return new MMC5983MA_I2C(config);
}
