/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file mpl3115a2_i2c.cpp
 *
 * I2C interface for MPL3115A2
 */

#include "mpl3115a2.h"

#define MPL3115A2_ADDRESS        0x60

device::Device *MPL3115A2_i2c_interface(uint8_t busnum);

class MPL3115A2_I2C : public device::I2C
{
public:
	MPL3115A2_I2C(uint8_t bus);
	virtual ~MPL3115A2_I2C() = default;

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	virtual int	probe();

private:

	/**
	 * Send a measure command to the MPL3115A2.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_measure(unsigned addr);

	int reg_read(uint8_t reg, void *data, unsigned count = 1);
	int reg_write(uint8_t reg, uint8_t data);
	int	reset();

};

device::Device *
MPL3115A2_i2c_interface(uint8_t busnum)
{
	return new MPL3115A2_I2C(busnum);
}

MPL3115A2_I2C::MPL3115A2_I2C(uint8_t bus) :
	I2C("MPL3115A2_I2C", nullptr, bus, 0, 400000)
{
}

int
MPL3115A2_I2C::init()
{
	/* this will call probe() */
	set_device_address(MPL3115A2_ADDRESS);
	return I2C::init();
}

int
MPL3115A2_I2C::reset()
{
	int max = 10;
	reg_write(MPL3115A2_CTRL_REG1, CTRL_REG1_RST);
	int rv = CTRL_REG1_RST;
	int ret = 1;

	while (ret == 1 && (rv & CTRL_REG1_RST) && max--) {
		usleep(4000);
		ret = reg_read(MPL3115A2_CTRL_REG1, &rv);
	}

	return ret == 1 ? PX4_OK : ret;
}

int
MPL3115A2_I2C::read(unsigned offset, void *data, unsigned count)
{

	int ret = -EINVAL;

	switch (offset) {
	case MPL3115A2_CTRL_REG1:
		ret = reg_read(offset, data, count);
		break;

	case OUT_P_MSB: {
			union _cvt {
				MPL3115A2_data_t reading;
			} *cvt = (_cvt *)data;

			/* read the most recent measurement
			 * 3 Pressure and 2 temprtture
			 */
			uint8_t	b[3 + 2];
			uint8_t reg = (uint8_t) offset;

			ret = transfer(&reg, 1, &b[0], sizeof(b));

			if (ret == PX4_OK) {
				cvt->reading.pressure.q = ((uint32_t)b[0]) << 18 | ((uint32_t) b[1]) <<  10 | (((uint32_t)b[2]) & 0xc0) << 2 | ((
								  b[2] & 0x30) >> 4);
				cvt->reading.temperature.w = ((uint16_t) b[3]) << 8 | (b[4] >> 4);

			}
		}
		break;
	}

	return ret;
}

int
MPL3115A2_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = reset();
		break;

	case IOCTL_MEASURE:
		ret = _measure(arg);
		break;

	default:
		ret = EINVAL;
	}

	return ret;
}

int
MPL3115A2_I2C::probe()
{
	_retries = 10;
	uint8_t whoami = 0;

	if ((reg_read(MPL3115A2_REG_WHO_AM_I, &whoami) > 0) && (whoami == MPL3115A2_WHO_AM_I)) {
		/*
		 * Disable retries; we may enable them selectively in some cases,
		 * but the device gets confused if we retry some of the commands.
		 */
		_retries = 0;
		return PX4_OK;
	}

	return -EIO;
}

int
MPL3115A2_I2C::_measure(unsigned addr)
{
	/*
	 * Disable retries on this command; we can't know whether failure
	 * means the device did or did not see the command.
	 */
	_retries = 0;
	return reg_write((addr >> 8) & 0xff, addr & 0xff);
}


int MPL3115A2_I2C::reg_read(uint8_t reg, void *data, unsigned count)
{
	uint8_t cmd = reg;
	int ret = transfer(&cmd, 1, (uint8_t *)data, count);
	return ret == PX4_OK ? count : ret;
}

int MPL3115A2_I2C::reg_write(uint8_t reg, uint8_t data)
{
	uint8_t buf[2] = { reg, data};
	int ret = transfer(buf, sizeof(buf), NULL, 0);
	return ret == PX4_OK ? 2 : ret;
}
