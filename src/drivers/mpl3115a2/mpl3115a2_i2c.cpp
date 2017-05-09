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

/* XXX trim includes */
#include <px4_config.h>
#include <px4_defines.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>

#include "mpl3115a2.h"

#include "board_config.h"

#define MPL3115A2_ADDRESS        0x60
#define MPL3115A2_REG_WHO_AM_I   0x0c
#define MPL3115A2_WHO_AM_I       0xC4

device::Device *MPL3115A2_i2c_interface(uint8_t busnum);

class MPL3115A2_I2C : public device::I2C
{
public:
	MPL3115A2_I2C(uint8_t bus);
	virtual ~MPL3115A2_I2C();

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

	int reg_read(unsigned reg, void *data, unsigned count);

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

MPL3115A2_I2C::~MPL3115A2_I2C()
{
}

int
MPL3115A2_I2C::init()
{
	/* this will call probe() */
	set_address(MPL3115A2_ADDRESS);
	return I2C::init();
}

int
MPL3115A2_I2C::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[3];

	/* read the most recent measurement */
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, &buf[0], 3);

	if (ret == PX4_OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[2];
		cvt->b[1] = buf[1];
		cvt->b[2] = buf[0];
		cvt->b[3] = 0;
	}

	return ret;
}

int
MPL3115A2_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		PX4_ERR("Not implemented");
		ret = EINVAL;
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

	if ((reg_read(MPL3115A2_REG_WHO_AM_I, &whoami, 1) > 0) && (whoami == MPL3115A2_WHO_AM_I)) {
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

	uint8_t cmd = addr;
	return transfer(&cmd, 1, nullptr, 0);
}


int MPL3115A2_I2C::reg_read(unsigned reg, void *data, unsigned count)
{
	uint8_t cmd = reg;
	int ret = transfer(&cmd, 1, (uint8_t *)data, count);
	return ret == PX4_OK ? count : ret;
}
