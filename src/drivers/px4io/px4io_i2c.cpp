/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
  * @file interface_i2c.cpp
  *
  * I2C interface for PX4IO
  */

/* XXX trim includes */
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <nuttx/i2c.h>

#include <mavlink/mavlink_log.h>
#include "uploader.h"
#include <modules/px4iofirmware/protocol.h>

#include "interface.h"

class PX4IO_I2C : public PX4IO_interface
{
public:
	PX4IO_I2C(int bus, uint8_t address);
	virtual ~PX4IO_I2C();

	virtual int	set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
	virtual int	get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	virtual bool	ok();

	virtual int	test(unsigned mode);

private:
	static const unsigned	_retries = 2;

	struct i2c_dev_s	*_dev;
	uint8_t			_address;
};

PX4IO_interface	*io_i2c_interface(int bus, uint8_t address)
{
	return new PX4IO_I2C(bus, address);
}

PX4IO_I2C::PX4IO_I2C(int bus, uint8_t address) :
	_dev(nullptr),
	_address(address)
{
	_dev = up_i2cinitialize(bus);
	if (_dev)
		I2C_SETFREQUENCY(_dev, 400000);
}

PX4IO_I2C::~PX4IO_I2C()
{
	if (_dev)
		up_i2cuninitialize(_dev);
}

bool
PX4IO_I2C::ok()
{
	if (!_dev)
		return false;

	/* check any other status here */

	return true;
}

int
PX4IO_I2C::test(unsigned mode)
{
	return 0;
}

int
PX4IO_I2C::set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	int ret;

	/* set up the transfer */
	uint8_t 	addr[2] = {
		page,
		offset
	};
	i2c_msg_s	msgv[2];

	msgv[0].addr = _address;
	msgv[0].flags = 0;
	msgv[0].buffer = addr;
	msgv[0].length = 2;

	msgv[1].addr = _address;
	msgv[1].flags = I2C_M_NORESTART;
	msgv[1].buffer = (uint8_t *)values;
	msgv[1].length = num_values * sizeof(*values);

	unsigned tries = 0;
	do {

		/* perform the transfer */
		ret = I2C_TRANSFER(_dev, msgv, 2);

		if (ret == OK)
			break;

	} while (tries++ < _retries);

	return ret;
}

int
PX4IO_I2C::get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	int ret;

	/* set up the transfer */
	uint8_t		addr[2] = {
		page,
		offset
	};
	i2c_msg_s	msgv[2];

	msgv[0].addr = _address;
	msgv[0].flags = 0;
	msgv[0].buffer = addr;
	msgv[0].length = 2;

	msgv[1].addr = _address;
	msgv[1].flags = I2C_M_READ;
	msgv[1].buffer = (uint8_t *)values;
	msgv[1].length = num_values * sizeof(*values);

	unsigned tries = 0;
	do {
		/* perform the transfer */
		ret = I2C_TRANSFER(_dev, msgv, 2);

		if (ret == OK)
			break;

	} while (tries++ < _retries);

	return ret;
}
