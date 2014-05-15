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
  * @file px4io_i2c.cpp
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
#include <crc32.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/i2c.h>

#ifdef PX4_I2C_OBDEV_PX4IO

device::Device	*PX4IO_i2c_interface();

class PX4IO_I2C : public device::I2C
{
public:
	PX4IO_I2C(int bus, uint8_t address);
	virtual ~PX4IO_I2C();

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count = 1);
	virtual int	write(unsigned address, void *data, unsigned count = 1);
	virtual int	ioctl(unsigned operation, unsigned &arg);

private:

};

device::Device
*PX4IO_i2c_interface()
{
	return new PX4IO_I2C(PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_PX4IO);
}

PX4IO_I2C::PX4IO_I2C(int bus, uint8_t address) :
	I2C("PX4IO_i2c", nullptr, bus, address, 320000)
{
	_retries = 3;
}

PX4IO_I2C::~PX4IO_I2C()
{
}

int
PX4IO_I2C::init()
{
	int ret;

	ret = I2C::init();
	if (ret != OK)
		goto out;

	/* XXX really should do something more here */

out:
	return 0;
}

int
PX4IO_I2C::ioctl(unsigned operation, unsigned &arg)
{
	return 0;
}

int
PX4IO_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	const uint16_t *values = reinterpret_cast<const uint16_t *>(data);

	/* set up the transfer */
	uint8_t 	addr[2] = {
		page,
		offset
	};
	uint32_t 	crc = 0;
	i2c_msg_s	msgv[3];

	msgv[0].flags = 0;
	msgv[0].buffer = addr;
	msgv[0].length = 2;

	msgv[1].flags = I2C_M_NORESTART;
	msgv[1].buffer = (uint8_t *)values;
	msgv[1].length = 2 * count;

	msgv[2].flags = I2C_M_NORESTART;
	msgv[2].buffer = (uint8_t *)&crc;
	msgv[2].length = sizeof(crc);

	crc = crc32part(&addr, 2, crc);
	crc = crc32part(data, 2 * count, crc);

	int ret = transfer(msgv, 3);
	if (ret == OK)
		ret = count;
	return ret;
}

int
PX4IO_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	const uint16_t *values = reinterpret_cast<const uint16_t *>(data);

	/* set up the transfer */
	uint8_t		addr[2] = {
		page,
		offset
	};
	uint32_t crc;
	i2c_msg_s	msgv[3];

	msgv[0].flags = 0;
	msgv[0].buffer = addr;
	msgv[0].length = 2;

	msgv[1].flags = I2C_M_READ;
	msgv[1].buffer = (uint8_t *)values;
	msgv[1].length = 2 * count;

	msgv[2].flags = I2C_M_READ | I2C_M_NORESTART;
	msgv[2].buffer = (uint8_t *)&crc;
	msgv[2].length = sizeof(crc);

	int ret = transfer(msgv, 3);
	if (ret == OK) {
		uint32_t tcrc = 0;

		tcrc = crc32part(&addr, 2, tcrc);
		tcrc = crc32part(data, 2 * count, tcrc);

		if (tcrc != crc) {
			ret = EIO;
		} else {
			ret = count;
		}
	}
	return ret;
}

#endif /* PX4_I2C_OBDEV_PX4IO */
