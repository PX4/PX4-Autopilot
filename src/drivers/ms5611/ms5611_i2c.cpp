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
 * @file ms5611_i2c.cpp
 *
 * I2C interface for MS5611
 */

/* XXX trim includes */
#include <px4_config.h>
#include <px4_defines.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
//#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>

#include "ms5611.h"

#include "board_config.h"

#define MS5611_ADDRESS_1		0x76	/* address select pins pulled high (PX4FMU series v1.6+) */
#define MS5611_ADDRESS_2		0x77    /* address select pins pulled low (PX4FMU prototypes) */



device::Device *MS5611_i2c_interface(ms5611::prom_u &prom_buf);

class MS5611_I2C : public device::I2C
{
public:
	MS5611_I2C(uint8_t bus, ms5611::prom_u &prom_buf);
	virtual ~MS5611_I2C();

	virtual int	init();
	virtual ssize_t	read(device::file_t *handlep, char *data, size_t count);
	virtual int	ioctl(device::file_t *handlep, int cmd, unsigned long arg);

#ifdef __PX4_NUTTX
protected:
	virtual int	probe();
#endif

private:
	ms5611::prom_u	&_prom;

#ifdef __PX4_NUTTX
	int		_probe_address(uint8_t address);
#endif

	/**
	 * Send a reset command to the MS5611.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the MS5611.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_measure(unsigned addr);

	/**
	 * Read the MS5611 PROM
	 *
	 * @return		PX4_OK if the PROM reads successfully.
	 */
	int		_read_prom();

};

device::Device *
MS5611_i2c_interface(ms5611::prom_u &prom_buf, uint8_t busnum)
{
	return new MS5611_I2C(busnum, prom_buf);
}

MS5611_I2C::MS5611_I2C(uint8_t bus, ms5611::prom_u &prom) :
	I2C("MS5611_I2C",
#ifdef __PX4_NUTTX
	    nullptr, bus, 0, 400000
#else
	    "/dev/MS5611_I2C", bus, 0
#endif
	   ),
	_prom(prom)
{
}

MS5611_I2C::~MS5611_I2C()
{
}

int
MS5611_I2C::init()
{
	/* this will call probe(), and thereby _probe_address */
	return I2C::init();
}

ssize_t
MS5611_I2C::read(device::file_t *handlep, char *data, size_t buflen)
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
MS5611_I2C::ioctl(device::file_t *handlep, int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case IOCTL_RESET:
		ret = _reset();
		break;

	case IOCTL_MEASURE:
		ret = _measure(arg);
		break;

	default:
		ret = EINVAL;
	}

	return ret;
}

#ifdef __PX4_NUTTX
int
MS5611_I2C::probe()
{
	_retries = 10;

	if ((PX4_OK == _probe_address(MS5611_ADDRESS_1)) ||
	    (PX4_OK == _probe_address(MS5611_ADDRESS_2))) {
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
MS5611_I2C::_probe_address(uint8_t address)
{
	/* select the address we are going to try */
	set_address(address);

	/* send reset command */
	if (PX4_OK != _reset()) {
		return -EIO;
	}

	/* read PROM */
	if (PX4_OK != _read_prom()) {
		return -EIO;
	}

	return PX4_OK;
}
#endif


int
MS5611_I2C::_reset()
{
	unsigned	old_retrycount = _retries;
	uint8_t		cmd = ADDR_RESET_CMD;
	int		result;

	/* bump the retry count */
	_retries = 10;
	result = transfer(&cmd, 1, nullptr, 0);
	_retries = old_retrycount;

	return result;
}

int
MS5611_I2C::_measure(unsigned addr)
{
	/*
	 * Disable retries on this command; we can't know whether failure
	 * means the device did or did not see the command.
	 */
	_retries = 0;

	uint8_t cmd = addr;
	return transfer(&cmd, 1, nullptr, 0);
}

int
MS5611_I2C::_read_prom()
{
	uint8_t		prom_buf[2];
	union {
		uint8_t		b[2];
		uint16_t	w;
	} cvt;

	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);

	/* read and convert PROM words */
	for (int i = 0; i < 8; i++) {
		uint8_t cmd = ADDR_PROM_SETUP + (i * 2);

		if (PX4_OK != transfer(&cmd, 1, &prom_buf[0], 2)) {
			break;
		}

		/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
		cvt.b[0] = prom_buf[1];
		cvt.b[1] = prom_buf[0];
		_prom.c[i] = cvt.w;
	}

	/* calculate CRC and return success/failure accordingly */
	return ms5611::crc4(&_prom.c[0]) ? PX4_OK : -EIO;
}
