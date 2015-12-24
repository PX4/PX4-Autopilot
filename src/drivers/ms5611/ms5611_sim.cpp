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
 * SIM interface for MS5611
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

#include <drivers/device/sim.h>
#include "ms5611.h"
#include "board_config.h"

device::Device *MS5611_sim_interface(ms5611::prom_u &prom_buf);

class MS5611_SIM : public device::SIM
{
public:
	MS5611_SIM(uint8_t bus, ms5611::prom_u &prom_buf);
	virtual ~MS5611_SIM();

	virtual int	init();
	virtual int	dev_read(unsigned offset, void *data, unsigned count);
	virtual int	dev_ioctl(unsigned operation, unsigned &arg);

	virtual int	transfer(const uint8_t *send, unsigned send_len,
				 uint8_t *recv, unsigned recv_len);
private:
	ms5611::prom_u	&_prom;

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
MS5611_sim_interface(ms5611::prom_u &prom_buf, uint8_t busnum)
{
	return new MS5611_SIM(busnum, prom_buf);
}

MS5611_SIM::MS5611_SIM(uint8_t bus, ms5611::prom_u &prom) :
	SIM("MS5611_SIM", "/dev/MS5611_SIM", bus, 0),
	_prom(prom)
{
}

MS5611_SIM::~MS5611_SIM()
{
}

int
MS5611_SIM::init()
{
	return SIM::init();
}

int
MS5611_SIM::dev_read(unsigned offset, void *data, unsigned count)
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
MS5611_SIM::dev_ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
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

int
MS5611_SIM::_reset()
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
MS5611_SIM::_measure(unsigned addr)
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
MS5611_SIM::_read_prom()
{
	int ret = 1;
	// TODO input simlation data
	return ret;
}

int
MS5611_SIM::transfer(const uint8_t *send, unsigned send_len,
		     uint8_t *recv, unsigned recv_len)
{
	// TODO add Simulation data connection so calls retrieve
	// data from the simulator
	return 0;
}
