/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file LPS25H_SPI.cpp
 *
 * SPI interface for LPS25H
 */

/* XXX trim includes */
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/spi.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "lps25h.h"
#include <board_config.h>

#ifdef PX4_SPIDEV_HMC

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

#define HMC_MAX_SEND_LEN		4
#define HMC_MAX_RCV_LEN			8

device::Device *LPS25H_SPI_interface(int bus);

class LPS25H_SPI : public device::SPI
{
public:
	LPS25H_SPI(int bus, spi_dev_e device);
	virtual ~LPS25H_SPI();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);

};

device::Device *
LPS25H_SPI_interface(int bus)
{
	return new LPS25H_SPI(bus, (spi_dev_e)PX4_SPIDEV_HMC);
}

LPS25H_SPI::LPS25H_SPI(int bus, spi_dev_e device) :
	SPI("LPS25H_SPI", nullptr, bus, device, SPIDEV_MODE3, 11 * 1000 * 1000 /* will be rounded to 10.4 MHz */)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LPS25H;
}

LPS25H_SPI::~LPS25H_SPI()
{
}

int
LPS25H_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// read WHO_AM_I value
	uint8_t id;

	if (read(ADDR_ID, &id, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	if (id != ID_WHO_AM_I) {
		DEVICE_DEBUG("ID byte mismatch (%02x != %02x)", ID_WHO_AM_I, id);
		return -EIO;
	}

	return OK;
}

int
LPS25H_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case MAGIOCGEXTERNAL:
		/*
		 * Even if this sensor is on the external SPI
		 * bus it is still internal to the autopilot
		 * assembly, so always return 0 for internal.
		 */
		return 0;

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

int
LPS25H_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}

int
LPS25H_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ | ADDR_INCREMENT;

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

#endif /* PX4_SPIDEV_HMC */
