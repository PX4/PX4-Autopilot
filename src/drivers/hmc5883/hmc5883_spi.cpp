/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
  * @file HMC5883_SPI.cpp
  *
  * SPI interface for HMC5983
  */

/* XXX trim includes */
#include <nuttx/config.h>

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

#include "hmc5883.h"
#include <board_config.h>

#ifdef PX4_SPIDEV_HMC

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

#define HMC_MAX_SEND_LEN		4
#define HMC_MAX_RCV_LEN			8

device::Device *HMC5883_SPI_interface(int bus);

class HMC5883_SPI : public device::SPI
{
public:
	HMC5883_SPI(int bus, spi_dev_e device);
	virtual ~HMC5883_SPI();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count); 
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);

};

device::Device *
HMC5883_SPI_interface(int bus)
{
	return new HMC5883_SPI(bus, (spi_dev_e)PX4_SPIDEV_HMC);
}

HMC5883_SPI::HMC5883_SPI(int bus, spi_dev_e device) :
	SPI("HMC5883_SPI", nullptr, bus, device, SPIDEV_MODE3, 11*1000*1000 /* will be rounded to 10.4 MHz */)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_HMC5883;
}

HMC5883_SPI::~HMC5883_SPI()
{
}

int
HMC5883_SPI::init()
{
	int ret;

	ret = SPI::init();
	if (ret != OK) {
		debug("SPI init failed");
		return -EIO;
	}

	// read WHO_AM_I value
	uint8_t data[3] = {0, 0, 0};

	if (read(ADDR_ID_A, &data[0], 1) ||
	    read(ADDR_ID_B, &data[1], 1) ||
	    read(ADDR_ID_C, &data[2], 1)) {
		debug("read_reg fail");
	}

	if ((data[0] != ID_A_WHO_AM_I) ||
	    (data[1] != ID_B_WHO_AM_I) ||
	    (data[2] != ID_C_WHO_AM_I)) {
		debug("ID byte mismatch (%02x,%02x,%02x)", data[0], data[1], data[2]);
		return -EIO;
	}

	return OK;
}

int
HMC5883_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case MAGIOCGEXTERNAL:

#ifdef PX4_SPI_BUS_EXT
		if (_bus == PX4_SPI_BUS_EXT) {
			return 1;
		} else 
#endif
		{
			return 0;
		}

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default:
	{
		ret = -EINVAL;
	}
	}

	return ret;
}

int
HMC5883_SPI::write(unsigned address, void *data, unsigned count)
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
HMC5883_SPI::read(unsigned address, void *data, unsigned count)
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
