/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file LPS22HB_SPI.cpp
 *
 * SPI interface for LPS22HB
 */

#include "LPS22HB.hpp"

#ifdef PX4_SPIDEV_LPS22HB

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)

device::Device *LPS22HB_SPI_interface(int bus);

class LPS22HB_SPI : public device::SPI
{
public:
	LPS22HB_SPI(int bus, uint32_t device);
	virtual ~LPS22HB_SPI() = default;

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

};

device::Device *
LPS22HB_SPI_interface(int bus)
{
	return new LPS22HB_SPI(bus, PX4_SPIDEV_LPS22HB);
}

LPS22HB_SPI::LPS22HB_SPI(int bus, uint32_t device) : SPI("LPS22HB_SPI", nullptr, bus, device, SPIDEV_MODE3, 10000000)
{
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_LPS22HB;
}

int
LPS22HB_SPI::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// read WHO_AM_I value
	uint8_t id = 0;

	if (read(WHO_AM_I, &id, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	if (id != LPS22HB_ID_WHO_AM_I) {
		DEVICE_DEBUG("ID byte mismatch (%02x != %02x)", LPS22HB_ID_WHO_AM_I, id);
		return -EIO;
	}

	return OK;
}

int
LPS22HB_SPI::write(unsigned address, void *data, unsigned count)
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
LPS22HB_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ;

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

#endif /* PX4_SPIDEV_LPS22HB */
