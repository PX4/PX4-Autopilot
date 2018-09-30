/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */

#include "SPI.hpp"

#include <px4_config.h>

#define DIR_READ    0x80
#define DIR_WRITE   0x00

namespace device
{

SPI::SPI(const char *name,
	 const char *devname,
	 int bus,
	 uint32_t device,
	 enum spi_mode_e mode,
	 uint32_t frequency) :
	// base class
	CDev(name, devname),
	// public
	// protected
	locking_mode(LOCK_PREEMPTION)
{
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SPI::~SPI()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int
SPI::init()
{
	// Assume the driver set the desired bus frequency. There is no standard
	// way to set it from user space.

	// do base class init, which will create device node, etc
	int ret = CDev::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("CDev::init failed");
		return ret;
	}

	// Open the actual I2C device
	char dev_path[16];
	snprintf(dev_path, sizeof(dev_path), "/dev/spi-%i", get_device_bus());
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("could not open %s", dev_path);
		px4_errno = errno;
		return PX4_ERROR;
	}

	return ret;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned length)
{
	uint8_t address = *send;

	PX4_DEBUG("transfer: length = %d", length);

	/* implement sensor interface via rpi spi */
	int transfer_bytes = 1 + length; // first byte is address

	// automatic write buffer
	uint8_t *write_buffer = (uint8_t *)alloca(transfer_bytes);
	memset(write_buffer, 0, transfer_bytes);
	// automatic read buffer
	uint8_t *read_buffer = (uint8_t *)alloca(transfer_bytes);
	memset(read_buffer, 0, transfer_bytes);

	write_buffer[0] = address | DIR_READ; // read mode

	struct spi_ioc_transfer spi_transfer; // datastructure for linux spi interface
	memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

	spi_transfer.rx_buf = (unsigned long)read_buffer;
	spi_transfer.len = transfer_bytes;
	spi_transfer.tx_buf = (unsigned long)write_buffer;
	// spi_transfer.speed_hz = SPI_FREQUENCY_1MHZ;
	spi_transfer.bits_per_word = 8;
	spi_transfer.delay_usecs = 0;

	int result = 0;
	result = ::ioctl(_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	if (result != transfer_bytes) {
		PX4_DEBUG("transfer error %d", result);
		return result;
	}

	PX4_DEBUG("transfer: read_buffer = %u, %u, %u, %u, %u, %u, %u, %u, %u",
		  read_buffer[1], read_buffer[2], read_buffer[3],
		  read_buffer[4], read_buffer[5], read_buffer[6],
		  read_buffer[7], read_buffer[8], read_buffer[9]);

	memcpy(recv, &read_buffer[1], transfer_bytes - 1);

	return 0;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	// TODO: implement

	return PX4_ERROR;
}

} // namespace device
