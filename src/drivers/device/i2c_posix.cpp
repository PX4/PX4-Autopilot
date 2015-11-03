/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file i2c.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "i2c.h"
#ifdef __PX4_LINUX
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define PX4_SIMULATE_I2C 0
static int simulate = PX4_SIMULATE_I2C;

namespace device
{

I2C::I2C(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address) :
	// base class
	VDev(name, devname),
	// public
	// protected
	_retries(0),
	// private
	_bus(bus),
	_address(address),
	_fd(-1)
{
	//warnx("I2C::I2C name = %s devname = %s", name, devname);
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

I2C::~I2C()
{
	if (_fd >= 0) {
#ifndef __PX4_QURT
		::close(_fd);
#endif
		_fd = -1;
	}
}

int
I2C::init()
{
	int ret = PX4_OK;

	// Assume the driver set the desired bus frequency. There is no standard
	// way to set it from user space.

	// do base class init, which will create device node, etc
	ret = VDev::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("VDev::init failed");
		return ret;
	}

	_fd = px4_open(get_devname(), PX4_F_RDONLY | PX4_F_WRONLY);

	if (_fd < 0) {
		DEVICE_DEBUG("px4_open failed of device %s", get_devname());
		return PX4_ERROR;
	}

#ifdef __PX4_QURT
	simulate = true;
#endif

	if (simulate) {
		_fd = 10000;

	} else {
#ifndef __PX4_QURT
		// Open the actual I2C device and map to the virtual dev name
		_fd = ::open(get_devname(), O_RDWR);

		if (_fd < 0) {
			warnx("could not open %s", get_devname());
			px4_errno = errno;
			return PX4_ERROR;
		}

#endif
	}

	return ret;
}

int
I2C::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
#ifndef __PX4_LINUX
	return 1;
#else
	struct i2c_msg msgv[2];
	unsigned msgs;
	struct i2c_rdwr_ioctl_data packets;
	int ret;
	unsigned retry_count = 0;

	if (_fd < 0) {
		warnx("I2C device not opened");
		return 1;
	}

	do {
		//	DEVICE_DEBUG("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);
		msgs = 0;

		if (send_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = 0;
			msgv[msgs].buf = const_cast<uint8_t *>(send);
			msgv[msgs].len = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = I2C_M_READ;
			msgv[msgs].buf = recv;
			msgv[msgs].len = recv_len;
			msgs++;
		}

		if (msgs == 0) {
			return -EINVAL;
		}

		packets.msgs  = msgv;
		packets.nmsgs = msgs;

		if (simulate) {
			//warnx("I2C SIM: transfer_4 on %s", get_devname());
			ret = PX4_OK;

		} else {
			ret = ::ioctl(_fd, I2C_RDWR, (unsigned long)&packets);

			if (ret < 0) {
				warnx("I2C transfer failed");
				return 1;
			}
		}

		/* success */
		if (ret == PX4_OK) {
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
#endif
}

int
I2C::transfer(struct i2c_msg *msgv, unsigned msgs)
{
#ifndef __PX4_LINUX
	return 1;
#else
	struct i2c_rdwr_ioctl_data packets;
	int ret;
	unsigned retry_count = 0;

	/* force the device address into the message vector */
	for (unsigned i = 0; i < msgs; i++) {
		msgv[i].addr = _address;
	}

	do {
		packets.msgs  = msgv;
		packets.nmsgs = msgs;

		if (simulate) {
			warnx("I2C SIM: transfer_2 on %s", get_devname());
			ret = PX4_OK;

		} else {
			ret = ::ioctl(_fd, I2C_RDWR, (unsigned long)&packets);
		}

		if (ret < 0) {
			warnx("I2C transfer failed");
			return 1;
		}

		/* success */
		if (ret == PX4_OK) {
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
#endif
}

int I2C::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	//struct i2c_rdwr_ioctl_data *packets = (i2c_rdwr_ioctl_data *)(void *)arg;
	switch (cmd) {
#ifdef __PX4_LINUX

	case I2C_RDWR:
		warnx("Use I2C::transfer, not ioctl");
		return 0;
#endif

	default:
		/* give it to the superclass */
		return VDev::ioctl(filp, cmd, arg);
	}
}

ssize_t	I2C::read(file_t *filp, char *buffer, size_t buflen)
{
	if (simulate) {
		// FIXME no idea what this should be
		warnx("2C SIM I2C::read");
		return 0;
	}

#ifndef __PX4_QURT
	return ::read(_fd, buffer, buflen);
#else
	return 0;
#endif
}

ssize_t	I2C::write(file_t *filp, const char *buffer, size_t buflen)
{
	if (simulate) {
		warnx("2C SIM I2C::write");
		return buflen;
	}

#ifndef __PX4_QURT
	return ::write(_fd, buffer, buflen);
#else
	return buflen;
#endif
}

} // namespace device
