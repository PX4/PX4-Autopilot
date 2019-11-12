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

#include "I2C.hpp"

#include "dev_fs_lib_i2c.h"

namespace device
{

I2C::I2C(const char *name, const char *devname, const int bus, const uint16_t address, const uint32_t frequency) :
	CDev(name, devname)
{
	DEVICE_DEBUG("I2C::I2C name = %s devname = %s", name, devname);
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
		::close(_fd);
		_fd = -1;
	}
}

int
I2C::init()
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
	snprintf(dev_path, sizeof(dev_path), "/dev/iic-%i", get_device_bus());
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("could not open %s", dev_path);
		px4_errno = errno;
		return PX4_ERROR;
	}

	return ret;
}

int
I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	dspal_i2c_ioctl_combined_write_read ioctl_write_read{};

	ioctl_write_read.write_buf = (uint8_t *)send;
	ioctl_write_read.write_buf_len = send_len;
	ioctl_write_read.read_buf = recv;
	ioctl_write_read.read_buf_len = recv_len;

	int bytes_read = ::ioctl(_fd, I2C_IOCTL_RDWR, &ioctl_write_read);

	if (bytes_read != (ssize_t)recv_len) {
		PX4_ERR("read register reports a read of %d bytes, but attempted to read %d bytes", bytes_read, recv_len);
		return -1;
	}

	return 0;
}

} // namespace device
