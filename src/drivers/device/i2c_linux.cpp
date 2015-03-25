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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

namespace device
{

I2C::I2C(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address) :
	// base class
	CDev(name, devname),
	// public
	// protected
	_retries(0),
	// private
	_bus(bus),
	_address(address),
	_fd(-1),
	_dname()
{
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;     

	if (devname)
		_dname = devname;
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
	int ret = PX4_OK;

	// Assume the driver set the desired bus frequency. There is no standard
	// way to set it from user space.

	// do base class init, which will create device node, etc
	ret = CDev::init();

	if (ret != PX4_OK) {
		debug("cdev init failed");
		return ret;
	}

	_fd = px4_open(_dname.c_str(), PX4_F_RDONLY | PX4_F_WRONLY);
	if (_fd < 0) {
		debug("px4_open failed of device %s", _dname.c_str());
		return PX4_ERROR;
	}
#if 0
	// Open the actual I2C device and map to the virtual dev name
	char str[22];

	// Fixme - not sure bus is the right mapping here
	// may have to go through /sys/bus/i2c interface to find the right map
	snprintf(str, sizeof(str), "/dev/i2c-%d", _bus);
	_fd = ::open(str, O_RDWR);
        if (_fd < 0) {
                warnx("could not open %s for virtual device %s", str, _dname.c_str());
                return -errno;
        }
#endif

	return ret;
}

int
I2C::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
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
		//	debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);
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

		if (msgs == 0)
			return -EINVAL;

		packets.msgs  = msgv;
		packets.nmsgs = msgs;

		ret = px4_ioctl(_fd, I2C_RDWR, (unsigned long)&packets);
		if (ret < 0) {
        		warnx("I2C transfer failed");
        		return 1;
    		}

		/* success */
		if (ret == PX4_OK)
			break;

// No way to reset device from userspace
#if 0
		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			px4_i2creset(_dev);
#endif

	} while (retry_count++ < _retries);

	return ret;
}

int
I2C::transfer(struct i2c_msg *msgv, unsigned msgs)
{
	struct i2c_rdwr_ioctl_data packets;
	int ret;
	unsigned retry_count = 0;

	/* force the device address into the message vector */
	for (unsigned i = 0; i < msgs; i++)
		msgv[i].addr = _address;

	do {
		packets.msgs  = msgv;
		packets.nmsgs = msgs;

		ret = px4_ioctl(_fd, I2C_RDWR, (unsigned long)&packets);
		if (ret < 0) {
        		warnx("I2C transfer failed");
        		return 1;
    		}

		/* success */
		if (ret == PX4_OK)
			break;

// No way to reset device from userspace
#if 0
		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			px4_i2creset(_dev);
#endif

	} while (retry_count++ < _retries);

	return ret;
}

int I2C::ioctl(device::px4_dev_handle_t *handlep, int cmd, unsigned long arg)
{
	//struct i2c_rdwr_ioctl_data *packets = (i2c_rdwr_ioctl_data *)(void *)arg;

	switch (cmd) {
	case I2C_RDWR:
        	warnx("I2C transfer request");
		return 0;
	default:
		/* give it to the superclass */
		return CDev::ioctl(handlep, cmd, arg);
	}
}

} // namespace device
