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
 * @file sim.cpp
 *
 * Base class for simulated devices.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include <px4_log.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "sim.h"

namespace device
{

SIM::SIM(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address) :
	// base class
	Device(name),
	// public
	// protected
	// private
	_bus(bus),
	_address(address),
	_devname(devname)
{

	PX4_DEBUG("SIM::SIM name = %s devname = %s", name, devname);
	// fill in _device_id fields for a SIM device
	_device_id.devid_s.bus_type = DeviceBusType_SIM;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SIM::~SIM()
{
}

int
SIM::init()
{
	int ret = PX4_OK;

	// Assume the driver set the desired bus frequency. There is no standard
	// way to set it from user space.

	// do base class init, which registers the virtual driver
	ret = Device::init();

	if (ret != PX4_OK) {
		PX4_ERR("VDev::init failed");
		return ret;
	}

	return ret;
}

int
SIM::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	if (send_len > 0) {
		PX4_DEBUG("SIM: sending %d bytes", send_len);
	}

	if (recv_len > 0) {
		PX4_DEBUG("SIM: receiving %d bytes", recv_len);

		// TODO - write data to recv;
	}

	PX4_DEBUG("I2C SIM: transfer_4 on %s", _devname);

	return PX4_OK;
}

} // namespace device
