/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file device.cpp
 *
 * Fundamental driver base class for the virtual device framework.
 */

#include "device.h"

#include <px4_defines.h>
#include <px4_posix.h>
#include <drivers/drv_device.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>

namespace device
{

Device::Device(const char *name) :
	// public
	// protected
	_name(name),
	_debug_enabled(false)
{
	int ret = px4_sem_init(&_lock, 0, 1);

	if (ret != 0) {
		PX4_WARN("SEM INIT FAIL: ret %d, %s", ret, strerror(errno));
	}

	/* setup a default device ID. When bus_type is UNKNOWN the
	   other fields are invalid */
	_device_id.devid = 0;
	_device_id.devid_s.bus_type = DeviceBusType_UNKNOWN;
	_device_id.devid_s.bus = 0;
	_device_id.devid_s.address = 0;
	_device_id.devid_s.devtype = 0;
}

Device::~Device()
{
	px4_sem_destroy(&_lock);
}

int
Device::init()
{
	int ret = OK;

	return ret;
}

int
Device::dev_read(unsigned offset, void *data, unsigned count)
{
	return -ENODEV;
}

int
Device::dev_write(unsigned offset, void *data, unsigned count)
{
	return -ENODEV;
}

int
Device::dev_ioctl(unsigned operation, unsigned arg)
{
	switch (operation) {
	case DEVIOCGDEVICEID:
		return (int)_device_id.devid;
	}

	return -ENODEV;
}

} // namespace device
