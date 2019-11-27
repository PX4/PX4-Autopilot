/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file gyro.cpp
 *
 * Driver for the Invensense mpu9250 connected via SPI.
 *
 * @author Andrew Tridgell
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>
#include <ecl/geo/geo.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include <perf/perf_counter.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "gyro.h"
#include "mpu9250.h"

MPU9250_accel::MPU9250_accel(MPU9250 *parent, const char *path) :
	CDev("MPU9250_accel", path),
	_parent(parent)
{
}

MPU9250_accel::~MPU9250_accel()
{
	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}
}

int
MPU9250_accel::init()
{
	// do base class init
	int ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("accel init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	return ret;
}

void
MPU9250_accel::parent_poll_notify()
{
	poll_notify(POLLIN);
}

int
MPU9250_accel::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	/*
	 * Repeated in MPU9250_mag::ioctl
	 * Both accel and mag CDev could be unused in case of magnetometer only mode or MPU6500
	 */

	switch (cmd) {
	case SENSORIOCRESET: {
			return _parent->reset();
		}

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU9250_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default:
				return _parent->_set_pollrate(arg);
			}
		}

	case ACCELIOCSSCALE: {
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			memcpy(&_parent->_accel_scale, s, sizeof(_parent->_accel_scale));
			return OK;
		}

	default:
		return CDev::ioctl(filp, cmd, arg);
	}
}
