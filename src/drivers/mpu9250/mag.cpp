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
 * @file mag.cpp
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include <systemlib/perf_counter.h>

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
#include "mpu9250.h"

MPU9250_mag::MPU9250_mag(MPU9250 *parent, const char *path) :
	CDev("MPU9250_mag", path),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
}

MPU9250_mag::~MPU9250_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}
}

int
MPU9250_mag::init()
{
	int ret;

	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	return ret;
}

void
MPU9250_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
MPU9250_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}

int
MPU9250_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->mag_ioctl(filp, cmd, arg);
	}
}
#if 0
int
HMC5883::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    unsigned dummy = arg;

    switch (cmd) {
    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _measure_ticks = 0;
                return OK;

            /* external signalling (DRDY) not supported */
            case SENSOR_POLLRATE_EXTERNAL:

            /* zero would be bad */
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
            case SENSOR_POLLRATE_DEFAULT: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* set interval for next measurement to minimum legal value */
                    _measure_ticks = USEC2TICK(HMC5883_CONVERSION_INTERVAL);

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* convert hz to tick interval via microseconds */
                    unsigned ticks = USEC2TICK(1000000 / arg);

                    /* check against maximum rate */
                    if (ticks < USEC2TICK(HMC5883_CONVERSION_INTERVAL)) {
                        return -EINVAL;
                    }

                    /* update interval for next measurement */
                    _measure_ticks = ticks;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_measure_ticks == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return 1000000 / TICK2USEC(_measure_ticks);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _reports->size();

    case SENSORIOCRESET:
        return reset();

    case MAGIOCSSAMPLERATE:
        /* same as pollrate because device is in single measurement mode*/
        return ioctl(filp, SENSORIOCSPOLLRATE, arg);

    case MAGIOCGSAMPLERATE:
        /* same as pollrate because device is in single measurement mode*/
        return 1000000 / TICK2USEC(_measure_ticks);

    case MAGIOCSRANGE:
        return set_range(arg);

    case MAGIOCGRANGE:
        return _range_ga;

    case MAGIOCSLOWPASS:
    case MAGIOCGLOWPASS:
        /* not supported, no internal filtering */
        return -EINVAL;

    case MAGIOCSSCALE:
        /* set new scale factors */
        memcpy(&_scale, (mag_scale *)arg, sizeof(_scale));
        /* check calibration, but not actually return an error */
        (void)check_calibration();
        return 0;

    case MAGIOCGSCALE:
        /* copy out scale factors */
        memcpy((mag_scale *)arg, &_scale, sizeof(_scale));
        return 0;

    case MAGIOCCALIBRATE:
        return calibrate(filp, arg);

    case MAGIOCEXSTRAP:
        return set_excitement(arg);

    case MAGIOCSELFTEST:
        return check_calibration();

    case MAGIOCGEXTERNAL:
        DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
        return _interface->ioctl(cmd, dummy);

    case MAGIOCSTEMPCOMP:
        return set_temperature_compensation(arg);

    case DEVIOCGDEVICEID:
        return _interface->ioctl(cmd, dummy);

    default:
        /* give it to the superclass */
        return CDev::ioctl(filp, cmd, arg);
    }
}
#endif
