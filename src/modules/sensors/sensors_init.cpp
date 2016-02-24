/****************************************************************************
 *
 *   Copyright (c) 2016 James Wilson. All rights reserved.
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
 * @file sensors_init.cpp
 *
 *	Sensor initialization code, used on everything but QURT.
 *
 * @author James Wilson <jywilson99@hotmail.com>
 */

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_adc.h>


#include <DevMgr.hpp>

#include "sensors_init.h"

using namespace DriverFramework;

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/**
 * Do accel-related initialisation.
 */
int		accel_init();

/**
 * Do gyro-related initialisation.
 */
int		gyro_init();

/**
 * Do mag-related initialisation.
 */
int		mag_init();

/**
 * Do baro-related initialisation.
 */
int		baro_init();


int
sensors_init(void)
{
	int ret;

	ret = accel_init();

	if (ret) { return ret; }

	ret = gyro_init();

	if (ret) { return ret; }

	ret = mag_init();

	if (ret) { return ret; }

	ret = baro_init();

	if (ret) { return ret; }

	return 0;
}


int
accel_init()
{
	DevHandle h_accel;
	DevMgr::getHandle(ACCEL0_DEVICE_PATH, h_accel);

	if (!h_accel.isValid()) {
		warnx("FATAL: no accelerometer found: %s (%d)", ACCEL0_DEVICE_PATH, h_accel.getError());
		return ERROR;

	} else {

		/* set the accel internal sampling rate to default rate */
		h_accel.ioctl(ACCELIOCSSAMPLERATE, ACCEL_SAMPLERATE_DEFAULT);

		/* set the driver to poll at default rate */
		h_accel.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
	}

	return OK;
}

int
gyro_init()
{
	DevHandle h_gyro;
	DevMgr::getHandle(GYRO0_DEVICE_PATH, h_gyro);

	if (!h_gyro.isValid()) {
		warnx("FATAL: no gyro found: %s (%d)", GYRO0_DEVICE_PATH, h_gyro.getError());
		return ERROR;

	}

	/* set the gyro internal sampling rate to default rate */
	h_gyro.ioctl(GYROIOCSSAMPLERATE, GYRO_SAMPLERATE_DEFAULT);

	/* set the driver to poll at default rate */
	h_gyro.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);

	return OK;
}

int
mag_init()
{
	int	ret;

	DevHandle h_mag;
	DevMgr::getHandle(MAG0_DEVICE_PATH, h_mag);

	if (!h_mag.isValid()) {
		warnx("FATAL: no magnetometer found: %s (%d)", MAG0_DEVICE_PATH, h_mag.getError());
		return ERROR;
	}

	/* try different mag sampling rates */


	ret = h_mag.ioctl(MAGIOCSSAMPLERATE, 150);

	if (ret == OK) {
		/* set the pollrate accordingly */
		h_mag.ioctl(SENSORIOCSPOLLRATE, 150);

	} else {
		ret = h_mag.ioctl(MAGIOCSSAMPLERATE, 100);

		/* if the slower sampling rate still fails, something is wrong */
		if (ret == OK) {
			/* set the driver to poll also at the slower rate */
			h_mag.ioctl(SENSORIOCSPOLLRATE, 100);

		} else {
			warnx("FATAL: mag sampling rate could not be set");
			return ERROR;
		}
	}

	return OK;
}

int
baro_init()
{
	DevHandle h_baro;
	DevMgr::getHandle(BARO0_DEVICE_PATH, h_baro);

	if (!h_baro.isValid()) {
		warnx("FATAL: No barometer found: %s (%d)", BARO0_DEVICE_PATH, h_baro.getError());
		return ERROR;
	}

	/* set the driver to poll at 150Hz */
	h_baro.ioctl(SENSORIOCSPOLLRATE, 150);

	return OK;
}

