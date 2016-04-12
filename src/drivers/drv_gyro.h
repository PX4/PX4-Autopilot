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
 * @file drv_gyro.h
 *
 * Gyroscope driver interface.
 */

#ifndef _DRV_GYRO_H
#define _DRV_GYRO_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define GYRO_BASE_DEVICE_PATH	"/dev/gyro"
#define GYRO0_DEVICE_PATH	"/dev/gyro0"
#define GYRO1_DEVICE_PATH	"/dev/gyro1"
#define GYRO2_DEVICE_PATH	"/dev/gyro2"

#include <uORB/topics/sensor_gyro.h>
#define gyro_report sensor_gyro_s

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
struct gyro_calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};

/*
 * ioctl() definitions
 */

#define _GYROIOCBASE		(0x2300)
#define _GYROIOC(_n)		(_PX4_IOC(_GYROIOCBASE, _n))

/** set the gyro internal sample rate to at least (arg) Hz */
#define GYROIOCSSAMPLERATE	_GYROIOC(0)

#define GYRO_SAMPLERATE_DEFAULT    1000003	/**< default sample rate */

/** return the gyro internal sample rate in Hz */
#define GYROIOCGSAMPLERATE	_GYROIOC(1)

/** set the gyro internal lowpass filter to no lower than (arg) Hz */
#define GYROIOCSLOWPASS		_GYROIOC(2)

/** set the gyro internal lowpass filter to no lower than (arg) Hz */
#define GYROIOCGLOWPASS		_GYROIOC(3)

/** set the gyro scaling constants to (arg) */
#define GYROIOCSSCALE		_GYROIOC(4)

/** get the gyro scaling constants into (arg) */
#define GYROIOCGSCALE		_GYROIOC(5)

/** set the gyro measurement range to handle at least (arg) degrees per second */
#define GYROIOCSRANGE		_GYROIOC(6)

/** get the current gyro measurement range in degrees per second */
#define GYROIOCGRANGE		_GYROIOC(7)

/** check the status of the sensor */
#define GYROIOCSELFTEST		_GYROIOC(8)

/** set the hardware low-pass filter cut-off no lower than (arg) Hz */
#define GYROIOCSHWLOWPASS	_GYROIOC(9)

/** get the hardware low-pass filter cut-off in Hz*/
#define GYROIOCGHWLOWPASS	_GYROIOC(10)

#endif /* _DRV_GYRO_H */
