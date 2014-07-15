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
 * @file Magnetometer driver interface.
 */

#ifndef _DRV_MAG_H
#define _DRV_MAG_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define MAG_DEVICE_PATH		"/dev/mag"

/**
 * mag report structure.  Reads from the device must be in multiples of this
 * structure.
 *
 * Output values are in gauss.
 */
struct mag_report {
	uint64_t timestamp;
	uint64_t error_count;
	float x;
	float y;
	float z;
	float range_ga;
	float scaling;

	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
};

/** mag scaling factors; Vout = (Vin * Vscale) + Voffset */
struct mag_scale {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};

/*
 * ObjDev tag for raw magnetometer data.
 */
ORB_DECLARE(sensor_mag0);
ORB_DECLARE(sensor_mag1);
ORB_DECLARE(sensor_mag2);

/*
 * mag device types, for _device_id
 */
#define DRV_MAG_DEVTYPE_HMC5883 1
#define DRV_MAG_DEVTYPE_LSM303D 2

/*
 * ioctl() definitions
 */

#define _MAGIOCBASE		(0x2400)
#define _MAGIOC(_n)		(_IOC(_MAGIOCBASE, _n))

/** set the mag internal sample rate to at least (arg) Hz */
#define MAGIOCSSAMPLERATE	_MAGIOC(0)

/** return the mag internal sample rate in Hz */
#define MAGIOCGSAMPLERATE	_MAGIOC(1)

/** set the mag internal lowpass filter to no lower than (arg) Hz */
#define MAGIOCSLOWPASS		_MAGIOC(2)

/** return the mag internal lowpass filter in Hz */
#define MAGIOCGLOWPASS		_MAGIOC(3)

/** set the mag scaling constants to the structure pointed to by (arg) */
#define MAGIOCSSCALE		_MAGIOC(4)

/** copy the mag scaling constants to the structure pointed to by (arg) */
#define MAGIOCGSCALE		_MAGIOC(5)

/** set the measurement range to handle (at least) arg Gauss */
#define MAGIOCSRANGE		_MAGIOC(6)

/** return the current mag measurement range in Gauss */
#define MAGIOCGRANGE		_MAGIOC(7)

/** perform self-calibration, update scale factors to canonical units */
#define MAGIOCCALIBRATE		_MAGIOC(8)

/** excite strap */
#define MAGIOCEXSTRAP		_MAGIOC(9)

/** perform self test and report status */
#define MAGIOCSELFTEST		_MAGIOC(10)

/** determine if mag is external or onboard */
#define MAGIOCGEXTERNAL		_MAGIOC(11)

#endif /* _DRV_MAG_H */
