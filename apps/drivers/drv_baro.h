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
 * @file Barometric pressure sensor driver interface.
 */

#ifndef _DRV_BARO_H
#define _DRV_BARO_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_orb_dev.h"

#define BARO_DEVICE_PATH	"/dev/baro"

/**
 * baro report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct baro_report {
	float pressure;
	float altitude;
	float temperature;
	uint64_t timestamp;
};

/*
 * ObjDev tag for raw barometer data.
 */
ORB_DECLARE(sensor_baro);

/*
 * ioctl() definitions
 */

#define _BAROIOCBASE		(0x2100)
#define _BAROIOC(_n)		(_IOC(_BAROIOCBASE, _n))

/** set the driver polling rate to (arg) Hz, or one of the BARO_POLLRATE constants */
#define BAROIOCSPOLLRATE	_BAROIOC(0)

#define BARO_POLLRATE_MANUAL		1000000	/**< poll when read */
#define BARO_POLLRATE_EXTERNAL		1000001	/**< poll when device signals ready */

/** set the internal queue depth to (arg) entries, must be at least 1 */
#define BAROIOCSQUEUEDEPTH	_BAROIOC(1)

/** set the report format to (arg); zero is the standard, 1-10 are reserved, all others are driver-specific. */
#define BAROIOCSREPORTFORMAT 	_BAROIOC(2)

#endif /* _DRV_BARO_H */
