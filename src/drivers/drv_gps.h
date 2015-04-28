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
 * @file drv_gps.h
 *
 * GPS driver interface.
 */

#ifndef _DRV_GPS_H
#define _DRV_GPS_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "board_config.h"

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#ifndef GPS_DEFAULT_UART_PORT
#define GPS_DEFAULT_UART_PORT "/dev/ttyS3"
#endif

#define GPS0_DEVICE_PATH	"/dev/gps0"

typedef enum {
	GPS_DRIVER_MODE_NONE = 0,
	GPS_DRIVER_MODE_UBX,
	GPS_DRIVER_MODE_MTK,
	GPS_DRIVER_MODE_ASHTECH
} gps_driver_mode_t;


/*
 * ObjDev tag for GPS data.
 */
ORB_DECLARE(gps);

/*
 * ioctl() definitions
 */
#define _GPSIOCBASE			(0x2800)            //TODO: arbitrary choice...
#define _GPSIOC(_n)		(_IOC(_GPSIOCBASE, _n))

#endif /* _DRV_GPS_H */
