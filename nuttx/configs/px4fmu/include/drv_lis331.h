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

/*
 * Driver for the ST LIS331 MEMS accelerometer
 */

#include <sys/ioctl.h>

#define _LIS331BASE	0x6900
#define LIS331C(_x)	_IOC(_LIS331BASE, _x)

/* 
 * Sets the sensor internal sampling rate, and if a buffer
 * has been configured, the rate at which entries will be
 * added to the buffer.
 */
#define LIS331_SETRATE		LIS331C(1)

#define LIS331_RATE_50Hz		(0<<3)
#define LIS331_RATE_100Hz		(1<<3)
#define LIS331_RATE_400Hz		(2<<3)
#define LIS331_RATE_1000Hz		(3<<3)

/*
 * Sets the sensor internal range.
 */
#define LIS331_SETRANGE		LIS331C(2)

#define LIS331_RANGE_2G			(0<<4)
#define LIS331_RANGE_4G			(1<<4)
#define LIS331_RANGE_8G			(3<<4)

/*
 * Sets the address of a shared lis331_buffer
 * structure that is maintained by the driver.
 *
 * If zero is passed as the address, disables
 * the buffer updating.
 */
#define LIS331_SETBUFFER	LIS331C(3)

struct lis331_buffer {
	uint32_t	size;		/* number of entries in the samples[] array */
	uint32_t	next;		/* the next entry that will be populated */
	struct {
		uint16_t	x;
		uint16_t	y;
		uint16_t	z;
	} samples[];
};

extern int	lis331_attach(struct spi_dev_s *spi, int spi_id);
