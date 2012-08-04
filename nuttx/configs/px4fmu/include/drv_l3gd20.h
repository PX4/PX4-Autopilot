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
 * Driver for the ST L3GD20 gyroscope
 */

/* IMPORTANT NOTES:
 *
 * SPI max. clock frequency: 10 Mhz
 * CS has to be high before transfer,
 * go low right before transfer and
 * go high again right after transfer
 *
 */

#include <sys/ioctl.h>

#define _L3GD20BASE	0x6200
#define L3GD20C(_x)	_IOC(_L3GD20BASE, _x)

/* 
 * Sets the sensor internal sampling rate, and if a buffer
 * has been configured, the rate at which entries will be
 * added to the buffer.
 */
#define L3GD20_SETRATE		L3GD20C(1)

#define L3GD20_RATE_95HZ_LP_12_5HZ		((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define L3GD20_RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define L3GD20_RATE_190HZ_LP_12_5HZ		((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define L3GD20_RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define L3GD20_RATE_190HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define L3GD20_RATE_190HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define L3GD20_RATE_380HZ_LP_20HZ		((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define L3GD20_RATE_380HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define L3GD20_RATE_380HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define L3GD20_RATE_380HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define L3GD20_RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define L3GD20_RATE_760HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define L3GD20_RATE_760HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define L3GD20_RATE_760HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

/*
 * Sets the sensor internal range.
 */
#define L3GD20_SETRANGE		L3GD20C(2)

#define L3GD20_RANGE_250DPS			(0<<4)
#define L3GD20_RANGE_500DPS			(1<<4)
#define L3GD20_RANGE_2000DPS		(3<<4)

#define L3GD20_RATE_95HZ			((0<<6) | (0<<4))
#define L3GD20_RATE_190HZ			((1<<6) | (0<<4))
#define L3GD20_RATE_380HZ			((2<<6) | (1<<4))
#define L3GD20_RATE_760HZ			((3<<6) | (2<<4))



/*
 * Sets the address of a shared l3gd20_buffer
 * structure that is maintained by the driver.
 *
 * If zero is passed as the address, disables
 * the buffer updating.
 */
#define L3GD20_SETBUFFER	L3GD20C(3)

struct l3gd20_buffer {
	uint32_t	size;		/* number of entries in the samples[] array */
	uint32_t	next;		/* the next entry that will be populated */
	struct {
		int16_t	x;
		int16_t	y;
		int16_t	z;
	} samples[];
};

extern int	l3gd20_attach(struct spi_dev_s *spi, int spi_id);
