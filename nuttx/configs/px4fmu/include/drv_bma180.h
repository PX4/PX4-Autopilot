/*
 *   Copyright (C) 2012 Lorenz Meier. All rights reserved.
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
 * 3. Neither the name of the author or the names of contributors may be
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
 */

/*
 * Driver for the BOSCH BMA180 MEMS accelerometer
 */

/* IMPORTANT NOTES:
 *
 * SPI max. clock frequency: 25 Mhz
 * CS has to be high before transfer,
 * go low right before transfer and
 * go high again right after transfer
 *
 */

#include <sys/ioctl.h>

#define _BMA180BASE	0x6300
#define BMA180C(_x)	_IOC(_BMA180BASE, _x)

/* 
 * Sets the sensor internal sampling rate, and if a buffer
 * has been configured, the rate at which entries will be
 * added to the buffer.
 */
#define BMA180_SETRATE		BMA180C(1)

#define BMA180_RATE_LP_10HZ		(0<<4)
#define BMA180_RATE_LP_20HZ		(1<<4)
#define BMA180_RATE_LP_40HZ		(2<<4)
#define BMA180_RATE_LP_75HZ		(3<<4)
#define BMA180_RATE_LP_150HZ	(4<<4)
#define BMA180_RATE_LP_300HZ	(5<<4)
#define BMA180_RATE_LP_600HZ	(6<<4)
#define BMA180_RATE_LP_1200HZ	(7<<4)

/*
 * Sets the sensor internal range.
 */
#define BMA180_SETRANGE		BMA180C(2)

#define BMA180_RANGE_1G			(0<<1)
#define BMA180_RANGE_1_5G		(1<<1)
#define BMA180_RANGE_2G			(2<<1)
#define BMA180_RANGE_3G			(3<<1)
#define BMA180_RANGE_4G			(4<<1)
#define BMA180_RANGE_8G			(5<<1)
#define BMA180_RANGE_16G		(6<<1)

/*
 * Sets the address of a shared BMA180_buffer
 * structure that is maintained by the driver.
 *
 * If zero is passed as the address, disables
 * the buffer updating.
 */
#define BMA180_SETBUFFER	BMA180C(3)

struct bma180_buffer {
	uint32_t	size;		/* number of entries in the samples[] array */
	uint32_t	next;		/* the next entry that will be populated */
	struct {
		uint16_t	x;
		uint16_t	y;
		uint16_t	z;
		uint8_t		temp;
	} samples[];
};

extern int	bma180_attach(struct spi_dev_s *spi, int spi_id);
