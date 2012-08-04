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
 * Driver for the Invense MPU-6000 gyroscope
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

#define _MPU6000BASE	0x7600
#define MPU6000C(_x)	_IOC(_MPU6000BASE, _x)

/* 
 * Sets the sensor internal sampling rate, and if a buffer
 * has been configured, the rate at which entries will be
 * added to the buffer.
 */
#define MPU6000_SETRATE		MPU6000C(1)

#define MPU6000_RATE_95HZ_LP_12_5HZ		((0<<7) | (0<<6) | (0<<5) | (0<<4))

/*
 * Sets the sensor internal range.
 */
#define MPU6000_SETRANGE		MPU6000C(2)

#define MPU6000_RANGE_250DPS			(0<<4)

#define MPU6000_RATE_95HZ			((0<<6) | (0<<4))



/*
 * Sets the address of a shared MPU6000_buffer
 * structure that is maintained by the driver.
 *
 * If zero is passed as the address, disables
 * the buffer updating.
 */
#define MPU6000_SETBUFFER	MPU6000C(3)

struct MPU6000_buffer {
	uint32_t	size;		/* number of entries in the samples[] array */
	uint32_t	next;		/* the next entry that will be populated */
	struct {
		uint16_t	x;
		uint16_t	y;
		uint16_t	z;
		uint16_t	roll;
		uint16_t	pitch;
		uint16_t	yaw;
	} samples[];
};

extern int	mpu6000_attach(struct spi_dev_s *spi, int spi_id);
