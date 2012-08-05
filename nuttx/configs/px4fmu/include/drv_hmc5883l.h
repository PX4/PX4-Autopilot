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
 * Driver for the ST HMC5883L gyroscope
 */

#include <sys/ioctl.h>

#define _HMC5883LBASE	0x6100
#define HMC5883LC(_x)	_IOC(_HMC5883LBASE, _x)

/* 
 * Sets the sensor internal sampling rate, and if a buffer
 * has been configured, the rate at which entries will be
 * added to the buffer.
 */
#define HMC5883L_SETRATE		HMC5883LC(1)

/* set rate (configuration A register */
#define HMC5883L_RATE_0_75HZ		(0 << 2) /* 0.75 Hz */
#define HMC5883L_RATE_1_50HZ		(1 << 2) /* 1.5 Hz */
#define HMC5883L_RATE_3_00HZ		(2 << 2) /* 3 Hz */
#define HMC5883L_RATE_7_50HZ		(3 << 2) /* 7.5 Hz */
#define HMC5883L_RATE_15HZ			(4 << 2) /* 15 Hz (default) */
#define HMC5883L_RATE_30HZ			(5 << 2) /* 30 Hz */
#define HMC5883L_RATE_75HZ			(6 << 2) /* 75 Hz */

/*
 * Sets the sensor internal range.
 */
#define HMC5883L_SETRANGE		HMC5883LC(2)

#define HMC5883L_RANGE_0_88GA			(0 << 5)
#define HMC5883L_RANGE_1_33GA			(1 << 5)
#define HMC5883L_RANGE_1_90GA			(2 << 5)
#define HMC5883L_RANGE_2_50GA			(3 << 5)
#define HMC5883L_RANGE_4_00GA			(4 << 5)

/*
 * Set the sensor measurement mode.
 */
#define HMC5883L_MODE_NORMAL			(0 << 0)
#define HMC5883L_MODE_POSITIVE_BIAS		(1 << 0)
#define HMC5883L_MODE_NEGATIVE_BIAS		(1 << 1)

/*
 * Sets the address of a shared HMC5883L_buffer
 * structure that is maintained by the driver.
 *
 * If zero is passed as the address, disables
 * the buffer updating.
 */
#define HMC5883L_SETBUFFER		HMC5883LC(3)

struct hmc5883l_buffer {
	uint32_t	size;		/* number of entries in the samples[] array */
	uint32_t	next;		/* the next entry that will be populated */
	struct {
		int16_t	x;
		int16_t	y;
		int16_t	z;
	} samples[];
};

#define HMC5883L_RESET			HMC5883LC(4)
#define HMC5883L_CALIBRATION_ON		HMC5883LC(5)
#define HMC5883L_CALIBRATION_OFF	HMC5883LC(6)

extern int	hmc5883l_attach(struct i2c_dev_s *i2c);
