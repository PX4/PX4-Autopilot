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
 * Driver for the Measurement Specialties MS5611 barometric pressure sensor
 */

#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "chip.h"
#include "px4fmu-internal.h"

#include <arch/board/up_hrt.h>
#include <arch/board/drv_ms5611.h>

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5611_MIN_INTER_MEASUREMENT_INTERVAL 9200

#define MS5611_ADDRESS_1		0x76    /* address select pins pulled high (PX4FMU series v1.6+) */
#define MS5611_ADDRESS_2		0x77    /* address select pins pulled low (PX4FMU prototypes) */

#define ADDR_RESET_CMD			0x1E /* read from this address to reset chip (0b0011110 on bus) */
#define ADDR_CMD_CONVERT_D1		0x48 /* 4096 samples to this address to start conversion (0b01001000 on bus) */
#define ADDR_CMD_CONVERT_D2		0x58 /* 4096 samples */
#define ADDR_DATA				0x00 /* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP			0xA0 /* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1			0xA2 /* address of 6x 2 bytes calibration data */

static FAR struct ms5611_dev_s	ms5611_dev;

static ssize_t ms5611_read(struct file *filp, FAR char *buffer, size_t buflen);
static int ms5611_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations ms5611_fops = {
	.read  = ms5611_read,
	.ioctl = ms5611_ioctl,
};

struct ms5611_prom_s
{
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
} __attribute__((packed));

union ms5611_prom_u
{
	uint16_t c[8];
	struct ms5611_prom_s s;
} __attribute__((packed));

struct ms5611_dev_s
{
	union ms5611_prom_u 	prom;
	struct i2c_dev_s		*i2c;
	struct ms5611_buffer	*buffer;
} __attribute__((packed));

static FAR uint8_t MS5611_ADDRESS;

static FAR struct {
		/* status register and data as read back from the device */
		float		pressure;
		float		altitude;
		float		temperature;
		uint32_t	d1_raw;
		uint32_t	d2_raw;
		uint32_t	measurements_count;
		uint8_t		last_state;
		uint64_t    last_read;
	} ms5611_report = {
			.pressure = 0.0f,
			.altitude = 0.0f,
			.temperature = 0.0f,
			.last_state = 0,
			/* make sure the first readout can be performed */
			.last_read = 0,
};

static int ms5611_read_prom(void);

static bool
read_values(float *data)
{
	int ret;
	uint8_t cmd_data[3];

	/* check validity of pointer */
	if (data == NULL)
	{
		*get_errno_ptr() = EINVAL;
		return -EINVAL;
	}

	/* only start reading when data is available */
	if (ms5611_report.measurements_count > 0)
	{
		/* do not read more often than at minimum 9.17 ms intervals */
		if ((hrt_absolute_time() - ms5611_report.last_read) < MS5611_MIN_INTER_MEASUREMENT_INTERVAL)
		{
			/* set errno to 'come back later' */
			ret = -EAGAIN;
			goto handle_return;
		}
		else
		{
			/* set new value */
			ms5611_report.last_read = hrt_absolute_time();
		}

		/* Read out last measurement */
		cmd_data[0] = 0x00;

		struct i2c_msg_s msgv[2] = {
				{
						.addr   = MS5611_ADDRESS,
						.flags  = 0,
						.buffer = cmd_data,
						.length = 1
				},
				{
						.addr   = MS5611_ADDRESS,
						.flags  = I2C_M_READ,
						.buffer = cmd_data,
						.length = 3
				}
		};
		ret = I2C_TRANSFER(ms5611_dev.i2c, msgv, 2);
		if (ret != OK) goto handle_return;


		/* at value 1 the last reading was temperature */
		if (ms5611_report.last_state == 1)
		{
			/* put temperature into the raw set */
			ms5611_report.d2_raw = (((uint32_t)cmd_data[0]) << 16) | (((uint32_t)cmd_data[1]) << 8) | ((uint32_t)cmd_data[2]);
		}
		else
		{
			/* put altitude into the raw set */
			ms5611_report.d1_raw = (((uint32_t)cmd_data[0]) << 16) | (((uint32_t)cmd_data[1]) << 8) | ((uint32_t)cmd_data[2]);
		}
	}
	ms5611_report.measurements_count++;

	/*
	 * this block reads four pressure values and one temp value,
	 * resulting in 80 Hz pressure update and 20 Hz temperature updates
	 * at 100 Hz continuous operation.
	 */
	if (ms5611_report.last_state == 0)
	{
		/* request first a temperature reading */
		cmd_data[0] = ADDR_CMD_CONVERT_D2;
	}
	else
	{
		/* request pressure reading */
		cmd_data[0] = ADDR_CMD_CONVERT_D1;
	}

	if (ms5611_report.last_state == 3)
	{
		ms5611_report.last_state = 0;
	}
	else
	{
		ms5611_report.last_state++;
	}


	/* write measurement command */
	struct i2c_msg_s conv_cmd[1] = {
			{
					.addr   = MS5611_ADDRESS,
					.flags  = 0,
					.buffer = cmd_data,
					.length = 1
			},
	};

	ret = I2C_TRANSFER(ms5611_dev.i2c, conv_cmd, 1);
	if (ret != OK) goto handle_return;

	/* only write back values after first complete set */
	if (ms5611_report.measurements_count > 2)
	{
		/* Calculate results */

		/* temperature calculation */
		int32_t dT = ms5611_report.d2_raw - (((int32_t)ms5611_dev.prom.s.c5_reference_temp)*256);
		int64_t temp_int64 = 2000 + (((int64_t)dT)*ms5611_dev.prom.s.c6_temp_coeff_temp)/8388608;

		/* pressure calculation */
		int64_t offset = (int64_t)ms5611_dev.prom.s.c2_pressure_offset * 65536 + ((int64_t)dT*ms5611_dev.prom.s.c4_temp_coeff_pres_offset)/128;
		int64_t sens = (int64_t)ms5611_dev.prom.s.c1_pressure_sens * 32768 + ((int64_t)dT*ms5611_dev.prom.s.c3_temp_coeff_pres_sens)/256;

		/* it's pretty cold, second order temperature compensation needed */
		if (temp_int64 < 2000)
		{
			/* second order temperature compensation */
			int64_t temp2 = (((int64_t)dT)*dT) >> 31;
			int64_t tmp_64 = (temp_int64-2000)*(temp_int64-2000);
			int64_t offset2 = (5*tmp_64)>>1;
			int64_t sens2 = (5*tmp_64)>>2;
			temp_int64 = temp_int64 - temp2;
			offset = offset - offset2;
			sens = sens - sens2;
		}

		int64_t press_int64 = (((ms5611_report.d1_raw*sens)/2097152-offset)/32768);

		ms5611_report.temperature = temp_int64 / 100.0f;
		ms5611_report.pressure = press_int64 / 100.0f;
		/* convert as double for max. precision, store as float (more than enough precision) */
		ms5611_report.altitude = (44330.0 * (1.0 - pow((press_int64 / 101325.0), 0.190295)));

		/* Write back float values */
		data[0] = ms5611_report.pressure;
		data[1] = ms5611_report.altitude;
		data[2] = ms5611_report.temperature;
	}
	else
	{
		/* not ready, try again */
		ret = -EINPROGRESS;
	}

	/* return 1 if new data is available, 0 else */
	handle_return:
	if (ret == OK)
	{
		return (sizeof(ms5611_report.d1_raw) + sizeof(ms5611_report.altitude) + sizeof(ms5611_report.d2_raw));
	}
	else
	{
		errno = -ret;
		return ret;
	}
}

static ssize_t
ms5611_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 12) {
		return read_values((float *)buffer);
	}

	/* buffer too small */
	errno = ENOSPC;
	return -ENOSPC;
}

static int
ms5611_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return -ENOSYS;

//	switch (cmd) {
//        case MS5611_SETRATE:
//            if ((arg & REG1_RATE_LP_MASK) == arg) {
//                set_rate(arg);
//                result = 0;
//                dev.rate = arg;
//            }
//            break;
//
//        case MS5611_SETBUFFER:
//            dev.buffer = (struct ms5611_buffer *)arg;
//            result = 0;
//            break;
//	}
//
//	if (result)
//		errno = EINVAL;
//	return result;
}



int ms5611_crc4(uint16_t n_prom[])
{
	/* routine ported from MS5611 application note */
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;
	n_rem = 0x00;
	/* save the read crc */
	crc_read = n_prom[7];
	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));
	for (cnt = 0; cnt < 16; cnt++)
	{
		/* uneven bytes */
		if (cnt & 1)
			{
			n_rem ^= (uint8_t) ((n_prom[cnt>>1]) & 0x00FF);
			}
		else
		{
			n_rem ^= (uint8_t) (n_prom[cnt>>1] >> 8);
		}
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & 0x8000)
			{
				n_rem = (n_rem << 1) ^ 0x3000;

			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return 0 == OK if CRCs match, 1 else */
	return !((0x000F & crc_read) == (n_rem ^ 0x00));
}


int ms5611_read_prom()
{
	/* read PROM data */
	uint8_t prom_buf[2] = {255,255};

	int retval = 0;

	for (int i = 0; i < 8; i++)
	{
		uint8_t cmd = {ADDR_PROM_SETUP + (i*2)};

		I2C_SETADDRESS(ms5611_dev.i2c, MS5611_ADDRESS, 7);
		retval = I2C_WRITEREAD(ms5611_dev.i2c, &cmd, 1, prom_buf, 2);

		/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
		ms5611_dev.prom.c[i] = (((uint16_t)prom_buf[0])<<8) | ((uint16_t)prom_buf[1]);

		if (retval != OK)
		{
			break;
		}
	}

	/* calculate CRC and return error if mismatch */
	return ms5611_crc4(ms5611_dev.prom.c);
}

int
ms5611_attach(struct i2c_dev_s *i2c)
{
	int	result = ERROR;

	ms5611_dev.i2c = i2c;

	MS5611_ADDRESS = MS5611_ADDRESS_1;

	/* write reset command */
	uint8_t cmd_data = ADDR_RESET_CMD;

	struct i2c_msg_s reset_cmd[1] = {
			{
					.addr   = MS5611_ADDRESS,
					.flags  = 0,
					.buffer = &cmd_data,
					.length = 1
			},
	};

	int ret = I2C_TRANSFER(ms5611_dev.i2c, reset_cmd, 1);

	if (ret == OK)
	{
		/* wait for PROM contents to be in the device (2.8 ms) */
		up_udelay(3000);

		/* read PROM */
		ret = ms5611_read_prom();
	}

	/* check if the address was wrong */
	if (ret != OK)
	{
		/* try second address */
		MS5611_ADDRESS = MS5611_ADDRESS_2;

		/* write reset command */
		cmd_data = ADDR_RESET_CMD;

		struct i2c_msg_s reset_cmd_2[1] = {
				{
						.addr   = MS5611_ADDRESS,
						.flags  = 0,
						.buffer = &cmd_data,
						.length = 1
				},
		};

		ret = I2C_TRANSFER(ms5611_dev.i2c, reset_cmd_2, 1);

		/* wait for PROM contents to be in the device (2.8 ms) */
		up_udelay(3000);

		/* read PROM */
		ret = ms5611_read_prom();
	}

	if (ret < 0) return -EIO;


	/* verify that the device is attached and functioning */
	if (ret == OK) {

		if (MS5611_ADDRESS == MS5611_ADDRESS_1)
		{
			printf("[ms5611 driver] Attached MS5611 at addr #1 (0x76)\n");
		}
		else
		{
			printf("[ms5611 driver] Attached MS5611 at addr #2 (0x77)\n");
		}

		/* trigger temperature read */
		(void)read_values(NULL);
		/* wait for conversion to complete */
		up_udelay(9200);

		/* trigger pressure read */
		(void)read_values(NULL);
		/* wait for conversion to complete */
		up_udelay(9200);
		/* now a read_values call would obtain valid results */

		/* make ourselves available */
		register_driver("/dev/ms5611", &ms5611_fops, 0666, NULL);

		result = OK;

	} else {
		errno = EIO;
	}

	return result;
}
