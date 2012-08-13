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
 * Driver for the ST L3GD20 MEMS gyroscope
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <arch/board/drv_l3gd20.h>

#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#define DIR_READ					(1<<7)
#define DIR_WRITE					(0<<7)
#define ADDR_INCREMENT				(1<<6)

#define ADDR_WHO_AM_I				0x0F
#define WHO_I_AM					0xD4
#define ADDR_CTRL_REG1				0x20
#define ADDR_CTRL_REG2				0x21
#define ADDR_CTRL_REG3				0x22
#define ADDR_CTRL_REG4				0x23
#define ADDR_CTRL_REG5				0x24
#define ADDR_REFERENCE				0x25
#define ADDR_OUT_TEMP				0x26
#define ADDR_STATUS_REG				0x27
#define ADDR_OUT_X_L				0x28
#define ADDR_OUT_X_H				0x29
#define ADDR_OUT_Y_L				0x2A
#define ADDR_OUT_Y_H				0x2B
#define ADDR_OUT_Z_L				0x2C
#define ADDR_OUT_Z_H				0x2D
#define ADDR_FIFO_CTRL_REG			0x2E
#define ADDR_FIFO_SRC_REG			0x2F
#define ADDR_INT1_CFG				0x30
#define ADDR_INT1_SRC				0x31
#define ADDR_INT1_TSH_XH			0x32
#define ADDR_INT1_TSH_XL			0x33
#define ADDR_INT1_TSH_YH			0x34
#define ADDR_INT1_TSH_YL			0x35
#define ADDR_INT1_TSH_ZH			0x36
#define ADDR_INT1_TSH_ZL			0x37
#define ADDR_INT1_DURATION			0x38

#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */

/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU					(1<<7)
#define REG4_BLE					(1<<6)
//#define REG4_SPI_3WIRE				(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR					(1<<6)
#define STATUS_YOR					(1<<5)
#define STATUS_XOR					(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA					(1<<2)
#define STATUS_YDA					(1<<1)
#define STATUS_XDA					(1<<0)

#define FIFO_CTRL_BYPASS_MODE				(0<<5)
#define FIFO_CTRL_FIFO_MODE					(1<<5)
#define FIFO_CTRL_STREAM_MODE				(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

static FAR struct l3gd20_dev_s	l3gd20_dev;

static ssize_t l3gd20_read(struct file *filp, FAR char *buffer, size_t buflen);
static int l3gd20_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations l3gd20_fops = {
	.open  = 0,
	.close = 0,
	.read  = l3gd20_read,
	.write = 0,
	.seek  = 0,
	.ioctl = l3gd20_ioctl,
#ifndef CONFIG_DISABLE_POLL
	.poll  = 0
#endif
};

struct l3gd20_dev_s
{
	struct spi_dev_s	*spi;
	int			spi_id;
	uint8_t			rate;
	struct l3gd20_buffer	*buffer;
};

static void	l3gd20_write_reg(uint8_t address, uint8_t data);
static uint8_t	l3gd20_read_reg(uint8_t address);

static void
l3gd20_write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	SPI_SELECT(l3gd20_dev.spi, l3gd20_dev.spi_id, true);
	SPI_SNDBLOCK(l3gd20_dev.spi, &cmd, sizeof(cmd));
	SPI_SELECT(l3gd20_dev.spi, l3gd20_dev.spi_id, false);
}

static uint8_t
l3gd20_read_reg(uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	SPI_SELECT(l3gd20_dev.spi, l3gd20_dev.spi_id, true);
	SPI_EXCHANGE(l3gd20_dev.spi, cmd, data, sizeof(cmd));
	SPI_SELECT(l3gd20_dev.spi, l3gd20_dev.spi_id, false);

	return data[1];
}

static int
set_range(uint8_t range)
{
	/* mask out illegal bit positions */
	uint8_t write_range = range & REG4_RANGE_MASK;
	/* immediately return if user supplied invalid value */
	if (write_range != range) return EINVAL;
	/* set remaining bits to a sane value */
	write_range |= REG4_BDU;
	/* write to device */
	l3gd20_write_reg(ADDR_CTRL_REG4, write_range);
	/* return 0 if register value is now written value, 1 if unchanged */
	return !(l3gd20_read_reg(ADDR_CTRL_REG4) == write_range);
}

static int
set_rate(uint8_t rate)
{
	/* mask out illegal bit positions */
	uint8_t write_rate = rate & REG1_RATE_LP_MASK;
	/* immediately return if user supplied invalid value */
	if (write_rate != rate) return EINVAL;
	/* set remaining bits to a sane value */
	write_rate |= REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
	/* write to device */
	l3gd20_write_reg(ADDR_CTRL_REG1, write_rate);
	/* return 0 if register value is now written value, 1 if unchanged */
	return !(l3gd20_read_reg(ADDR_CTRL_REG1) == write_rate);
}

static int
read_fifo(int16_t *data)
{

	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		uint8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} __attribute__((packed))	report = {.status = 11};

	report.cmd = 0x26 | DIR_READ | ADDR_INCREMENT;

	SPI_LOCK(l3gd20_dev.spi, true);
	SPI_SELECT(l3gd20_dev.spi, PX4_SPIDEV_GYRO, true);
	SPI_SETFREQUENCY(l3gd20_dev.spi, 25000000);

	SPI_EXCHANGE(l3gd20_dev.spi, &report, &report, sizeof(report));

	/* XXX if the status value is unchanged, attempt a second exchange */
	if (report.status == 11) SPI_EXCHANGE(l3gd20_dev.spi, &report, &report, sizeof(report));
	/* XXX set magic error value if this still didn't succeed */
	if (report.status == 11) report.status = 12;

	SPI_SETFREQUENCY(l3gd20_dev.spi, 10000000);
	SPI_SELECT(l3gd20_dev.spi, PX4_SPIDEV_GYRO, false);
	SPI_LOCK(l3gd20_dev.spi, false);

	data[0] = report.x;
	data[1] = report.y;
	data[2] = report.z;

	/* if all axes are valid, return buflen (6), else return negative status */
	int ret = -((int)report.status);
	if (STATUS_ZYXDA == (report.status & STATUS_ZYXDA) || STATUS_ZYXOR == (report.status & STATUS_ZYXOR))
	{
		ret = 6;
	}

	return ret;
}

static ssize_t
l3gd20_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 6) {
		/* return buflen or a negative value */
		int ret = read_fifo((int16_t *)buffer);
		if (ret != 6) *get_errno_ptr() = EAGAIN;
		return ret;
	}

	/* buffer too small */
	*get_errno_ptr() = ENOSPC;
	return ERROR;
}

static int
l3gd20_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

	switch (cmd) {
        case L3GD20_SETRATE:
            if ((arg & REG1_RATE_LP_MASK) == arg) {
            	SPI_LOCK(l3gd20_dev.spi, true);
                set_rate(arg);
                SPI_LOCK(l3gd20_dev.spi, false);
                result = 0;
                l3gd20_dev.rate = arg;
            }
            break;

        case L3GD20_SETRANGE:
            if ((arg & REG4_RANGE_MASK) == arg) {
            	SPI_LOCK(l3gd20_dev.spi, true);
                set_range(arg);
                SPI_LOCK(l3gd20_dev.spi, false);
                result = 0;
            }
            break;

        case L3GD20_SETBUFFER:
            l3gd20_dev.buffer = (struct l3gd20_buffer *)arg;
            result = 0;
            break;
	}

	if (result)
		errno = EINVAL;
	return result;
}

int
l3gd20_attach(struct spi_dev_s *spi, int spi_id)
{
	int	result = ERROR;

	l3gd20_dev.spi = spi;
	l3gd20_dev.spi_id = spi_id;

	SPI_LOCK(l3gd20_dev.spi, true);
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)l3gd20_read_reg(ADDR_WHO_AM_I);

	/* verify that the device is attached and functioning */
	if (l3gd20_read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {

		/* reset device memory */
		//l3gd20_write_reg(ADDR_CTRL_REG5, REG5_REBOOT_MEMORY);
		//up_udelay(1000);

		/* set default configuration */
 	 	l3gd20_write_reg(ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
		l3gd20_write_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
		l3gd20_write_reg(ADDR_CTRL_REG3, 0);		/* no interrupts - we don't use them */
		l3gd20_write_reg(ADDR_CTRL_REG4, 0x10);
		l3gd20_write_reg(ADDR_CTRL_REG5, 0);

		l3gd20_write_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);	  /* disable wake-on-interrupt */
		l3gd20_write_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_STREAM_MODE); /* Enable FIFO, old data is overwritten */

		if ((set_range(L3GD20_RANGE_500DPS) != 0) ||
				(set_rate(L3GD20_RATE_760HZ_LP_100HZ) != 0))	/* takes device out of low-power mode */
		{
			errno = EIO;
		} else {
			/* Read out the first few funky values */
			struct {					/* status register and data as read back from the device */
				uint8_t		cmd;
				uint8_t		temp;
				uint8_t		status;
				int16_t		x;
				int16_t		y;
				int16_t		z;
			} __attribute__((packed))	report;

			report.cmd = 0x26 | DIR_READ | ADDR_INCREMENT;

			SPI_SELECT(spi, PX4_SPIDEV_GYRO, true);
			SPI_EXCHANGE(spi, &report, &report, sizeof(report));
			SPI_SELECT(spi, PX4_SPIDEV_GYRO, false);
			up_udelay(500);
			/* And read another set */
			SPI_SELECT(spi, PX4_SPIDEV_GYRO, true);
			SPI_EXCHANGE(spi, &report, &report, sizeof(report));
			SPI_SELECT(spi, PX4_SPIDEV_GYRO, false);


			/* make ourselves available */
			register_driver("/dev/l3gd20", &l3gd20_fops, 0666, NULL);

			result = 0;
		}

	} else {

		errno = EIO;
	}

	SPI_LOCK(l3gd20_dev.spi, false);

	return result;
}
