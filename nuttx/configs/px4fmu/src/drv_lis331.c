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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_lis331.h>

/*
 * LIS331 registers
 */

#define DIR_READ		(1<<7)
#define DIR_WRITE		(0<<7)
#define ADDR_INCREMENT		(1<<6)

#define ADDR_WHO_AM_I		0x0f
#define WHO_I_AM		0x32

#define ADDR_CTRL_REG1		0x20		/* sample rate constants are in the public header */
#define REG1_POWER_NORMAL		(1<<5)
#define REG1_RATE_MASK			(3<<3)
#define REG1_Z_ENABLE			(1<<2)
#define REG1_Y_ENABLE			(1<<1)
#define REG1_X_ENABLE			(1<<0)

#define ADDR_CTRL_REG2		0x21

#define ADDR_CTRL_REG3		0x22

#define ADDR_CTRL_REG4		0x23
#define REG4_BDU			(1<<7)
#define REG4_BIG_ENDIAN			(1<<6)
#define REG4_RANGE_MASK			(3<<4)
#define REG4_SPI_3WIRE			(1<<0)

#define ADDR_CTRL_REG5		0x24

#define ADDR_HP_FILTER_RESET	0x25
#define ADDR_REFERENCE		0x26
#define ADDR_STATUS_REG		0x27
#define STATUS_ZYXOR			(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA			(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define ADDR_OUT_X		0x28	/* 16 bits */
#define ADDR_OUT_Y		0x2A	/* 16 bits */
#define ADDR_OUT_Z		0x2C	/* 16 bits */


static ssize_t	lis331_read(struct file *filp, FAR char *buffer, size_t buflen);
static int	lis331_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations lis331_fops = {
	.read  = lis331_read,
	.ioctl = lis331_ioctl,
};

struct lis331_dev_s
{
	struct spi_dev_s	*spi;
	int			spi_id;

	uint8_t			rate;
	struct lis331_buffer	*buffer;
};

static struct lis331_dev_s	lis331_dev;

static void	write_reg(uint8_t address, uint8_t data);
static uint8_t	read_reg(uint8_t address);
static bool	read_fifo(uint16_t *data);
static void	set_range(uint8_t range);
static void	set_rate(uint8_t rate);

static void
write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	SPI_SELECT(lis331_dev.spi, lis331_dev.spi_id, true);
      	SPI_SNDBLOCK(lis331_dev.spi, &cmd, sizeof(cmd));
	SPI_SELECT(lis331_dev.spi, lis331_dev.spi_id, false);
}

static uint8_t
read_reg(uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	SPI_SELECT(lis331_dev.spi, lis331_dev.spi_id, true);
	SPI_EXCHANGE(lis331_dev.spi, cmd, data, sizeof(cmd));
	SPI_SELECT(lis331_dev.spi, lis331_dev.spi_id, false);

	return data[1];	
}

static bool
read_fifo(uint16_t *data)
{
	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} __attribute__((packed))	report;

	report.cmd = ADDR_STATUS_REG | DIR_READ | ADDR_INCREMENT;

	/* exchange the report structure with the device */
	SPI_LOCK(lis331_dev.spi, true);
	SPI_SELECT(lis331_dev.spi, lis331_dev.spi_id, true);
	SPI_EXCHANGE(lis331_dev.spi, &report, &report, sizeof(report));
	SPI_SELECT(lis331_dev.spi, lis331_dev.spi_id, false);
	SPI_LOCK(lis331_dev.spi, false);

	data[0] = report.x;
	data[1] = report.y;
	data[2] = report.z;

	return report.status & STATUS_ZYXDA;
}

static void
set_range(uint8_t range)
{
	range &= REG4_RANGE_MASK;
	write_reg(ADDR_CTRL_REG4, range | REG4_BDU);
}

static void
set_rate(uint8_t rate)
{
	rate &= REG1_RATE_MASK;
	write_reg(ADDR_CTRL_REG1, rate | REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
}

static ssize_t
lis331_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 12) {
		if (read_fifo((uint16_t *)buffer))
			return 12;

		/* no data */
		return 0;
	}

	/* buffer too small */
	errno = ENOSPC;
	return ERROR;
}

static int
lis331_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

	switch (cmd) {
	case LIS331_SETRATE:
		if ((arg & REG1_RATE_MASK) == arg) {
			set_rate(arg);
			result = 0;
			lis331_dev.rate = arg;
		}
		break;

	case LIS331_SETRANGE:
		if ((arg & REG4_RANGE_MASK) == arg) {
			set_range(arg);
			result = 0;
		}
		break;

	case LIS331_SETBUFFER:
		lis331_dev.buffer = (struct lis331_buffer *)arg;
		result = 0;
		break;
	}

	if (result)
		errno = EINVAL;
	return result;
}

int
lis331_attach(struct spi_dev_s *spi, int spi_id)
{
	int	result = ERROR;
	
	lis331_dev.spi = spi;

	SPI_LOCK(lis331_dev.spi, true);

	/* verify that the device is attached and functioning */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {

		/* set default configuration */
		write_reg(ADDR_CTRL_REG2, 0);	/* disable interrupt-generating high-pass filters */
		write_reg(ADDR_CTRL_REG3, 0);	/* no interrupts - we don't use them */
		write_reg(ADDR_CTRL_REG5, 0);	/* disable wake-on-interrupt */

		set_range(LIS331_RANGE_4G);
		set_rate(LIS331_RATE_400Hz);	/* takes device out of low-power mode */

		/* make ourselves available */
		register_driver("/dev/lis331", &lis331_fops, 0666, NULL);

		result = 0;
	} else {
		errno = EIO;
	}

	SPI_LOCK(lis331_dev.spi, false);

	return result;
}

