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
 * Driver for the Honeywell/ST HMC5883L MEMS magnetometer
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c.h>
#include <arch/board/board.h>

#include "chip.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_hmc5883l.h>

#define ADDR_CONF_A				0x00
#define ADDR_CONF_B				0x01
#define ADDR_MODE				0x02
#define ADDR_DATA_OUT_X_MSB		0x03
#define ADDR_DATA_OUT_X_LSB		0x04
#define ADDR_DATA_OUT_Z_MSB		0x05
#define ADDR_DATA_OUT_Z_LSB		0x06
#define ADDR_DATA_OUT_Y_MSB		0x07
#define ADDR_DATA_OUT_Y_LSB		0x08
#define ADDR_STATUS				0x09
#define ADDR_ID_A				0x10
#define ADDR_ID_B				0x11
#define ADDR_ID_C				0x12

#define HMC5883L_ADDRESS		0x1E

/* modes not changeable outside of driver */
#define HMC5883L_MODE_NORMAL			(0 << 0)  /* default */
#define HMC5883L_MODE_POSITIVE_BIAS		(1 << 0)  /* positive bias */
#define HMC5883L_MODE_NEGATIVE_BIAS		(1 << 1)  /* negative bias */

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

#define ID_A_WHO_AM_I			'H'
#define ID_B_WHO_AM_I			'4'
#define ID_C_WHO_AM_I			'3'

static FAR struct hmc5883l_dev_s	hmc5883l_dev;

static ssize_t hmc5883l_read(struct file *filp, FAR char *buffer, size_t buflen);
static int hmc5883l_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations hmc5883l_fops = {
	.open  = 0,
	.close = 0,
	.read  = hmc5883l_read,
	.write = 0,
	.seek  = 0,
	.ioctl = hmc5883l_ioctl,
#ifndef CONFIG_DISABLE_POLL
	.poll  = 0
#endif
};

struct hmc5883l_dev_s
{
	struct i2c_dev_s	*i2c;
	uint8_t			rate;
	struct hmc5883l_buffer	*buffer;
};

static int hmc5883l_write_reg(uint8_t address, uint8_t data);
static int hmc5883l_read_reg(uint8_t address);
static int hmc5883l_reset(void);

static int
hmc5883l_write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[] = {address, data};
	return I2C_WRITE(hmc5883l_dev.i2c, cmd, 2);
}

static int
hmc5883l_read_reg(uint8_t address)
{
	uint8_t cmd = address;
	uint8_t data;

	int ret = I2C_WRITEREAD(hmc5883l_dev.i2c, &cmd, 1, &data, 1);
	/* return data on success, error code on failure */
	if (ret == OK) {
		ret = data;
	}
	return ret;
}

static int
hmc5883l_set_range(uint8_t range)
{
	I2C_SETADDRESS(hmc5883l_dev.i2c, HMC5883L_ADDRESS, 7);


	/* mask out illegal bit positions */
	uint8_t write_range = range; //& REG4_RANGE_MASK;
	/* immediately return if user supplied invalid value */
	if (write_range != range) return EINVAL;
	/* set remaining bits to a sane value */
//	write_range |= REG4_BDU;
	/* write to device */
	hmc5883l_write_reg(ADDR_CONF_B, write_range);
	/* return 0 if register value is now written value, 1 if unchanged */
	return !(hmc5883l_read_reg(ADDR_CONF_B) == write_range);
}

static int
hmc5883l_set_rate(uint8_t rate)
{
	I2C_SETADDRESS(hmc5883l_dev.i2c, HMC5883L_ADDRESS, 7);
	/* mask out illegal bit positions */
	uint8_t write_rate = rate;// & REG1_RATE_LP_MASK;
	/* immediately return if user supplied invalid value */
	if (write_rate != rate) return EINVAL;
	/* set remaining bits to a sane value */
//	write_rate |= REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
	write_rate |= HMC5883L_AVERAGING_8;
	/* write to device */
	hmc5883l_write_reg(ADDR_CONF_A, write_rate);
	/* return 0 if register value is now written value, 1 if unchanged */
	return !(hmc5883l_read_reg(ADDR_CONF_A) == write_rate);
}

static bool
read_values(int16_t *data)
{
	struct { /* status register and data as read back from the device */
		int16_t		x;
		int16_t		z;
		int16_t		y;
		uint8_t		status;
	} __attribute__((packed))	hmc_report;
	hmc_report.status = 0;

	static int read_err_count = 0;

	/* exchange the report structure with the device */

	uint8_t cmd = ADDR_DATA_OUT_X_MSB;

	int ret = 0;

	I2C_SETADDRESS(hmc5883l_dev.i2c, HMC5883L_ADDRESS, 7);

	/* set device into single mode, trigger next measurement */
	ret = hmc5883l_write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	/* Only execute consecutive steps on success */
	if (ret == OK)
	  {
		cmd = ADDR_DATA_OUT_X_MSB;
		ret = I2C_WRITEREAD(hmc5883l_dev.i2c, &cmd, 1, (uint8_t*)&hmc_report, 6);
		if (ret == OK)
		  {
			/* Six bytes to read, stop if timed out */
			int hmc_status = hmc5883l_read_reg(ADDR_STATUS);
			if (hmc_status < 0)
			  {
			  	if (hmc_status == ETIMEDOUT) hmc5883l_reset();
			  	ret = hmc_status;
			  }
			else
			  {
			  	hmc_report.status = hmc_status;
			  	ret = OK;
			  }
		  }
		  else
		  {
		  	if (ret == ETIMEDOUT) hmc5883l_reset();
		  }
	  }
	  else
	  {
	  	if (ret == ETIMEDOUT) hmc5883l_reset();
	  }

	if (ret != OK)
	{
		read_err_count++;
		/* If the last reads failed as well, reset the bus and chip */
		if (read_err_count > 3) hmc5883l_reset();

		*get_errno_ptr() = -ret;
	} else {
		read_err_count = 0;
		/* write values, and exchange the two 8bit blocks (big endian to little endian) */
		data[0] = ((hmc_report.x & 0x00FF) << 8) | ((hmc_report.x & 0xFF00) >> 8);
		data[1] = ((hmc_report.y & 0x00FF) << 8) | ((hmc_report.y & 0xFF00) >> 8);
		data[2] = ((hmc_report.z & 0x00FF) << 8) | ((hmc_report.z & 0xFF00) >> 8);
		if ((hmc_report.status & STATUS_REG_DATA_READY) > 0)
		{
			ret = 6;
		} else {
			ret = -EAGAIN;
		}
	}

	/* return len if new data is available, error else. hmc_report.status is 0 on errors */
	return ret;
}

static ssize_t
hmc5883l_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 6) {
		return read_values((int16_t *)buffer);
	}

	/* buffer too small */
	*get_errno_ptr() = ENOSPC;
	return -ERROR;
}

static int
hmc5883l_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

	switch (cmd) {
       case HMC5883L_SETRATE:
        	result = hmc5883l_set_rate(arg);
        	break;

       case HMC5883L_SETRANGE:
        	result = hmc5883l_set_range(arg);
        	break;
//
//    case HMC5883L_SETBUFFER:
//        hmc5883l_dev.buffer = (struct hmc5883l_buffer *)arg;
//        result = 0;
//        break;

        case HMC5883L_RESET:
            result = hmc5883l_reset();
            break;
	}

	if (result)
		errno = EINVAL;
	return result;
}

int hmc5883l_reset()
{
	int ret;
	printf("[hmc5883l drv] Resettet I2C2 BUS\n");
	up_i2cuninitialize(hmc5883l_dev.i2c);
	hmc5883l_dev.i2c = up_i2cinitialize(2);
	I2C_SETFREQUENCY(hmc5883l_dev.i2c, 400000);
	// up_i2creset(hmc5883l_dev.i2c);
	//I2C_SETADDRESS(hmc5883l_dev.i2c, HMC5883L_ADDRESS, 7);
	//hmc5883l_set_range(HMC5883L_RANGE_0_88GA);
	//hmc5883l_set_rate(HMC5883L_RATE_75HZ);
	/* set device into single mode, start measurement */
	//ret = hmc5883l_write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);
	return ret;
}

int
hmc5883l_attach(struct i2c_dev_s *i2c)
{
	int	result = ERROR;

	hmc5883l_dev.i2c = i2c;

//	I2C_LOCK(hmc5883l_dev.i2c, true);
	I2C_SETADDRESS(hmc5883l_dev.i2c, HMC5883L_ADDRESS, 7);

	uint8_t cmd = ADDR_STATUS;
	uint8_t status_id[4] = {0, 0, 0, 0};


	int ret = I2C_WRITEREAD(i2c, &cmd, 1, status_id, 4);

	/* verify that the device is attached and functioning */
	if ((ret >= 0) && (status_id[1] == ID_A_WHO_AM_I) && (status_id[2] == ID_B_WHO_AM_I) && (status_id[3] == ID_C_WHO_AM_I)) {

		/* set update rate to 75 Hz */
		/* set 0.88 Ga range */
		if ((ret != 0) || (hmc5883l_set_range(HMC5883L_RANGE_0_88GA) != 0) ||
				(hmc5883l_set_rate(HMC5883L_RATE_75HZ) != 0))
		{
			errno = EIO;
		} else {

			/* set device into single mode, start measurement */
			ret = hmc5883l_write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

			/* make ourselves available */
			register_driver("/dev/hmc5883l", &hmc5883l_fops, 0666, NULL);

			result = 0;
		}

	} else {
		errno = EIO;
	}



	return result;
}
