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
 * Driver for the ST mpu6000 MEMS gyroscope
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_mpu6000.h>

#define DIR_READ					(0x80)
#define DIR_WRITE					(0<<7)
#define ADDR_INCREMENT				(1<<6)

#define WHO_I_AM					0xD4

// MPU 6000 registers
#define MPUREG_WHOAMI 0x75 //
#define MPUREG_SMPLRT_DIV 0x19 //
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_INT_STATUS 0x3A
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 //
#define MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define MPUREG_PWR_MGMT_1 0x6B //
#define MPUREG_PWR_MGMT_2 0x6C //
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_PRODUCT_ID 			0x0C	// Product ID Register


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR      0x10
#define BIT_RAW_RDY_EN        0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA   0x01
											// Product ID Description for MPU6000
											// high 4 bits 	low 4 bits
											// Product Name	Product Revision
#define MPU6000ES_REV_C4 			0x14 	// 0001			0100
#define MPU6000ES_REV_C5 			0x15 	// 0001			0101
#define MPU6000ES_REV_D6 			0x16	// 0001			0110
#define MPU6000ES_REV_D7 			0x17	// 0001			0111
#define MPU6000ES_REV_D8 			0x18	// 0001			1000	
#define MPU6000_REV_C4 				0x54	// 0101			0100 
#define MPU6000_REV_C5 				0x55	// 0101			0101
#define MPU6000_REV_D6 				0x56	// 0101			0110	
#define MPU6000_REV_D7 				0x57	// 0101			0111
#define MPU6000_REV_D8 				0x58	// 0101			1000
#define MPU6000_REV_D9 				0x59	// 0101			1001
#define MPU6000_REV_D10 			0x5A	// 0101			1010

static FAR struct mpu6000_dev_s	mpu6000_dev;

static ssize_t mpu6000_read(struct file *filp, FAR char *buffer, size_t buflen);
static int mpu6000_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations mpu6000_fops = {
	.open  = 0,
	.close = 0,
	.read  = mpu6000_read,
	.write = 0,
	.seek  = 0,
	.ioctl = mpu6000_ioctl,
#ifndef CONFIG_DISABLE_POLL
	.poll  = 0
#endif
};

struct mpu6000_dev_s
{
	struct spi_dev_s	*spi;
	int			spi_id;
	uint8_t			rate;
	struct mpu6000_buffer	*buffer;
};

static void	mpu6000_write_reg(uint8_t address, uint8_t data);
static uint8_t	mpu6000_read_reg(uint8_t address);

static void
mpu6000_write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, true);
	SPI_SNDBLOCK(mpu6000_dev.spi, &cmd, sizeof(cmd));
	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, false);
}

static uint8_t
mpu6000_read_reg(uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, true);
	SPI_EXCHANGE(mpu6000_dev.spi, cmd, data, sizeof(cmd));
	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, false);

	return data[1];
}

static int16_t
mpu6000_read_int16(uint8_t address)
{
	uint8_t	cmd[3] = {address | DIR_READ, 0, 0};
	uint8_t data[3];

	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, true);
	SPI_EXCHANGE(mpu6000_dev.spi, cmd, data, sizeof(cmd));
	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, false);

	return (((int16_t)data[1])<<8) | data[2];
}

static int
mpu6000_set_range(uint8_t range)
{
//	/* mask out illegal bit positions */
//	uint8_t write_range = range & REG4_RANGE_MASK;
//	/* immediately return if user supplied invalid value */
//	if (write_range != range) return EINVAL;
//	/* set remaining bits to a sane value */
//	write_range |= REG4_BDU;
//	/* write to device */
//	write_reg(ADDR_CTRL_REG4, write_range);
//	/* return 0 if register value is now written value, 1 if unchanged */
//	return !(read_reg(ADDR_CTRL_REG4) == write_range);
}

static int
mpu6000_set_rate(uint8_t rate)
{
//	/* mask out illegal bit positions */
//	uint8_t write_rate = rate & REG1_RATE_LP_MASK;
//	/* immediately return if user supplied invalid value */
//	if (write_rate != rate) return EINVAL;
//	/* set remaining bits to a sane value */
//	write_rate |= REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
//	/* write to device */
//	write_reg(ADDR_CTRL_REG1, write_rate);
//	/* return 0 if register value is now written value, 1 if unchanged */
//	return !(read_reg(ADDR_CTRL_REG1) == write_rate);
}

static int
mpu6000_read_fifo(int16_t *data)
{
//	struct {					/* status register and data as read back from the device */
//		uint8_t		cmd;
//		uint8_t		temp;
//		uint8_t		status;
//		int16_t		x;
//		int16_t		y;
//		int16_t		z;
//	} __attribute__((packed))	report;
//
//	report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
//
//	/* exchange the report structure with the device */
//	SPI_LOCK(mpu6000_dev.spi, true);
//
//	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, true);
//
//	read_reg(ADDR_WHO_AM_I);
//
//	SPI_EXCHANGE(mpu6000_dev.spi, &report, &report, sizeof(report));
//	SPI_SELECT(mpu6000_dev.spi, mpu6000_dev.spi_id, false);
//
//	SPI_LOCK(mpu6000_dev.spi, false);
//
//
//
	//

	// Device has MSB first at lower address (big endian)


	struct {					/* status register and data as read back from the device */
		// uint8_t		cmd;
		// uint8_t		int_status;
		int16_t		xacc;
		int16_t		yacc;
		int16_t		zacc;
		int16_t		temp;
		int16_t		rollspeed;
		int16_t		pitchspeed;
		int16_t		yawspeed;
	} report;

	uint8_t cmd[sizeof(report)];
	cmd[0] = MPUREG_ACCEL_XOUT_H | DIR_READ; // was addr_incr

	SPI_LOCK(mpu6000_dev.spi, true);
	SPI_SELECT(mpu6000_dev.spi, PX4_SPIDEV_MPU, true);
	report.xacc = mpu6000_read_int16(MPUREG_ACCEL_XOUT_H);
	report.yacc = mpu6000_read_int16(MPUREG_ACCEL_YOUT_H);
	report.zacc = mpu6000_read_int16(MPUREG_ACCEL_ZOUT_H);
	report.rollspeed = mpu6000_read_int16(MPUREG_GYRO_XOUT_H);
	report.pitchspeed = mpu6000_read_int16(MPUREG_GYRO_YOUT_H);
	report.yawspeed = mpu6000_read_int16(MPUREG_GYRO_ZOUT_H);
	SPI_SELECT(mpu6000_dev.spi, PX4_SPIDEV_MPU, false);
	SPI_LOCK(mpu6000_dev.spi, false);

	data[0] = report.xacc;
	data[1] = report.yacc;
	data[2] = report.zacc;
	data[3] = report.rollspeed;
	data[4] = report.pitchspeed;
	data[5] = report.yawspeed;

	return 1;//(report.int_status & 0x01);
}

static ssize_t
mpu6000_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 12) {
		if (mpu6000_read_fifo((int16_t *)buffer))
			return 12;

		/* no data */
		return 0;
	}

	/* buffer too small */
	errno = ENOSPC;
	return ERROR;
}

static int
mpu6000_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

	switch (cmd) {
        case MPU6000_SETRATE:
            if ((arg & 0x00/* XXX REG MASK MISSING */) == arg) {
            	SPI_LOCK(mpu6000_dev.spi, true);
                mpu6000_set_rate(arg);
                SPI_LOCK(mpu6000_dev.spi, false);
                result = 0;
                mpu6000_dev.rate = arg;
            }
            break;

        case MPU6000_SETRANGE:
            if ((arg & 0x00/* XXX REG MASK MISSING */) == arg) {
            	SPI_LOCK(mpu6000_dev.spi, true);
                mpu6000_set_range(arg);
                SPI_LOCK(mpu6000_dev.spi, false);
                result = 0;
            }
            break;

        case MPU6000_SETBUFFER:
            mpu6000_dev.buffer = (struct mpu6000_buffer *)arg;
            result = 0;
            break;
	}

	if (result)
		errno = EINVAL;
	return result;
}

int
mpu6000_attach(struct spi_dev_s *spi, int spi_id)
{
	int	result = ERROR;

	mpu6000_dev.spi = spi;
	mpu6000_dev.spi_id = spi_id;

	SPI_LOCK(mpu6000_dev.spi, true);

	// Set sensor-specific SPI mode
	SPI_SETFREQUENCY(mpu6000_dev.spi, 10000000); // 500 KHz
	SPI_SETBITS(mpu6000_dev.spi, 8);
	// Either mode 1 or mode 3
	SPI_SETMODE(mpu6000_dev.spi, SPIDEV_MODE3);

	// Chip reset
	mpu6000_write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	up_udelay(10000);
	// Wake up device and select GyroZ clock (better performance)
	mpu6000_write_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	up_udelay(1000);
	// Disable I2C bus (recommended on datasheet)
	mpu6000_write_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
	up_udelay(1000);
    // SAMPLE RATE
	mpu6000_write_reg(MPUREG_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
    usleep(1000);
    // FS & DLPF   FS=2000¼/s, DLPF = 98Hz (low pass filter)
    mpu6000_write_reg(MPUREG_CONFIG, BITS_DLPF_CFG_98HZ);
    usleep(1000);
    mpu6000_write_reg(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000¼/s
    usleep(1000);

	uint8_t _product_id = mpu6000_read_reg(MPUREG_PRODUCT_ID);
	printf("MPU-6000 product id: %d\n", (int)_product_id);

	if ((_product_id == MPU6000ES_REV_C4) || (_product_id == MPU6000ES_REV_C5) ||
		(_product_id == MPU6000_REV_C4)   || (_product_id == MPU6000_REV_C5)){
		// Accel scale 8g (4096 LSB/g)
		// Rev C has different scaling than rev D
		mpu6000_write_reg(MPUREG_ACCEL_CONFIG,1<<3);
	} else {
		// Accel scale 8g (4096 LSB/g)
		mpu6000_write_reg(MPUREG_ACCEL_CONFIG,2<<3);
	}
    usleep(1000);

    // INT CFG => Interrupt on Data Ready
    mpu6000_write_reg(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    usleep(1000);
    mpu6000_write_reg(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
    usleep(1000);
    // Oscillator set
    // write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
    usleep(1000);

    /* revert back to normal bus mode */
	SPI_SETFREQUENCY(mpu6000_dev.spi, 10000000);
	SPI_SETBITS(mpu6000_dev.spi, 8);
	SPI_SETMODE(mpu6000_dev.spi, SPIDEV_MODE3);

	/* verify that the device is attached and functioning */
	if ((_product_id == MPU6000ES_REV_C4) || (_product_id == MPU6000ES_REV_C5) ||
		(_product_id == MPU6000_REV_C4)   || (_product_id == MPU6000_REV_C5) ||
		(_product_id == MPU6000_REV_D7)	  || (_product_id == MPU6000_REV_D8) ||
		(_product_id == MPU6000_REV_D9)   || (_product_id == MPU6000_REV_D10)){

		/* make ourselves available */
		register_driver("/dev/mpu6000", &mpu6000_fops, 0666, NULL);

		result = OK;
	} else {

		errno = EIO;
	}

	SPI_LOCK(mpu6000_dev.spi, false);

	SPI_LOCK(mpu6000_dev.spi, false);

	return result;
}
