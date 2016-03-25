/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file mpu9250.cpp
 *
 * Driver for the Invensense MPU9250 connected via SPI.
 *
 * @author Andrew Tridgell
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>

#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>

#include "mag.h"
#include "mpu9250.h"

#define DIR_READ			0x80
#define DIR_WRITE			0x00

// MPU 9250 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10

#define MPU_WHOAMI_9250			0x71

MPU9250::MPU9250(int filter) :
	SPI("MPU9250", "/dev/mpu9250_accel", PX4_SPI_BUS_SENSORS, (spi_dev_e)PX4_SPIDEV_MPU, SPIDEV_MODE3, MPU9250_LOW_BUS_SPEED),
	_mag(new MPU9250_mag(this, "/dev/mpu9250_mag")),
	_whoami(0),
	_call{},
	_last_accel_data{},
	_got_duplicate(false),
	_filter(filter)
//	mag()
{
	// disable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MPU9250;

	/* Prime _mag with parents devid. */
	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_MPU9250;

	memset(&_call, 0, sizeof(_call));
}

MPU9250::~MPU9250()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the magnetometer subdriver */
	delete _mag;
}

int
MPU9250::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		goto out;
	}

	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 probe failed");
		goto out;
	}

	ret = reset();

	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 reset failed");
		goto out;
	}

	ret = _mag->init();

	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		goto out;
	}

out:
	return ret;
}

int MPU9250::reset()
{
	write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	up_udelay(10000);

	write_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_AUTO);
	up_udelay(1000);

	write_reg(MPUREG_PWR_MGMT_2, 0);
	up_udelay(1000);

	// SAMPLE RATE
	_set_sample_rate(1000);
	usleep(1000);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	switch (_filter) {
		case 1:
			break;
		case 0:
		default:
			write_reg(MPUREG_CONFIG, BITS_DLPF_CFG_41HZ);
			break;
	}
	usleep(1000);


	// Gyro scale 2000 deg/s ()
	write_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	usleep(1000);

	write_reg(MPUREG_ACCEL_CONFIG, 3 << 3);

	usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	usleep(1000);
	write_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	usleep(1000);

	return OK;
}

int
MPU9250::probe()
{
	/* look for device ID */
	_whoami = read_reg(MPUREG_WHOAMI);

	// verify product revision
	switch (_whoami) {
	case MPU_WHOAMI_9250:
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MPU9250::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }
	if (div < 1) { div = 1; }

	write_reg(MPUREG_SMPLRT_DIV, div - 1);
}

uint8_t
MPU9250::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	set_frequency(speed);

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
MPU9250::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	// general register transfer at low clock speed
	set_frequency(MPU9250_LOW_BUS_SPEED);

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
MPU9250::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	set_frequency(MPU9250_LOW_BUS_SPEED);

	transfer(cmd, nullptr, sizeof(cmd));
}

void
MPU9250::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU9250::start(int interval)
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified interval */
	hrt_call_every(&_call,
		       1000,
		       interval,
		       (hrt_callout)&MPU9250::measure_trampoline, this);
}

void
MPU9250::stop()
{
	hrt_cancel(&_call);
}

void measure_callback(struct Report &report);

void
MPU9250::measure_trampoline(void *arg)
{
	MPU9250 *dev = reinterpret_cast<MPU9250 *>(arg);

	struct Report report;

	/* make another measurement */
	memset(&report, 0, sizeof(report));
	if (dev->measure(report)) {
		measure_callback(report);
	}
}

void MPU9250::set_frequency_high()
{
	set_frequency(MPU9250_HIGH_BUS_SPEED);
}

bool MPU9250::check_duplicate(uint8_t *accel_data)
{
	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
//	if (!_got_duplicate && memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
	if (memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		// it isn't new data - wait for next timer
		_got_duplicate = true;

	} else {
		memcpy(&_last_accel_data, accel_data, sizeof(_last_accel_data));
		_got_duplicate = false;
	}

	return _got_duplicate;
}

bool
MPU9250::measure(struct Report &report)
{
	bool ret = false;
	struct MPUReport mpu_report;

	/*
	 * Fetch the full set of measurements from the MPU9250 in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report))) {
		printf("SPI transfer error\n");
		return false;
	}

	if (mpu_report.status & 0x01) {
	} else {
//		duplicates++;
		return false;
	}

	// pass the report structure in for debugging (update dbg cnt's)
	if (_mag->measure(mpu_report.mag, report)) {
		report.mag_x = mpu_report.mag.x;
		report.mag_y = mpu_report.mag.y;
		report.mag_z = mpu_report.mag.z;
		ret = true;
	}
	ret = true; // while debugging we want to capture every updates data


	/*
	 * Convert from big to little endian
	 */
	report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);
	report.temp    = int16_t_from_bytes(mpu_report.temp);
	report.gyro_x  = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y  = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z  = int16_t_from_bytes(mpu_report.gyro_z);

	return ret;
}

void
MPU9250::print_registers()
{
	printf("MPU9250 registers\n");

	for (uint8_t reg = 0; reg <= 126; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if (reg % 13 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}
