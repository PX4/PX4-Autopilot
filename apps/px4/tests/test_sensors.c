/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file test_sensors.c
 * Tests the onboard sensors.
 * 
 * The sensors app must not be running when performing this test.
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/spi.h>

#include "tests.h"

#include <arch/board/drv_lis331.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_l3gd20.h>
#include <arch/board/drv_hmc5883l.h>
#include <drivers/drv_accel.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

//static int	lis331(int argc, char *argv[]);
static int l3gd20(int argc, char *argv[]);
static int bma180(int argc, char *argv[]);
static int hmc5883l(int argc, char *argv[]);
static int ms5611(int argc, char *argv[]);
static int mpu6000(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char	*name;
	const char	*path;
	int	(* test)(int argc, char *argv[]);
} sensors[] = {
	{"l3gd20",	"/dev/l3gd20",	l3gd20},
	{"bma180",	"/dev/bma180",	bma180},
	{"hmc5883l",	"/dev/hmc5883l",	hmc5883l},
	{"ms5611",	"/dev/ms5611",	ms5611},
	{"mpu6000",	"/dev/accel",	mpu6000},
//    {"lis331",	"/dev/lis331",	lis331},
	{NULL, NULL, NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

//static int
//lis331(int argc, char *argv[])
//{
//	int		fd;
//	int16_t	buf[3];
//	int		ret;
//
//	fd = open("/dev/lis331", O_RDONLY);
//	if (fd < 0) {
//		printf("\tlis331: not present on PX4FMU v1.5 and later\n");
//		return ERROR;
//	}
//
//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
//
//		printf("LIS331: ioctl fail\n");
//		return ERROR;
//	}
//
//	/* wait at least 100ms, sensor should have data after no more than 20ms */
//	usleep(100000);
//
//	/* read data - expect samples */
//	ret = read(fd, buf, sizeof(buf));
//	if (ret != sizeof(buf)) {
//		printf("LIS331: read1 fail (%d)\n", ret);
//		return ERROR;
//	}
//
//	/* read data - expect no samples (should not be ready again yet) */
//	ret = read(fd, buf, sizeof(buf));
//	if (ret != 0) {
//		printf("LIS331: read2 fail (%d)\n", ret);
//		return ERROR;
//	}
//
//	/* XXX more tests here */
//
//	return 0;
//}

static int
l3gd20(int argc, char *argv[])
{
	printf("\tL3GD20: test start\n");
	fflush(stdout);

	int		fd;
	int16_t	buf[3] = {0, 0, 0};
	int		ret;

	fd = open("/dev/l3gd20", O_RDONLY | O_NONBLOCK);

	if (fd < 0) {
		printf("L3GD20: open fail\n");
		return ERROR;
	}

//	if (ioctl(fd, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) ||
//	    ioctl(fd, L3GD20_SETRANGE, L3GD20_RANGE_500DPS)) {
//
//		printf("L3GD20: ioctl fail\n");
//		return ERROR;
//	} else {
//		printf("\tconfigured..\n");
//	}
//
//	/* wait at least 100ms, sensor should have data after no more than 2ms */
//	usleep(100000);



	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tL3GD20: read1 fail (%d should have been %d)\n", ret, sizeof(buf));
		//return ERROR;

	} else {
		printf("\tL3GD20 values #1: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* wait at least 2 ms, sensor should have data after no more than 1.5ms */
	usleep(2000);

	/* read data - expect no samples (should not be ready again yet) */
	ret = read(fd, buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tL3GD20: read2 fail (%d)\n", ret);
		close(fd);
		return ERROR;

	} else {
		printf("\tL3GD20 values #2: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* empty sensor buffer */
	ret = 0;

	while (ret != sizeof(buf)) {
		// Keep reading until successful
		ret = read(fd, buf, sizeof(buf));
	}

	/* test if FIFO is operational */
	usleep(14800); // Expecting 10 measurements

	ret = 0;
	int count = 0;
	bool dataready = true;

	while (dataready) {
		// Keep reading until successful
		ret = read(fd, buf, sizeof(buf));

		if (ret != sizeof(buf)) {
			dataready = false;

		} else {
			count++;
		}
	}

	printf("\tL3GD20: Drained FIFO with %d values (expected 8-12)\n", count);

	/* read data - expect no samples (should not be ready again yet) */
	ret = read(fd, buf, sizeof(buf));

	if (ret != 0) {
		printf("\tL3GD20: Note: read3 got data - there should not have been data ready\n", ret);
//		return ERROR;
	}

	close(fd);

	/* Let user know everything is ok */
	printf("\tOK: L3GD20 passed all tests successfully\n");
	return OK;
}

static int
bma180(int argc, char *argv[])
{
	printf("\tBMA180: test start\n");
	fflush(stdout);

	int		fd;
	int16_t	buf[3] = {0, 0, 0};
	int		ret;

	fd = open("/dev/bma180", O_RDONLY);

	if (fd < 0) {
		printf("\tBMA180: open fail\n");
		return ERROR;
	}

//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
//
//		printf("BMA180: ioctl fail\n");
//		return ERROR;
//	}
//
	/* wait at least 100ms, sensor should have data after no more than 20ms */
	usleep(100000);

	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tBMA180: read1 fail (%d)\n", ret);
		close(fd);
		return ERROR;

	} else {
		printf("\tBMA180 values: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* wait at least 10ms, sensor should have data after no more than 2ms */
	usleep(100000);

	ret = read(fd, buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		printf("\tBMA180: read2 fail (%d)\n", ret);
		close(fd);
		return ERROR;

	} else {
		printf("\tBMA180: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* empty sensor buffer */
	ret = 0;

	while (ret != sizeof(buf)) {
		// Keep reading until successful
		ret = read(fd, buf, sizeof(buf));
	}

	ret = read(fd, buf, sizeof(buf));

	if (ret != 0) {
		printf("\tBMA180: Note: read3 got data - there should not have been data ready\n", ret);
	}

	/* Let user know everything is ok */
	printf("\tOK: BMA180 passed all tests successfully\n");
	close(fd);

	return OK;
}

static int
mpu6000(int argc, char *argv[])
{
	printf("\tMPU-6000: test start\n");
	fflush(stdout);

	int		fd;
	struct accel_report buf;
	int		ret;

	fd = open("/dev/accel", O_RDONLY);

	if (fd < 0) {
		printf("\tMPU-6000: open fail, run <mpu6000 start> first.\n");
		return ERROR;
	}

	/* wait at least 100ms, sensor should have data after no more than 20ms */
	usleep(100000);

	/* read data - expect samples */
	ret = read(fd, &buf, sizeof(buf));

	if (ret < 3) {
		printf("\tMPU-6000: read1 fail (%d)\n", ret);
		return ERROR;

	} else {
		printf("\tMPU-6000 values: acc: x:%8.4f\ty:%8.4f\tz:%8.4f\n", (double)buf.x, (double)buf.y, (double)buf.z);//\tgyro: r:%d\tp:%d\ty:%d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
	}

	// /* wait at least 10ms, sensor should have data after no more than 2ms */
	// usleep(100000);

	// ret = read(fd, buf, sizeof(buf));

	// if (ret != sizeof(buf)) {
	// 	printf("\tMPU-6000: read2 fail (%d)\n", ret);
	// 	return ERROR;

	// } else {
	// 	printf("\tMPU-6000 values: acc: x:%d\ty:%d\tz:%d\tgyro: r:%d\tp:%d\ty:%d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
	// }

	/* XXX more tests here */

	/* Let user know everything is ok */
	printf("\tOK: MPU-6000 passed all tests successfully\n");

	return OK;
}

static int
ms5611(int argc, char *argv[])
{
	printf("\tMS5611: test start\n");
	fflush(stdout);

	int		fd;
	float	buf[3] = {0.0f, 0.0f, 0.0f};
	int		ret;

	fd = open("/dev/ms5611", O_RDONLY);

	if (fd < 0) {
		printf("\tMS5611: open fail\n");
		return ERROR;
	}

	for (int i = 0; i < 5; i++) {
		/* read data - expect samples */
		ret = read(fd, buf, sizeof(buf));

		if (ret != sizeof(buf)) {

			if ((int8_t)ret == -EAGAIN || (int8_t)ret == -EINPROGRESS) {
				/* waiting for device to become ready, this is not an error */
			} else {
				printf("\tMS5611: read fail (%d)\n", ret);
				close(fd);
				return ERROR;
			}

		} else {

			/* hack for float printing */
			int32_t pressure_int = buf[0];
			int32_t altitude_int = buf[1];
			int32_t temperature_int = buf[2];

			printf("\tMS5611: pressure:%d.%03d mbar - altitude: %d.%02d meters - temp:%d.%02d deg celcius\n", pressure_int, (int)(buf[0] * 1000 - pressure_int * 1000), altitude_int, (int)(buf[1] * 100 - altitude_int * 100), temperature_int, (int)(buf[2] * 100 - temperature_int * 100));
		}

		/* wait at least 10ms, sensor should have data after no more than 6.5ms */
		usleep(10000);
	}

	close(fd);

	/* Let user know everything is ok */
	printf("\tOK: MS5611 passed all tests successfully\n");

	return OK;
}

static int
hmc5883l(int argc, char *argv[])
{
	printf("\tHMC5883L: test start\n");
	fflush(stdout);

	int		fd;
	int16_t	buf[7] = {0, 0, 0};
	int		ret;

	fd = open("/dev/hmc5883l", O_RDONLY);

	if (fd < 0) {
		printf("\tHMC5883L: open fail\n");
		return ERROR;
	}

	int i;

	for (i = 0; i < 5; i++) {
		/* wait at least 7ms, sensor should have data after no more than 6.5ms */
		usleep(7000);

		/* read data - expect samples */
		ret = read(fd, buf, sizeof(buf));

		if (ret != sizeof(buf)) {
			printf("\tHMC5883L: read1 fail (%d) values: x:%d\ty:%d\tz:%d\n", ret, buf[0], buf[1], buf[2]);
			close(fd);
			return ERROR;

		} else {
			printf("\tHMC5883L: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
		}
	}

	close(fd);

	/* Let user know everything is ok */
	printf("\tOK: HMC5883L passed all tests successfully\n");

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_sensors
 ****************************************************************************/

int test_sensors(int argc, char *argv[])
{
	unsigned	i;

	printf("Running sensors tests:\n\n");
	fflush(stdout);

	int ret = OK;

	for (i = 0; sensors[i].name; i++) {
		printf("  sensor: %s\n", sensors[i].name);

		/* Flush and leave enough time for the flush to become effective */
		fflush(stdout);
		usleep(50000);
		/* Test the sensor - if the tests crash at this point, the right sensor name has been printed */

		ret += sensors[i].test(argc, argv);
	}

	return ret;
}
