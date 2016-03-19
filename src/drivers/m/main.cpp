
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>

#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
//#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>

#include "mag.h"
//#include "gyro.h"
#include "mpu9250.h"

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_MAG		"/dev/mpu9250_mag"

/** driver 'main' command */
extern "C" { __EXPORT int m_main(int argc, char *argv[]); }


MPU9250	*g_dev;

extern int32_t period;

extern uint8_t cnt0;
extern uint8_t cnt1;
extern uint8_t cnt2;
extern uint8_t cnt3;
extern uint8_t cnt4;

extern int16_t x_raw;
extern int16_t y_raw;
extern int16_t z_raw;

extern int glitched;

extern struct Report report;

extern bool measured;

void mpu(int rate);

void
mpu(int rate)
{
	int i = 0;

	g_dev->set_frequency_high();
	if (rate != 0) {
		g_dev->start(rate);
	}
	while (true) {

		g_dev->set_frequency_high();

		if (rate == 0) {
			measured = g_dev->measure();
		}

		if (measured || glitched) {
			measured = false;
			if (i++ > 10 || glitched != 0) {
				i = 0;
//				printf("%5d %5d %5d : ", report.accel_x, report.accel_y, report.accel_z);
				printf("%5d %5d %5d ", x_raw, y_raw, z_raw);
				printf("[%x %x %x %x %x] ", cnt0, cnt1, cnt2, cnt3, cnt4);
				printf("%2u ms ", period / 1000);
				if (glitched) {
					printf("%u", glitched);
					glitched = 0;
				}
				printf("\n");
			}
			cnt0 = cnt1 = cnt2 = cnt3 = cnt4 = 0;
		}
/*
		struct pollfd fds;
		int ret;
		fds.fd = 0; // stdin
		fds.events = POLLIN;
		ret = poll(&fds, 1, 0);
		if (ret > 0) {
			char c;

			read(0, &c, 1);
			switch (c) {
			case 0x03: // ctrl-c
			case 0x1b: // esc
			case 'c':
			case 'q':
				return;
			}
		}
 */
//		usleep(10000);
//		usleep(1);
	}
}

int
m_main(int argc, char *argv[])
{
	int rate = 0;
	if (argc > 1) {
		rate = atoi(argv[1]);
	}
	if (rate > 2000 || rate < 0) {
		rate = 1000;
	}
	printf("%s %u\n", argv[0], rate);


	const char *verb = argv[optind];

	/* create the driver */
	g_dev = new MPU9250(PX4_SPI_BUS_SENSORS, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG, (spi_dev_e)PX4_SPIDEV_MPU);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	if (!strcmp(verb, "regdump")) {
		g_dev->print_registers();
	}

	mpu(rate);

	if (g_dev != nullptr) {
		delete(g_dev);
	}
	exit(0);

fail:
	if (g_dev != nullptr) {
		delete g_dev;
	}
	errx(1, "driver start failed");
	exit(1);
}
