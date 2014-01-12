/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file mtd.c
 *
 * mtd service and utility app.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/spi.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/param/param.h"
#include "systemlib/err.h"

__EXPORT int mtd_main(int argc, char *argv[]);

#ifndef CONFIG_MTD_RAMTRON

/* create a fake command with decent message to not confuse users */
int mtd_main(int argc, char *argv[])
{
	errx(1, "RAMTRON not enabled, skipping.");
}

#else

static void	mtd_attach(void);
static void	mtd_start(void);
static void	mtd_erase(void);
static void	mtd_ioctl(unsigned operation);
static void	mtd_save(const char *name);
static void	mtd_load(const char *name);
static void	mtd_test(void);

static bool attached = false;
static bool started = false;
static struct mtd_dev_s *mtd_dev;
static char *_mtdname = "/dev/mtd_params";
static char *_wpname = "/dev/mtd_waypoints";

int mtd_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "start"))
			mtd_start();

		if (!strcmp(argv[1], "test"))
			mtd_test();
	}

	errx(1, "expected a command, try 'start' or 'test'");
}

struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);


static void
mtd_attach(void)
{
	/* find the right spi */
	struct spi_dev_s *spi = up_spiinitialize(2);
	/* this resets the spi bus, set correct bus speed again */
	SPI_SETFREQUENCY(spi, 40 * 1000 * 1000);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE3);
	SPI_SELECT(spi, SPIDEV_FLASH, false);

	if (spi == NULL)
		errx(1, "failed to locate spi bus");

	/* start the MTD driver, attempt 5 times */
	for (int i = 0; i < 5; i++) {
		mtd_dev = ramtron_initialize(spi);

		if (mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				warnx("warning: mtd needed %d attempts to attach", i + 1);
			}

			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (mtd_dev == NULL)
		errx(1, "failed to initialize mtd driver");

	attached = true;
}

static void
mtd_start(void)
{
	int ret;

	if (started)
		errx(1, "mtd already mounted");

	if (!attached)
		mtd_attach();

	if (!mtd_dev) {
		warnx("ERROR: Failed to create RAMTRON FRAM MTD instance\n");
		exit(1);
	}

	/* Initialize to provide an FTL block driver on the MTD FLASH interface.
	 *
	 * NOTE:  We could just skip all of this FTL and BCH stuff.  We could
	 * instead just use the MTD drivers bwrite and bread to perform this
	 * test.  Creating the character drivers, however, makes this test more
	 * interesting.
	 */

	ret = ftl_initialize(0, mtd_dev);

	if (ret < 0) {
		warnx("Creating /dev/mtdblock0 failed: %d\n", ret);
		exit(2);
	}

	/* Now create a character device on the block device */

	ret = bchdev_register("/dev/mtdblock0", _mtdname, false);

	if (ret < 0) {
		warnx("ERROR: bchdev_register %s failed: %d\n", _mtdname, ret);
		exit(3);
	}

	/* mount the mtd */
	ret = mount(NULL, "/mtd", "nxffs", 0, NULL);

	if (ret < 0)
		errx(1, "failed to mount /mtd - erase mtd to reformat");

	started = true;
	warnx("mounted mtd at /mtd");
	exit(0);
}

static void
mtd_ioctl(unsigned operation)
{
	int fd;

	fd = open("/mtd/.", 0);

	if (fd < 0)
		err(1, "open /mtd");

	if (ioctl(fd, operation, 0) < 0)
		err(1, "ioctl");

	exit(0);
}

static void
mtd_test(void)
{
//	at24c_test();
	exit(0);
}

#endif
