/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file ramtron.c
 *
 * ramtron service and utility app.
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

__EXPORT int ramtron_main(int argc, char *argv[]);

#ifndef CONFIG_MTD_RAMTRON

/* create a fake command with decent message to not confuse users */
int ramtron_main(int argc, char *argv[])
{
	errx(1, "RAMTRON not enabled, skipping.");
}
#else

static void	ramtron_attach(void);
static void	ramtron_start(void);
static void	ramtron_erase(void);
static void	ramtron_ioctl(unsigned operation);
static void	ramtron_save(const char *name);
static void	ramtron_load(const char *name);
static void	ramtron_test(void);

static bool attached = false;
static bool started = false;
static struct mtd_dev_s *ramtron_mtd;

int ramtron_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "start"))
			ramtron_start();

		if (!strcmp(argv[1], "save_param"))
			ramtron_save(argv[2]);

		if (!strcmp(argv[1], "load_param"))
			ramtron_load(argv[2]);

		if (!strcmp(argv[1], "erase"))
			ramtron_erase();

		if (!strcmp(argv[1], "test"))
			ramtron_test();

		if (0) {	/* these actually require a file on the filesystem... */

			if (!strcmp(argv[1], "reformat"))
				ramtron_ioctl(FIOC_REFORMAT);

			if (!strcmp(argv[1], "repack"))
				ramtron_ioctl(FIOC_OPTIMIZE);
		}
	}

	errx(1, "expected a command, try 'start'\n\t'save_param /ramtron/parameters'\n\t'load_param /ramtron/parameters'\n\t'erase'\n");
}

struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);


static void
ramtron_attach(void)
{
	/* find the right spi */
	struct spi_dev_s *spi = up_spiinitialize(2);
	/* this resets the spi bus, set correct bus speed again */
    // xxx set in ramtron driver, leave this out
//	SPI_SETFREQUENCY(spi, 4000000);
	SPI_SETFREQUENCY(spi, 375000000);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE3);
	SPI_SELECT(spi, SPIDEV_FLASH, false);

	if (spi == NULL)
		errx(1, "failed to locate spi bus");

	/* start the MTD driver, attempt 5 times */
	for (int i = 0; i < 5; i++) {
		ramtron_mtd = ramtron_initialize(spi);
		if (ramtron_mtd) {
			/* abort on first valid result */
			if (i > 0) {
				warnx("warning: ramtron needed %d attempts to attach", i+1);
			}
			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (ramtron_mtd == NULL)
		errx(1, "failed to initialize ramtron driver");

	attached = true;
}

static void
ramtron_start(void)
{
	int ret;

	if (started)
		errx(1, "ramtron already mounted");

	if (!attached)
		ramtron_attach();

	/* start NXFFS */
	ret = nxffs_initialize(ramtron_mtd);

	if (ret < 0)
		errx(1, "failed to initialize NXFFS - erase ramtron to reformat");

	/* mount the ramtron */
	ret = mount(NULL, "/ramtron", "nxffs", 0, NULL);

	if (ret < 0)
		errx(1, "failed to mount /ramtron - erase ramtron to reformat");

	started = true;
	warnx("mounted ramtron at /ramtron");
	exit(0);
}

//extern int at24c_nuke(void);

static void
ramtron_erase(void)
{
	if (!attached)
		ramtron_attach();

//	if (at24c_nuke())
		errx(1, "erase failed");

	errx(0, "erase done, reboot now");
}

static void
ramtron_ioctl(unsigned operation)
{
	int fd;

	fd = open("/ramtron/.", 0);

	if (fd < 0)
		err(1, "open /ramtron");

	if (ioctl(fd, operation, 0) < 0)
		err(1, "ioctl");

	exit(0);
}

static void
ramtron_save(const char *name)
{
	if (!started)
		errx(1, "must be started first");

	if (!name)
		err(1, "missing argument for device name, try '/ramtron/parameters'");

	warnx("WARNING: 'ramtron save_param' deprecated - use 'param save' instead");

	/* delete the file in case it exists */
	unlink(name);

	/* create the file */
	int fd = open(name, O_WRONLY | O_CREAT | O_EXCL);

	if (fd < 0)
		err(1, "opening '%s' failed", name);

	int result = param_export(fd, false);
	close(fd);

	if (result < 0) {
		unlink(name);
		errx(1, "error exporting to '%s'", name);
	}

	exit(0);
}

static void
ramtron_load(const char *name)
{
	if (!started)
		errx(1, "must be started first");

	if (!name)
		err(1, "missing argument for device name, try '/ramtron/parameters'");

	warnx("WARNING: 'ramtron load_param' deprecated - use 'param load' instead");

	int fd = open(name, O_RDONLY);

	if (fd < 0)
		err(1, "open '%s'", name);

	int result = param_load(fd);
	close(fd);

	if (result < 0)
		errx(1, "error importing from '%s'", name);

	exit(0);
}

//extern void at24c_test(void);

static void
ramtron_test(void)
{
//	at24c_test();
	exit(0);
}

#endif
