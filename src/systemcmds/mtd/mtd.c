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

/* create a fake command with decent warnx to not confuse users */
int mtd_main(int argc, char *argv[])
{
	errx(1, "RAMTRON not enabled, skipping.");
}

#else

static void	mtd_attach(void);
static void	mtd_start(char *partition_names[], unsigned n_partitions);
static void	mtd_test(void);

static bool attached = false;
static bool started = false;
static struct mtd_dev_s *mtd_dev;
static const int n_partitions_default = 2;

/* note, these will be equally sized */
static char *partition_names_default[n_partitions] = {"/dev/mtd_params", "/dev/mtd_waypoints"};

int mtd_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "start")) {

			/* start mapping according to user request */
			if (argc > 3) {
				mtd_start(argv + 2, argc - 2);

			} else {
				mtd_start(partition_names_default, n_partitions_default);
			}
		}

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
mtd_start(char *partition_names[], unsigned n_partitions)
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


	/* Get the geometry of the FLASH device */

	FAR struct mtd_geometry_s geo;

	ret = mtd_dev->ioctl(master, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

	if (ret < 0) {
		fdbg("ERROR: mtd->ioctl failed: %d\n", ret);
		exit(3);
	}

	warnx("Flash Geometry:\n");
	warnx("  blocksize:      %lu\n", (unsigned long)geo.blocksize);
	warnx("  erasesize:      %lu\n", (unsigned long)geo.erasesize);
	warnx("  neraseblocks:   %lu\n", (unsigned long)geo.neraseblocks);

	/* Determine the size of each partition.  Make each partition an even
	 * multiple of the erase block size (perhaps not using some space at the
	 * end of the FLASH).
	 */

	unsigned blkpererase = geo.erasesize / geo.blocksize;
	unsigned nblocks     = (geo.neraseblocks / n_partitions) * blkpererase;
	unsigned partsize    = nblocks * geo.blocksize;

	warnx("  No. partitions: %u\n", n_partitions);
	warnx("  Partition size: %lu Blocks (%lu bytes)\n", nblocks, partsize);

	/* Now create MTD FLASH partitions */

	warnx("Creating partitions\n");
	FAR struct mtd_dev_s *part[n_partitions];
	char blockname[32];

	for (unsigned offset = 0, unsigned i = 0; i < n_partitions; offset += nblocks, i++) {

		warnx("  Partition %d. Block offset=%lu, size=%lu\n",
		      i, (unsigned long)offset, (unsigned long)nblocks);

		/* Create the partition */

		part[i] = mtd_partition(mtd_dev, offset, nblocks);

		if (!part[i]) {
			warnx("ERROR: mtd_partition failed. offset=%lu nblocks=%lu\n",
			      (unsigned long)offset, (unsigned long)nblocks);
			fsync(stderr);
			exit(4);
		}

		/* Initialize to provide an FTL block driver on the MTD FLASH interface */

		snprintf(blockname, 32, "/dev/mtdblock%d", i);

		ret = ftl_initialize(i, part[i]);

		if (ret < 0) {
			warnx("ERROR: ftl_initialize %s failed: %d\n", blockname, ret);
			fsync(stderr);
			exit(5);
		}

		/* Now create a character device on the block device */

		ret = bchdev_register(blockname, partition_names[i], false);

		if (ret < 0) {
			warnx("ERROR: bchdev_register %s failed: %d\n", charname, ret);
			fsync(stderr);
			exit(6);
		}
	}

	started = true;
	exit(0);
}

static void
mtd_test(void)
{
	warnx("This test routine does not test anything yet!");
	exit(1);
}

#endif
