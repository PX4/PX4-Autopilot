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

#include <px4_config.h>

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

#include <board_config.h>

__EXPORT int mtd_main(int argc, char *argv[]);

#ifndef CONFIG_MTD

/* create a fake command with decent warnx to not confuse users */
int mtd_main(int argc, char *argv[])
{
	errx(1, "MTD not enabled, skipping.");
}

#else

#ifdef CONFIG_MTD_RAMTRON
static void	ramtron_attach(void);
#else

#ifndef PX4_I2C_BUS_ONBOARD
#  error PX4_I2C_BUS_ONBOARD not defined, cannot locate onboard EEPROM
#endif

static void	at24xxx_attach(void);
#endif
static void	mtd_start(char *partition_names[], unsigned n_partitions);
static void	mtd_test(void);
static void	mtd_erase(char *partition_names[], unsigned n_partitions);
static void	mtd_readtest(char *partition_names[], unsigned n_partitions);
static void	mtd_rwtest(char *partition_names[], unsigned n_partitions);
static void	mtd_print_info(void);
static int	mtd_get_geometry(unsigned long *blocksize, unsigned long *erasesize, unsigned long *neraseblocks,
				 unsigned *blkpererase, unsigned *nblocks, unsigned *partsize, unsigned n_partitions);

static bool attached = false;
static bool started = false;
static struct mtd_dev_s *mtd_dev;
static unsigned n_partitions_current = 0;

/* note, these will be equally sized */
static char *partition_names_default[] = {"/fs/mtd_params", "/fs/mtd_waypoints"};
static const int n_partitions_default = sizeof(partition_names_default) / sizeof(partition_names_default[0]);

static void
mtd_status(void)
{
	if (!attached) {
		errx(1, "MTD driver not started");
	}

	mtd_print_info();
	exit(0);
}

int mtd_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "start")) {

			/* start mapping according to user request */
			if (argc >= 3) {
				mtd_start(argv + 2, argc - 2);

			} else {
				mtd_start(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "test")) {
			mtd_test();
		}

		if (!strcmp(argv[1], "readtest")) {
			if (argc >= 3) {
				mtd_readtest(argv + 2, argc - 2);

			} else {
				mtd_readtest(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "rwtest")) {
			if (argc >= 3) {
				mtd_rwtest(argv + 2, argc - 2);

			} else {
				mtd_rwtest(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "status")) {
			mtd_status();
		}

		if (!strcmp(argv[1], "erase")) {
			if (argc >= 3) {
				mtd_erase(argv + 2, argc - 2);

			} else {
				mtd_erase(partition_names_default, n_partitions_default);
			}
		}
	}

	errx(1, "expected a command, try 'start', 'erase', 'status', 'readtest', 'rwtest' or 'test'");
}

struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);
struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
				off_t firstblock, off_t nblocks);

#ifdef CONFIG_MTD_RAMTRON
static void
ramtron_attach(void)
{
	/* find the right spi */
#ifdef CONFIG_ARCH_BOARD_AEROCORE
	struct spi_dev_s *spi = up_spiinitialize(4);
#else
	struct spi_dev_s *spi = up_spiinitialize(2);
#endif
	/* this resets the spi bus, set correct bus speed again */
	SPI_SETFREQUENCY(spi, 10 * 1000 * 1000);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE3);
	SPI_SELECT(spi, SPIDEV_FLASH, false);

	if (spi == NULL) {
		errx(1, "failed to locate spi bus");
	}

	/* start the RAMTRON driver, attempt 5 times */
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
	if (mtd_dev == NULL) {
		errx(1, "failed to initialize mtd driver");
	}

	int ret = mtd_dev->ioctl(mtd_dev, MTDIOC_SETSPEED, (unsigned long)10 * 1000 * 1000);

	if (ret != OK) {
		// FIXME: From the previous warnx call, it looked like this should have been an errx instead. Tried
		// that but setting the bug speed does fail all the time. Which was then exiting and the board would
		// not run correctly. So changed to warnx.
		warnx("failed to set bus speed");
	}

	attached = true;
}
#else

static void
at24xxx_attach(void)
{
	/* find the right I2C */
	struct i2c_dev_s *i2c = up_i2cinitialize(PX4_I2C_BUS_ONBOARD);
	/* this resets the I2C bus, set correct bus speed again */
	I2C_SETFREQUENCY(i2c, 400000);

	if (i2c == NULL) {
		errx(1, "failed to locate I2C bus");
	}

	/* start the MTD driver, attempt 5 times */
	for (int i = 0; i < 5; i++) {
		mtd_dev = at24c_initialize(i2c);

		if (mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				warnx("warning: EEPROM needed %d attempts to attach", i + 1);
			}

			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (mtd_dev == NULL) {
		errx(1, "failed to initialize EEPROM driver");
	}

	attached = true;
}
#endif

static void
mtd_start(char *partition_names[], unsigned n_partitions)
{
	int ret;

	if (started) {
		errx(1, "mtd already mounted");
	}

	if (!attached) {
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
		at24xxx_attach();
#else
		ramtron_attach();
#endif
	}

	if (!mtd_dev) {
		warnx("ERROR: Failed to create RAMTRON FRAM MTD instance");
		exit(1);
	}

	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize;

	ret = mtd_get_geometry(&blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize, n_partitions);

	if (ret) {
		exit(3);
	}

	/* Now create MTD FLASH partitions */

	FAR struct mtd_dev_s *part[n_partitions];
	char blockname[32];

	unsigned offset;
	unsigned i;

	for (offset = 0, i = 0; i < n_partitions; offset += nblocks, i++) {

		/* Create the partition */

		part[i] = mtd_partition(mtd_dev, offset, nblocks);

		if (!part[i]) {
			warnx("ERROR: mtd_partition failed. offset=%lu nblocks=%lu",
			      (unsigned long)offset, (unsigned long)nblocks);
			exit(4);
		}

		/* Initialize to provide an FTL block driver on the MTD FLASH interface */

		snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", i);

		ret = ftl_initialize(i, part[i]);

		if (ret < 0) {
			warnx("ERROR: ftl_initialize %s failed: %d", blockname, ret);
			exit(5);
		}

		/* Now create a character device on the block device */

		ret = bchdev_register(blockname, partition_names[i], false);

		if (ret < 0) {
			warnx("ERROR: bchdev_register %s failed: %d", partition_names[i], ret);
			exit(6);
		}
	}

	n_partitions_current = n_partitions;

	started = true;
	exit(0);
}

int mtd_get_geometry(unsigned long *blocksize, unsigned long *erasesize, unsigned long *neraseblocks,
		     unsigned *blkpererase, unsigned *nblocks, unsigned *partsize, unsigned n_partitions)
{
	/* Get the geometry of the FLASH device */

	FAR struct mtd_geometry_s geo;

	int ret = mtd_dev->ioctl(mtd_dev, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

	if (ret < 0) {
		warnx("ERROR: mtd->ioctl failed: %d", ret);
		return ret;
	}

	*blocksize = geo.blocksize;
	*erasesize = geo.blocksize;
	*neraseblocks = geo.neraseblocks;

	/* Determine the size of each partition.  Make each partition an even
	 * multiple of the erase block size (perhaps not using some space at the
	 * end of the FLASH).
	 */

	*blkpererase = geo.erasesize / geo.blocksize;
	*nblocks     = (geo.neraseblocks / n_partitions) * *blkpererase;
	*partsize    = *nblocks * geo.blocksize;

	return ret;
}

/*
  get partition size in bytes
 */
static ssize_t mtd_get_partition_size(void)
{
	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize = 0;

	int ret = mtd_get_geometry(&blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize,
				   n_partitions_current);

	if (ret != OK) {
		errx(1, "Failed to get geometry");
	}

	return partsize;
}

void mtd_print_info(void)
{
	if (!attached) {
		exit(1);
	}

	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize;

	int ret = mtd_get_geometry(&blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize,
				   n_partitions_current);

	if (ret) {
		exit(3);
	}

	warnx("Flash Geometry:");

	printf("  blocksize:      %lu\n", blocksize);
	printf("  erasesize:      %lu\n", erasesize);
	printf("  neraseblocks:   %lu\n", neraseblocks);
	printf("  No. partitions: %u\n", n_partitions_current);
	printf("  Partition size: %u Blocks (%u bytes)\n", nblocks, partsize);
	printf("  TOTAL SIZE: %u KiB\n", neraseblocks * erasesize / 1024);

}

void
mtd_test(void)
{
	warnx("This test routine does not test anything yet!");
	exit(1);
}

void
mtd_erase(char *partition_names[], unsigned n_partitions)
{
	uint8_t v[64];
	memset(v, 0xFF, sizeof(v));

	for (uint8_t i = 0; i < n_partitions; i++) {
		uint32_t count = 0;
		printf("Erasing %s\n", partition_names[i]);
		int fd = open(partition_names[i], O_WRONLY);

		if (fd == -1) {
			errx(1, "Failed to open partition");
		}

		while (write(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		printf("Erased %lu bytes\n", (unsigned long)count);
		close(fd);
	}

	exit(0);
}

/*
  readtest is useful during startup to validate the device is
  responding on the bus. It relies on the driver returning an error on
  bad reads (the ramtron driver does return an error)
 */
void
mtd_readtest(char *partition_names[], unsigned n_partitions)
{
	ssize_t expected_size = mtd_get_partition_size();

	uint8_t v[128];

	for (uint8_t i = 0; i < n_partitions; i++) {
		ssize_t count = 0;
		printf("reading %s expecting %u bytes\n", partition_names[i], expected_size);
		int fd = open(partition_names[i], O_RDONLY);

		if (fd == -1) {
			errx(1, "Failed to open partition");
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		if (count != expected_size) {
			errx(1, "Failed to read partition - got %u/%u bytes", count, expected_size);
		}

		close(fd);
	}

	printf("readtest OK\n");
	exit(0);
}

/*
  rwtest is useful during startup to validate the device is
  responding on the bus for both reads and writes. It reads data in
  blocks and writes the data back, then reads it again, failing if the
  data isn't the same
 */
void
mtd_rwtest(char *partition_names[], unsigned n_partitions)
{
	ssize_t expected_size = mtd_get_partition_size();

	uint8_t v[128], v2[128];

	for (uint8_t i = 0; i < n_partitions; i++) {
		ssize_t count = 0;
		off_t offset = 0;
		printf("rwtest %s testing %u bytes\n", partition_names[i], expected_size);
		int fd = open(partition_names[i], O_RDWR);

		if (fd == -1) {
			errx(1, "Failed to open partition");
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);

			if (lseek(fd, offset, SEEK_SET) != offset) {
				errx(1, "seek failed");
			}

			if (write(fd, v, sizeof(v)) != sizeof(v)) {
				errx(1, "write failed");
			}

			if (lseek(fd, offset, SEEK_SET) != offset) {
				errx(1, "seek failed");
			}

			if (read(fd, v2, sizeof(v2)) != sizeof(v2)) {
				errx(1, "read failed");
			}

			if (memcmp(v, v2, sizeof(v2)) != 0) {
				errx(1, "memcmp failed");
			}

			offset += sizeof(v);
		}

		if (count != expected_size) {
			errx(1, "Failed to read partition - got %u/%u bytes", count, expected_size);
		}

		close(fd);
	}

	printf("rwtest OK\n");
	exit(0);
}

#endif
