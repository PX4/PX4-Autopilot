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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/spi.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/board/board.h>

#include "systemlib/px4_macros.h"
#include <parameters/param.h>

#include <board_config.h>

extern "C" __EXPORT int mtd_main(int argc, char *argv[]);

#ifndef CONFIG_MTD

/* create a fake command with decent warning to not confuse users */
int mtd_main(int argc, char *argv[])
{
	PX4_WARN("MTD not enabled, skipping.");
	return 1;
}

#else

struct mtd_instance_s;

#  if defined(BOARD_HAS_MTD_PARTITION_OVERRIDE)
#    define MTD_PARTITION_TABLE  BOARD_HAS_MTD_PARTITION_OVERRIDE
#  else
#   define MTD_PARTITION_TABLE  {"/fs/mtd_params", "/fs/mtd_waypoints"}
#  endif

/* note, these will be equally sized */
static const char *partition_names_default[] = MTD_PARTITION_TABLE;
#if defined(BOARD_MTD_PARTITION_TABLE_SIZES)
static const float partition_sizes_default[] = BOARD_MTD_PARTITION_TABLE_SIZES;
#else
#  define partition_sizes_default nullptr
#endif


#ifdef CONFIG_MTD_RAMTRON
static int	ramtron_attach(mtd_instance_s &instance);
#else

#ifndef PX4_I2C_BUS_MTD
#error "Board needs to define PX4_I2C_BUS_MTD for onboard EEPROM bus"
#endif

#endif

#ifdef PX4_I2C_BUS_MTD
static int	at24xxx_attach(mtd_instance_s &instance);
#endif

static int	mtd_start(mtd_instance_s &instance, const char *partition_names[], unsigned n_partitions);
static int	mtd_erase(const char *partition_names[], unsigned n_partitions);
static int	mtd_readtest(const mtd_instance_s &instance, const char *partition_names[], unsigned n_partitions);
static int	mtd_rwtest(const mtd_instance_s &instance, const char *partition_names[], unsigned n_partitions);
static int	mtd_print_info(int instance);
static int	mtd_get_geometry(const mtd_instance_s &instance, unsigned long *blocksize, unsigned long *erasesize,
				 unsigned long *neraseblocks,
				 unsigned *blkpererase, unsigned *nblocks, unsigned *partsize);

struct mtd_instance_s {
	int (*attach)(mtd_instance_s &instance);
	bool attached;
	bool started;
	struct mtd_dev_s *mtd_dev;
	unsigned n_partitions_current;
	int *partition_block_counts;
	const float *partition_percentages;
	const char **partition_names;
};

static mtd_instance_s instances[] = {
#ifdef CONFIG_MTD_RAMTRON
	{&ramtron_attach, false, false, nullptr, 0, nullptr, partition_sizes_default, partition_names_default},
#endif
#ifdef PX4_I2C_BUS_MTD
	{&at24xxx_attach, false, false, nullptr, 0, nullptr, nullptr, nullptr},
#endif
};
static constexpr int num_instances = arraySize(instances);
static const int n_partitions_default = arraySize(partition_names_default);

static int
mtd_status(void)
{
	int ret = 0;
	bool running = false;

	for (int i = 0; i < num_instances; ++i) {
		if (instances[i].attached) {
			ret |= mtd_print_info(i);
			running = true;
		}
	}

	if (!running) {
		PX4_INFO("MTD driver not started");
		return 1;
	}

	return ret;
}

static void	print_usage(void)
{
	PRINT_MODULE_DESCRIPTION("Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board)");

	PRINT_MODULE_USAGE_NAME("mtd", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Mount partitions");
	PRINT_MODULE_USAGE_COMMAND_DESCR("readtest", "Perform read test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("rwtest", "Perform read-write test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("erase", "Erase partition(s)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("has-secondary", "Check if the board has configured a secondary device");

	PRINT_MODULE_USAGE_PARAM_COMMENT("The commands 'start', 'readtest' and 'rwtest' have an optional instance index:");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 1, "storage index (if the board has multiple storages)", true);

	PRINT_MODULE_USAGE_PARAM_COMMENT("The commands 'start', 'readtest', 'rwtest' and 'erase' have an optional parameter:");
	PRINT_MODULE_USAGE_ARG("<partition_name1> [<partition_name2> ...]",
			       "Partition names (eg. /fs/mtd_params), use system default if not provided", true);
}

int mtd_main(int argc, char *argv[])
{
	if (argc >= 2) {
		int instance = 0;
		int partition_index = 2;

		if (argc > 3 && !strcmp(argv[2], "-i")) {
			instance = strtol(argv[3], nullptr, 10);

			if (instance < 0 || instance >= num_instances) {
				PX4_ERR("invalid instance");
				return -1;
			}

			partition_index += 2;
		}

		if (!strcmp(argv[1], "start")) {

			/* start mapping according to user request */
			if (argc > partition_index) {
				return mtd_start(instances[instance], (const char **)(argv + partition_index), argc - partition_index);

			} else {
				return mtd_start(instances[instance], partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "readtest")) {
			if (argc > partition_index) {
				return mtd_readtest(instances[instance], (const char **)(argv + partition_index), argc - partition_index);

			} else {
				return mtd_readtest(instances[instance], partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "rwtest")) {
			if (argc > partition_index) {
				return mtd_rwtest(instances[instance], (const char **)(argv + partition_index), argc - partition_index);

			} else {
				return mtd_rwtest(instances[instance], partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "status")) {
			return mtd_status();
		}

		if (!strcmp(argv[1], "erase")) {
			if (argc > partition_index) {
				return mtd_erase((const char **)(argv + partition_index), argc - partition_index);

			} else {
				return mtd_erase(partition_names_default, n_partitions_default);
			}
		}

		if (!strcmp(argv[1], "has-secondary")) {
			return num_instances > 1 ? 0 : 1;
		}
	}

	print_usage();
	return 1;
}

struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);
struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
				off_t firstblock, off_t nblocks);

#ifdef CONFIG_MTD_RAMTRON
static int
ramtron_attach(mtd_instance_s &instance)
{
	/* initialize the right spi */
	struct spi_dev_s *spi = px4_spibus_initialize(px4_find_spi_bus(SPIDEV_FLASH(0)));

	if (spi == nullptr) {
		PX4_ERR("failed to locate spi bus");
		return 1;
	}

	/* this resets the spi bus, set correct bus speed again */
	SPI_SETFREQUENCY(spi, 10 * 1000 * 1000);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE3);
	SPI_SELECT(spi, SPIDEV_FLASH(0), false);

	/* start the RAMTRON driver, attempt 5 times */

	for (int i = 0; i < 5; i++) {
		instance.mtd_dev = ramtron_initialize(spi);

		if (instance.mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				PX4_WARN("mtd needed %d attempts to attach", i + 1);
			}

			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (instance.mtd_dev == nullptr) {
		PX4_ERR("failed to initialize mtd driver");
		return 1;
	}

	int ret = instance.mtd_dev->ioctl(instance.mtd_dev, MTDIOC_SETSPEED, (unsigned long)10 * 1000 * 1000);

	if (ret != OK) {
		// FIXME: From the previous warning call, it looked like this should have been fatal error instead. Tried
		// that but setting the bus speed does fail all the time. Which was then exiting and the board would
		// not run correctly. So changed to PX4_WARN.
		PX4_WARN("failed to set bus speed");
	}

	instance.attached = true;
	return 0;
}
#endif

#ifdef PX4_I2C_BUS_MTD

static int
at24xxx_attach(mtd_instance_s &instance)
{
	/* find the right I2C */
	struct i2c_master_s *i2c = px4_i2cbus_initialize(PX4_I2C_BUS_MTD);

	if (i2c == nullptr) {
		PX4_ERR("failed to locate I2C bus");
		return 1;
	}

	/* start the MTD driver, attempt 5 times */
	for (int i = 0; i < 5; i++) {
		instance.mtd_dev = at24c_initialize(i2c);

		if (instance.mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				PX4_WARN("EEPROM needed %d attempts to attach", i + 1);
			}

			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (instance.mtd_dev == nullptr) {
		PX4_ERR("failed to initialize EEPROM driver");
		return 1;
	}

	instance.attached = true;
	return 0;
}
#endif

static int
mtd_start(mtd_instance_s &instance, const char *partition_names[], unsigned n_partitions)
{
	int ret;

	if (instance.started) {
		PX4_ERR("mtd already mounted");
		return 1;
	}

	if (!instance.attached) {
		ret = instance.attach(instance);

		if (ret != 0) {
			return ret;
		}
	}

	if (!instance.mtd_dev) {
		PX4_ERR("Failed to create MTD instance");
		return 1;
	}

	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize;

	instance.n_partitions_current = n_partitions;
	ret = mtd_get_geometry(instance, &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

	if (ret) {
		return ret;
	}

	/* Now create MTD FLASH partitions */

	FAR struct mtd_dev_s *part[n_partitions];
	char blockname[32];

	unsigned offset;
	unsigned i;


	instance.partition_block_counts = new int[n_partitions];

	if (instance.partition_block_counts == nullptr) {
		PX4_ERR("mtd_partition failed allocation counts");
		return 1;
	}

	instance.partition_names = new const char *[n_partitions];

	if (instance.partition_names == nullptr) {
		PX4_ERR("mtd_partition failed allocation for names");
		return 1;
	}

	for (unsigned int n = 0; n < n_partitions; n++) {
		float percentage = instance.partition_percentages == nullptr  ?
				   100.0f / n_partitions : instance.partition_percentages[n];
		volatile int nb = neraseblocks * (percentage / 100.0f);
		instance.partition_block_counts[n] = (nb == 0 ? 1 : nb);
		instance.partition_names[n] = nullptr;
	}

	for (offset = 0, i = 0; i < n_partitions; offset += instance.partition_block_counts[i], i++) {

		/* Create the partition */

		part[i] = mtd_partition(instance.mtd_dev, offset, instance.partition_block_counts[i]);

		if (!part[i]) {
			PX4_ERR("mtd_partition failed. offset=%lu nblocks=%lu",
				(unsigned long)offset, (unsigned long)nblocks);
			return 1;
		}

		instance.partition_names[i] = strdup(partition_names[i]);

		/* Initialize to provide an FTL block driver on the MTD FLASH interface */

		ret = -1;

		for (int dev_index = 0; ret != 0; ++dev_index) {
			snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", i + dev_index);

			ret = ftl_initialize(i + dev_index, part[i]);

			if (ret == -EEXIST) {
				continue;
			}

			if (ret < 0 || dev_index >= 9) {
				PX4_ERR("ftl_initialize %s failed: %d", blockname, ret);
				return 1;
			}
		}

		/* Now create a character device on the block device */

		ret = bchdev_register(blockname, partition_names[i], false);

		if (ret < 0) {
			PX4_ERR("bchdev_register %s failed: %d", partition_names[i], ret);
			return 1;
		}
	}

	instance.started = true;
	return 0;
}

int mtd_get_geometry(const mtd_instance_s &instance, unsigned long *blocksize, unsigned long *erasesize,
		     unsigned long *neraseblocks,
		     unsigned *blkpererase, unsigned *nblocks, unsigned *partsize)
{
	/* Get the geometry of the FLASH device */

	FAR struct mtd_geometry_s geo;

	int ret = instance.mtd_dev->ioctl(instance.mtd_dev, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

	if (ret < 0) {
		PX4_ERR("mtd->ioctl failed: %d", ret);
		return ret;
	}

	*blocksize = geo.blocksize;
	*erasesize = geo.erasesize;
	*neraseblocks = geo.neraseblocks;

	/* Determine the size of each partition.  Make each partition an even
	 * multiple of the erase block size (perhaps not using some space at the
	 * end of the FLASH).
	 */

	*blkpererase = geo.erasesize / geo.blocksize;
	*nblocks     = (geo.neraseblocks / instance.n_partitions_current) * *blkpererase;
	*partsize    = *nblocks * geo.blocksize;

	return ret;
}

/*
  get partition size in bytes
 */
static ssize_t mtd_get_partition_size(const mtd_instance_s &instance, const char *partname)
{
	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize = 0;

	int ret = mtd_get_geometry(instance, &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

	if (ret != OK) {
		PX4_ERR("Failed to get geometry");
		return 0;
	}

	unsigned partn = 0;

	for (unsigned n = 0; n < instance.n_partitions_current; n++) {
		if (instance.partition_names[n] != nullptr &&
		    partname != nullptr &&
		    strcmp(instance.partition_names[n], partname) == 0) {
			partn = n;
			break;
		}
	}

	return instance.partition_block_counts[partn] * erasesize;
}

int mtd_print_info(int instance)
{
	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize;

	int ret = mtd_get_geometry(instances[instance], &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks,
				   &partsize);

	if (ret) {
		return ret;
	}

	PX4_INFO("Flash Geometry of instance %i:", instance);

	printf("  blocksize:      %lu\n", blocksize);
	printf("  erasesize:      %lu\n", erasesize);
	printf("  neraseblocks:   %lu\n", neraseblocks);
	printf("  No. partitions: %u\n", instances[instance].n_partitions_current);
	printf("  Partition size: %u Blocks (%u bytes)\n", nblocks, partsize);
	printf("  TOTAL SIZE: %u KiB\n", neraseblocks * erasesize / 1024);

	return 0;
}

int
mtd_erase(const char *partition_names[], unsigned n_partitions)
{
	uint8_t v[64];
	memset(v, 0xFF, sizeof(v));

	for (uint8_t i = 0; i < n_partitions; i++) {
		uint32_t count = 0;
		printf("Erasing %s\n", partition_names[i]);
		int fd = open(partition_names[i], O_WRONLY);

		if (fd == -1) {
			PX4_ERR("Failed to open partition");
			return 1;
		}

		while (write(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		printf("Erased %lu bytes\n", (unsigned long)count);
		close(fd);
	}

	return 0;
}

/*
  readtest is useful during startup to validate the device is
  responding on the bus. It relies on the driver returning an error on
  bad reads (the ramtron driver does return an error)
 */
int
mtd_readtest(const mtd_instance_s &instance, const char *partition_names[], unsigned n_partitions)
{


	uint8_t v[128];

	for (uint8_t i = 0; i < n_partitions; i++) {
		ssize_t count = 0;

		ssize_t expected_size = mtd_get_partition_size(instance, partition_names[i]);

		if (expected_size == 0) {
			PX4_ERR("Failed partition size is 0");
			return 1;
		}

		printf("reading %s expecting %u bytes\n", partition_names[i], expected_size);
		int fd = open(partition_names[i], O_RDONLY);

		if (fd == -1) {
			PX4_ERR("Failed to open partition");
			return 1;
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		if (count != expected_size) {
			PX4_ERR("Failed to read partition - got %u/%u bytes", count, expected_size);
			return 1;
		}

		close(fd);
	}

	printf("readtest OK\n");
	return 0;
}

/*
  rwtest is useful during startup to validate the device is
  responding on the bus for both reads and writes. It reads data in
  blocks and writes the data back, then reads it again, failing if the
  data isn't the same
 */
int
mtd_rwtest(const mtd_instance_s &instance, const char *partition_names[], unsigned n_partitions)
{
	uint8_t v[128], v2[128];

	for (uint8_t i = 0; i < n_partitions; i++) {
		ssize_t count = 0;
		off_t offset = 0;

		ssize_t expected_size = mtd_get_partition_size(instance, partition_names[i]);

		if (expected_size == 0) {
			PX4_ERR("Failed partition size is 0");
			return 1;
		}

		printf("rwtest %s testing %u bytes\n", partition_names[i], expected_size);
		int fd = open(partition_names[i], O_RDWR);

		if (fd == -1) {
			PX4_ERR("Failed to open partition");
			return 1;
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);

			if (lseek(fd, offset, SEEK_SET) != offset) {
				PX4_ERR("seek failed");
				return 1;
			}

			if (write(fd, v, sizeof(v)) != sizeof(v)) {
				PX4_ERR("write failed");
				return 1;
			}

			if (lseek(fd, offset, SEEK_SET) != offset) {
				PX4_ERR("seek failed");
				return 1;
			}

			if (read(fd, v2, sizeof(v2)) != sizeof(v2)) {
				PX4_ERR("read failed");
				return 1;
			}

			if (memcmp(v, v2, sizeof(v2)) != 0) {
				PX4_ERR("memcmp failed");
				return 1;
			}

			offset += sizeof(v);
		}

		if (count != expected_size) {
			PX4_ERR("Failed to read partition - got %u/%u bytes", count, expected_size);
			return 1;
		}

		close(fd);
	}

	printf("rwtest OK\n");
	return 0;
}

#endif
