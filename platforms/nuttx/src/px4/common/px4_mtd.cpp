/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file px4_mtd.cpp
 *
 * mtd services.
 *
 * @author David Sidrane <david.sidrane@nscdg.com>
 */

#ifndef MODULE_NAME
#define MODULE_NAME "PX4_MTD"
#endif

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/spi.h>

#include <inttypes.h>
#include <errno.h>
#include <stdbool.h>
#include "systemlib/px4_macros.h"

#include <nuttx/drivers/drivers.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

extern "C" {
	struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);
	struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
					off_t firstblock, off_t nblocks);
}
static int num_instances = 0;
static int total_blocks = 0;
static mtd_instance_s *instances[MAX_MTD_INSTANCES] = {};


static int ramtron_attach(mtd_instance_s &instance)
{
#if !defined(CONFIG_MTD_RAMTRON)
	PX4_ERR("Misconfiguration CONFIG_MTD_RAMTRON not set");
	return ENXIO;
#else

	/* start the RAMTRON driver, attempt 10 times */

	int spi_speed_mhz = 10;

	for (int i = 0; i < 10; i++) {
		/* initialize the right spi */
		struct spi_dev_s *spi = px4_spibus_initialize(px4_find_spi_bus(instance.devid));

		if (spi == nullptr) {
			PX4_ERR("failed to locate spi bus");
			return -ENXIO;
		}

		/* this resets the spi bus, set correct bus speed again */
		SPI_LOCK(spi, true);
		SPI_SETFREQUENCY(spi, spi_speed_mhz * 1000 * 1000);
		SPI_SETBITS(spi, 8);
		SPI_SETMODE(spi, SPIDEV_MODE3);
		SPI_SELECT(spi, instance.devid, false);
		SPI_LOCK(spi, false);

		instance.mtd_dev = ramtron_initialize(spi);

		if (instance.mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				PX4_WARN("mtd needed %d attempts to attach", i + 1);
			}

			break;
		}

		// try reducing speed for next attempt
		spi_speed_mhz--;
		px4_usleep(10000);
	}

	/* if last attempt is still unsuccessful, abort */
	if (instance.mtd_dev == nullptr) {
		PX4_ERR("failed to initialize mtd driver");
		return -EIO;
	}

	int ret = instance.mtd_dev->ioctl(instance.mtd_dev, MTDIOC_SETSPEED, (unsigned long)spi_speed_mhz * 1000 * 1000);

	if (ret != OK) {
		// FIXME: From the previous warning call, it looked like this should have been fatal error instead. Tried
		// that but setting the bus speed does fail all the time. Which was then exiting and the board would
		// not run correctly. So changed to PX4_WARN.
		PX4_WARN("failed to set bus speed");
	}

	return 0;
#endif
}


static int at24xxx_attach(mtd_instance_s &instance)
{
#if !defined(PX4_I2C_BUS_MTD)
	PX4_ERR("Misconfiguration PX4_I2C_BUS_MTD not set");
	return -ENXIO;
#else

	struct i2c_master_s *i2c = px4_i2cbus_initialize(PX4_I2C_DEVID_BUS(instance.devid));

	if (i2c == nullptr) {
		PX4_ERR("failed to locate I2C bus");
		return -ENXIO;
	}

	/* start the MTD driver, attempt 5 times */
	for (int i = 0; i < 5; i++) {
		int ret_val = px4_at24c_initialize(i2c, PX4_I2C_DEVID_ADDR(instance.devid), &(instance.mtd_dev));

		if (ret_val == 0) {
			/* abort on first valid result */
			if (i > 0) {
				PX4_WARN("EEPROM needed %d attempts to attach", i + 1);
			}

			break;

		} else if (ret_val == -ENOMEM) {
			PX4_ERR("Number of at24c EEPROM instances reached the board limit of %d", BOARD_MTD_NUM_EEPROM);
			break;
		}
	}

	/* if last attempt is still unsuccessful, abort */
	if (instance.mtd_dev == nullptr) {
		PX4_ERR("failed to initialize EEPROM driver");
		return -EIO;
	}

	return 0;
#endif
}


int px4_mtd_get_geometry(const mtd_instance_s *instance, unsigned long *blocksize, unsigned long *erasesize,
			 unsigned long *neraseblocks,
			 unsigned *blkpererase, unsigned *nblocks, unsigned *partsize)
{
	/* Get the geometry of the FLASH device */

	FAR struct mtd_geometry_s geo;

	int ret = instance->mtd_dev->ioctl(instance->mtd_dev, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

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
	*nblocks     = (geo.neraseblocks / instance->n_partitions_current) * *blkpererase;
	*partsize    = *nblocks * geo.blocksize;

	return ret;
}

/*
  get partition size in bytes
 */
ssize_t px4_mtd_get_partition_size(const mtd_instance_s *instance, const char *partname)
{
	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize = 0;

	int ret = px4_mtd_get_geometry(instance, &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

	if (ret != OK) {
		PX4_ERR("Failed to get geometry");
		return 0;
	}

	unsigned partn = 0;

	for (unsigned n = 0; n < instance->n_partitions_current; n++) {
		if (instance->partition_names[n] != nullptr &&
		    partname != nullptr &&
		    strcmp(instance->partition_names[n], partname) == 0) {
			partn = n;
			break;
		}
	}

	return instance->partition_block_counts[partn] * blocksize;
}

mtd_instance_s **px4_mtd_get_instances(unsigned int *count)
{
	*count = num_instances;
	return instances;
}

// Define the default FRAM usage
#if !defined(CONFIG_MTD_RAMTRON)

static const px4_mtd_manifest_t default_mtd_config = {
};

#else

const px4_mft_device_t spifram  = {             // FM25V02A on FMUM 32K 512 X 64
	.bus_type = px4_mft_device_t::SPI,
	.devid    = SPIDEV_FLASH(0)
};

const px4_mtd_entry_t fram = {
	.device = &spifram,
	.npart = 2,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/fs/mtd_params",
			.nblocks = 32
		},
		{
			.type = MTD_WAYPOINTS,
			.path = "/fs/mtd_waypoints",
			.nblocks = 32

		}
	},
};


static const px4_mtd_manifest_t default_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&fram,
	}
};
#endif

int px4_mtd_config(const px4_mtd_manifest_t *mft_mtd)
{
	int rv = -EINVAL;

	const px4_mtd_manifest_t *mtd_list = mft_mtd ? mft_mtd : &default_mtd_config;

	if (mtd_list == nullptr) {
		PX4_ERR("Invalid mtd configuration!");
		return rv;
	}

	if (mtd_list->nconfigs == 0) {
		return 0;
	}

	rv = -ENOMEM;
	uint8_t total_new_instances = mtd_list->nconfigs + num_instances;

	if (total_new_instances >= MAX_MTD_INSTANCES) {
		PX4_ERR("reached limit of max %u mtd instances", MAX_MTD_INSTANCES);
		return rv;
	}

	for (uint8_t i = num_instances, num_entry = 0u; i < total_new_instances; ++i, ++num_entry) {

		instances[i] = new mtd_instance_s;

		if (instances[i] == nullptr) {
memoryout:
			PX4_ERR("failed to allocate memory!");
			return rv;
		}

		num_instances++;

		uint32_t nparts = mtd_list->entries[num_entry]->npart;
		instances[i]->devid = mtd_list->entries[num_entry]->device->devid;
		instances[i]->mtd_dev = nullptr;
		instances[i]->n_partitions_current = 0;

		rv = -ENOMEM;
		instances[i]->part_dev = new FAR struct mtd_dev_s *[nparts];

		if (instances[i]->part_dev == nullptr) {
			goto memoryout;
		}

		instances[i]->partition_block_counts = new int[nparts];

		if (instances[i]->partition_block_counts == nullptr) {
			goto memoryout;
		}

		instances[i]->partition_types = new int[nparts];

		if (instances[i]->partition_types == nullptr) {
			goto memoryout;
		}

		instances[i]->partition_names = new const char *[nparts];

		if (instances[i]->partition_names == nullptr) {
			goto memoryout;
		}

		for (uint32_t p = 0; p < nparts; p++) {
			instances[i]->partition_block_counts[p] =  mtd_list->entries[num_entry]->partd[p].nblocks;
			instances[i]->partition_names[p] = mtd_list->entries[num_entry]->partd[p].path;
			instances[i]->partition_types[p] = mtd_list->entries[num_entry]->partd[p].type;
		}

		if (mtd_list->entries[num_entry]->device->bus_type == px4_mft_device_t::I2C) {
			rv = at24xxx_attach(*instances[i]);

		} else if (mtd_list->entries[num_entry]->device->bus_type == px4_mft_device_t::SPI) {
			rv = ramtron_attach(*instances[i]);
#if defined(HAS_FLEXSPI)

		} else if (mtd_list->entries[num_entry]->device->bus_type == px4_mft_device_t::FLEXSPI) {
			rv = flexspi_attach(instances[i]);
#endif

		} else if (mtd_list->entries[num_entry]->device->bus_type == px4_mft_device_t::ONCHIP) {
			instances[i]->n_partitions_current++;
			return 0;
		}

		if (rv != 0) {
			goto errout;
		}

		unsigned long blocksize;
		unsigned long erasesize;
		unsigned long neraseblocks;
		unsigned int  blkpererase;
		unsigned int  nblocks;
		unsigned int  partsize;

		rv = px4_mtd_get_geometry(instances[i], &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

		if (rv != 0) {
			goto errout;
		}

		/* Now create MTD FLASH partitions */

		char blockname[32];

		unsigned long offset;
		unsigned part;

		for (offset = 0, part = 0; rv == 0 && part < nparts; offset += instances[i]->partition_block_counts[part], part++) {

			/* Create the partition */

			instances[i]->part_dev[part] = mtd_partition(instances[i]->mtd_dev, offset, instances[i]->partition_block_counts[part]);

			if (instances[i]->part_dev[part] == nullptr) {
				PX4_ERR("mtd_partition failed. offset=%lu nblocks=%u",
					offset, nblocks);
				rv = -ENOSPC;
				goto errout;
			}

			/* Initialize to provide an FTL block driver on the MTD FLASH interface */

			snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", total_blocks);

			rv = ftl_initialize(total_blocks, instances[i]->part_dev[part]);

			if (rv < 0) {
				PX4_ERR("ftl_initialize %s failed: %d", blockname, rv);
				goto errout;
			}

			total_blocks++;

			/* Now create a character device on the block device */

			rv = bchdev_register(blockname, instances[i]->partition_names[part], false);

			if (rv < 0) {
				PX4_ERR("bchdev_register %s failed: %d", instances[i]->partition_names[part], rv);
				goto errout;
			}

			instances[i]->n_partitions_current++;
		}

errout:

		if (rv < 0) {
			PX4_ERR("mtd failure: %d bus %" PRId32 " address %" PRId32 " class %d",
				rv,
				PX4_I2C_DEVID_BUS(instances[i]->devid),
				PX4_I2C_DEVID_ADDR(instances[i]->devid),
				mtd_list->entries[num_entry]->partd[instances[i]->n_partitions_current].type);
			break;
		}
	}

	return rv;
}

__EXPORT int px4_mtd_query(const char *sub, const char *val, const char **get)
{
	int rv = -ENODEV;

	static const char *keys[] = PX4_MFT_MTD_STR_TYPES;
	static const px4_mtd_types_t types[] = PX4_MFT_MTD_TYPES;
	int key = 0;

	for (unsigned int k = 0; k < arraySize(keys); k++) {
		if (!strcmp(keys[k], sub)) {
			key = types[k];
			break;
		}
	}


	rv = -EINVAL;

	if (key != 0) {
		rv = -ENOENT;

		for (int i = 0; i < num_instances; i++) {
			for (unsigned n = 0; n < instances[i]->n_partitions_current; n++) {
				if (instances[i]->partition_types[n] == key) {
					if (get != nullptr && val == nullptr) {
						*get =  instances[i]->partition_names[n];
						return 0;
					}

					if (val != nullptr && strcmp(instances[i]->partition_names[n], val) == 0) {
						return 0;
					}
				}
			}
		}
	}


	return rv;
}
