/****************************************************************************
 * boards/nxp/mr-tropic/src/imxrt_flexspi_storage.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include <px4_platform_common/px4_mtd.h>
#include "px4_log.h"

#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>
#include "board_config.h"
#include "hardware/imxrt_pinmux.h"
#include "barriers.h"

/* Used sectors must be multiple of the flash block size
 * i.e. W25Q32JV has a block size of 64KB
*/

#define NOR_USED_SECTORS  (0x40U)   /* 64 * 4KB = 256KB */
#define NOR_TOTAL_SECTORS (0x0400U)
#define NOR_PAGE_SIZE     (0x0100U) /* 256 bytes */
#define NOR_SECTOR_SIZE   (0x1000U) /* 4KB */
#define NOR_START_SECTOR  (NOR_TOTAL_SECTORS - NOR_USED_SECTORS)
#define NOR_START_PAGE    ((NOR_START_SECTOR * NOR_SECTOR_SIZE) / NOR_PAGE_SIZE)
#define NOR_STORAGE_ADDR  (IMXRT_FLEX2CIPHER_BASE + NOR_START_SECTOR * NOR_SECTOR_SIZE)
#define NOR_STORAGE_END   (IMXRT_FLEX2CIPHER_BASE + (NOR_START_SECTOR + NOR_TOTAL_SECTORS) * NOR_SECTOR_SIZE)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

extern struct flexspi_nor_config_s g_bootConfig;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* FlexSPI NOR device private data */

struct imxrt_flexspi_storage_dev_s {
	struct mtd_dev_s mtd;
	uint8_t *ahb_base;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int imxrt_flexspi_storage_erase(struct mtd_dev_s *dev,
				       off_t startblock,
				       size_t nblocks);
static ssize_t imxrt_flexspi_storage_read(struct mtd_dev_s *dev,
		off_t offset,
		size_t nbytes,
		uint8_t *buffer);
static ssize_t imxrt_flexspi_storage_bread(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		uint8_t *buffer);
static ssize_t imxrt_flexspi_storage_bwrite(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		const uint8_t *buffer);
static int imxrt_flexspi_storage_ioctl(struct mtd_dev_s *dev,
				       int cmd,
				       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_flexspi_storage_dev_s g_flexspi_nor = {
	.mtd =
	{
		.erase  = imxrt_flexspi_storage_erase,
		.bread  = imxrt_flexspi_storage_bread,
		.bwrite = imxrt_flexspi_storage_bwrite,
		.read   = imxrt_flexspi_storage_read,
		.ioctl  = imxrt_flexspi_storage_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
		.write  = NULL,
#endif
		.name   = "imxrt_flexspi_storage"
	},
	.ahb_base = (uint8_t *) NOR_STORAGE_ADDR,
};

/* Ensure exclusive access to the driver */

static sem_t g_exclsem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imxrt_flexspi_storage_erase_chip(
	const struct imxrt_flexspi_storage_dev_s *dev)
{
	/* We can't erase the chip we're executing from */
	return -EINVAL;
}

static ssize_t imxrt_flexspi_storage_read(struct mtd_dev_s *dev,
		off_t offset,
		size_t nbytes,
		uint8_t *buffer)
{
	ssize_t ret;
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	uint8_t *src;

	finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

	if (offset < 0) {
		return -EIO;
	}

	src = priv->ahb_base + offset;

	if (src + nbytes > (uint8_t *)NOR_STORAGE_END) {
		return -EIO;
	}

	ret = nxsem_wait(&g_exclsem);

	if (ret < 0) {
		return ret;
	}

	memcpy(buffer, src, nbytes);

	nxsem_post(&g_exclsem);

	finfo("return nbytes: %d\n", (int)nbytes);
	return (ssize_t)nbytes;
}

static ssize_t imxrt_flexspi_storage_bread(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		uint8_t *buffer)
{
	ssize_t nbytes;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	/* On this device, we can handle the block read just like the byte-oriented
	 * read
	 */

	nbytes = imxrt_flexspi_storage_read(dev, startblock * NOR_PAGE_SIZE,
					    nblocks * NOR_PAGE_SIZE, buffer);

	if (nbytes > 0) {
		nbytes /= NOR_PAGE_SIZE;
	}

	return nbytes;
}

locate_code(".ramfunc")
static ssize_t imxrt_flexspi_storage_bwrite(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		const uint8_t *buffer)
{
	ssize_t ret;
	struct flexspi_nor_config_s *pConfig = &g_bootConfig;
	size_t len = nblocks * NOR_PAGE_SIZE;
	off_t offset = (startblock + NOR_START_PAGE) * NOR_PAGE_SIZE;
	uint8_t *src = (uint8_t *) buffer;
#ifdef CONFIG_ARMV7M_DCACHE
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	uint8_t *dst = priv->ahb_base + startblock * NOR_PAGE_SIZE;
#endif
	int i;

	if (((uintptr_t)buffer % 4) != 0) {
		return -EINVAL; // Byte aligned write not supported
	}

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	ret = nxsem_wait(&g_exclsem);

	if (ret < 0) {
		return ret;
	}

	while (len) {
		i = MIN(NOR_PAGE_SIZE, len);
		cpsid(); // Disable interrupts
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
		volatile uint32_t status = ROM_FLEXSPI_NorFlash_ProgramPage(NOR_INSTANCE, pConfig, offset, (const uint32_t *)src);
#pragma GCC diagnostic pop
		cpsie(); // Enable interrupts
		usleep(0); // Yield to scheduler
		UNUSED(status);

		offset += i;
		src += i;
		len -= i;
	}

#ifdef CONFIG_ARMV7M_DCACHE
	up_invalidate_dcache((uintptr_t)dst,
			     (uintptr_t)dst + nblocks * NOR_PAGE_SIZE);
#endif

	nxsem_post(&g_exclsem);

	return nblocks;
}

locate_code(".ramfunc")
static int imxrt_flexspi_storage_erase(struct mtd_dev_s *dev,
				       off_t startblock,
				       size_t nblocks)
{
	struct flexspi_nor_config_s *pConfig = &g_bootConfig;
	size_t blocksleft = nblocks;
#ifdef CONFIG_ARMV7M_DCACHE
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	uint8_t *dst = priv->ahb_base + startblock * NOR_SECTOR_SIZE;
#endif
	ssize_t ret;

	startblock += NOR_START_SECTOR;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	ret = nxsem_wait(&g_exclsem);

	if (ret < 0) {
		return ret;
	}

	while (blocksleft-- > 0) {
		/* Erase each sector */
		cpsid(); // Disable interrupts
		volatile uint32_t status = ROM_FLEXSPI_NorFlash_Erase(NOR_INSTANCE, pConfig,
					   (startblock * NOR_SECTOR_SIZE), NOR_SECTOR_SIZE);
		cpsie(); // Enable interrupts
		usleep(0); // Yield to scheduler
		UNUSED(status);
		startblock++;
	}

#ifdef CONFIG_ARMV7M_DCACHE
	up_invalidate_dcache((uintptr_t)dst,
			     (uintptr_t)dst + nblocks * NOR_SECTOR_SIZE);
#endif
	nxsem_post(&g_exclsem);

	return (int)nblocks;
}

static int imxrt_flexspi_storage_ioctl(struct mtd_dev_s *dev,
				       int cmd,
				       unsigned long arg)
{
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	int ret = -EINVAL; /* Assume good command with bad parameters */

	finfo("cmd: %d\n", cmd);

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			struct mtd_geometry_s *geo =
				(struct mtd_geometry_s *)((uintptr_t)arg);

			if (geo) {
				/* Populate the geometry structure with information need to
				 * know the capacity and how to access the device.
				 *
				 * NOTE:
				 * that the device is treated as though it where just an array
				 * of fixed size blocks.  That is most likely not true, but the
				 * client will expect the device logic to do whatever is
				 * necessary to make it appear so.
				 */

				geo->blocksize    = (NOR_PAGE_SIZE);
				geo->erasesize    = (NOR_SECTOR_SIZE);
				geo->neraseblocks = (NOR_USED_SECTORS);

				ret               = OK;

				finfo("blocksize: %lu erasesize: %lu neraseblocks: %lu\n",
				      geo->blocksize, geo->erasesize, geo->neraseblocks);
			}
		}
		break;

	case BIOC_PARTINFO: {
			struct partition_info_s *info =
				(struct partition_info_s *)arg;

			if (info != NULL) {
				info->numsectors  = (NOR_USED_SECTORS * NOR_SECTOR_SIZE) / NOR_PAGE_SIZE;
				info->sectorsize  = NOR_PAGE_SIZE;
				info->startsector = 0;
				info->parent[0]   = '\0';
				ret               = OK;
			}
		}
		break;

	case MTDIOC_BULKERASE: {
			/* Erase the entire device */

			imxrt_flexspi_storage_erase_chip(priv);
			ret               = OK;
		}
		break;

	default:
		ret = -ENOTTY; /* Bad/unsupported command */
		break;
	}

	finfo("return %d\n", ret);
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: imxrt_flexspi_storage_initialize
 *
 * Description:
 *  This function is called by board-bringup logic to configure the
 *  flash device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int imxrt_flexspi_storage_initialize(void)
{
	struct mtd_dev_s *mtd_dev = &g_flexspi_nor.mtd;
	int ret = -ENODEV;

	/* Register the MTD driver so that it can be accessed from the
	 * VFS.
	 */

	ret = register_mtddriver("/dev/nor", mtd_dev, 0755, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
		       ret);
	}

#ifdef CONFIG_FS_LITTLEFS

	/* Mount the LittleFS file system */

	ret = nx_mount("/dev/nor", "/fs/nor", "littlefs", 0,
		       "autoformat");

	if (ret < 0) {
		syslog(LOG_ERR,
		       "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
		       ret);
	}

#endif

	return ret;
}
