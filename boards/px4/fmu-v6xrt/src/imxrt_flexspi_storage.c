/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

/* MTD over the storage region of the boot NOR, mounted as littlefs */

#include <nuttx/config.h>

#ifdef CONFIG_BOARD_FLEXSPI_FLASH_STORAGE

#include <errno.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "board_config.h"
#include "hw_config.h"
#include "imxrt_flexspi_nor_l0.h"

#define FLASH_STORAGE_DEV      "/dev/nor"
#define FLASH_STORAGE_MOUNT    "/fs/flash"
#define FLASH_STORAGE_PAGES    (FLASH_STORAGE_SIZE / FLEXSPI_NOR_PAGE_SIZE)

struct flash_storage_dev_s {
	struct mtd_dev_s mtd;
	struct flexspi_nor_region_s region;
};

static bool in_range(off_t start, size_t count, off_t total)
{
	return start >= 0 && (off_t)count <= total && start <= total - (off_t)count;
}

static ssize_t flash_storage_read(struct mtd_dev_s *dev, off_t offset, size_t nbytes, uint8_t *buffer)
{
	struct flash_storage_dev_s *priv = (struct flash_storage_dev_s *)dev;

	if (!in_range(offset, nbytes, FLASH_STORAGE_SIZE)) {
		return -EIO;
	}

	return flexspi_nor_l0_read(&priv->region, (uint32_t)offset, buffer, nbytes);
}

static ssize_t flash_storage_bread(struct mtd_dev_s *dev, off_t startblock, size_t nblocks, uint8_t *buffer)
{
	if (!in_range(startblock, nblocks, FLASH_STORAGE_PAGES)) {
		return -EIO;
	}

	ssize_t nbytes = flash_storage_read(dev, startblock * FLEXSPI_NOR_PAGE_SIZE, nblocks * FLEXSPI_NOR_PAGE_SIZE, buffer);
	return nbytes > 0 ? nbytes / (ssize_t)FLEXSPI_NOR_PAGE_SIZE : nbytes;
}

static ssize_t flash_storage_bwrite(struct mtd_dev_s *dev, off_t startblock, size_t nblocks, const uint8_t *buffer)
{
	struct flash_storage_dev_s *priv = (struct flash_storage_dev_s *)dev;

	if (!in_range(startblock, nblocks, FLASH_STORAGE_PAGES)) {
		return -EIO;
	}

	const size_t len = nblocks * FLEXSPI_NOR_PAGE_SIZE;
	ssize_t written = flexspi_nor_l0_program(&priv->region, (uint32_t)startblock * FLEXSPI_NOR_PAGE_SIZE, buffer, len);

	if (written < 0) {
		return written;
	}

	return (size_t)written == len ? (ssize_t)nblocks : -EIO;
}

static int flash_storage_erase(struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
	struct flash_storage_dev_s *priv = (struct flash_storage_dev_s *)dev;

	if (!in_range(startblock, nblocks, FLASH_STORAGE_SECTORS)) {
		return -EIO;
	}

	int ret = flexspi_nor_l0_erase(&priv->region, (uint32_t)startblock, (uint32_t)nblocks);
	return ret < 0 ? ret : (int)nblocks;
}

static int flash_storage_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	if (cmd != MTDIOC_GEOMETRY) {
		return -ENOTTY;
	}

	struct mtd_geometry_s *geo = (struct mtd_geometry_s *)(uintptr_t)arg;

	if (geo == NULL) {
		return -EINVAL;
	}

	geo->blocksize = FLEXSPI_NOR_PAGE_SIZE;
	geo->erasesize = FLEXSPI_NOR_SECTOR_SIZE;
	geo->neraseblocks = FLASH_STORAGE_SECTORS;
	return OK;
}

static struct flash_storage_dev_s g_flash_storage_dev = {
	.mtd = {
		.erase  = flash_storage_erase,
		.bread  = flash_storage_bread,
		.bwrite = flash_storage_bwrite,
		.read   = flash_storage_read,
		.ioctl  = flash_storage_ioctl,
		.name   = "flash_storage"
	},
	.region = {
		.offset = FLASH_STORAGE_OFFSET,
		.size   = FLASH_STORAGE_SIZE,
	},
};

int fmuv6xrt_flash_storage_initialize(void)
{
	struct flexspi_nor_region_s *region = &g_flash_storage_dev.region;
	int ret = flexspi_nor_l0_init(region, FLEXSPI_NOR_L0_PERF("flash_storage"));

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] Bad " FLASH_STORAGE_DEV " geometry: %d\n", ret);
		return ret;
	}

	ret = register_mtddriver(FLASH_STORAGE_DEV, &g_flash_storage_dev.mtd, 0755, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] Failed to register " FLASH_STORAGE_DEV ": %d\n", ret);
		return ret;
	}

	/* autoformat: a blank or unmountable volume is formatted, as on the other littlefs boards */
	ret = nx_mount(FLASH_STORAGE_DEV, FLASH_STORAGE_MOUNT, "littlefs", 0, "autoformat");

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] Failed to mount " FLASH_STORAGE_MOUNT ": %d\n", ret);

	} else {
		syslog(LOG_INFO, "[boot] littlefs mounted at " FLASH_STORAGE_MOUNT "\n");
	}

	return ret;
}

#endif /* CONFIG_BOARD_FLEXSPI_FLASH_STORAGE */
