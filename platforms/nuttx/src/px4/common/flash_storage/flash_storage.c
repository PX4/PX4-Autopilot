/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>

#ifdef CONFIG_BOARD_FLASH_STORAGE

#include <errno.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#include <px4_platform/flash_storage.h>

#ifdef CONFIG_BOARD_FLASH_STORAGE_ARMED_READONLY
bool flash_storage_armed(void);

/* Program and erase mask interrupts for the whole operation; while armed they are refused */
static bool refused(struct px4_flash_storage_s *dev)
{
	if (flash_storage_armed()) {
		perf_count(dev->refused);
		return true;
	}

	return false;
}
#else
static inline bool refused(struct px4_flash_storage_s *dev) { return false; }
#endif

static bool in_range(off_t start, size_t count, off_t total)
{
	return start >= 0 && (off_t)count <= total && start <= total - (off_t)count;
}

static ssize_t storage_read(struct mtd_dev_s *mtd, off_t offset, size_t nbytes, uint8_t *buffer)
{
	struct px4_flash_storage_s *dev = (struct px4_flash_storage_s *)mtd;

	if (!in_range(offset, nbytes, dev->size)) {
		return -EIO;
	}

	return dev->ops->read(dev->ctx, (uint32_t)offset, buffer, nbytes);
}

static ssize_t storage_bread(struct mtd_dev_s *mtd, off_t startblock, size_t nblocks, uint8_t *buffer)
{
	struct px4_flash_storage_s *dev = (struct px4_flash_storage_s *)mtd;

	if (!in_range(startblock, nblocks, dev->size / dev->page_size)) {
		return -EIO;
	}

	ssize_t nbytes = storage_read(mtd, startblock * dev->page_size, nblocks * dev->page_size, buffer);
	return nbytes > 0 ? nbytes / (ssize_t)dev->page_size : nbytes;
}

static ssize_t storage_bwrite(struct mtd_dev_s *mtd, off_t startblock, size_t nblocks, const uint8_t *buffer)
{
	struct px4_flash_storage_s *dev = (struct px4_flash_storage_s *)mtd;

	if (!in_range(startblock, nblocks, dev->size / dev->page_size)) {
		return -EIO;
	}

	if (refused(dev)) {
		return -EBUSY;
	}

	const size_t len = nblocks * dev->page_size;
	ssize_t written = dev->ops->program(dev->ctx, (uint32_t)startblock * dev->page_size, buffer, len);

	if (written < 0) {
		return written;
	}

	return (size_t)written == len ? (ssize_t)nblocks : -EIO;
}

static int storage_erase(struct mtd_dev_s *mtd, off_t startblock, size_t nblocks)
{
	struct px4_flash_storage_s *dev = (struct px4_flash_storage_s *)mtd;

	if (!in_range(startblock, nblocks, dev->size / dev->sector_size)) {
		return -EIO;
	}

	if (refused(dev)) {
		return -EBUSY;
	}

	int ret = dev->ops->erase(dev->ctx, (uint32_t)startblock, (uint32_t)nblocks);
	return ret < 0 ? ret : (int)nblocks;
}

static int storage_ioctl(struct mtd_dev_s *mtd, int cmd, unsigned long arg)
{
	struct px4_flash_storage_s *dev = (struct px4_flash_storage_s *)mtd;

	if (cmd != MTDIOC_GEOMETRY) {
		return -ENOTTY;
	}

	struct mtd_geometry_s *geo = (struct mtd_geometry_s *)(uintptr_t)arg;

	if (geo == NULL) {
		return -EINVAL;
	}

	geo->blocksize = dev->page_size;
	geo->erasesize = dev->sector_size;
	geo->neraseblocks = dev->size / dev->sector_size;
	return OK;
}

int px4_flash_storage_register(struct px4_flash_storage_s *dev, const char *devpath, const char *mountpoint)
{
	if (dev->ops == NULL || dev->page_size == 0 || dev->sector_size % dev->page_size != 0 ||
	    dev->size % dev->sector_size != 0) {
		return -EINVAL;
	}

	dev->mtd.erase = storage_erase;
	dev->mtd.bread = storage_bread;
	dev->mtd.bwrite = storage_bwrite;
	dev->mtd.read = storage_read;
	dev->mtd.ioctl = storage_ioctl;
	dev->mtd.name = "flash_storage";

#ifdef CONFIG_BOARD_FLASH_STORAGE_ARMED_READONLY
	dev->refused = perf_alloc(PC_COUNT, "flash_storage: refused while armed");
#endif

	int ret = register_mtddriver(devpath, &dev->mtd, 0755, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] Failed to register %s: %d\n", devpath, ret);
		return ret;
	}

	/* autoformat: a blank or unmountable volume is formatted, as on the other littlefs boards */
	ret = nx_mount(devpath, mountpoint, "littlefs", 0, "autoformat");

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] Failed to mount %s: %d\n", mountpoint, ret);

	} else {
		syslog(LOG_INFO, "[boot] littlefs mounted at %s\n", mountpoint);
	}

	return ret;
}

#endif /* CONFIG_BOARD_FLASH_STORAGE */
