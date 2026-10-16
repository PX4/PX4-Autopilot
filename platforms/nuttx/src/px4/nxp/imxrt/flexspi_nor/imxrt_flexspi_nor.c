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

#ifdef CONFIG_BOARD_FLEXSPI_NOR_L0

#include <errno.h>
#include <sched.h>
#include <string.h>

#include <nuttx/cache.h>
#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>
#include <arch/irq.h>

#include <px4_arch/imxrt_flexspi_nor.h>
#include <px4_arch/imxrt_romapi.h>    /* after the FlexSPI config types it depends on */

/* The ROM API keeps shared state, and an AHB read anywhere stalls while any region is busy */
static sem_t g_exclsem = SEM_INITIALIZER(1);

static inline bool blank_get(const struct imxrt_flexspi_nor_region_s *region, uint32_t sector)
{
	return region->blank[sector / 8] & (1u << (sector % 8));
}

static inline void blank_set(struct imxrt_flexspi_nor_region_s *region, uint32_t sector, bool blank)
{
	if (blank) {
		region->blank[sector / 8] |= 1u << (sector % 8);

	} else {
		region->blank[sector / 8] &= ~(1u << (sector % 8));
	}
}

/* Caller holds g_exclsem */
static bool sector_blank(struct imxrt_flexspi_nor_region_s *region, uint32_t sector)
{
	if (blank_get(region, sector)) {
		return true;
	}

	const uint32_t *p = (const uint32_t *)(uintptr_t)(region->ahb + sector * IMXRT_FLEXSPI_NOR_SECTOR_SIZE);

	for (unsigned i = 0; i < IMXRT_FLEXSPI_NOR_SECTOR_SIZE / sizeof(uint32_t); i++) {
		if (p[i] != 0xffffffffu) {
			return false;
		}
	}

	blank_set(region, sector, true);
	return true;
}

/* ClearCache: the ROM routines leave the AHB prefetch buffers stale */
locate_code(".ramfunc")
static uint32_t rom_program_page(uint32_t instance, struct flexspi_nor_config_s *config, uint32_t offset,
				 const uint32_t *src)
{
	cpsid();
	uint32_t status = ROM_FLEXSPI_NorFlash_ProgramPage(instance, config, offset, src);
	ROM_FLEXSPI_NorFlash_ClearCache(instance);
	cpsie();
	return status;
}

locate_code(".ramfunc")
static uint32_t rom_erase_sector(uint32_t instance, struct flexspi_nor_config_s *config, uint32_t offset)
{
	cpsid();
	uint32_t status = ROM_FLEXSPI_NorFlash_Erase(instance, config, offset, IMXRT_FLEXSPI_NOR_SECTOR_SIZE);
	ROM_FLEXSPI_NorFlash_ClearCache(instance);
	cpsie();
	return status;
}

static void invalidate(const struct imxrt_flexspi_nor_region_s *region, uint32_t offset, size_t len)
{
#ifdef CONFIG_ARMV7M_DCACHE
	const uintptr_t start = (uintptr_t)region->ahb + offset;
	up_invalidate_dcache(start, start + len);
#endif
}

int imxrt_flexspi_nor_init(struct imxrt_flexspi_nor_region_s *region, const char *program_name,
			   const char *erase_name, const char *erase_skip_name)
{
	if (region->ahb != NULL) {
		return -EALREADY;
	}

	if (region->config == NULL || region->ahb_base == 0 ||
	    region->size == 0 ||
	    region->size / IMXRT_FLEXSPI_NOR_SECTOR_SIZE > IMXRT_FLEXSPI_NOR_MAX_SECTORS ||
	    region->offset % IMXRT_FLEXSPI_NOR_SECTOR_SIZE != 0 ||
	    region->size % IMXRT_FLEXSPI_NOR_SECTOR_SIZE != 0 ||
	    region->offset > UINT32_MAX - region->size) {
		return -EINVAL;
	}

	region->ahb = (const uint8_t *)(region->ahb_base + region->offset);
	region->perf_program = perf_alloc(PC_ELAPSED, program_name);
	region->perf_erase = perf_alloc(PC_ELAPSED, erase_name);
	region->perf_erase_skip = perf_alloc(PC_COUNT, erase_skip_name);

	return 0;
}

ssize_t imxrt_flexspi_nor_read(struct imxrt_flexspi_nor_region_s *region, uint32_t offset, void *dst, size_t len)
{
	if (region->ahb == NULL) {
		return -EINVAL;
	}

	if (len > region->size || offset > region->size - len) {
		return -EIO;
	}

	int ret = nxsem_wait_uninterruptible(&g_exclsem);

	if (ret < 0) {
		return ret;
	}

	memcpy(dst, region->ahb + offset, len);
	nxsem_post(&g_exclsem);
	return (ssize_t)len;
}

ssize_t imxrt_flexspi_nor_program(struct imxrt_flexspi_nor_region_s *region, uint32_t offset, const void *src,
				  size_t len)
{
	if (region->ahb == NULL || (uintptr_t)src % 4 != 0 ||
	    offset % IMXRT_FLEXSPI_NOR_PAGE_SIZE != 0 || len % IMXRT_FLEXSPI_NOR_PAGE_SIZE != 0) {
		return -EINVAL;
	}

	if (len > region->size || offset > region->size - len) {
		return -EIO;
	}

	if (len == 0) {
		return 0;
	}

	int ret = nxsem_wait_uninterruptible(&g_exclsem);

	if (ret < 0) {
		return ret;
	}

	for (uint32_t s = offset / IMXRT_FLEXSPI_NOR_SECTOR_SIZE; s <= (offset + len - 1) / IMXRT_FLEXSPI_NOR_SECTOR_SIZE;
	     s++) {
		blank_set(region, s, false);
	}

	const uint8_t *p = src;
	size_t written = 0;

	while (written < len) {
		perf_begin(region->perf_program);
		uint32_t status = rom_program_page(region->instance, region->config, region->offset + offset + written,
						   (const uint32_t *)(uintptr_t)(p + written));
		perf_end(region->perf_program);

		if (status != 0) {
			break;
		}

		written += IMXRT_FLEXSPI_NOR_PAGE_SIZE;
	}

	invalidate(region, offset, written);
	nxsem_post(&g_exclsem);
	return (ssize_t)written;
}

int imxrt_flexspi_nor_erase(struct imxrt_flexspi_nor_region_s *region, uint32_t sector, uint32_t nsectors)
{
	if (region->ahb == NULL) {
		return -EINVAL;
	}

	const uint32_t region_sectors = region->size / IMXRT_FLEXSPI_NOR_SECTOR_SIZE;

	if (nsectors > region_sectors || sector > region_sectors - nsectors) {
		return -EIO;
	}

	int ret = nxsem_wait_uninterruptible(&g_exclsem);

	if (ret < 0) {
		return ret;
	}

	for (uint32_t s = sector; s < sector + nsectors; s++) {
		if (sector_blank(region, s)) {
			perf_count(region->perf_erase_skip);
			continue;
		}

		perf_begin(region->perf_erase);
		uint32_t status = rom_erase_sector(region->instance, region->config,
						   region->offset + s * IMXRT_FLEXSPI_NOR_SECTOR_SIZE);
		perf_end(region->perf_erase);

		invalidate(region, s * IMXRT_FLEXSPI_NOR_SECTOR_SIZE, IMXRT_FLEXSPI_NOR_SECTOR_SIZE);

		if (status != 0) {
			ret = -EIO;
			break;
		}

		blank_set(region, s, true);
		sched_yield();
	}

	nxsem_post(&g_exclsem);
	return ret;
}

static ssize_t storage_read(void *ctx, uint32_t offset, void *dst, size_t len)
{
	return imxrt_flexspi_nor_read(ctx, offset, dst, len);
}

static ssize_t storage_program(void *ctx, uint32_t offset, const void *src, size_t len)
{
	return imxrt_flexspi_nor_program(ctx, offset, src, len);
}

static int storage_erase(void *ctx, uint32_t sector, uint32_t nsectors)
{
	return imxrt_flexspi_nor_erase(ctx, sector, nsectors);
}

const struct px4_flash_storage_ops_s g_imxrt_flexspi_nor_storage_ops = {
	.read = storage_read,
	.program = storage_program,
	.erase = storage_erase,
};

#endif /* CONFIG_BOARD_FLEXSPI_NOR_L0 */
