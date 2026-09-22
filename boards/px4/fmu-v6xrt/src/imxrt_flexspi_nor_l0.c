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

#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>

#include "hardware/rt117x/imxrt117x_memorymap.h"
#include "imxrt_flexspi_nor_l0.h"

#define FLEXSPI_NOR_INSTANCE  1u    /* FlexSPI1 */
#define FLEXSPI_NOR_SECTORS   (FLEXSPI_NOR_TOTAL_SIZE / FLEXSPI_NOR_SECTOR_SIZE)

extern struct flexspi_nor_config_s g_bootConfig;

static sem_t g_exclsem = SEM_INITIALIZER(1);
static uint8_t g_blank[FLEXSPI_NOR_SECTORS / 8];    /* set: sector known blank */

static inline uint32_t device_sector(const struct flexspi_nor_region_s *region, uint32_t sector)
{
	return region->offset / FLEXSPI_NOR_SECTOR_SIZE + sector;
}

static inline bool blank_get(const struct flexspi_nor_region_s *region, uint32_t sector)
{
	const uint32_t s = device_sector(region, sector);
	return g_blank[s / 8] & (1u << (s % 8));
}

static inline void blank_set(const struct flexspi_nor_region_s *region, uint32_t sector, bool blank)
{
	const uint32_t s = device_sector(region, sector);

	if (blank) {
		g_blank[s / 8] |= 1u << (s % 8);

	} else {
		g_blank[s / 8] &= ~(1u << (s % 8));
	}
}

/* Caller holds g_exclsem */
static bool sector_blank(const struct flexspi_nor_region_s *region, uint32_t sector)
{
	if (blank_get(region, sector)) {
		return true;
	}

	const uint32_t *p = (const uint32_t *)(uintptr_t)(region->ahb + sector * FLEXSPI_NOR_SECTOR_SIZE);

	for (unsigned i = 0; i < FLEXSPI_NOR_SECTOR_SIZE / sizeof(uint32_t); i++) {
		if (p[i] != 0xffffffffu) {
			return false;
		}
	}

	blank_set(region, sector, true);
	return true;
}

/* ClearCache: the ROM routines leave the AHB prefetch buffers stale */
locate_code(".ramfunc")
static uint32_t rom_program_page(uint32_t offset, const uint32_t *src)
{
	cpsid();
	uint32_t status = ROM_FLEXSPI_NorFlash_ProgramPage(FLEXSPI_NOR_INSTANCE, &g_bootConfig, offset, src);
	ROM_FLEXSPI_NorFlash_ClearCache(FLEXSPI_NOR_INSTANCE);
	cpsie();
	return status;
}

locate_code(".ramfunc")
static uint32_t rom_erase_sector(uint32_t offset)
{
	cpsid();
	uint32_t status = ROM_FLEXSPI_NorFlash_Erase(FLEXSPI_NOR_INSTANCE, &g_bootConfig, offset, FLEXSPI_NOR_SECTOR_SIZE);
	ROM_FLEXSPI_NorFlash_ClearCache(FLEXSPI_NOR_INSTANCE);
	cpsie();
	return status;
}

static void invalidate(const struct flexspi_nor_region_s *region, uint32_t offset, size_t len)
{
#ifdef CONFIG_ARMV7M_DCACHE
	const uintptr_t start = (uintptr_t)region->ahb + offset;
	up_invalidate_dcache(start, start + len);
#endif
}

int flexspi_nor_l0_init(struct flexspi_nor_region_s *region, const char *program_name,
			const char *erase_name, const char *erase_skip_name)
{
	if (region->ahb != NULL) {
		return -EALREADY;
	}

	if (region->size == 0 ||
	    region->size > FLEXSPI_NOR_TOTAL_SIZE ||
	    region->offset % FLEXSPI_NOR_SECTOR_SIZE != 0 ||
	    region->size % FLEXSPI_NOR_SECTOR_SIZE != 0 ||
	    region->offset > FLEXSPI_NOR_TOTAL_SIZE - region->size) {
		return -EINVAL;
	}

	region->ahb = (const uint8_t *)(uintptr_t)(IMXRT_FLEXSPI1_CIPHER_BASE + region->offset);
	region->perf_program = perf_alloc(PC_ELAPSED, program_name);
	region->perf_erase = perf_alloc(PC_ELAPSED, erase_name);
	region->perf_erase_skip = perf_alloc(PC_COUNT, erase_skip_name);

	return 0;
}

ssize_t flexspi_nor_l0_read(struct flexspi_nor_region_s *region, uint32_t offset, void *dst, size_t len)
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

ssize_t flexspi_nor_l0_program(struct flexspi_nor_region_s *region, uint32_t offset, const void *src, size_t len)
{
	if (region->ahb == NULL || (uintptr_t)src % 4 != 0 ||
	    offset % FLEXSPI_NOR_PAGE_SIZE != 0 || len % FLEXSPI_NOR_PAGE_SIZE != 0) {
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

	for (uint32_t s = offset / FLEXSPI_NOR_SECTOR_SIZE; s <= (offset + len - 1) / FLEXSPI_NOR_SECTOR_SIZE; s++) {
		blank_set(region, s, false);
	}

	const uint8_t *p = src;
	size_t written = 0;

	while (written < len) {
		perf_begin(region->perf_program);
		uint32_t status = rom_program_page(region->offset + offset + written, (const uint32_t *)(uintptr_t)(p + written));
		perf_end(region->perf_program);

		if (status != 0) {
			break;
		}

		written += FLEXSPI_NOR_PAGE_SIZE;
	}

	invalidate(region, offset, written);
	nxsem_post(&g_exclsem);
	return (ssize_t)written;
}

int flexspi_nor_l0_erase(struct flexspi_nor_region_s *region, uint32_t sector, uint32_t nsectors)
{
	if (region->ahb == NULL) {
		return -EINVAL;
	}

	const uint32_t region_sectors = region->size / FLEXSPI_NOR_SECTOR_SIZE;

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
		uint32_t status = rom_erase_sector(region->offset + s * FLEXSPI_NOR_SECTOR_SIZE);
		perf_end(region->perf_erase);

		invalidate(region, s * FLEXSPI_NOR_SECTOR_SIZE, FLEXSPI_NOR_SECTOR_SIZE);

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

#endif /* CONFIG_BOARD_FLEXSPI_NOR_L0 */
