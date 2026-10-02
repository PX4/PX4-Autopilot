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

/* littlefs over storage_partition of the FlexSPI1 boot NOR at /fs/flash */

#include <nuttx/config.h>

#ifdef CONFIG_BOARD_FLASH_STORAGE

#include <syslog.h>

#include <px4_arch/imxrt_flexspi_nor.h>
#include <px4_platform/flash_storage.h>

#include "board_config.h"
#include "hw_config.h"
#include "hardware/rt117x/imxrt117x_memorymap.h"

extern struct flexspi_nor_config_s g_bootConfig;

/* Filled at init so both stay in .bss: the region carries a 2 KiB blank bitmap */
static struct imxrt_flexspi_nor_region_s g_region;
static struct px4_flash_storage_s g_storage;

int fmuv6xrt_flash_storage_initialize(void)
{
	g_region.instance = 1;
	g_region.ahb_base = IMXRT_FLEXSPI1_CIPHER_BASE;
	g_region.config = &g_bootConfig;
	g_region.offset = FLASH_STORAGE_PARTITION_OFFSET;
	g_region.size = FLASH_STORAGE_PARTITION_SIZE;

	int ret = imxrt_flexspi_nor_init(&g_region, IMXRT_FLEXSPI_NOR_PERF("flash_storage"));

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] Bad flash storage region: %d\n", ret);
		return ret;
	}

	g_storage.ops = &g_imxrt_flexspi_nor_storage_ops;
	g_storage.ctx = &g_region;
	g_storage.size = FLASH_STORAGE_PARTITION_SIZE;
	g_storage.page_size = IMXRT_FLEXSPI_NOR_PAGE_SIZE;
	g_storage.sector_size = IMXRT_FLEXSPI_NOR_SECTOR_SIZE;

	return px4_flash_storage_register(&g_storage, "/dev/nor", "/fs/flash");
}

#endif /* CONFIG_BOARD_FLASH_STORAGE */
