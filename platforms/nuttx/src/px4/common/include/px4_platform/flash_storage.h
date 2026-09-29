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

/*
 * Flash storage: an MTD device over a flash region mounted as littlefs, for
 * boards whose filesystem shares the die the CPU executes from. The board
 * supplies the region geometry and a backend (read, program, erase); this
 * layer provides the MTD surface, the mount, and refuses program and erase
 * while the vehicle is armed (CONFIG_BOARD_FLASH_STORAGE_ARMED_READONLY).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/mtd/mtd.h>
#include <perf/perf_counter.h>

__BEGIN_DECLS

struct px4_flash_storage_ops_s {
	ssize_t (*read)(void *ctx, uint32_t offset, void *dst, size_t len);
	ssize_t (*program)(void *ctx, uint32_t offset, const void *src, size_t len);   /* page multiples, target erased */
	int (*erase)(void *ctx, uint32_t sector, uint32_t nsectors);
};

struct px4_flash_storage_s {
	struct mtd_dev_s mtd;                          /* first: the MTD handle is the device */

	/* set by the board */
	const struct px4_flash_storage_ops_s *ops;
	void *ctx;
	uint32_t size;
	uint32_t page_size;
	uint32_t sector_size;

	/* set by px4_flash_storage_register() */
	perf_counter_t refused;
};

/* Registers the MTD device at devpath and mounts it as littlefs at mountpoint with autoformat */
int px4_flash_storage_register(struct px4_flash_storage_s *dev, const char *devpath, const char *mountpoint);

__END_DECLS
