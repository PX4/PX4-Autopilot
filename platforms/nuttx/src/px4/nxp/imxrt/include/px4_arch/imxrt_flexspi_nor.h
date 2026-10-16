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
 * Region-relative read, program and erase over a FlexSPI NOR the CPU executes
 * from, via the i.MX RT ROM API, from .ramfunc with interrupts masked, one
 * page or one sector per window. The board supplies the FlexSPI instance, the
 * AHB (XIP) base and the ROM configuration the device was brought up with.
 * Exclusion is device wide. Requires ARCH_RAMFUNCS and ARCH_RAMVECTORS.
 *
 * Constraints this layer cannot enforce:
 * - Masking is PRIMASK, not BASEPRI: zero-latency handlers execute from the
 *   busy flash.
 * - DMA is not masked. No descriptor may reference the XIP window while an
 *   operation can be in flight.
 * - Nothing else programs or erases this NOR.
 * - The D-cache is write-through over the AHB window: invalidate-only suffices.
 * - Reads use the AHB mapping, program and erase physical offsets: invalid for
 *   a slot under FlexSPI address remap.
 * - The system tick stops per window; hrt does not. Budget windows from the
 *   datasheet maximum, not the bench: sector erase is 25 ms typical and
 *   400 ms maximum, page program 0.15 ms typical and 0.75 ms maximum, and
 *   both grow with program/erase cycles.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <perf/perf_counter.h>
#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_platform/flash_storage.h>

__BEGIN_DECLS

#define IMXRT_FLEXSPI_NOR_PAGE_SIZE     256u
#define IMXRT_FLEXSPI_NOR_SECTOR_SIZE   4096u
#define IMXRT_FLEXSPI_NOR_MAX_SECTORS   16384u    /* 64 MiB; bounds the blank bitmap */

/* perf_alloc() keeps the pointer: names must be literals */
#define IMXRT_FLEXSPI_NOR_PERF(prefix) \
	(prefix ": program"), (prefix ": erase"), (prefix ": erase blank skip")

struct imxrt_flexspi_nor_region_s {
	/* set by the board */
	uint32_t instance;                          /* FlexSPI instance the device is on */
	uintptr_t ahb_base;                         /* AHB (XIP) base of the device */
	struct flexspi_nor_config_s *config;        /* ROM API configuration in use for the device */
	uint32_t offset;                            /* sector aligned */
	uint32_t size;                              /* sector multiple */

	/* set by imxrt_flexspi_nor_init() */
	const uint8_t *ahb;
	perf_counter_t perf_program;
	perf_counter_t perf_erase;
	perf_counter_t perf_erase_skip;
	uint8_t blank[IMXRT_FLEXSPI_NOR_MAX_SECTORS / 8];   /* set: sector known blank */
};

/* -EINVAL on bad geometry, -EALREADY if bound; other calls fail until this succeeds */
int imxrt_flexspi_nor_init(struct imxrt_flexspi_nor_region_s *region, const char *program_name,
			   const char *erase_name, const char *erase_skip_name);

ssize_t imxrt_flexspi_nor_read(struct imxrt_flexspi_nor_region_s *region, uint32_t offset, void *dst, size_t len);

/* offset and len page multiples, src word aligned, target erased; returns bytes programmed */
ssize_t imxrt_flexspi_nor_program(struct imxrt_flexspi_nor_region_s *region, uint32_t offset, const void *src,
				  size_t len);

/* Blank sectors are skipped; yields between erases */
int imxrt_flexspi_nor_erase(struct imxrt_flexspi_nor_region_s *region, uint32_t sector, uint32_t nsectors);

/* Flash storage backend over a region; ctx is the struct imxrt_flexspi_nor_region_s */
extern const struct px4_flash_storage_ops_s g_imxrt_flexspi_nor_storage_ops;

__END_DECLS
