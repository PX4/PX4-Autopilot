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
 * Region-relative read, program and erase over the FlexSPI1 boot NOR via the
 * ROM API, from .ramfunc with interrupts masked, one page or one sector per
 * window. Exclusion is device wide. Regions come from flash_layout.h.
 *
 * Constraints L0 cannot enforce:
 * - Masking is PRIMASK, not BASEPRI: zero-latency handlers execute from the
 *   busy flash.
 * - DMA is not masked. No descriptor may reference the XIP window
 *   (0x30000000-0x34000000) while an operation can be in flight.
 * - Nothing else programs or erases this NOR.
 * - The D-cache is write-through over the AHB window: invalidate-only suffices.
 * - Reads use the AHB mapping, program and erase physical offsets: invalid for
 *   a slot under FlexSPI address remap.
 * - The system tick stops per window (~25 ms per sector erase); hrt does not.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <perf/perf_counter.h>

#include "flash_layout.h"

/* perf_alloc() keeps the pointer: names must be literals */
#define FLEXSPI_NOR_L0_PERF(prefix) \
	(prefix ": program"), (prefix ": erase"), (prefix ": erase blank skip")

struct flexspi_nor_region_s {
	uint32_t offset;        /* sector aligned */
	uint32_t size;          /* sector multiple */

	/* set by flexspi_nor_l0_init() */
	const uint8_t *ahb;
	perf_counter_t perf_program;
	perf_counter_t perf_erase;
	perf_counter_t perf_erase_skip;
};

/* -EINVAL on bad geometry, -EALREADY if bound; other calls fail until this succeeds */
int flexspi_nor_l0_init(struct flexspi_nor_region_s *region, const char *program_name,
			const char *erase_name, const char *erase_skip_name);

ssize_t flexspi_nor_l0_read(struct flexspi_nor_region_s *region, uint32_t offset, void *dst, size_t len);

/* offset and len page multiples, src word aligned, target erased; returns bytes programmed */
ssize_t flexspi_nor_l0_program(struct flexspi_nor_region_s *region, uint32_t offset, const void *src, size_t len);

/* Blank sectors are skipped; yields between erases */
int flexspi_nor_l0_erase(struct flexspi_nor_region_s *region, uint32_t sector, uint32_t nsectors);
