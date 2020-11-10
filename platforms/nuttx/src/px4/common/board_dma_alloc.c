/****************************************************************************
 *
 *   Copyright (C) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file board_dma_alloc.c
 *
 * Provide the board dma allocator interface.
 */

#include <px4_platform_common/px4_config.h>
#include "board_config.h"

#include <stdint.h>
#include <errno.h>
#include <nuttx/mm/gran.h>

#include <perf/perf_counter.h>

/************************************************************************************
 * Name: board_dma_alloc_init
 *
 * Description:
 *   All boards may optionally provide this API to instantiate a pool of
 *   memory for uses with FAST FS DMA operations.
 *
 ************************************************************************************/
#if defined(BOARD_DMA_ALLOC_POOL_SIZE)

#if defined(CONFIG_GRAN)

static GRAN_HANDLE dma_allocator;

/*
 * The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
 * one sector-sized buffer per file.
 *
 * We use a fundamental alignment / granule size of 64B; this is sufficient
 * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
 */
static uint8_t g_dma_heap[BOARD_DMA_ALLOC_POOL_SIZE] __attribute__((aligned(64)));
static perf_counter_t g_dma_perf;
static uint16_t dma_heap_inuse;
static uint16_t dma_heap_peak_use;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
__EXPORT int
board_dma_alloc_init(void)
{
	dma_allocator = gran_initialize(g_dma_heap,
					sizeof(g_dma_heap),
					7,  /* 128B granule - must be > alignment (XXX bug?) */
					6); /* 64B alignment */

	if (dma_allocator == NULL) {
		return -ENOMEM;

	} else {
		dma_heap_inuse = 0;
		dma_heap_peak_use = 0;
		g_dma_perf = perf_alloc(PC_COUNT, "dma_alloc");
	}

	return OK;
}

__EXPORT int
board_get_dma_usage(uint16_t *dma_total, uint16_t *dma_used, uint16_t *dma_peak_used)
{
	*dma_total = sizeof(g_dma_heap);
	*dma_used = dma_heap_inuse;
	*dma_peak_used = dma_heap_peak_use;

	return OK;
}

__EXPORT void *
board_dma_alloc(size_t size)
{
	void *rv = NULL;
	perf_count(g_dma_perf);
	rv = gran_alloc(dma_allocator, size);

	if (rv != NULL) {
		dma_heap_inuse += size;

		if (dma_heap_inuse > dma_heap_peak_use) {
			dma_heap_peak_use = dma_heap_inuse;
		}
	}

	return rv;
}

__EXPORT void
board_dma_free(FAR void *memory, size_t size)
{
	gran_free(dma_allocator, memory, size);
	dma_heap_inuse -= size;
}

#endif /* CONFIG_GRAN */
#endif /* BOARD_DMA_ALLOC_POOL_SIZE */
