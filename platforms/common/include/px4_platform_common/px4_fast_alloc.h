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
 *    the distribution.
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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>

/**
 * Allocate from fast (DTCM) memory where available, falling back to the
 * regular heap. On platforms without DTCM this is equivalent to malloc/free.
 *
 * Allocations are one-shot at module startup — the fallback to malloc on a
 * full DTCM heap adds no runtime overhead to the hot path.
 */
__BEGIN_DECLS

void *px4_fast_malloc(size_t size);
void *px4_fast_memalign(size_t alignment, size_t size);
void  px4_fast_free(void *ptr);

__END_DECLS

/* Cache-aligned fast alloc: uses DTCM (cache-bypass, alignment irrelevant) when
 * available, falls back to memalign() on cached SRAM so cache maintenance stays safe. */
#if defined(ARMV7M_DCACHE_LINESIZE)
#  define px4_fast_cache_aligned_alloc(s) px4_fast_memalign(ARMV7M_DCACHE_LINESIZE, (s))
#else
#  define px4_fast_cache_aligned_alloc(s) px4_fast_malloc(s)
#endif
