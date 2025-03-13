/****************************************************************************
 *
 *   Copyright (c) 2025 Technology Innovation Institute. All rights reserved.
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
 * @file secure_heap.c
 *
 * Implementation for heap placed in secure ram area defined by linker
 * script
 */

#if defined(__PX4_NUTTX)

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

static struct mm_heap_s *sec_heap __attribute__((section(".secmem")));
static uint32_t memory_pool[6 * 1024 - sizeof(sec_heap) / 4] __attribute__((section(".secmem"))); /* 24 KB */

void secure_heap_init(void)
{
	sec_heap = mm_initialize("Secmem", memory_pool, sizeof(memory_pool));
}

void *sec_malloc(size_t size)
{
	return mm_zalloc(sec_heap, size);
}

void *sec_calloc(size_t nemb, size_t size)
{
	return mm_calloc(sec_heap, nemb, size);
}

void *sec_realloc(void *ptr, size_t size)
{
	return mm_realloc(sec_heap, ptr, size);
}

void sec_free(void *ptr)
{
	if (ptr) {
		mm_free(sec_heap, ptr);
	}
}

#endif
