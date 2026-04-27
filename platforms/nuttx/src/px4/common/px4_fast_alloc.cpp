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

#include <px4_platform_common/px4_fast_alloc.h>
#include <px4_arch/dtcm.h>
#include <stdlib.h>

void *px4_fast_malloc(size_t size)
{
#ifdef HAVE_DTCM_HEAP
	if (g_dtcm_heap != nullptr) {
		void *p = dtcm_malloc(size);

		if (p != nullptr) {
			return p;
		}
	}

#endif
	return malloc(size);
}

void *px4_fast_memalign(size_t alignment, size_t size)
{
	void *p = px4_fast_malloc(size);
	return p ? p : memalign(alignment, size);
}

void px4_fast_free(void *ptr)
{
#ifdef HAVE_DTCM_HEAP

	if (g_dtcm_heap != nullptr
	    && ptr >= (void *)DTCM_START
	    && ptr <  (void *)DTCM_END) {
		dtcm_free(ptr);
		return;
	}

#endif
	free(ptr);
}
