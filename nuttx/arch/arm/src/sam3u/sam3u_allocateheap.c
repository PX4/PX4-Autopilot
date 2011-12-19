/****************************************************************************
 * arch/arm/src/common/sam3u_allocateheap.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "sam3u_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#if CONFIG_MM_REGIONS < 2
#  warning "CONFIG_MM_REGIONS < 2: SRAM1 not included in HEAP"
#endif

#if CONFIG_MM_REGIONS < 3 && !defined(CONFIG_SAM3U_NAND)
#  warning "CONFIG_MM_REGIONS < 3: NFC SRAM not included in HEAP"
#endif

#if CONFIG_MM_REGIONS > 2 && defined(CONFIG_SAM3U_NAND)
#  error "CONFIG_MM_REGIONS > 3 but cannot used NFC SRAM"
#  undef CONFIG_MM_REGIONS
#  define CONFIG_MM_REGIONS 2
#endif

#if CONFIG_DRAM_END > (SAM3U_INTSRAM0_BASE+CONFIG_SAM3U_SRAM0_SIZE)
#  error "CONFIG_DRAM_END is beyond the end of SRAM0"
#  undef CONFIG_DRAM_END
#  define CONFIG_DRAM_END (SAM3U_INTSRAM0_BASE+CONFIG_SAM3U_SRAM0_SIZE)
#elif CONFIG_DRAM_END < (SAM3U_INTSRAM0_BASE+CONFIG_SAM3U_SRAM0_SIZE)
#  warning "CONFIG_DRAM_END is before end of SRAM0... not all of SRAM0 used"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   The heap may be statically allocated by
 *   defining CONFIG_HEAP_BASE and CONFIG_HEAP_SIZE.  If these
 *   are not defined, then this function will be called to
 *   dynamically set aside the heap region.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  size_t size = CONFIG_DRAM_END - g_heapbase;

  /* Return the heap settings */

  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_heapbase;
  *heap_size  = size;

  /* Allow access to the heap memory */

   sam3u_mpuheap((uintptr_t)g_heapbase, size);
}

/************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ************************************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{
  /* Add the region */

  kmm_addregion((FAR void*)SAM3U_INTSRAM1_BASE, CONFIG_SAM3U_SRAM1_SIZE);

  /* Allow access to the heap memory */

  sam3u_mpuheap(SAM3U_INTSRAM1_BASE, CONFIG_SAM3U_SRAM1_SIZE);

  /* Add the region */

#if CONFIG_MM_REGIONS > 2
  kmm_addregion((FAR void*)SAM3U_NFCSRAM_BASE, CONFIG_SAM3U_NFCSRAM_SIZE);

  /* Allow access to the heap memory */

  sam3u_mpuheap(SAM3U_NFCSRAM_BASE, CONFIG_SAM3U_NFCSRAM_SIZE);
#endif
}
#endif
