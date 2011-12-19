/****************************************************************************
 * arch/arm/src/stm32/up_allocateheap.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/mm.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* For the STM312F10xxx family, all SRAM is in a contiguous block starting
 * at g_heapbase and extending through CONFIG_DRAM_END (my apologies for
 * the bad naming).
 */

#if defined(CONFIG_STM32_STM32F10XX)
#  define SRAM1_END CONFIG_DRAM_END

/* All members of the STM32F40xxx family have 192Kb in three banks:
 *
 * 1) 112Kb of SRAM beginning at address 0x2000:0000
 * 2)  16Kb of SRAM beginning at address 0x2001:c000
 * 3)  64Kb of TCM SRAM beginning at address 0x1000:0000
 *
 * As determined by ld.script, g_heapbase lies in the 112Kb memory
 * region and that extends to 0x2001:0000.  But the  first and second memory
 * regions are contiguous and treated as one in this logic that extends to
 * 0x2002:0000.
 */

#elif defined(CONFIG_STM32_STM32F40XX)
#  define SRAM1_END   0x20020000
#  define SRAM2_START 0x10000000
#  define SRAM2_END   0x10010000
#else
#  error "Unsupported STM32 chip"
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
  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_heapbase;
  *heap_size  = SRAM1_END - g_heapbase;
}

/****************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
#  if defined(CONFIG_STM32_STM32F40XX)
#    if defined(CONFIG_HEAP2_BASE) && defined(CONFIG_HEAP2_END)
#      if CONFIG_MM_REGIONS > 3
#        error "CONFIG_MM_REGIONS > 3 but I don't know what all of the regions are"
#      endif
#    elif CONFIG_MM_REGIONS > 2
#      error "CONFIG_MM_REGIONS > 2 but I don't know what all of the regions are"
#    endif

void up_addregion(void)
{
  /* Add the STM32F40xxx TCM SRAM heap region. */

   mm_addregion((FAR void*)SRAM2_START, SRAM2_END-SRAM2_START);

   /* Add the user specified heap region. */

#  if CONFIG_MM_REGIONS > 2 && defined(CONFIG_HEAP2_BASE) && defined(CONFIG_HEAP2_END)
   mm_addregion((FAR void*)CONFIG_HEAP2_BASE, CONFIG_HEAP2_END - CONFIG_HEAP2_BASE);
#  endif
}

#  elif defined(CONFIG_HEAP2_BASE) && defined(CONFIG_HEAP2_END)
#    if CONFIG_MM_REGIONS > 2
#      error "CONFIG_MM_REGIONS > 2 but I don't know what all of the regions are"
#    endif

void up_addregion(void)
{
  /* Add the user specified heap region. */

  mm_addregion((FAR void*)CONFIG_HEAP2_BASE, CONFIG_HEAP2_END - CONFIG_HEAP2_BASE);
}

#  else
#    error "CONFIG_MM_REGIONS > 1 but I don't know what any of the region(s) are"
#  endif
#endif
