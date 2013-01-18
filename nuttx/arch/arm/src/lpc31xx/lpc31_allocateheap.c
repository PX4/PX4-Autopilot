/************************************************************************
 * arch/arm/src/lpc31xx/lpc31_allocateheap.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lpc31_memorymap.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/* Configuration ********************************************************/

/* Some sanity checking.  If external memory regions are defined, verify
 * that CONFIG_MM_REGIONS is set to match, exactly, the number of external
 * memory regions that we have been asked to add to the heap.
 */

#if defined(CONFIG_ARCH_EXTSRAM0) && defined(CONFIG_ARCH_EXTSRAM0HEAP)
#  if defined(CONFIG_ARCH_EXTSRAM1) && defined(CONFIG_ARCH_EXTSRAM1HEAP)
#    if defined(CONFIG_ARCH_EXTDRAM) && defined(CONFIG_ARCH_EXTDRAMHEAP)
#      /* SRAM+EXTSRAM0+EXTSRAM1+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 4
#    else
#      /* SRAM+EXTSRAM0+EXTSRAM1 */
#      define LPC31_NEXT_REGIONS 3
#    endif
#  elif defined(CONFIG_ARCH_EXTDRAM) && defined(CONFIG_ARCH_EXTDRAMHEAP)
#      /* SRAM+EXTSRAM0+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 3
#  else
#      /* SRAM+EXTSRAM0 */
#      define LPC31_NEXT_REGIONS 2
#  endif
#elif defined(CONFIG_ARCH_EXTSRAM1) && defined(CONFIG_ARCH_EXTSRAM1HEAP)
#  if defined(CONFIG_ARCH_EXTDRAM) && defined(CONFIG_ARCH_EXTDRAMHEAP)
#      /* SRAM+EXTSRAM1+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 3
#  else
#      /* SRAM+EXTSRAM1 */
#      define LPC31_NEXT_REGIONS 2
#  endif
#elif defined(CONFIG_ARCH_EXTDRAM) && defined(CONFIG_ARCH_EXTDRAMHEAP)
#      /* SRAM+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 2
#else
#      /* SRAM */
#      define LPC31_NEXT_REGIONS 1
#endif

#if CONFIG_MM_REGIONS != LPC31_NEXT_REGIONS
#  if CONFIG_MM_REGIONS < LPC31_NEXT_REGIONS
#    error "CONFIG_MM_REGIONS is large enough for the selected memory regions"
#  else
#    error "CONFIG_MM_REGIONS is too large for the selected memory regions"
#  endif
#  if defined(CONFIG_ARCH_EXTSRAM0) && defined(CONFIG_ARCH_EXTSRAM0HEAP)
#    error "External SRAM0 is selected for heap"
#  endif
#  if defined(CONFIG_ARCH_EXTSRAM1) && defined(CONFIG_ARCH_EXTSRAM1HEAP)
#    error "External SRAM1 is selected for heap"
#  endif
#  if defined(CONFIG_ARCH_EXTDRAM) && defined(CONFIG_ARCH_EXTDRAMHEAP)
#    error "External SRAM1 is selected for heap"
#  endif
#endif

/* The following determines the end+1 address the heap (in SRAM0 or SRAM1)
 * and that, in turn, determines the size of the heap.  Specifically, this
 * logic it checks if a page table has been allocated at the end of SRAM
 * and, if so, subtracts that the size of the page table from the end+1
 * address of heap.
 *
 * If the page table was not allocated at the end of SRAM, then this logic
 * will let the heap run all the way to the end of SRAM.
 */

#ifdef CONFIG_PAGING
#  ifdef PGTABLE_IN_HIGHSRAM
#    define LPC31_HEAP_VEND (PG_LOCKED_VBASE + PG_TOTAL_VSIZE - PGTABLE_SIZE)  
#  else
#    define LPC31_HEAP_VEND (PG_LOCKED_VBASE + PG_TOTAL_VSIZE)  
#  endif
#else
#  ifdef PGTABLE_IN_HIGHSRAM
#    define LPC31_HEAP_VEND (LPC31_INTSRAM_VSECTION + LPC31_ISRAM_SIZE - PGTABLE_SIZE)  
#  else
#    define LPC31_HEAP_VEND (LPC31_INTSRAM_VSECTION + LPC31_ISRAM_SIZE)  
#  endif
#endif

/************************************************************************
 * Private Data
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   The heap may be statically allocated by defining CONFIG_HEAP_BASE
 *   and CONFIG_HEAP_SIZE.  If these are not defined, then this function
 *   will be called to dynamically set aside the heap region to the end
 *   of SRAM.
 *
 *   SRAM layout:
 *   Start of SRAM:   .data
 *                    .bss
 *                    IDLE thread stack
 *   End of SRAm:     heap
 *
 *   NOTE: Ignore the erroneous nomenclature DRAM and SDRAM.  That names
 *   date back to an earlier platform that had SDRAM.
 *
 ************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_heapbase;
  *heap_size  = LPC31_HEAP_VEND - g_heapbase;
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
#if defined(CONFIG_ARCH_EXTSRAM0) && defined(CONFIG_ARCH_EXTSRAM0HEAP)
  mm_addregion((FAR void*)LPC31_EXTSRAM0_VSECTION, CONFIG_ARCH_EXTSRAM0SIZE);
#endif

#if defined(CONFIG_ARCH_EXTSRAM1) && defined(CONFIG_ARCH_EXTSRAM1HEAP)
  mm_addregion((FAR void*)LPC31_EXTSRAM1_VSECTION, CONFIG_ARCH_EXTSRAM1SIZE);
#endif

#if defined(CONFIG_ARCH_EXTDRAM) && defined(CONFIG_ARCH_EXTDRAMHEAP)
  mm_addregion((FAR void*)LPC31_EXTSDRAM_VSECTION, CONFIG_ARCH_EXTDRAMSIZE);
#endif
}
#endif
