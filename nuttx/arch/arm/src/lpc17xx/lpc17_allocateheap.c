/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_allocateheap.c
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

#include "chip/lpc17_memorymap.h"
#include "lpc17_emacram.h"
#include "lpc17_ohciram.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* The configured RAM start address must be the beginning of CPU SRAM */

#if CONFIG_DRAM_START != LPC17_SRAM_BASE
#  warning "CONFIG_DRAM_START is not at LPC17_SRAM_BASE"
#  undef CONFIG_DRAM_START
#  undef CONFIG_DRAM_END
#  define CONFIG_DRAM_START LPC17_SRAM_BASE
#  define CONFIG_DRAM_END  (LPC17_SRAM_BASE+LPC17_CPUSRAM_SIZE)
#endif

/* The configured RAM size must be less then or equal to the CPU SRAM size */

#if CONFIG_DRAM_SIZE > LPC17_CPUSRAM_SIZE
#  warning "CONFIG_DRAM_SIZE is larger than the size of CPU SRAM"
#  undef CONFIG_DRAM_SIZE
#  undef CONFIG_DRAM_END
#  define CONFIG_DRAM_SIZE LPC17_CPUSRAM_SIZE
#  define CONFIG_DRAM_END (LPC17_SRAM_BASE+LPC17_CPUSRAM_SIZE)
#elif CONFIG_DRAM_SIZE < LPC17_CPUSRAM_SIZE
#  warning "CONFIG_DRAM_END is before end of CPU SRAM... not all of CPU SRAM used"
#endif

/* Figure out how much heap be have in AHB SRAM (if any).  Complications:
 * 1) AHB SRAM Bank 0 or 1 may on may not be supported in the hardware.
 * 2) Some or all of Bank 0 may be used for Ethernet Packet buffering.
 *    Ethernet packet buffering is used from the beginning of Bank 0 and
 *    any free memory at the end of Bank 0 will be contiguous with any
 *    free memory at the beginning of Bank 1.
 * 3) Some or all of Bank 1 may be used for OHCI descriptor memory.  OCHI
 *    memory is used from the end of Bank 1 and any free memory at the
 *    beginning of Bank 1 will be contiguous with any free memory at the
 *    end of Bank 0.
 */

#undef LPC17_AHB_HEAPBASE /* Assume that nothing is available */
#undef LPC17_AHB_HEAPSIZE

/* If we have Bank 0, then we may possibly also have Bank 1 */

#ifdef LPC17_HAVE_BANK0

  /* We have BANK0 (and, hence, possibly Bank1).  Is Bank0 all used for
   * Ethernet packet buffering?  Or is there any part of Bank0 available for
   * the heap.
   */

#  ifdef LPC17_BANK0_HEAPSIZE

     /* Some or all of Bank0 is available for the heap.  The heap will begin
      * in bank 1.
      */

#     define LPC17_AHB_HEAPBASE LPC17_BANK0_HEAPBASE

     /* Is Bank1 present? Has there available heap memory in Bank 1? */

#    if defined(LPC17_HAVE_BANK1) && defined(LPC17_BANK1_HEAPSIZE)

       /* Yes... the heap space available is the unused memory at the end
        * of Bank0 plus the unused memory at the beginning of Bank 1.
        */

#       define LPC17_AHB_HEAPSIZE (LPC17_BANK0_HEAPSIZE + LPC17_BANK1_HEAPSIZE)
#    else

        /* No... the heap space available is only the unused memory at the
         * end of Bank 0.
         */

#       define LPC17_AHB_HEAPSIZE LPC17_BANK0_HEAPSIZE

#    endif /* LPC17_HAVE_BANK1 && LPC17_BANK1_HEAPSIZE */
#  else /* !LPC17_BANK0_HEAPSIZE */

     /* We have Bnak 0, but no memory is available for the heap there.
      * Do we have Bank 1? Is any heap memory available in Bank 1?
      */

#    if defined(LPC17_HAVE_BANK1) && defined(LPC17_BANK1_HEAPSIZE)

       /* Yes... the heap space available is the unused memory at the
        * beginning of Bank1.
        */

#      define LPC17_AHB_HEAPBASE LPC17_BANK1_HEAPBASE
#      define LPC17_AHB_HEAPSIZE LPC17_BANK1_HEAPSIZE

#    endif /* LPC17_HAVE_BANK1 && LPC17_BANK1_HEAPSIZE */
#  endif /* LPC17_BANK0_HEAPSIZE */
#endif /* LPC17_HAVE_BANK0 */

/* Sanity checking */

#ifdef LPC17_AHB_HEAPBASE
#  if CONFIG_MM_REGIONS < 2
#    warning "CONFIG_MM_REGIONS < 2: Available AHB SRAM Bank(s) not included in HEAP"
#  endif
#  if CONFIG_MM_REGIONS > 2
#    warning "CONFIG_MM_REGIONS > 2: Are additional regions handled by application?"
#  endif
#else
#  if CONFIG_MM_REGIONS > 1
#    warning "CONFIG_MM_REGIONS > 1: This configuration has no available AHB SRAM Bank0/1"
#    warning "CONFIG_MM_REGIONS > 1: Are additional regions handled by application?"
#  endif
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
  *heap_size = CONFIG_DRAM_END - g_heapbase;
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
  /* Banks 0 and 1 are each 16Kb.  If both are present, they occupy a
   * contiguous 32Kb memory region.
   *
   * If Ethernet is enabled, it will take some or all of bank 0 for packet
   * buffering and descriptor tables; If USB host is enabled, it will take
   * some or all of bank 1 for descriptor memory.  The complex conditional
   * compilation above should boil this all down to a very simple check
   * here:
   *
   * Is any memory available in AHB SRAM for the heap?
   */

#ifdef LPC17_AHB_HEAPBASE

  /* Yes... Add the AHB SRAM heap region. */

   mm_addregion((FAR void*)LPC17_AHB_HEAPBASE, LPC17_AHB_HEAPSIZE);
#endif
}
#endif
