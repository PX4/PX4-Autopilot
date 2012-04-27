/****************************************************************************
 * arch/arm/src/stm32/up_allocateheap.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
/* Internal SRAM is available in all members of the STM32 family. The
 * following definitions must be provided to specify the size and
 * location of internal(system) SRAM:
 *
 * CONFIG_DRAM_END            : End address (+1) of SRAM (F1 family only, the
 *                            : F4 family uses the a priori end of SRAM)
 *
 * The F4 family also contains internal CCM SRAM.  This SRAM is different
 * because it cannot be used for DMA.  So if DMA needed, then the following
 * should be defined to exclude CCM SRAM from the heap:
 *
 * CONFIG_STM32_CCMEXCLUDE    : Exclude CCM SRAM from the HEAP
 *
 * In addition to internal SRAM, SRAM may also be available through the FSMC.
 * In order to use FSMC SRAM, the following additional things need to be
 * present in the NuttX configuration file:
 *
 * CONFIG_STM32_FSMC=y        : Enables the FSMC
 * CONFIG_STM32_FSMC_SRAM=y   : Indicates that SRAM is available via the
 *   FSMC (as opposed to an LCD or FLASH).
 * CONFIG_HEAP2_BASE          : The base address of the SRAM in the FSMC
 *   address space
 * CONFIG_HEAP2_END           : The end (+1) of the SRAM in the FSMC
 *   address space
 * CONFIG_MM_REGIONS          : Must be set to a large enough value to
 *   include the FSMC SRAM (as determined by the rules provided below)
 */

#ifndef CONFIG_STM32_FSMC
#  undef CONFIG_STM32_FSMC_SRAM
#endif

/* For the STM312F10xxx family, all internal SRAM is in one contiguous block
 * starting at g_heapbase and extending through CONFIG_DRAM_END (my apologies for
 * the bad naming).  In addition, external FSMC SRAM may be available.
 */

#if defined(CONFIG_STM32_STM32F10XX)

   /* Set the end of system SRAM */

#  define SRAM1_END CONFIG_DRAM_END

   /* Check if external FSMC SRAM is provided */

#  if CONFIG_STM32_FSMC_SRAM
#    if CONFIG_MM_REGIONS < 2
#      warning "FSMC SRAM not included in the heap"
#      undef CONFIG_STM32_FSMC_SRAM
#    elif CONFIG_MM_REGIONS > 2
#      error "CONFIG_MM_REGIONS > 2 but I don't know what some of the region(s) are"
#      undef CONFIG_MM_REGIONS
#      define CONFIG_MM_REGIONS 2
#    endif
#  elif CONFIG_MM_REGIONS > 1
#    error "CONFIG_MM_REGIONS > 1 but I don't know what the other region(s) are"
#  endif

   /* The STM32 F1 has not CCM SRAM */

#  undef CONFIG_STM32_CCMEXCLUDE
#  define CONFIG_STM32_CCMEXCLUDE 1

/* All members of the STM32F20xxx and STM32F40xxx families have 192Kb in three banks:
 *
 * 1) 112Kb of System SRAM beginning at address 0x2000:0000
 * 2)  16Kb of System SRAM beginning at address 0x2001:c000
 * 3)  64Kb of CCM SRAM beginning at address 0x1000:0000
 *
 * As determined by ld.script, g_heapbase lies in the 112Kb memory
 * region and that extends to 0x2001:0000.  But the  first and second memory
 * regions are contiguous and treated as one in this logic that extends to
 * 0x2002:0000.
 *
 * As a complication, it appears that CCM SRAM cannot be used for DMA.  So, if
 * STM32 DMA is enabled, CCM SRAM should probably be excluded from the heap.
 *
 * In addition, external FSMC SRAM may be available.
 */

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)

   /* Set the end of system SRAM */

#  define SRAM1_END   0x20020000

   /* Set the range of CCM SRAM as well (although we may not use it) */

#  define SRAM2_START 0x10000000
#  define SRAM2_END   0x10010000

   /* There are 4 possible SRAM configurations:
    *
    * Configuration 1. System SRAM (only)
    *                  CONFIG_MM_REGIONS == 1
    *                  CONFIG_STM32_FSMC_SRAM NOT defined
    *                  CONFIG_STM32_CCMEXCLUDE defined
    * Configuration 2. System SRAM and CCM SRAM
    *                  CONFIG_MM_REGIONS == 2
    *                  CONFIG_STM32_FSMC_SRAM NOT defined
    *                  CONFIG_STM32_CCMEXCLUDE NOT defined
    * Configuration 3. System SRAM and FSMC SRAM
    *                  CONFIG_MM_REGIONS == 2
    *                  CONFIG_STM32_FSMC_SRAM defined
    *                  CONFIG_STM32_CCMEXCLUDE defined
    * Configuration 4. System SRAM, CCM SRAM, and FSMC SRAM
    *                  CONFIG_MM_REGIONS == 3
    *                  CONFIG_STM32_FSMC_SRAM defined
    *                  CONFIG_STM32_CCMEXCLUDE NOT defined
    *
    * Let's make sure that all definitions are consitent before doing
    * anything else
    */

#  if defined(CONFIG_STM32_FSMC_SRAM)

   /* Configuration 3 or 4. External SRAM is available.  CONFIG_MM_REGIONS
    * should be at least 2.
    */

#    if CONFIG_MM_REGIONS < 2
       /* Only one memory region.  Force Configuration 1 */

#      warning "FSMC SRAM (and CCM SRAM) excluded from the heap"
#      undef CONFIG_STM32_FSMC_SRAM
#      undef CONFIG_STM32_CCMEXCLUDE
#      define CONFIG_STM32_CCMEXCLUDE 1

   /* CONFIG_MM_REGIONS may be 3 if CCM SRAM is included in the head */

#    elif CONFIG_MM_REGIONS > 2

       /* More than two memory regions.  This is okay if CCM SRAM is not
        * disabled.
        */

#      if defined(CONFIG_STM32_CCMEXCLUDE)

         /* Configuration 3: CONFIG_MM_REGIONS should have been 2 */

#        error "CONFIG_MM_REGIONS > 2 but I don't know what some of the region(s) are"
#        undef CONFIG_MM_REGIONS
#        define CONFIG_MM_REGIONS 2
#      else

         /* Configuration 4: DMA should be disabled and CONFIG_MM_REGIONS
          * should be 3.
          */

#        ifdef CONFIG_STM32_DMA
#          warning "CCM SRAM is included in the heap AND DMA is enabled"
#        endif
#        if CONFIG_MM_REGIONS != 3
#          error "CONFIG_MM_REGIONS > 3 but I don't know what some of the region(s) are"
#          undef CONFIG_MM_REGIONS
#          define CONFIG_MM_REGIONS 3
#        endif
#      endif

   /* CONFIG_MM_REGIONS is exactly 2.  We cannot support both CCM SRAM and
    * FSMC SRAM.
    */

#    elif !defined(CONFIG_STM32_CCMEXCLUDE)
#      error "CONFIG_MM_REGIONS == 2, cannot support both CCM SRAM and FSMC SRAM"
#      undef CONFIG_STM32_CCMEXCLUDE
#      define CONFIG_STM32_CCMEXCLUDE 1
#    endif
   
#  elif !defined(CONFIG_STM32_CCMEXCLUDE)

   /* Configuration 2: FSMC SRAM is not used, but CCM SRAM is requested.  DMA
    * should be disabled and CONFIG_MM_REGIONS should be 2.
    */

#    ifdef CONFIG_STM32_DMA
#      warning "CCM SRAM is included in the heap AND DMA is enabled"
#    endif
#    if CONFIG_MM_REGIONS < 2
#      error "CCM SRAM excluded from the heap because CONFIG_MM_REGIONS < 2"
#      undef CONFIG_STM32_CCMEXCLUDE
#      define CONFIG_STM32_CCMEXCLUDE 1
#    elif CONFIG_MM_REGIONS > 2
#      error "CONFIG_MM_REGIONS > 2 but I don't know what some of the region(s) are"
#      undef CONFIG_MM_REGIONS
#      define CONFIG_MM_REGIONS 2
#    endif
#  endif
#else
#  error "Unsupported STM32 chip"
#endif

/* If FSMC SRAM is going to be used as heap, then verify that the starting
 * address and size of the external SRAM region has been provided in the
 * configuration (as CONFIG_HEAP2_BASE and CONFIG_HEAP2_END).
 */

#ifdef CONFIG_STM32_FSMC_SRAM
#  if !defined(CONFIG_HEAP2_BASE) || !defined(CONFIG_HEAP2_END)
#    error "CONFIG_HEAP2_BASE and CONFIG_HEAP2_END must be provided"
#    undef CONFIG_STM32_FSMC_SRAM
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
void up_addregion(void)
{
  /* Add the STM32F20xxx/STM32F40xxx CCM SRAM heap region. */

#ifndef CONFIG_STM32_CCMEXCLUDE
   mm_addregion((FAR void*)SRAM2_START, SRAM2_END-SRAM2_START);
#endif

   /* Add the external FSMC SRAM heap region. */

#ifdef CONFIG_STM32_FSMC_SRAM
   mm_addregion((FAR void*)CONFIG_HEAP2_BASE, CONFIG_HEAP2_END - CONFIG_HEAP2_BASE);
#endif
}
#endif
