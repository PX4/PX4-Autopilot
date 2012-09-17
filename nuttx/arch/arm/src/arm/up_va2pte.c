/****************************************************************************
 * arch/arm/src/arm/up_va2pte.c
 * Utility to map a virtual address to a L2 page table entry.
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/page.h>

#include "chip.h"
#include "pg_macros.h"
#include "up_internal.h"

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: up_va2pte()
 *
 * Description:
 *  Convert a virtual address within the paged text region into a pointer to
 *  the corresponding page table entry.
 *
 * Input Parameters:
 *   vaddr - The virtual address within the paged text region.
 *
 * Returned Value:
 *   A pointer to  the corresponding page table entry.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

uint32_t *up_va2pte(uintptr_t vaddr)
{
  uint32_t L1;
  uint32_t *L2;
  unsigned int ndx;

  /* The virtual address is expected to lie in the paged text region */

  DEBUGASSERT(vaddr >= PG_PAGED_VBASE && vaddr < PG_PAGED_VEND);

  /* Get the L1 table entry associated with this virtual address */

  L1 = *(uint32_t*)PG_POOL_VA2L1VADDR(vaddr);

  /* Get the address of the L2 page table from the L1 entry */

  L2 = (uint32_t*)PG_POOL_L12VPTABLE(L1);

  /* Get the index into the L2 page table.  Each L1 entry maps
   * 256 x 4Kb or 1024 x 1Kb pages.
   */

  ndx = (vaddr & 0x000fffff) >> PAGESHIFT;

  /* Return true if this virtual address is mapped. */

  return &L2[ndx];
}

#endif /* CONFIG_PAGING */
