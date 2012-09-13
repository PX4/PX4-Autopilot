/****************************************************************************
 * arch/arm/src/arm/up_checkmapping.c
 * Check if the current task's fault address has been mapped into the virtual
 * address space.
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
 * Name: up_checkmapping()
 *
 * Description:
 *  The function up_checkmapping() returns an indication if the page fill
 *  still needs to performed or not. In certain conditions, the page fault
 *  may occur on several threads and be queued multiple times. This function
 *  will prevent the same page from be filled multiple times.
 *
 * Input Parameters:
 *   tcb - A reference to the task control block of the task that we believe
 *         needs to have a page fill.  Architecture-specific logic can
 *         retrieve page fault information from the architecture-specific
 *         context information in this TCB and can consult processor resources
 *         (page tables or TLBs or ???) to determine if the fill still needs
 *         to be performed or not.
 *
 * Returned Value:
 *   This function will return true if the mapping is in place and false
 *   if the mapping is still needed.  Errors encountered should be
 *   interpreted as fatal.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

bool up_checkmapping(FAR _TCB *tcb)
{
  uintptr_t vaddr;
  uint32_t *pte;

  /* Since interrupts are disabled, we don't need to anything special. */

  DEBUGASSERT(tcb);

  /* Get the virtual address that caused the fault */

  vaddr = tcb->xcp.far;
  DEBUGASSERT(vaddr >= PG_PAGED_VBASE && vaddr < PG_PAGED_VEND);

  /* Get the PTE associated with this virtual address */

  pte = up_va2pte(vaddr);

  /* Return true if this virtual address is mapped. */

  return (*pte != 0);
}

#endif /* CONFIG_PAGING */
