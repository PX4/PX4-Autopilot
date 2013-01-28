/****************************************************************************
 *  arch/arm/src/src/up_prefetchabort.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Output debug info if stack dump is selected -- even if 
 * debug is not selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  lldbg
# define lldbg lowsyslog
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
 * Name: up_prefetchabort
 *
 * Description;
 *   This is the prefetch abort exception handler. The ARM prefetch abort
 *   exception occurs when a memory fault is detected during an an
 *   instruction fetch.
 *
 ****************************************************************************/

void up_prefetchabort(uint32_t *regs)
{
#ifdef CONFIG_PAGING
   uint32_t *savestate;

  /* Save the saved processor context in current_regs where it can be accessed
   * for register dumps and possibly context switching.
   */

  savestate    = (uint32_t*)current_regs;
#endif
  current_regs = regs;

#ifdef CONFIG_PAGING
  /* Get the (virtual) address of instruction that caused the prefetch abort.
   * When the exception occurred, this address was provided in the lr register
   * and this value was saved in the context save area as the PC at the
   * REG_R15 index.
   *
   * Check to see if this miss address is within the configured range of
   * virtual addresses.
   */

  pglldbg("VADDR: %08x VBASE: %08x VEND: %08x\n",
          regs[REG_PC], PG_PAGED_VBASE, PG_PAGED_VEND);

  if (regs[REG_R15] >= PG_PAGED_VBASE && regs[REG_R15] < PG_PAGED_VEND)
    {
      /* Save the offending PC as the fault address in the TCB of the currently
       * executing task.  This value is, of course, already known in regs[REG_R15],
       * but saving it in this location will allow common paging logic for both
       * prefetch and data aborts.
       */

      FAR _TCB *tcb = (FAR _TCB *)g_readytorun.head;
      tcb->xcp.far  = regs[REG_R15];

      /* Call pg_miss() to schedule the page fill.  A consequences of this
       * call are:
       *
       * (1) The currently executing task will be blocked and saved on
       *     on the g_waitingforfill task list.
       * (2) An interrupt-level context switch will occur so that when
       *     this function returns, it will return to a different task,
       *     most likely the page fill worker thread.
       * (3) The page fill worker task has been signalled and should
       *     execute immediately when we return from this exception.
       */

      pg_miss();

      /* Restore the previous value of current_regs.  NULL would indicate that
       * we are no longer in an interrupt handler.  It will be non-NULL if we
       * are returning from a nested interrupt.
       */

      current_regs = savestate;
    }
  else
#endif
    {
      lldbg("Prefetch abort. PC: %08x\n", regs[REG_PC]);
      PANIC(OSERR_ERREXCEPTION);
    }
}
