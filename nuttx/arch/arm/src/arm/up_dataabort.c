/****************************************************************************
 * arch/arm/src/arm/up_dataabort.c
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

#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "arm.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: up_dataabort
 *
 * Input parameters:
 *   regs - The standard, ARM register save array.
 *
 * If CONFIG_PAGING is selected in the NuttX configuration file, then these
 * additional input values are expected:
 *
 *   far - Fault address register.  On a data abort, the ARM MMU places the
 *     miss virtual address (MVA) into the FAR register.  This is the address
 *     of the data which, when accessed, caused the fault.
 *   fsr - Fault status register.  On a data a abort, the ARM MMU places an
 *     encoded four-bit value, the fault status, along with the four-bit
 *     encoded domain number, in the data FSR
 *
 * Description:
 *   This is the data abort exception handler. The ARM data abort exception
 *   occurs when a memory fault is detected during a data transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
void up_dataabort(uint32_t *regs, uint32_t far, uint32_t fsr)
{
  FAR _TCB *tcb = (FAR _TCB *)g_readytorun.head;
#ifdef CONFIG_PAGING
   uint32_t *savestate;

  /* Save the saved processor context in current_regs where it can be accessed
   * for register dumps and possibly context switching.
   */


  savestate    = (uint32_t*)current_regs;
#endif
  current_regs = regs;

#ifdef CONFIG_PAGING
  /* In the NuttX on-demand paging implementation, only the read-only, .text
   * section is paged.  However, the ARM compiler generated PC-relative data
   * fetches from within the .text sections.  Also, it is customary to locate
   * read-only data (.rodata) within the same section as .text so that it
   * does not require copying to RAM. Misses in either of these case should
   * cause a data abort.
   *
   * We are only interested in data aborts due to page translations faults.
   * Sections should already be in place and permissions should already be
   * be set correctly (to read-only) so any other data abort reason is a
   * fatal error.
   */

  pglldbg("FSR: %08x FAR: %08x\n", fsr, far);
  if ((fsr & FSR_MASK) != FSR_PAGE)
    {
      goto segfault;
    }

  /* Check the (virtual) address of data that caused the data abort. When
   * the exception occurred, this address was provided in the FAR register.
   * (It has not yet been saved in the register context save area).
   */
 
  pgllvdbg("VBASE: %08x VEND: %08x\n", PG_PAGED_VBASE, PG_PAGED_VEND);
  if (far < PG_PAGED_VBASE || far >= PG_PAGED_VEND)
    {
      goto segfault;
    }

  /* Save the offending data address as the fault address in the TCB of
   * the currently task.  This fault address is also used by the prefetch
   * abort handling; this will allow common paging logic for both
   * prefetch and data aborts.
   */

  tcb->xcp.far = regs[REG_R15];

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
  return;

segfault:
#endif
  lldbg("Data abort. PC: %08x FAR: %08x FSR: %08x\n", regs[REG_PC], far, fsr);
  PANIC(OSERR_ERREXCEPTION);
}

#else /* CONFIG_PAGING */

void up_dataabort(uint32_t *regs)
{
  /* Save the saved processor context in current_regs where it can be accessed
   * for register dumps and possibly context switching.
   */

  current_regs = regs;

  /* Crash -- possibly showing diagnost debug information. */

  lldbg("Data abort. PC: %08x\n", regs[REG_PC]);
  PANIC(OSERR_ERREXCEPTION);
}

#endif /* CONFIG_PAGING */
