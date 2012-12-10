/****************************************************************************
 * arch/z80/src/z180/z180_schedulesigaction.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "chip/switch.h"
#include "os_internal.h"
#include "up_internal.h"

#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_sigsetup
 ****************************************************************************/

static void z180_sigsetup(FAR _TCB *tcb, sig_deliver_t sigdeliver, FAR chipreg_t *regs)
{
  /* Save the return address and interrupt state. These will be restored by
   * the signal trampoline after the signals have been delivered.
   */

  tcb->xcp.sigdeliver    = sigdeliver;
  tcb->xcp.saved_pc      = regs[XCPT_PC];
  tcb->xcp.saved_i       = regs[XCPT_I];

  /* Then set up to vector to the trampoline with interrupts disabled */

  regs[XCPT_PC]  = (chipreg_t)up_sigdeliver;
  regs[XCPT_I]   = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'igdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 ****************************************************************************/

void up_schedule_sigaction(FAR _TCB *tcb, sig_deliver_t sigdeliver)
{
  dbg("tcb=0x%p sigdeliver=0x%04x\n", tcb, (uint16_t)sigdeliver);

  /* Refuse to handle nested signal actions */

  if (tcb->xcp.sigdeliver == NULL)
    {
      irqstate_t flags;

      /* Make sure that interrupts are disabled */

      flags = irqsave();

      /* First, handle some special cases when the signal is being delivered
       * to the currently executing task.
       */

      if (tcb == (FAR _TCB*)g_readytorun.head)
        {
          /* CASE 1:  We are not in an interrupt handler and a task is
           * signalling itself for some reason.
           */

          if (!IN_INTERRUPT())
            {
              /* In this case just deliver the signal now. */

              sigdeliver(tcb);
            }

          /* CASE 2:  We are in an interrupt handler AND the interrupted task
           * is the same as the one that must receive the signal, then we
           * will have to modify the return state as well as the state in
           * the TCB.
           */

          else
            {
              /* Set up to vector to the trampoline with interrupts disabled. */

              z180_sigsetup(tcb, sigdeliver, IRQ_STATE());

              /* And make sure that the saved context in the TCB
               * is the same as the interrupt return context.
               */

              SAVE_IRQCONTEXT(tcb);
            }
        }

      /* Otherwise, we are (1) signaling a task is not running from an interrupt
       * handler or (2) we are not in an interrupt handler and the running task
       * is signalling some non-running task.
       */

      else
        {
          /* Set up to vector to the trampoline with interrupts disabled. */

          z180_sigsetup(tcb, sigdeliver, tcb->xcp.regs);
        }

      irqrestore(flags);
    }
}

#endif /* CONFIG_DISABLE_SIGNALS */

