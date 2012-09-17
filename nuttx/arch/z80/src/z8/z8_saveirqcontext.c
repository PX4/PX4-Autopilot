/****************************************************************************
 * arch/z80/src/z8/z8_saveirqcontext.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
#include <arch/irq.h>

#include "chip/switch.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
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
 * Name: z8_saveirqcontext
 *
 * Description:
 *   In order to provide faster interrupt handling, the interrupt logic does
 *   "lazy" context saving as described below:
 *
 *   (1) At the time of the interrupt, minimum information is saved and the
 *       register pointer is changed so that the interrupt logic does not
 *       alter the state of the interrupted task's registers.
 *   (2) If no context switch occurs during the interrupt processing, then
 *       the return from interrupt is also simple.
 *   (3) If a context switch occurs during interrupt processing, then
 *       (a) The full context of the interrupt task is saved, and
 *       (b) A full context switch is performed when the interrupt exits
 *           (see z8_vector.S).
 *
 * This function implements the full-context switch of bullet 3a.
 *
 ****************************************************************************/

void z8_saveirqcontext(FAR chipreg_t *regs)
{
  /* If we have already saved the interrupted task's registers in the TCB,
   * then we do not need to do anything.
   */

  if (g_z8irqstate.state == Z8_IRQSTATE_ENTRY)
    {
      /* Calculate the source address based on the saved RP value */

      uint16_t       rp  = g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS] >> 8;
      FAR chipreg_t *src = (FAR uint16_t*)(rp & 0xf0);
      FAR chipreg_t *dest = &regs[XCPT_RR0];

      /* Copy the interrupted tasks register into the TCB register save area. */

      int i;
      for (i = 0; i < XCPTCONTEXT_REGS; i++)
        {
          *dest++ = *src++;
        }

      /* Since the task was interrupted, we know that interrupts were enabled */

      regs[XCPT_IRQCTL] = 0x0080; /* IRQE bit will enable interrupts */

      /* The g_z8irqstate.regs pointer is the value of the stack pointer at
       * the time that up_doirq() was called.  Therefore, we can calculate
       * the correct value for the stack pointer on return from interrupt:
       */

      regs[XCPT_SP] = ((chipreg_t)g_z8irqstate.regs) + Z8_IRQSAVE_SIZE;

      /* Copy the PC, RP, and FLAGS information from the lazy save to the TCB
       * register save area.
       */

      regs[XCPT_RPFLAGS] = g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS];
      regs[XCPT_PC]      = g_z8irqstate.regs[Z8_IRQSAVE_PC];

      /* Now update the IRQ save area so that we will know that we have already
       * done this.
       */

      g_z8irqstate.state = Z8_IRQSTATE_SAVED;
      g_z8irqstate.regs  = regs;
    }
}

