/****************************************************************************
 * arch/x86/src/i486/up_savestate.c
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

#include <debug.h>

#include <arch/arch.h>
#include <arch/irq.h>

#include "up_internal.h"

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
 * Name: up_savestate
 *
 * Description:
 *   This function saves the interrupt level context information in the
 *   TCB.  This would just be a up_copystate but we have to handle one
 *   special case.  In the case where the privilige level changes, the
 *   value of sp and ss will not be saved on stack by the interrupt handler.
 *   So, in that case, we will have to fudge those values here.
 *
 ****************************************************************************/

void up_savestate(uint32_t *regs)
{
  uint8_t cpl;
  uint8_t rpl;
  
  /* First, just copy all of the registers */

  up_copystate(regs, (uint32_t*)current_regs);

  /* The RES_SP and REG_SS values will not be saved by the interrupt handling
   * logic if there is no change in privilege level.  In that case, we will
   * have to "fudge" those values here.  For now, just overwrite the REG_SP
   * and REG_SS values with what we believe to be correct.  Obviously, this
   * will have to change in the future to support multi-segment operation.
   *
   * Check for a change in privilege level.
   */

  rpl = regs[REG_CS] & 3;
  cpl = up_getcs() & 3;
  DEBUGASSERT(rpl >= cpl);

  if (rpl == cpl)
    {
      /* No priority change, SP and SS are not present in the stack frame.
       *
       * The value saved in the REG_ESP will be the stackpointer value prior to
       * the execution of the PUSHA.  It will point at REG_IRQNO.
       */

      regs[REG_SP] = current_regs[REG_ESP] + 4*BOTTOM_NOPRIO;
      regs[REG_SS] = up_getss();
    }
  else
    {
	  DEBUGASSERT(regs[REG_SP] == current_regs[REG_ESP] + 4*BOTTOM_PRIO);
	}
}
