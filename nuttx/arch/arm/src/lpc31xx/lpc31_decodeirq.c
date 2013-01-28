/********************************************************************************
 * arch/arm/src/lpc31xx/lpc31_decodeirq.c
 * arch/arm/src/chip/lpc31_decodeirq.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "up_arch.h"

#include "os_internal.h"
#include "up_internal.h"

#include "lpc31_intc.h"

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

void up_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  lowsyslog("Unexpected IRQ\n");
  current_regs = regs;
  PANIC(OSERR_ERREXCEPTION);
#else
  int index;
  int irq;

  /* Read the IRQ vector status register.  Bits 3-10 provide the IRQ number
   * of the interrupt (the TABLE_ADDR was initialized to zero, so the
   * following masking should be unnecessary)
   */

  index = getreg32(LPC31_INTC_VECTOR0) & INTC_VECTOR_INDEX_MASK;
  if (index != 0)
    {
      /* Shift the index so that the range of IRQ numbers are in bits 0-7 (values
       * 1-127) and back off the IRQ number by 1 so that the numbering is zero-based
       */

      irq = (index >> INTC_VECTOR_INDEX_SHIFT) -1;

      /* Verify that the resulting IRQ number is valid */

      if ((unsigned)irq < NR_IRQS)
        {
          uint32_t* savestate;

          /* Mask and acknowledge the interrupt */

          up_maskack_irq(irq);

          /* Current regs non-zero indicates that we are processing an interrupt;
           * current_regs is also used to manage interrupt level context switches.
           */

          savestate    = (uint32_t*)current_regs;
          current_regs = regs;

          /* Deliver the IRQ */

          irq_dispatch(irq, regs);

          /* Restore the previous value of current_regs.  NULL would indicate that
           * we are no longer in an interrupt handler.  It will be non-NULL if we
           * are returning from a nested interrupt.
           */

          current_regs = savestate;

          /* Unmask the last interrupt (global interrupts are still
           * disabled).
           */

          up_enable_irq(irq);
        }
    }
#endif
}
