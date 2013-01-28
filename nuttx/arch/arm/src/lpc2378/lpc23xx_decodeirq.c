/********************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_decodeirq.c
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
 *
 *   Copyright (C) 2010, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "os_internal.h"
#include "internal.h"
#include "lpc23xx_vic.h"

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Types
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
 * Public Funstions
 ********************************************************************************/

/********************************************************************************
 * up_decodeirq() and/or lpc23xx_decodeirq()
 *
 * Description:
 *   The vectored interrupt controller (VIC) takes 32 interrupt request inputs
 *   and programmatically assigns them into 2 categories:  FIQ, vectored IRQ.
 *
 *   - FIQs have the highest priority.  There is a single FIQ vector, but multiple
 *     interrupt sources can be ORed to this FIQ vector.
 *
 *   - Vectored IRQs have the middle priority.  Any of the 32 interrupt sources
 *     can be assigned to vectored IRQs.
 *
 *   - Non-vectored IRQs have the lowest priority.
 *
 *   The general flow of IRQ processing is to simply read the VICAddress
 *   and jump to the address of the vector provided in the register.  The VIC will
 *   provide the address of the highest priority vectored IRQ.  If a non-vectored
 *   IRQ is requesting, the address of a default handler is provided.
 *
 ********************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
void up_decodeirq(uint32_t *regs)
#else
static void lpc23xx_decodeirq(uint32_t *regs)
#endif
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  lowsyslog("Unexpected IRQ\n");
  current_regs = regs;
  PANIC(OSERR_ERREXCEPTION);
#else

  /* Check which IRQ fires */

  uint32_t irqbits = vic_getreg(VIC_IRQSTATUS_OFFSET) & 0xFFFFFFFF;
  unsigned int irq;

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      if (irqbits & (uint32_t) (1 << irq))
        break;
    }

  /* Verify that the resulting IRQ number is valid */

  if (irq < NR_IRQS)            /* redundant check ?? */
    {
       uint32_t *savestate;

      /* Current regs non-zero indicates that we are processing an interrupt;
       * current_regs is also used to manage interrupt level context switches.
       */

      savestate    = (uint32_t*)current_regs;
      current_regs = regs;

      /* Mask and acknowledge the interrupt */

      up_maskack_irq(irq);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Restore the previous value of current_regs.  NULL would indicate that
       * we are no longer in an interrupt handler.  It will be non-NULL if we
       * are returning from a nested interrupt.
       */

      current_regs = savestate;
    }

#endif
}

#ifdef CONFIG_VECTORED_INTERRUPTS
void up_decodeirq(uint32_t *regs)
{
  vic_vector_t vector = (vic_vector_t) vic_getreg(VIC_ADDRESS_OFFSET);

  /* Mask and acknowledge the interrupt */

  up_maskack_irq(irq);

  /* Valid Interrupt */

  if (vector != NULL)
    (vector) (regs);
}
#endif
