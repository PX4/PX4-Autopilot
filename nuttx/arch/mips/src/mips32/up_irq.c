/****************************************************************************
 * arch/mips/src/mips32/up_irq.c
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

#include <arch/irq.h>
#include <arch/types.h>
#include <arch/mips32/cp0.h>

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
 * Name: irqsave
 *
 * Description:
 *   Save the current interrupt state and disable interrupts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Interrupt state prior to disabling interrupts.
 *
 ****************************************************************************/

irqstate_t irqsave(void)
{
  register irqstate_t status;
  register irqstate_t ret;

  status  = cp0_getstatus();       /* Get CP0 status */
  ret     = status;                /* Save the status */
  status &= ~CP0_STATUS_IM_MASK;   /* Clear all interrupt mask bits */
  status |= CP0_STATUS_IM_SWINTS;  /* Keep S/W interrupts enabled */
  cp0_putstatus(status);           /* Disable interrupts */
  return ret;                      /* Return status before interrupts disabled */
}

/****************************************************************************
 * Name: irqrestore
 *
 * Description:
 *   Restore the previous interrutp state (i.e., the one previously returned
 *   by irqsave())
 *
 * Input Parameters:
 *   state - The interrupt state to be restored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void irqrestore(irqstate_t irqstate)
{
  register irqstate_t status;

  status    = cp0_getstatus();      /* Get CP0 status */
  status   &= ~CP0_STATUS_IM_MASK;  /* Clear all interrupt mask bits */
  irqstate &= CP0_STATUS_IM_MASK;   /* Retain interrupt mask bits only */
  status   |= irqstate;             /* Set new interrupt mask bits */
  status   |= CP0_STATUS_IM_SWINTS; /* Make sure that S/W interrupts enabled */
  cp0_putstatus(status);            /* Restore interrupt state */
}
