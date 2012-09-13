/****************************************************************************
 *  arch/hc/src/m9s12/m9s12_initialstate.c
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

#include <string.h>

#include <nuttx/sched.h>
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
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(_TCB *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Save the initial stack pointer */

  xcp->regs[REG_SPH]      = (uint16_t)tcb->adj_stack_ptr >> 8;
  xcp->regs[REG_SPL]      = (uint16_t)tcb->adj_stack_ptr & 0xff;

  /* Save the task entry point */

  xcp->regs[REG_PCH]      = (uint16_t)tcb->start >> 8;
  xcp->regs[REG_PCL]      = (uint16_t)tcb->start & 0xff;

#ifndef CONFIG_HCS12_NONBANKED
  /* Can only directly start in PPAGE 0x30 */

  xcp->regs[REG_PPAGE]    = 0x30;
#endif

  /* Condition code register:
   *
   *   Bit 0: C — Carry/Borrow status bit
   *   Bit 1: V — Two’s complement overflow status bit
   *   Bit 2: Z — Zero status bit
   *   Bit 3: N — Negative status bit
   *   Bit 4: I — Maskable interrupt control bit
   *   Bit 5: H — Half-carry status bit
   *   Bit 6: X — Non-maskable interrupt control bit
   *   Bit 7: S — STOP instruction control bit
   */

# ifdef CONFIG_SUPPRESS_INTERRUPTS
  /* Disable STOP, Mask I- and Z- interrupts */

  xcp->regs[REG_CCR]      = HCS12_CCR_S|HCS12_CCR_X|HCS12_CCR_I;  
# else
  /* Disable STOP, Enable I- and Z-interrupts */

  xcp->regs[REG_CCR]      = HCS12_CCR_S;
# endif
}

