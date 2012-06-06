/****************************************************************************
 * arch/mips/src/mips32/up_initialstate.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/mips32/cp0.h>

#include "up_internal.h"
#include "up_arch.h"

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
  uint32_t regval;

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Save the initial stack pointer.  Hmmm.. the stack is set to the very
   * beginning of the stack region.  Some functions may want to store data on
   * the caller's stack and it might be good to reserve some space.  However,
   * only the start function would do that and we have control over that one
   */

  xcp->regs[REG_SP]      = (uint32_t)tcb->adj_stack_ptr;

  /* Save the task entry point */

  xcp->regs[REG_EPC]     = (uint32_t)tcb->start;
  
  /* If this task is running PIC, then set the PIC base register to the
   * address of the allocated D-Space region.
   */

#ifdef CONFIG_PIC
#  warning "Missing logic"
#endif

  /* Set privileged- or unprivileged-mode, depending on how NuttX is
   * configured and what kind of thread is being started.
   *
   * If the kernel build is not selected, then all threads run in
   * privileged thread mode.
   */

#ifdef CONFIG_NUTTX_KERNEL
#  warning "Missing logic"
#endif

  /* Set the initial value of the status register.  It will be the same
   * as the current status register with some changes:
   *
   * 1. Make sure the IE is set
   * 2. Clear the BEV bit (This bit should already be clear)
   * 3. Clear the UM bit so that the new task executes in kernel mode
   *   (This bit should already be clear)
   * 4. Set the interrupt mask bits (depending on configuration)
   * 5. Set the EXL bit (This will not be set)
   *
   * The EXL bit is set because this new STATUS register will be
   * instantiated in kernel mode inside of an interrupt handler. EXL
   * will be automatically cleared by the eret instruction.
   */

  regval  = cp0_getstatus();
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  regval &= ~(CP0_STATUS_IM_ALL | CP0_STATUS_BEV | CP0_STATUS_UM);
  regval |=  (CP0_STATUS_IE | CP0_STATUS_EXL | CP0_STATUS_IM_SWINTS);
#else
  regval &= ~(CP0_STATUS_BEV | CP0_STATUS_UM);
  regval |=  (CP0_STATUS_IE | CP0_STATUS_EXL | CP0_STATUS_IM_ALL);
#endif
  xcp->regs[REG_STATUS] = regval;
}

