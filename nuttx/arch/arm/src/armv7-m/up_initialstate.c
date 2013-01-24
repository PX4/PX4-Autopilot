/****************************************************************************
 * arch/arm/src/armv7-m/up_initialstate.c
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#include "up_internal.h"
#include "up_arch.h"

#include "psr.h"
#include "exc_return.h"

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

  xcp->regs[REG_SP]      = (uint32_t)tcb->adj_stack_ptr;

  /* Set the stack limit value */

  xcp->regs[REG_R10]     = (uint32_t)tcb->stack_alloc_ptr + 64;

  /* Fill the stack with a watermark value */

  memset(tcb->stack_alloc_ptr, 0xff, tcb->adj_stack_size);

  /* Save the task entry point (stripping off the thumb bit) */

  xcp->regs[REG_PC]      = (uint32_t)tcb->start & ~1;
  
  /* Specify thumb mode */

  xcp->regs[REG_XPSR]    = ARMV7M_XPSR_T;

  /* If this task is running PIC, then set the PIC base register to the
   * address of the allocated D-Space region.
   */

#ifdef CONFIG_PIC
  if (tcb->dspace != NULL)
    {
      /* Set the PIC base register (probably R10) to the address of the
       * alloacated D-Space region.
       */

      xcp->regs[REG_PIC] = (uint32_t)tcb->dspace->region;
    }

  /* Make certain that bit 0 is set in the main entry address.  This
   * is only an issue when NXFLAT is enabled.  NXFLAT doesn't know
   * anything about thumb; the addresses that NXFLAT sets are based
   * on file header info and won't have bit 0 set.
   */

#ifdef CONFIG_NXFLAT
  tcb->entry.main = (main_t)((uint32_t)tcb->entry.main | 1);
#endif
#endif /* CONFIG_PIC */

#ifdef CONFIG_ARMV7M_CMNVECTOR
  /* Set privileged- or unprivileged-mode, depending on how NuttX is
   * configured and what kind of thread is being started.
   *
   * If the kernel build is not selected, then all threads run in
   * privileged thread mode.
   *
   * If FPU support is not configured, set the bit that indicates that
   * the context does not include the volatile FP registers.
   */

  xcp->regs[REG_EXC_RETURN] = EXC_RETURN_BASE | EXC_RETURN_THREAD_MODE;

#ifndef CONFIG_ARCH_FPU

  xcp->regs[REG_EXC_RETURN] |= EXC_RETURN_STD_CONTEXT;

#else

  xcp->regs[REG_FPSCR] = 0; // XXX initial FPSCR should be configurable
  xcp->regs[REG_FPReserved] = 0;

#endif /* CONFIG_ARCH_FPU */

#ifdef CONFIG_NUTTX_KERNEL
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      /* It is a normal task or a pthread.  Set user mode */

      xcp->regs[REG_EXC_RETURN] = EXC_RETURN_PROCESS_STACK;
    }
#endif /* CONFIG_NUTTX_KERNEL */

#else /* CONFIG_ARMV7M_CMNVECTOR */

  /* Set privileged- or unprivileged-mode, depending on how NuttX is
   * configured and what kind of thread is being started.
   *
   * If the kernel build is not selected, then all threads run in
   * privileged thread mode.
   */

#ifdef CONFIG_NUTTX_KERNEL
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      /* It is a kernel thread.. set privileged thread mode */

      xcp->regs[REG_EXC_RETURN] = EXC_RETURN_PRIVTHR;
    }
  else
    {
      /* It is a normal task or a pthread.  Set user mode */

      xcp->regs[REG_EXC_RETURN] = EXC_RETURN_UNPRIVTHR;
    }
#endif /* CONFIG_NUTTX_KERNEL */
#endif /* CONFIG_ARMV7M_CMNVECTOR */

  /* Enable or disable interrupts, based on user configuration */

#ifdef CONFIG_SUPPRESS_INTERRUPTS
#ifdef CONFIG_ARMV7M_USEBASEPRI
  xcp->regs[REG_BASEPRI] = NVIC_SYSH_DISABLE_PRIORITY;
#else
  xcp->regs[REG_PRIMASK] = 1;
#endif
#endif /* CONFIG_SUPPRESS_INTERRUPTS */
}
