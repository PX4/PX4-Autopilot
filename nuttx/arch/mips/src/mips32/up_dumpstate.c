/****************************************************************************
 * arch/mips/src/mips32/up_dumpstate.c
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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Output debug info if stack dump is selected -- even if debug is not
 * selected.
 */

#undef  lldbg
#define lldbg lowsyslog

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getsp
 ****************************************************************************/

/* I don't know if the builtin to get SP is enabled */

static inline uint32_t up_getsp(void)
{
  register uint32_t sp;
  __asm__
  (
  	"\tadd  %0, $0, $29\n"
    : "=r"(sp)
  );
  return sp;
}

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint32_t sp, uint32_t stack_base)
{
  uint32_t stack ;

  for (stack = sp & ~0x1f; stack < stack_base; stack += 32)
    {
      uint32_t *ptr = (uint32_t*)stack;
      lldbg("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

static inline void up_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (current_regs)
    {
      lldbg("MFLO:%08x MFHI:%08x EPC:%08x STATUS:%08x\n",
            current_regs[REG_MFLO], current_regs[REG_MFHI], current_regs[REG_EPC],
            current_regs[REG_STATUS]);
      lldbg("AT:%08x V0:%08x V1:%08x A0:%08x A1:%08x A2:%08x A3:%08x\n",
            current_regs[REG_AT], current_regs[REG_V0], current_regs[REG_V1],
            current_regs[REG_A0], current_regs[REG_A1], current_regs[REG_A2],
            current_regs[REG_A3]);
      lldbg("T0:%08x T1:%08x T2:%08x T3:%08x T4:%08x T5:%08x T6:%08x T7:%08x\n",
            current_regs[REG_T0], current_regs[REG_T1], current_regs[REG_T2],
            current_regs[REG_T3], current_regs[REG_T4], current_regs[REG_T5],
            current_regs[REG_T6], current_regs[REG_T7]);
      lldbg("S0:%08x S1:%08x S2:%08x S3:%08x S4:%08x S5:%08x S6:%08x S7:%08x\n",
            current_regs[REG_S0], current_regs[REG_S1], current_regs[REG_S2],
            current_regs[REG_S3], current_regs[REG_S4], current_regs[REG_S5],
            current_regs[REG_S6], current_regs[REG_S7]);
#ifdef MIPS32_SAVE_GP
      lldbg("T8:%08x T9:%08x GP:%08x SP:%08x FP:%08x RA:%08x\n",
            current_regs[REG_T8], current_regs[REG_T9], current_regs[REG_GP],
            current_regs[REG_SP], current_regs[REG_FP], current_regs[REG_RA]);
#else
      lldbg("T8:%08x T9:%08x SP:%08x FP:%08x RA:%08x\n",
            current_regs[REG_T8], current_regs[REG_T9], current_regs[REG_SP],
            current_regs[REG_FP], current_regs[REG_RA]);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  _TCB    *rtcb = (_TCB*)g_readytorun.head;
  uint32_t sp   = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Get the limits on the user stack memory */

  if (rtcb->pid == 0)
    {
      ustackbase = g_heapbase - 4;
      ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      ustackbase = (uint32_t)rtcb->adj_stack_ptr;
      ustacksize = (uint32_t)rtcb->adj_stack_size;
    }

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)&g_userstack;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4;

  /* Show interrupt stack info */

  lldbg("sp:     %08x\n", sp);
  lldbg("IRQ stack:\n");
  lldbg("  base: %08x\n", istackbase);
  lldbg("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_userstack;
      lldbg("sp:     %08x\n", sp);
    }

  /* Show user stack info */

  lldbg("User stack:\n");
  lldbg("  base: %08x\n", ustackbase);
  lldbg("  size: %08x\n", ustacksize);
#else
  lldbg("sp:         %08x\n", sp);
  lldbg("stack base: %08x\n", ustackbase);
  lldbg("stack size: %08x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
#if !defined(CONFIG_ARCH_INTERRUPTSTACK) || CONFIG_ARCH_INTERRUPTSTACK < 4
      lldbg("ERROR: Stack pointer is not within allocated stack\n");
#endif
    }
  else
    {
      up_stackdump(sp, ustackbase);
    }

  /* Then dump the registers (if available) */

  up_registerdump();
}

#endif /* CONFIG_ARCH_STACKDUMP */
