/****************************************************************************
 * arch/sh/src/sh1/sh1_assert.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Output debug info if stack dump is selected -- even if 
 * debug is not selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  lldbg
# define lldbg lowsyslog
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sh1_getsp
 ****************************************************************************/

static inline uint32_t sh1_getsp(void)
{
  uint32_t sp;

  __asm__ __volatile__
    (
      "mov   r15, %0\n\t"
      : "=&z" (sp)
      :
      : "memory"
    );
  return sp;
}

/****************************************************************************
 * Name: sh1_stackdump
 ****************************************************************************/

static void sh1_stackdump(uint32_t sp, uint32_t stack_base)
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
 * Name: sh1_registerdump
 ****************************************************************************/

static inline void sh1_registerdump(void)
{
  uint32_t *ptr = (uint32_t*)current_regs;

  /* Are user registers available from interrupt processing? */

  if (ptr)
    {
      /* Yes.. dump the interrupt registers */

      lldbg("PC: %08x SR=%08x\n",
            ptr[REG_PC], ptr[REG_SR]);

      lldbg("PR: %08x GBR: %08x MACH: %08x MACL: %08x\n",
            ptr[REG_PR], ptr[REG_GBR], ptr[REG_MACH], ptr[REG_MACL]);

      lldbg("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n", 0,
            ptr[REG_R0], ptr[REG_R1], ptr[REG_R2], ptr[REG_R3],
            ptr[REG_R4], ptr[REG_R5], ptr[REG_R6], ptr[REG_R7]);

      lldbg("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n", 8,
            ptr[REG_R8], ptr[REG_R9], ptr[REG_R10], ptr[REG_R11],
            ptr[REG_R12], ptr[REG_R13], ptr[REG_R14], ptr[REG_R15]);
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
  _TCB    *rtcb     = (_TCB*)g_readytorun.head;
  uint32_t sp       = sh1_getsp();
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

      sh1_stackdump(sp, istackbase);

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
      sh1_stackdump(sp, ustackbase);
    }

  /* Then dump the registers (if available) */

  sh1_registerdump();
}

#endif /* CONFIG_ARCH_STACKDUMP */
