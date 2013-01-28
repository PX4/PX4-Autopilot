/****************************************************************************
 * arch/avr/src/avr/up_dumpstate.c
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

/* There may be a built-in to do this, but I don't know if it is enabled */

static inline uint16_t up_getsp(void)
{
  uint8_t spl;
  uint8_t sph;

  __asm__ __volatile__
  (
    "in %0, __SP_L__\n\t"
    "in %1, __SP_H__\n"
    : "=r" (spl), "=r" (sph)
    :
  );
  return (uint16_t)sph << 8 | spl;
}

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint16_t sp, uint16_t stack_base)
{
  uint16_t stack ;

  for (stack = sp & ~3; stack < stack_base; stack += 12)
    {
      uint8_t *ptr = (uint8_t*)stack;
      lldbg("%04x: %02x %02x %02x %02x %02x %02x %02x %02x"
            " %02x %02x %02x %02x\n",
             stack,
             ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7],
             ptr[8], ptr[9], ptr[10], ptr[11]);
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
      lldbg("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            0,
            current_regs[REG_R0],  current_regs[REG_R1],
            current_regs[REG_R2],  current_regs[REG_R3],
            current_regs[REG_R4],  current_regs[REG_R5],
            current_regs[REG_R6],  current_regs[REG_R7]);

      lldbg("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            8,
            current_regs[REG_R8],  current_regs[REG_R9],
            current_regs[REG_R10], current_regs[REG_R11],
            current_regs[REG_R12], current_regs[REG_R13],
            current_regs[REG_R14], current_regs[REG_R15]);

      lldbg("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            16,
            current_regs[REG_R16], current_regs[REG_R17],
            current_regs[REG_R18], current_regs[REG_R19],
            current_regs[REG_R20], current_regs[REG_R21],
            current_regs[REG_R22], current_regs[REG_R23]);

      lldbg("R%02d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            24,
            current_regs[REG_R24], current_regs[REG_R25],
            current_regs[REG_R26], current_regs[REG_R27],
            current_regs[REG_R28], current_regs[REG_R29],
            current_regs[REG_R30], current_regs[REG_R31]);

      lldbg("PC:  %02x%02x  SP: %02x%02x SREG: %02x\n",
            current_regs[REG_PCH], current_regs[REG_PCL],
            current_regs[REG_SPH], current_regs[REG_SPL],
            current_regs[REG_SREG]);
    }
}

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  _TCB    *rtcb = (_TCB*)g_readytorun.head;
  uint16_t sp   = up_getsp();
  uint16_t ustackbase;
  uint16_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 0
  uint16_t istackbase;
  uint16_t istacksize;
#endif

  /* Get the limits on the user stack memory */

  if (rtcb->pid == 0)
    {
      ustackbase = g_heapbase - 1;
      ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      ustackbase = (uint16_t)rtcb->adj_stack_ptr;
      ustacksize = (uint16_t)rtcb->adj_stack_size;
    }

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 0
  istackbase = (uint16_t)&g_intstackbase - 1;
  istacksize = CONFIG_ARCH_INTERRUPTSTACK;

  /* Show interrupt stack info */

  lldbg("sp:     %04x\n", sp);
  lldbg("IRQ stack:\n");
  lldbg("  base: %04x\n", istackbase);
  lldbg("  size: %04x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase);
    }

  /* Extract the user stack pointer if we are in an interrupt handler.
   * If we are not in an interrupt handler.  Then sp is the user stack
   * pointer (and the above range check should have failed).
   */

  if (current_regs)
    {
      sp = current_regs[REG_R13];
      lldbg("sp:     %04x\n", sp);
    }

  lldbg("User stack:\n");
  lldbg("  base: %04x\n", ustackbase);
  lldbg("  size: %04x\n", ustacksize);

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp <= ustackbase && sp > ustackbase - ustacksize)
    {
      up_stackdump(sp, ustackbase);
    }
#else
  lldbg("sp:         %04x\n", sp);
  lldbg("stack base: %04x\n", ustackbase);
  lldbg("stack size: %04x\n", ustacksize);

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
      lldbg("ERROR: Stack pointer is not within allocated stack\n");
    }
  else
    {
      up_stackdump(sp, ustackbase);
    }
#endif

  /* Then dump the registers (if available) */

  up_registerdump();
}
#endif
