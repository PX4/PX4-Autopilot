/****************************************************************************
 * arch/arm/src/armv7-m/up_assert.c
 *
 *   Copyright (C) 2009-2010, 2012-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Output debug info if stack dump is selected -- even if 
 * debug is not selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  lldbg
# define lldbg lowsyslog
#endif

/* The following is just intended to keep some ugliness out of the mainline
 * code.  We are going to print the task name if:
 *
 *  CONFIG_TASK_NAME_SIZE > 0 &&    <-- The task has a name
 *  (defined(CONFIG_DEBUG)    ||    <-- And the debug is enabled (lldbg used)
 *   defined(CONFIG_ARCH_STACKDUMP) <-- Or lowsyslog() is used
 */

#undef CONFIG_PRINT_TASKNAME
#if CONFIG_TASK_NAME_SIZE > 0 && (defined(CONFIG_DEBUG) || defined(CONFIG_ARCH_STACKDUMP))
#  define CONFIG_PRINT_TASKNAME 1
#endif

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
  uint32_t sp;
  __asm__
  (
    "\tmov %0, sp\n\t"
    : "=r"(sp)
  );
  return sp;
}

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
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
#else
# define up_stackdump()
#endif

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static inline void up_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (current_regs)
    {
      /* Yes.. dump the interrupt registers */

      lldbg("R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
            current_regs[REG_R0],  current_regs[REG_R1],
            current_regs[REG_R2],  current_regs[REG_R3],
            current_regs[REG_R4],  current_regs[REG_R5],
            current_regs[REG_R6],  current_regs[REG_R7]);
      lldbg("R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
            current_regs[REG_R8],  current_regs[REG_R9],
            current_regs[REG_R10], current_regs[REG_R11],
            current_regs[REG_R12], current_regs[REG_R13],
            current_regs[REG_R14], current_regs[REG_R15]);
#ifdef CONFIG_ARMV7M_USEBASEPRI
      lldbg("xPSR: %08x BASEPRI: %08x\n",
            current_regs[REG_XPSR],  current_regs[REG_BASEPRI]);
#else
      lldbg("xPSR: %08x PRIMASK: %08x\n",
            current_regs[REG_XPSR],  current_regs[REG_PRIMASK]);
#endif
    }
}
#else
# define up_registerdump()
#endif

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
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
  istackbase = (uint32_t)&g_intstackbase;
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
    }

  /* Extract the user stack pointer if we are in an interrupt handler.
   * If we are not in an interrupt handler.  Then sp is the user stack
   * pointer (and the above range check should have failed).
   */

  if (current_regs)
    {
      sp = current_regs[REG_R13];
      lldbg("sp:     %08x\n", sp);
    }

  lldbg("User stack:\n");
  lldbg("  base: %08x\n", ustackbase);
  lldbg("  size: %08x\n", ustacksize);

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp <= ustackbase && sp > ustackbase - ustacksize)
    {
      up_stackdump(sp, ustackbase);
    }
#else
  lldbg("sp:         %08x\n", sp);
  lldbg("stack base: %08x\n", ustackbase);
  lldbg("stack size: %08x\n", ustacksize);

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
#else
# define up_dumpstate()
#endif

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(int errorcode) /* noreturn_function */
{
  /* Are we in an interrupt handler or the idle task? */

  if (current_regs || ((_TCB*)g_readytorun.head)->pid == 0)
    {
       (void)irqsave();
        for(;;)
          {
#ifdef CONFIG_ARCH_LEDS
            up_ledon(LED_PANIC);
            up_mdelay(250);
            up_ledoff(LED_PANIC);
            up_mdelay(250);
#endif
          }
    }
  else
    {
      exit(errorcode);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const uint8_t *filename, int lineno)
{
#ifdef CONFIG_PRINT_TASKNAME
  _TCB *rtcb = (_TCB*)g_readytorun.head;
#endif

  up_ledon(LED_ASSERTION);
#ifdef CONFIG_PRINT_TASKNAME
  lldbg("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  lldbg("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif
  up_dumpstate();
  _up_assert(EXIT_FAILURE);
}

/****************************************************************************
 * Name: up_assert_code
 ****************************************************************************/

void up_assert_code(const uint8_t *filename, int lineno, int errorcode)
{
#ifdef CONFIG_PRINT_TASKNAME
  _TCB *rtcb = (_TCB*)g_readytorun.head;
#endif

  up_ledon(LED_ASSERTION);
#ifdef CONFIG_PRINT_TASKNAME
  lldbg("Assertion failed at file:%s line: %d task: %s error code: %d\n",
        filename, lineno, rtcb->name, errorcode);
#else
  lldbg("Assertion failed at file:%s line: %d error code: %d\n",
        filename, lineno, errorcode);
#endif
  up_dumpstate();
  _up_assert(errorcode);
}
