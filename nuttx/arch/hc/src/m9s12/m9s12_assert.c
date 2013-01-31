/****************************************************************************
 * arch/hc/src/m9s12/m9s12_assert.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 * Name: up_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_stackdump(uint16_t sp, uint16_t stack_base)
{
  uint16_t stack;

  for (stack = sp; stack < stack_base; stack += 16)
    {
      uint8_t *ptr = (uint8_t*)stack;
      lldbg("%04x: %02x %02x %02x %02x %02x %02x %02x %02x"
            "   %02x %02x %02x %02x %02x %02x %02x %02x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7],
             ptr[8], ptr[9], ptr[10], ptr[11], ptr[12], ptr[13], ptr[14], ptr[15]);
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
      lldbg("A:%02x B:%02x X:%02x%02x Y:%02x%02x PC:%02x%02x CCR:%02x\n",
             current_regs[REG_A], current_regs[REG_B], current_regs[REG_XH],
             current_regs[REG_XL], current_regs[REG_YH], current_regs[REG_YL],
             current_regs[REG_PCH], current_regs[REG_PCL], current_regs[REG_CCR]);
      lldbg("SP:%02x%02x FRAME:%02x%02x TMP:%02x%02x Z:%02x%02x XY:%02x\n",
             current_regs[REG_SPH], current_regs[REG_SPL],
             current_regs[REG_FRAMEH], current_regs[REG_FRAMEL],
             current_regs[REG_TMPL], current_regs[REG_TMPH], current_regs[REG_ZL],
             current_regs[REG_ZH], current_regs[REG_XY], current_regs[REG_XY+1]);

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#elif CONFIG_HCS12_MSOFTREGS == 2
      lldbg("SOFTREGS: %02x%02x :%02x%02x\n",
            current_regs[REG_SOFTREG1], current_regs[REG_SOFTREG1+1],
            current_regs[REG_SOFTREG2], current_regs[REG_SOFTREG2+1]);
#elif CONFIG_HCS12_MSOFTREGS == 1
      lldbg("SOFTREGS: %02x%02x\n", current_regs[REG_SOFTREG1],
            current_regs[REG_SOFTREG1+1]);
#endif

#ifndef CONFIG_HCS12_NONBANKED
      lldbg("PPAGE: %02x\n", current_regs[REG_PPAGE],);
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
  uint16_t sp   = up_getsp();
  uint16_t ustackbase;
  uint16_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint16_t istackbase;
  uint16_t istacksize;
#endif

  /* Get the limits on the user stack memory */

  if (rtcb->pid == 0)
    {
      ustackbase = g_heapbase - 4;
      ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      ustackbase = (uint16_t)rtcb->adj_stack_ptr;
      ustacksize = (uint16_t)rtcb->adj_stack_size;
    }

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint16_t)&g_userstack;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4;

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

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_userstack;
      lldbg("sp:     %04x\n", sp);
    }

  /* Show user stack info */

  lldbg("User stack:\n");
  lldbg("  base: %04x\n", ustackbase);
  lldbg("  size: %04x\n", ustacksize);
#else
  lldbg("sp:         %04x\n", sp);
  lldbg("stack base: %04x\n", ustackbase);
  lldbg("stack size: %04x\n", ustacksize);
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
#else
# define up_dumpstate()
#endif

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(int errorcode) noreturn_function;
static void _up_assert(int errorcode)
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
