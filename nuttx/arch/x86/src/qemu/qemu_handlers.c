/****************************************************************************
 *  arch/x86/src/qemu/qemu_handlers.c
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
#include <nuttx/compiler.h>

#include <nuttx/arch.h>
#include <arch/io.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void idt_outb(uint8_t val, uint16_t addr) noinline_function;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name idt_outb
 *
 * Description:
 *   A slightly slower version of outb
 *
 ****************************************************************************/

static void idt_outb(uint8_t val, uint16_t addr)
{
  outb(val, addr);
}

/****************************************************************************
 * Name: common_handler
 *
 * Description:
 *   Common logic for the ISR/IRQ handlers
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_INTERRUPTS
static uint32_t *common_handler(int irq, uint32_t *regs)
{
  uint32_t *savestate;

  up_ledon(LED_INIRQ);

  /* Nested interrupts are not supported in this implementation.  If you want
   * implemented nested interrupts, you would have to (1) change the way that
   * current regs is handled and (2) the design associated with
   * CONFIG_ARCH_INTERRUPTSTACK.
   */

  /* Current regs non-zero indicates that we are processing an interrupt;
   * current_regs is also used to manage interrupt level context switches.
   */

  savestate    = (uint32_t*)current_regs;
  current_regs = regs;

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

  /* If a context switch occurred while processing the interrupt then
   * current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint32_t*)current_regs;

  /* Restore the previous value of current_regs.  NULL would indicate that
   * we are no longer in an interrupt handler.  It will be non-NULL if we
   * are returning from a nested interrupt.
   */

  current_regs = savestate;
  return regs;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isr_handler
 *
 * Description:
 *   This gets called from ISR vector handling logic in qemu_vectors.S
 *
 ****************************************************************************/

uint32_t *isr_handler(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  up_ledon(LED_INIRQ);
  PANIC(OSERR_ERREXCEPTION); /* Doesn't return */
  return regs;               /* To keep the compiler happy */
#else
  uint32_t *ret;

  /* Dispatch the interrupt */

  up_ledon(LED_INIRQ);
  ret = common_handler((int)regs[REG_IRQNO], regs);
  up_ledoff(LED_INIRQ);
  return ret;
#endif
}

/****************************************************************************
 * Name: isr_handler
 *
 * Description:
 *   This gets called from IRQ vector handling logic in qemu_vectors.S
 *
 ****************************************************************************/

uint32_t *irq_handler(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  up_ledon(LED_INIRQ);
  PANIC(OSERR_ERREXCEPTION); /* Doesn't return */
  return regs;               /* To keep the compiler happy */
#else
  uint32_t *ret;
  int irq;

  up_ledon(LED_INIRQ);

  /* Get the IRQ number */

  irq = (int)regs[REG_IRQNO];

  /* Send an EOI (end of interrupt) signal to the PICs if this interrupt
   * involved the slave.
   */

  if (irq >= IRQ8)
    {
      /* Send reset signal to slave */

      idt_outb(PIC_OCW2_EOI_NONSPEC, PIC2_OCW2);
    }

  /* Send reset signal to master */

  idt_outb(PIC_OCW2_EOI_NONSPEC, PIC1_OCW2);

  /* Dispatch the interrupt */

  ret = common_handler(irq, regs);
  up_ledoff(LED_INIRQ);
  return ret;
#endif
}

