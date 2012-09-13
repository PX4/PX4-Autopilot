/****************************************************************************
 * arch/x86/include/i486/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_X86_INCLUDE_I486_IRQ_H
#define __ARCH_X86_INCLUDE_I486_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <arch/arch.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* ISR and IRQ numbers */

#define ISR0     0 /* Division by zero exception */
#define ISR1     1 /* Debug exception */
#define ISR2     2 /* Non maskable interrupt */
#define ISR3     3 /* Breakpoint exception */
#define ISR4     4 /* 'Into detected overflow' */
#define ISR5     5 /* Out of bounds exception */
#define ISR6     6 /* Invalid opcode exception */
#define ISR7     7 /* No coprocessor exception */
#define ISR8     8 /* Double fault (pushes an error code) */
#define ISR9     9 /* Coprocessor segment overrun */
#define ISR10   10 /* Bad TSS (pushes an error code) */
#define ISR11   11 /* Segment not present (pushes an error code) */
#define ISR12   12 /* Stack fault (pushes an error code) */
#define ISR13   13 /* General protection fault (pushes an error code) */
#define ISR14   14 /* Page fault (pushes an error code) */
#define ISR15   15 /* Unknown interrupt exception */
#define ISR16   16 /* Coprocessor fault */
#define ISR17   17 /* Alignment check exception */
#define ISR18   18 /* Machine check exception */
#define ISR19   19 /* Reserved */
#define ISR20   20 /* Reserved */
#define ISR21   21 /* Reserved */
#define ISR22   22 /* Reserved */
#define ISR23   23 /* Reserved */
#define ISR24   24 /* Reserved */
#define ISR25   25 /* Reserved */
#define ISR26   26 /* Reserved */
#define ISR27   27 /* Reserved */
#define ISR28   28 /* Reserved */
#define ISR29   29 /* Reserved */
#define ISR30   30 /* Reserved */
#define ISR31   31 /* Reserved */

#define IRQ0    32 /* System timer (cannot be changed) */
#define IRQ1    33 /* Keyboard controller (cannot be changed) */
#define IRQ2    34 /* Cascaded signals from IRQs 8–15 */
#define IRQ3    35 /* Serial port controller for COM2/4 */
#define IRQ4    36 /* serial port controller for COM1/3 */
#define IRQ5    37 /* LPT port 2 or sound card */
#define IRQ6    38 /* Floppy disk controller */
#define IRQ7    39 /* LPT port 1 or sound card */
#define IRQ8    40 /* Real time clock (RTC) */
#define IRQ9    41 /* Open interrupt/available or SCSI host adapter */
#define IRQ10   42 /* Open interrupt/available or SCSI or NIC */
#define IRQ11   43 /* Open interrupt/available or SCSI or NIC */
#define IRQ12   44 /* Mouse on PS/2 connector */
#define IRQ13   45 /* Math coprocessor */
#define IRQ14   46 /* Primary ATA channel */
#define IRQ15   47 /* Secondary ATA channel */

#define NR_IRQS 48

/* Common register save structgure created by up_saveusercontext() and by
 * ISR/IRQ interrupt processing.
 */

#define REG_DS            (0)  /* Data segment selector */
#define REG_EDI           (1)  /* Saved by pusha */
#define REG_ESI           (2)  /* "   " "" "   " */
#define REG_EBP           (3)  /* "   " "" "   " */
#define REG_ESP           (4)  /* "   " "" "   " (NOTE 1)*/
#define REG_EBX           (5)  /* "   " "" "   " */
#define REG_EDX           (6)  /* "   " "" "   " */
#define REG_ECX           (7)  /* "   " "" "   " */
#define REG_EAX           (8)  /* "   " "" "   " */
#define REG_IRQNO         (9)  /* Interrupt number (NOTE 2) */
#define REG_ERRCODE      (10)  /* Error code (NOTE 2) */
#define REG_EIP          (11)  /* Pushed by process on interrupt processing */
#define REG_CS           (12)  /* "    " "" "     " "" "       " "        " */
#define REG_EFLAGS       (13)  /* "    " "" "     " "" "       " "        " */
#define REG_SP           (14)  /* "    " "" "     " "" "       " "        " */
#define REG_SS           (15)  /* "    " "" "     " "" "       " "        " */

/* NOTE 1: Two versions of the ESP are saved:  One from the interrupt
 *   processing and one from pusha.  Only the interrupt ESP (REG_SP) is used.
 * NOTE 2: This is not really state data.  Rather, this is just a convenient
 *   way to pass parameters from the interrupt handler to C cod.
 */

#define XCPTCONTEXT_REGS (16)
#define XCPTCONTEXT_SIZE (4 * XCPTCONTEXT_REGS)

/* Some special landmarks in the stack frame:
 *
 * TOP_PUSHA - The offset (in 32-bit words) from the beginning of the
 *   save area on the stack to the value that should be in REG_ESP.
 * BOTTOM_PUSHA - The offset (in 32-bit words) from the stack position before
 *   the interrupt occurred to the value that should be in REG_ESP.
 *   save area on the stack to the value that should be in REG_ESP.
 * OFFSET_PRIO - The offset from the value of REG_ESP to the value of the
 *   stack pointer before the interrupt occurred (assuming that a priority
 *   change occurred.
 * OFFSET_PRIO - The offset from the value of REG_ESP to the value of the
 *   stack pointer before the interrupt occurred (assuming that NO priority
 *   change occurred.
 */

#define TOP_PUSHA    	 REG_IRQNO
#define BOTTOM_PRIO      (XCPTCONTEXT_REGS-REG_IRQNO)
#define BOTTOM_NOPRIO    (REG_SP-REG_IRQNO)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of instruction pointer and EFLAGS used during
   * signal processing.
   */

  uint32_t saved_eip;
  uint32_t saved_eflags;
#endif

  /* Register save area */

   uint32_t regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Get the current FLAGS register contents */

static inline irqstate_t irqflags()
{
  irqstate_t flags;

  asm volatile(
    "\tpushf\n"
    "\tpop %0\n"
    : "=rm" (flags)
    :
    : "memory");
  return flags;
}

/* Get a sample of the FLAGS register, determine if interrupts are disabled.
 * If the X86_FLAGS_IF is cleared by cli, then interrupts are disabled.  If
 * if the X86_FLAGS_IF is set by sti, then interrupts are enable.
 */

static inline bool irqdisabled(irqstate_t flags)
{
  return ((flags & X86_FLAGS_IF) == 0);
}

static inline bool irqenabled(irqstate_t flags)
{
  return ((flags & X86_FLAGS_IF) != 0);
}

/* Disable interrupts unconditionally */

static inline void irqdisable(void)
{
  asm volatile("cli": : :"memory");
}

/* Enable interrupts unconditionally */

static inline void irqenable(void)
{
  asm volatile("sti": : :"memory");
}

/* Disable interrupts, but return previous interrupt state */

static inline irqstate_t irqsave(void)
{
  irqstate_t flags = irqflags();
  irqdisable();
  return flags;
}

/* Conditionally disable interrupts */

static inline void irqrestore(irqstate_t flags)
{
  if (irqenabled(flags))
    {
      irqenable();
    }
}

static inline void system_call3(unsigned int nbr, uintptr_t parm1,
			                    uintptr_t parm2, uintptr_t parm3)
{
  /* To be provided */
}

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_INCLUDE_I486_IRQ_H */

