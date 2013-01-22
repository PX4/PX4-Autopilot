/****************************************************************************
 * arch/arm/src/lpc43/lpc43_irq.c
 * arch/arm/src/chip/lpc43_irq.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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
#include <arch/irq.h>

#include "chip.h"
#include "nvic.h"
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "lpc43_irq.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (LPC43M4_SYSH_PRIORITY_DEFAULT << 24 |\
   LPC43M4_SYSH_PRIORITY_DEFAULT << 16 |\
   LPC43M4_SYSH_PRIORITY_DEFAULT << 8  |\
   LPC43M4_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/* This is the address of the vector table */

extern unsigned _vectors[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ) && defined (CONFIG_DEBUG)
static void lpc43_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = irqsave();
  slldbg("NVIC (%s, irq=%d):\n", msg, irq);
  slldbg("  INTCTRL:    %08x VECTAB: %08x\n",
         getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
#if 0
  slldbg("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
         getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
         getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
#endif
  slldbg("  IRQ ENABLE: %08x %08x\n",
         getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE));
  slldbg("  SYSH_PRIO:  %08x %08x %08x\n",
         getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
         getreg32(NVIC_SYSH12_15_PRIORITY));
  slldbg("  IRQ PRIO:   %08x %08x %08x %08x\n", 
        getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
        getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  slldbg("              %08x %08x %08x %08x\n", 
        getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
        getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
  slldbg("              %08x %08x %08x %08x\n", 
        getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
        getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
  slldbg("              %08x %08x %08x\n", 
        getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
        getreg32(NVIC_IRQ56_59_PRIORITY));
  irqrestore(flags);
}
#else
#  define lpc43_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: lpc43_nmi, lpc43_busfault, lpc43_usagefault, lpc43_pendsv,
 *       lpc43_dbgmonitor, lpc43_pendsv, lpc43_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int lpc43_nmi(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! NMI received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc43_busfault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Bus fault recived\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc43_usagefault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Usage fault received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc43_pendsv(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! PendSV received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc43_dbgmonitor(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Debug Monitor receieved\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc43_reserved(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Reserved interrupt\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}
#endif

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_IRQPRIO) && defined(CONFIG_ARMV7M_USEBASEPRI)
static int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  irq        -= 4;
  regaddr     = NVIC_SYSH_PRIORITY(irq);
  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  stm32_dumpnvic("prioritize", irq);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc43_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int lpc43_irqinfo(int irq, uint32_t *regaddr, uint32_t *bit)
{
  DEBUGASSERT(irq >= LPC43_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= LPC43_IRQ_EXTINT)
    {
      if (irq < (LPC43_IRQ_EXTINT + 32))
        {
           *regaddr = NVIC_IRQ0_31_ENABLE;
           *bit     = 1 << (irq - LPC43_IRQ_EXTINT);
        }
      else if (irq < LPC43M4_IRQ_NIRQS)
        {
           *regaddr = NVIC_IRQ32_63_ENABLE;
           *bit     = 1 << (irq - LPC43_IRQ_EXTINT - 32);
        }
      else
        {
          return ERROR; /* Invalid interrupt */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
       *regaddr = NVIC_SYSHCON;
       if (irq == LPC43_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == LPC43_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == LPC43_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == LPC43_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Complete initialization of the interrupt system and enable normal,
 *   interrupt processing.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
#ifdef CONFIG_DEBUG
  uint32_t regval;
#endif
  int num_priority_registers;

  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);
  putreg32(0, NVIC_IRQ32_63_ENABLE);

  /* Make sure that we are using the correct vector table.  The default
   * vector address is 0x0000:0000 but if we are executing code that is
   * positioned in SRAM or in external FLASH, then we may need to reset
   * the interrupt vector so that it refers to the table in SRAM or in
   * external FLASH.
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(LPC43_IRQ_SVCALL, up_svcall);
  irq_attach(LPC43_IRQ_HARDFAULT, up_hardfault);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
/* up_prioritize_irq(LPC43_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
   up_prioritize_irq(LPC43_IRQ_SVCALL, NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARMV7M_MPU
  irq_attach(LPC43_IRQ_MEMFAULT, up_memfault);
  up_enable_irq(LPC43_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(LPC43_IRQ_NMI, lpc43_nmi);
#ifndef CONFIG_ARMV7M_MPU
  irq_attach(LPC43_IRQ_MEMFAULT, up_memfault);
#endif
  irq_attach(LPC43_IRQ_BUSFAULT, lpc43_busfault);
  irq_attach(LPC43_IRQ_USAGEFAULT, lpc43_usagefault);
  irq_attach(LPC43_IRQ_PENDSV, lpc43_pendsv);
  irq_attach(LPC43_IRQ_DBGMONITOR, lpc43_dbgmonitor);
  irq_attach(LPC43_IRQ_RESERVED, lpc43_reserved);
#endif

  lpc43_dumpnvic("initial", LPC43M4_IRQ_NIRQS);

  /* If a debugger is connected, try to prevent it from catching hardfaults */

#ifdef CONFIG_DEBUG
  regval  = getreg32(NVIC_DEMCR);
  regval &= ~NVIC_DEMCR_VCHARDERR;
  putreg32(regval, NVIC_DEMCR);
#endif

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  setbasepri(LPC43M4_SYSH_PRIORITY_MAX);
  irqrestore(0);
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (lpc43_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Clear the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval &= ~bit;
      putreg32(regval, regaddr);
    }
#ifdef CONFIG_GPIO_IRQ
  else if (irq >= LPC43_VALID_FIRST0L)
    {
      /* Maybe it is a (derived) GPIO IRQ */

      lpc43_gpioint_disable(irq);
    }
#endif
  lpc43_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (lpc43_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Set the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval |= bit;
      putreg32(regval, regaddr);
    }
#ifdef CONFIG_GPIO_IRQ
  else if (irq >= LPC43_VALID_FIRST0L)
    {
      /* Maybe it is a (derived) GPIO IRQ */

      lpc43_gpioint_enable(irq);
    }
#endif
  lpc43_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  up_disable_irq(irq);

#if 0 /* Does not appear to be necessary in most cases */
  lpc43_clrpend(irq);
#endif
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

#ifdef CONFIG_ARMV7M_USEBASEPRI
  DEBUGASSERT(irq >= LPC43_IRQ_MEMFAULT && irq < NR_IRQS &&
              priority >= NVIC_SYSH_DISABLE_PRIORITY &&
              priority <= NVIC_SYSH_PRIORITY_MIN);
#else
  DEBUGASSERT(irq >= LPC43_IRQ_MEMFAULT && irq < NR_IRQS &&
             (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);
#endif

  if (irq < LPC43_IRQ_EXTINT)
    {
      irq    -= 4;
      regaddr = NVIC_SYSH_PRIORITY(irq);
    }
  else
    {
      irq    -= LPC43_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  lpc43_dumpnvic("prioritize", irq);
  return OK;
}
#endif
