/****************************************************************************
 * arch/arm/src/lpc17/lpc17_irq.c
 * arch/arm/src/chip/lpc17_irq.c
 *
 *   Copyright (C) 2010-2011, 2013 Gregory Nutt. All rights reserved.
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

#include "nvic.h"
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "lpc17_gpio.h"
#include "lpc17_clrpend.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enable NVIC debug features that are probably only desireable during
 * bringup
 */

#undef LPC17_IRQ_DEBUG

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 16 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 8  |\
   NVIC_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(LPC17_IRQ_DEBUG) && defined (CONFIG_DEBUG)
static void lpc17_dumpnvic(const char *msg, int irq)
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
  slldbg("  IRQ ENABLE: %08x\n", getreg32(NVIC_IRQ0_31_ENABLE));
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
  irqrestore(flags);
}
#else
#  define lpc17_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: lpc17_nmi, lpc17_busfault, lpc17_usagefault, lpc17_pendsv,
 *       lpc17_dbgmonitor, lpc17_pendsv, lpc17_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int lpc17_nmi(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! NMI received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc17_busfault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Bus fault recived\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc17_usagefault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Usage fault received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc17_pendsv(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! PendSV received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc17_dbgmonitor(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Debug Monitor receieved\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int lpc17_reserved(int irq, FAR void *context)
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
 * Name: lpc17_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int lpc17_irqinfo(int irq, uint32_t *regaddr, uint32_t *bit)
{
  DEBUGASSERT(irq >= LPC17_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= LPC17_IRQ_EXTINT)
    {
      if (irq < (LPC17_IRQ_EXTINT+32))
        {
           *regaddr = NVIC_IRQ0_31_ENABLE;
           *bit     = 1 << (irq - LPC17_IRQ_EXTINT);
        }
      else if (irq < LPC17_IRQ_NIRQS)
        {
           *regaddr = NVIC_IRQ32_63_ENABLE;
           *bit     = 1 << (irq - LPC17_IRQ_EXTINT - 32);
        }
      else
        {
          return ERROR; /* Invalid irq */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
       *regaddr = NVIC_SYSHCON;
       if (irq == LPC17_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == LPC17_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == LPC17_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == LPC17_IRQ_SYSTICK)
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
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);

  /* Set all interrrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  putreg32(DEFPRIORITY32, NVIC_IRQ0_3_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ12_15_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ16_19_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ20_23_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ24_27_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ28_31_PRIORITY);

  putreg32(DEFPRIORITY32, NVIC_IRQ32_35_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ36_39_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ40_43_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ44_47_PRIORITY);

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(LPC17_IRQ_SVCALL, up_svcall);
  irq_attach(LPC17_IRQ_HARDFAULT, up_hardfault);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
/* up_prioritize_irq(LPC17_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
   up_prioritize_irq(LPC17_IRQ_SVCALL, NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARMV7M_MPU
  irq_attach(LPC17_IRQ_MEMFAULT, up_memfault);
  up_enable_irq(LPC17_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(LPC17_IRQ_NMI, lpc17_nmi);
#ifndef CONFIG_ARMV7M_MPU
  irq_attach(LPC17_IRQ_MEMFAULT, up_memfault);
#endif
  irq_attach(LPC17_IRQ_BUSFAULT, lpc17_busfault);
  irq_attach(LPC17_IRQ_USAGEFAULT, lpc17_usagefault);
  irq_attach(LPC17_IRQ_PENDSV, lpc17_pendsv);
  irq_attach(LPC17_IRQ_DBGMONITOR, lpc17_dbgmonitor);
  irq_attach(LPC17_IRQ_RESERVED, lpc17_reserved);
#endif

  lpc17_dumpnvic("initial", LPC17_IRQ_NIRQS);

  /* Initialize logic to support a second level of interrupt decoding for
   * GPIO pins.
   */
 
#ifdef CONFIG_GPIO_IRQ
  lpc17_gpioirqinitialize();
#endif

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqenable();
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

  if (lpc17_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Clear the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval &= ~bit;
      putreg32(regval, regaddr);
    }
#ifdef CONFIG_GPIO_IRQ
  else if (irq >= LPC17_VALID_FIRST0L)
    {
      /* Maybe it is a (derived) GPIO IRQ */

      lpc17_gpioirqdisable(irq);
    }
#endif
  lpc17_dumpnvic("disable", irq);
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

  if (lpc17_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Set the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval |= bit;
      putreg32(regval, regaddr);
    }
#ifdef CONFIG_GPIO_IRQ
  else if (irq >= LPC17_VALID_FIRST0L)
    {
      /* Maybe it is a (derived) GPIO IRQ */

      lpc17_gpioirqenable(irq);
    }
#endif
  lpc17_dumpnvic("enable", irq);
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
  lpc17_clrpend(irq);
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
  DEBUGASSERT(irq >= LPC17_IRQ_MEMFAULT && irq < LPC17_IRQ_NIRQS &&
              priority >= NVIC_SYSH_DISABLE_PRIORITY &&
              priority <= NVIC_SYSH_PRIORITY_MIN);
#else
  DEBUGASSERT(irq >= LPC17_IRQ_MEMFAULT && irq < LPC17_IRQ_NIRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);
#endif

  if (irq < LPC17_IRQ_EXTINT)
    {
      irq    -= 4;
      regaddr = NVIC_SYSH_PRIORITY(irq);
    }
  else
    {
      irq    -= LPC17_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  lpc17_dumpnvic("prioritize", irq);
  return OK;
}
#endif
