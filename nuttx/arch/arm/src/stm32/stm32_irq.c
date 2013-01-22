/****************************************************************************
 * arch/arm/src/stm32/stm32_irq.c
 * arch/arm/src/chip/stm32_irq.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
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
#include "stm32_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enable NVIC debug features that are probably only desireable during
 * bringup
 */

#undef STM32_IRQ_DEBUG

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
 * Name: stm32_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(STM32_IRQ_DEBUG) && defined (CONFIG_DEBUG)
static void stm32_dumpnvic(const char *msg, int irq)
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
  slldbg("  IRQ ENABLE: %08x %08x %08x\n",
         getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
         getreg32(NVIC_IRQ64_95_ENABLE));
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
  slldbg("              %08x %08x %08x %08x\n", 
        getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
        getreg32(NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
  slldbg("              %08x\n", 
        getreg32(NVIC_IRQ64_67_PRIORITY));
  irqrestore(flags);
}
#else
#  define stm32_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: stm32_nmi, stm32_busfault, stm32_usagefault, stm32_pendsv,
 *       stm32_dbgmonitor, stm32_pendsv, stm32_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int stm32_nmi(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! NMI received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int stm32_busfault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Bus fault recived\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int stm32_usagefault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Usage fault received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int stm32_pendsv(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! PendSV received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int stm32_dbgmonitor(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Debug Monitor receieved\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

static int stm32_reserved(int irq, FAR void *context)
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
 * Name: stm32_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int stm32_irqinfo(int irq, uint32_t *regaddr, uint32_t *bit)
{
  DEBUGASSERT(irq >= STM32_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= STM32_IRQ_INTERRUPTS)
    {
      if (irq < STM32_IRQ_INTERRUPTS + 32)
        {
           *regaddr = NVIC_IRQ0_31_ENABLE;
           *bit     = 1 << (irq - STM32_IRQ_INTERRUPTS);
        }
      else if (irq < STM32_IRQ_INTERRUPTS + 64)
        {
           *regaddr = NVIC_IRQ32_63_ENABLE;
           *bit     = 1 << (irq - STM32_IRQ_INTERRUPTS - 32);
        }
      else if (irq < NR_IRQS)
        {
           *regaddr = NVIC_IRQ64_95_ENABLE;
           *bit     = 1 << (irq - STM32_IRQ_INTERRUPTS - 64);
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
       if (irq == STM32_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == STM32_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == STM32_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == STM32_IRQ_SYSTICK)
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
  uint32_t regaddr;
  int num_priority_registers;

  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);
  putreg32(0, NVIC_IRQ32_63_ENABLE);

  /* The standard location for the vector table is at the beginning of FLASH
   * at address 0x0800:0000.  If we are using the STMicro DFU bootloader, then
   * the vector table will be offset to a different location in FLASH and we
   * will need to set the NVIC vector location to this alternative location.
   */

#ifdef CONFIG_STM32_DFU
  putreg32((uint32_t)stm32_vectors, NVIC_VECTAB);
#endif

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

  irq_attach(STM32_IRQ_SVCALL, up_svcall);
  irq_attach(STM32_IRQ_HARDFAULT, up_hardfault);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
/* up_prioritize_irq(STM32_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
   up_prioritize_irq(STM32_IRQ_SVCALL, NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARMV7M_MPU
  irq_attach(STM32_IRQ_MEMFAULT, up_memfault);
  up_enable_irq(STM32_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(STM32_IRQ_NMI, stm32_nmi);
#ifndef CONFIG_ARMV7M_MPU
  irq_attach(STM32_IRQ_MEMFAULT, up_memfault);
#endif
  irq_attach(STM32_IRQ_BUSFAULT, stm32_busfault);
  irq_attach(STM32_IRQ_USAGEFAULT, stm32_usagefault);
  irq_attach(STM32_IRQ_PENDSV, stm32_pendsv);
  irq_attach(STM32_IRQ_DBGMONITOR, stm32_dbgmonitor);
  irq_attach(STM32_IRQ_RESERVED, stm32_reserved);
#endif

  stm32_dumpnvic("initial", NR_IRQS);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

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

  if (stm32_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Clear the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval &= ~bit;
      putreg32(regval, regaddr);
    }
  stm32_dumpnvic("disable", irq);
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

  if (stm32_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Set the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval |= bit;
      putreg32(regval, regaddr);
    }
  stm32_dumpnvic("enable", irq);
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
  DEBUGASSERT(irq >= STM32_IRQ_MEMFAULT && irq < NR_IRQS &&
              priority >= NVIC_SYSH_DISABLE_PRIORITY &&
              priority <= NVIC_SYSH_PRIORITY_MIN);
#else
  DEBUGASSERT(irq >= STM32_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);
#endif

  if (irq < STM32_IRQ_INTERRUPTS)
    {
      irq    -= 4;
      regaddr = NVIC_SYSH_PRIORITY(irq);
    }
  else
    {
      irq    -= STM32_IRQ_INTERRUPTS;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  stm32_dumpnvic("prioritize", irq);
  return OK;
}
#endif
