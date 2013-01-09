/****************************************************************************
 * arch/arm/src/lm/lm_gpioirq.c
 * arch/arm/src/chip/lm_gpioirq.c
 *
 *   Copyright (C) 2009-2010, 2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "irq_internal.h"

#include "lm_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A table of handlers for each GPIO interrupt */

static FAR xcpt_t g_gpioirqvector[NR_GPIO_IRQS];

/* A table that maps a GPIO group to a GPIO base address.  Overly complicated
 * because we support disabling interrupt support for arbitrary ports.  This
 * must carefully match the IRQ numbers assigned in arch/arm/include/lm3s/irq.h
 */

static const uint32_t g_gpiobase[] =
{
#ifndef CONFIG_LM_DISABLE_GPIOA_IRQS
   LM_GPIOA_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOB_IRQS
  LM_GPIOB_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOC_IRQS
  LM_GPIOC_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOD_IRQS
  LM_GPIOD_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOE_IRQS
  LM_GPIOE_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOF_IRQS
  LM_GPIOF_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOG_IRQS
  LM_GPIOG_BASE,
#endif

  /* NOTE: Not all Stellaris architectures support GPIOs above GPIOG.  If the
   * chip does not support these higher ports, then they must be disabled in
   * the configuration.  Otherwise, the following will likely cause compilation
   * errors!
   */

#ifndef CONFIG_LM_DISABLE_GPIOH_IRQS
  LM_GPIOH_BASE,
#endif
#ifndef CONFIG_LM_DISABLE_GPIOJ_IRQS
  LM_GPIOJ_BASE,
#endif
};

#define GPIO_NADDRS (sizeof(g_gpiobase)/sizeof(uint32_t))

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_gpiobaseaddress
 *
 * Input:
 *   gpioirq - A pin number in the range of 0 to NR_GPIO_IRQS.
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.  NOTE that range checking was provided by
 *   callee
 *
 ****************************************************************************/

static uint32_t lm_gpiobaseaddress(unsigned int gpioirq)
{
  unsigned int ndx = gpioirq >> 3;
  if (ndx < GPIO_NADDRS)
    {
      return g_gpiobase[ndx];
    }
  return 0;
}

/****************************************************************************
 * Name: lm_gpio*handler
 *
 * Description:
 *   Handle interrupts on each enabled GPIO port
 *
 ****************************************************************************/

static int lm_gpiohandler(uint32_t regbase, int irqbase, void *context)
{
  uint32_t mis;
  int irq;
  int pin;

  /* Handle each pending GPIO interrupt.  "The GPIO MIS register is the masked
   * interrupt status register. Bits read High in GPIO MIS reflect the status
   * of input lines triggering an interrupt. Bits read as Low indicate that
   * either no interrupt has been generated, or the interrupt is masked."
   */

  mis = getreg32(regbase + LM_GPIO_MIS_OFFSET) & 0xff;

  /* Clear all GPIO interrupts that we are going to process.  "The GPIO ICR
   * register is the interrupt clear register. Writing a 1 to a bit in this
   * register clears the corresponding interrupt edge detection logic register.
   * Writing a 0 has no effect."
   */

  putreg32(mis, regbase + LM_GPIO_ICR_OFFSET);

  /* Now process each IRQ pending in the MIS */

  for (pin = 0; pin < 8 && mis != 0; pin++, mis >>= 1)
    {
      if ((mis & 1) != 0)
        {
          irq = irqbase + pin;
          g_gpioirqvector[irq - NR_IRQS](irq, context);
        }
    }
  return OK;
}

#ifndef CONFIG_LM_DISABLE_GPIOA_IRQS
static int lm_gpioahandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOA_BASE, LM_IRQ_GPIOA_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOB_IRQS
static int lm_gpiobhandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOB_BASE, LM_IRQ_GPIOB_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOC_IRQS
static int lm_gpiochandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOC_BASE, LM_IRQ_GPIOC_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOD_IRQS
static int lm_gpiodhandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOD_BASE, LM_IRQ_GPIOD_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOE_IRQS
static int lm_gpioehandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOE_BASE, LM_IRQ_GPIOE_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOF_IRQS
static int lm_gpiofhandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOF_BASE, LM_IRQ_GPIOF_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOG_IRQS
static int lm_gpioghandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOG_BASE, LM_IRQ_GPIOG_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOH_IRQS
static int lm_gpiohhandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOH_BASE, LM_IRQ_GPIOH_0, context);
}
#endif

#ifndef CONFIG_LM_DISABLE_GPIOJ_IRQS
static int lm_gpiojhandler(int irq, FAR void *context)
{
  return lm_gpiohandler(LM_GPIOJ_BASE, LM_IRQ_GPIOJ_0, context);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

int gpio_irqinitialize(void)
{
  int i;

  /* Point all interrupt vectors to the unexpected interrupt */

  for (i = 0; i < NR_GPIO_IRQS; i++)
    {
      g_gpioirqvector[i] = irq_unexpected_isr;
    }

  /* Then attach each GPIO interrupt handlers and enable corresponding GPIO
   * interrupts
   */

#ifndef CONFIG_LM_DISABLE_GPIOA_IRQS
  irq_attach(LM_IRQ_GPIOA, lm_gpioahandler);
  up_enable_irq(LM_IRQ_GPIOA);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOB_IRQS
  irq_attach(LM_IRQ_GPIOB, lm_gpiobhandler);
  up_enable_irq(LM_IRQ_GPIOB);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOC_IRQS
  irq_attach(LM_IRQ_GPIOC, lm_gpiochandler);
  up_enable_irq(LM_IRQ_GPIOC);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOD_IRQS
  irq_attach(LM_IRQ_GPIOD, lm_gpiodhandler);
  up_enable_irq(LM_IRQ_GPIOD);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOE_IRQS
  irq_attach(LM_IRQ_GPIOE, lm_gpioehandler);
  up_enable_irq(LM_IRQ_GPIOE);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOF_IRQS
  irq_attach(LM_IRQ_GPIOF, lm_gpiofhandler);
  up_enable_irq(LM_IRQ_GPIOF);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOG_IRQS
  irq_attach(LM_IRQ_GPIOG, lm_gpioghandler);
  up_enable_irq(LM_IRQ_GPIOG);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOH_IRQS
  irq_attach(LM_IRQ_GPIOH, lm_gpiohhandler);
  up_enable_irq(LM_IRQ_GPIOH);
#endif
#ifndef CONFIG_LM_DISABLE_GPIOJ_IRQS
  irq_attach(LM_IRQ_GPIOJ, lm_gpiojhandler);
  up_enable_irq(LM_IRQ_GPIOJ);
#endif

  return OK;
}

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 ****************************************************************************/

int gpio_irqattach(int irq, xcpt_t isr)
{
  irqstate_t flags;
  int        gpioirq = irq - NR_IRQS;
  int        ret     = ERROR;

  if ((unsigned)gpioirq < NR_GPIO_IRQS)
    {
      flags = irqsave();

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      if (isr == NULL)
        {
#ifndef CONFIG_ARCH_NOINTC
           gpio_irqdisable(gpioirq);
#endif
           isr = irq_unexpected_isr;
        }

      /* Save the new ISR in the table. */

      g_irqvector[gpioirq] = isr;
      irqrestore(flags);
      ret = OK;
    }
  return ret;
}

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqenable(int irq)
{
  irqstate_t flags;
  int        gpioirq = irq - NR_IRQS;
  uint32_t   base;
  uint32_t   regval;
  int        pin;

  if ((unsigned)gpioirq < NR_GPIO_IRQS)
    {
      /* Get the base address of the GPIO module associated with this IRQ */

      base = lm_gpiobaseaddress(gpioirq);
      DEBUGASSERT(base != 0);
      pin  = (1 << (gpioirq & 7));

      /* Disable the GPIO interrupt. "The GPIO IM register is the interrupt
       * mask register. Bits set to High in GPIO IM allow the corresponding
       * pins to trigger their individual interrupts and the combined GPIO INTR
       * line. Clearing a bit disables interrupt triggering on that pin. All
       * bits are cleared by a reset.
       */

      flags   = irqsave();
      regval  = getreg32(base + LM_GPIO_IM_OFFSET);
      regval |= pin;
      putreg32(regval, base + LM_GPIO_IM_OFFSET);
      irqrestore(flags);
    }
}

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqdisable(int irq)
{
  irqstate_t flags;
  int        gpioirq = irq - NR_IRQS;
  uint32_t   base;
  uint32_t   regval;
  int        pin;

  if ((unsigned)gpioirq < NR_GPIO_IRQS)
    {
      /* Get the base address of the GPIO module associated with this IRQ */

      base = lm_gpiobaseaddress(gpioirq);
      DEBUGASSERT(base != 0);
      pin  = (1 << (gpioirq & 7));

      /* Disable the GPIO interrupt. "The GPIO IM register is the interrupt
       * mask register. Bits set to High in GPIO IM allow the corresponding
       * pins to trigger their individual interrupts and the combined GPIO INTR
       * line. Clearing a bit disables interrupt triggering on that pin. All
       * bits are cleared by a reset.
       */

      flags   = irqsave();
      regval  = getreg32(base + LM_GPIO_IM_OFFSET);
      regval &= ~pin;
      putreg32(regval, base + LM_GPIO_IM_OFFSET);
      irqrestore(flags);
    }
}
