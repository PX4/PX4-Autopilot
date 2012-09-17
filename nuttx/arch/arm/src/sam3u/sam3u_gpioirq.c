/****************************************************************************
 * arch/arm/src/sam3u/sam3u_gpioirq.c
 * arch/arm/src/chip/sam3u_gpioirq.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam3u_internal.h"
#include "sam3u_pio.h"
#include "sam3u_pmc.h"

#ifdef CONFIG_GPIO_IRQ

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_gpiobase
 *
 * Description:
 *   Return the base address of the GPIO register set
 *
 ****************************************************************************/

static inline uint32_t sam3u_gpiobase(uint16_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  return SAM3U_PION_BASE(port >> GPIO_PORT_SHIFT);
}

/****************************************************************************
 * Name: sam3u_gpiopin
 *
 * Description:
 *   Returun the base address of the GPIO register set
 *
 ****************************************************************************/

static inline int sam3u_gpiopin(uint16_t pinset)
{
  return 1 << ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam3u_irqbase
 *
 * Description:
 *   Return gpio information associated with this IRQ
 *
 ****************************************************************************/

static int sam3u_irqbase(int irq, uint32_t *base, int *pin)
{
  if (irq >= SAM3U_IRQ_NIRQS)
    {
#ifdef CONFIG_GPIOA_IRQ
      if (irq <= SAM3U_IRQ_PA31)
        {
          *base = SAM3U_PIOA_BASE;
          *pin  = irq - SAM3U_IRQ_PA0;
          return OK;
        }
#endif
#ifdef CONFIG_GPIOB_IRQ
      if (irq <= SAM3U_IRQ_PB31)
        {
          *base = SAM3U_PIOB_BASE;
          *pin  = irq - SAM3U_IRQ_PB0;
          return OK;
        }
#endif
#ifdef CONFIG_GPIOC_IRQ
      if (irq <= SAM3U_IRQ_PC31)
        {
          *base = SAM3U_PIOC_BASE;
          *pin  = irq - SAM3U_IRQ_PC0;
          return OK;
        }
#endif
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: up_gpioa/b/cinterrupt
 *
 * Description:
 *   Receive GPIOA/B/C interrupts
 *
 ****************************************************************************/

static int up_gpiointerrupt(uint32_t base, int irq0, void *context)
{
  uint32_t pending;
  uint32_t bit;
  int      irq;

  pending = getreg32(base+SAM3U_PIO_ISR_OFFSET) & getreg32(base+SAM3U_PIO_IMR_OFFSET);
  for (bit = 1, irq = irq0; pending != 0; bit <<= 1, irq++)
    {
      if ((pending & bit) != 0)
        {
          /* Re-deliver the IRQ (recurses! We got here from irq_dispatch!) */

          irq_dispatch(irq, context);

          /* Remove this from the set of pending interrupts */

          pending &= ~bit;
        }
    }
  return OK;
}

#ifdef CONFIG_GPIOA_IRQ
static int up_gpioainterrupt(int irq, void *context)
{
  return up_gpiointerrupt(SAM3U_PIOA_BASE, SAM3U_IRQ_PA0, context);
}
#endif

#ifdef CONFIG_GPIOB_IRQ
static int up_gpiobinterrupt(int irq, void *context)
{
  return up_gpiointerrupt(SAM3U_PIOB_BASE, SAM3U_IRQ_PB0, context);
}
#endif

#ifdef CONFIG_GPIOC_IRQ
static int up_gpiocinterrupt(int irq, void *context)
{
  return up_gpiointerrupt(SAM3U_PIOC_BASE, SAM3U_IRQ_PC0, context);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void sam3u_gpioirqinitialize(void)
{
  uint32_t pcer;

  /* Configure GPIOA interrupts */

#ifdef CONFIG_GPIOA_IRQ
  /* Enable GPIOA clocking */

  pcer |= (1 << SAM3U_PID_PIOA);
  putreg32(pcer, SAM3U_PMC_PCER);

  /* Clear and disable all GPIOA interrupts */

  (void)getreg32(SAM3U_PIOA_ISR);
  putreg32(0xffffffff, SAM3U_PIOA_IDR);

  /* Attach and enable the GPIOA IRQ */

  (void)irq_attach(SAM3U_IRQ_PIOA, up_gpioainterrupt);
  up_enable_irq(SAM3U_IRQ_PIOA);
#endif

  /* Configure GPIOB interrupts */

#ifdef CONFIG_GPIOB_IRQ
  /* Enable GPIOB clocking */

  pcer |= (1 << SAM3U_PID_PIOB);
  putreg32(pcer, SAM3U_PMC_PCER);

  /* Clear and disable all GPIOB interrupts */

  (void)getreg32(SAM3U_PIOB_ISR);
  putreg32(0xffffffff, SAM3U_PIOB_IDR);

  /* Attach and enable the GPIOB IRQ */

  (void)irq_attach(SAM3U_IRQ_PIOB, up_gpiobinterrupt);
  up_enable_irq(SAM3U_IRQ_PIOB);
#endif

  /* Configure GPIOC interrupts */

#ifdef CONFIG_GPIOC_IRQ
  /* Enable GPIOC clocking */

  pcer |= (1 << SAM3U_PID_PIOC);
  putreg32(pcer, SAM3U_PMC_PCER);

  /* Clear and disable all GPIOC interrupts */

  (void)getreg32(SAM3U_PIOC_ISR);
  putreg32(0xffffffff, SAM3U_PIOC_IDR);

  /* Attach and enable the GPIOC IRQ */

  (void)irq_attach(SAM3U_IRQ_PIOC, up_gpioainterrupt);
  up_enable_irq(SAM3U_IRQ_PIOC);
#endif
}

/************************************************************************************
 * Name: sam3u_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

void sam3u_gpioirq(uint16_t pinset)
{
  uint32_t base = sam3u_gpiobase(pinset);
  int      pin  = sam3u_gpiopin(pinset);
 
   /* Are any additional interrupt modes selected? */

   if ((pinset & GPIO_INT_MASK) != 0)
     {
       /* Yes.. Enable additional interrupt mode */
 
       putreg32(pin, base+SAM3U_PIO_AIMER_OFFSET);

       /* Level or edge detected interrupt? */

       if ((pinset & GPIO_INT_LEVEL) != 0)
         {
           putreg32(pin, base+SAM3U_PIO_LSR_OFFSET); /* Level */
         }
       else
         {
           putreg32(pin, base+SAM3U_PIO_ESR_OFFSET); /* Edge */
         }

      /* High level/rising edge or low level /falling edge? */

       if ((pinset & GPIO_INT_HIGHLEVEL) != 0)
         {
           putreg32(pin, base+SAM3U_PIO_REHLSR_OFFSET); /* High level/Rising edge */
         }
       else
         {
           putreg32(pin, base+SAM3U_PIO_FELLSR_OFFSET); /* Low level/Falling edge */
         }
     }
   else
     {
       /* No.. Disable additional interrupt mode */
 
       putreg32(pin, base+SAM3U_PIO_AIMDR_OFFSET);
     }
}

/************************************************************************************
 * Name: sam3u_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void sam3u_gpioirqenable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam3u_irqbase(irq, &base, &pin) == OK)
    {
       /* Clear (all) pending interrupts and enable this pin interrupt */

       (void)getreg32(base+SAM3U_PIO_ISR_OFFSET);
       putreg32((1 << pin), base+SAM3U_PIO_IER_OFFSET);
    }
}

/************************************************************************************
 * Name: sam3u_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void sam3u_gpioirqdisable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam3u_irqbase(irq, &base, &pin) == OK)
    {
       /* Disable this pin interrupt */

       putreg32((1 << pin), base+SAM3U_PIO_IDR_OFFSET);
    }
}

#endif /* CONFIG_GPIO_IRQ */