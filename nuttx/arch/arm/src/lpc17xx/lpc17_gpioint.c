/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_gpioint.c
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "chip.h"
#include "lpc17_gpio.h"


#ifdef CONFIG_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Name: lpc17_getintedge
 *
 * Description:
 *  Get the stored interrupt edge configuration.
 *
 ****************************************************************************/

static unsigned int lpc17_getintedge(unsigned int port, unsigned int pin)
{
  uint64_t *intedge;

  /* Which word to we use? */

  if (port == 0)
    {
      intedge = &g_intedge0;
    }
  else if (port == 2)
    {
      intedge  = &g_intedge2;
    }
  else
    {
      return 0;
    }

  /* Return the value for the PINSEL */

  return (unsigned int)(((*intedge) >> (pin << 1)) & 3);
}

/****************************************************************************
 * Name: lpc17_setintedge
 *
 * Description:
 *  Set the edge interrupt enabled bits for this pin.
 *
 ****************************************************************************/

static void lpc17_setintedge(uint32_t intbase, unsigned int pin, unsigned int edges)
{
  int regval;

  /* Set/clear the rising edge enable bit */

  regval = getreg32(intbase + LPC17_GPIOINT_INTENR_OFFSET);
  if ((edges & 2) != 0)
    {
      regval |= GPIOINT(pin);
    }
  else
    {
      regval &= ~GPIOINT(pin);
    }
  putreg32(regval, intbase + LPC17_GPIOINT_INTENR_OFFSET);

  /* Set/clear the rising edge enable bit */

  regval = getreg32(intbase + LPC17_GPIOINT_INTENF_OFFSET);
  if ((edges & 1) != 0)
    {
      regval |= GPIOINT(pin);
    }
  else
    {
      regval &= ~GPIOINT(pin);
    }
  putreg32(regval, intbase + LPC17_GPIOINT_INTENF_OFFSET);
}

/****************************************************************************
 * Name: lpc17_irq2port
 *
 * Description:
 *  Given an IRQ number, return the GPIO port number (0 or 2) of the interrupt.
 *
 ****************************************************************************/

static int lpc17_irq2port(int irq)
{
 /* Set 1: 12 interrupts p0.0-p0.11 */

  if (irq >= LPC17_VALID_FIRST0L && irq < (LPC17_VALID_FIRST0L+LPC17_VALID_NIRQS0L))
    {
      return 0;
    }

  /* Set 2: 16 interrupts p0.15-p0.30 */

  else if (irq >= LPC17_VALID_FIRST0H && irq < (LPC17_VALID_FIRST0H+LPC17_VALID_NIRQS0H))
    {
      return 0;
    }

  /* Set 3: 14 interrupts p2.0-p2.13 */

  else if (irq >= LPC17_VALID_FIRST2 && irq < (LPC17_VALID_FIRST2+LPC17_VALID_NIRQS2))
    {
      return 2;
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_irq2pin
 *
 * Description:
 *  Given an IRQ number, return the GPIO pin number (0..31) of the interrupt.
 *
 ****************************************************************************/

static int lpc17_irq2pin(int irq)
{
  /* Set 1: 12 interrupts p0.0-p0.11
   * 
   * See arch/arm/include/lpc17xx/irq.h:
   * LPC17_VALID_SHIFT0L   0    - Bit 0 is thre first bit in the group of 12 interrupts
   * LPC17_VALID_FIRST0L   irq  - IRQ number associated with p0.0
   * LPC17_VALID_NIRQS0L   12   - 12 interrupt bits in the group
   */

  if (irq >= LPC17_VALID_FIRST0L && irq < (LPC17_VALID_FIRST0L+LPC17_VALID_NIRQS0L))
    {
      return irq - LPC17_VALID_FIRST0L + LPC17_VALID_SHIFT0L;
    }

  /* Set 2: 16 interrupts p0.15-p0.30
   * 
   * LPC17_VALID_SHIFT0H   15   - Bit 15 is the first bit in a group of 16 interrupts
   * LPC17_VALID_FIRST0L   irq  - IRQ number associated with p0.15
   * LPC17_VALID_NIRQS0L   16   - 16 interrupt bits in the group
   */

  else if (irq >= LPC17_VALID_FIRST0H && irq < (LPC17_VALID_FIRST0H+LPC17_VALID_NIRQS0H))
    {
      return irq - LPC17_VALID_FIRST0H + LPC17_VALID_SHIFT0H;
    }

  /* Set 3: 14 interrupts p2.0-p2.13
   * 
   * LPC17_VALID_SHIFT2    0    - Bit 0 is the first bit in a group of 14 interrupts
   * LPC17_VALID_FIRST2    irq  - IRQ number associated with p2.0
   * LPC17_VALID_NIRQS2    14   - 14 interrupt bits in the group
   */

  else if (irq >= LPC17_VALID_FIRST2 && irq < (LPC17_VALID_FIRST2+LPC17_VALID_NIRQS2))
    {
      return irq - LPC17_VALID_FIRST2 + LPC17_VALID_SHIFT2;
    }
  return -EINVAL;
}
   
/****************************************************************************
 * Name: lpc17_gpiodemux
 *
 * Description:
 *  Demux all interrupts on one GPIO interrupt status register.
 *
 ****************************************************************************/

static void lpc17_gpiodemux(uint32_t intbase, uint32_t intmask,
                            int irqbase, void *context)
{
  uint32_t intstatr;
  uint32_t intstatf;
  uint32_t intstatus;
  uint32_t bit;
  int      irq;

  /* Get the interrupt rising and falling edge status and mask out only the
   * interrupts that are enabled.
   */

  intstatr  = getreg32(intbase + LPC17_GPIOINT_INTSTATR_OFFSET);
  intstatr &= getreg32(intbase + LPC17_GPIOINT_INTENR_OFFSET);

  intstatf  = getreg32(intbase + LPC17_GPIOINT_INTSTATF_OFFSET);
  intstatf &= getreg32(intbase + LPC17_GPIOINT_INTENF_OFFSET);

  /* And get the OR of the enabled interrupt sources.  We do not make any
   * distinction between rising and falling edges (but the hardware does support
   * the ability to differently if needed.
   */

  intstatus = intstatr | intstatf;

  /* Now march through the (valid) bits and dispatch each interrupt */

  irq = irqbase;
  bit = 1;
  while (intstatus != 0)
    {
      /* Does this pin support an interrupt?  If no, skip over it WITHOUT
       * incrementing irq.
       */

      if ((intmask & bit) != 0)
        {
           /* This pin can support an interrupt.  Is there an interrupt pending
            * and enabled?
            */

           if ((intstatus & bit) != 0)
             {
               /* Clear the interrupt status */

               putreg32(bit, intbase + LPC17_GPIOINT_INTCLR_OFFSET);

               /* And dispatch the interrupt */

               irq_dispatch(irq, context);
             }

           /* Increment the IRQ number on each interrupt pin */

           irq++;
        }
 
      /* Next bit */

      intstatus &= ~bit;
      bit      <<= 1;
    }
}

/****************************************************************************
 * Name: lpc17_gpiointerrupt
 *
 * Description:
 *  Handle the EINT3 interrupt that also indicates that a GPIO interrupt has
 *  occurred.  NOTE:  This logic will have to be extended if EINT3 is
 *  actually used for External Interrupt 3.
 *
 ****************************************************************************/

static int lpc17_gpiointerrupt(int irq, void *context)
{
  /* Get the GPIO interrupt status */

  uint32_t intstatus = getreg32(LPC17_GPIOINT_IOINTSTATUS);

  /* Check for an interrupt on GPIO0 */

  if ((intstatus & GPIOINT_IOINTSTATUS_P0INT) != 0)
    {
      lpc17_gpiodemux(LPC17_GPIOINT0_BASE, LPC17_VALID_GPIOINT0,
                      LPC17_VALID_FIRST0L, context);
    }

  /* Check for an interrupt on GPIO2 */

  if ((intstatus & GPIOINT_IOINTSTATUS_P2INT) != 0)
    {
      lpc17_gpiodemux(LPC17_GPIOINT2_BASE, LPC17_VALID_GPIOINT2,
                      LPC17_VALID_FIRST2, context);
    }

  return OK;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void lpc17_gpioirqinitialize(void)
{
   /* Disable all GPIO interrupts */

  putreg32(0, LPC17_GPIOINT0_INTENR);
  putreg32(0, LPC17_GPIOINT0_INTENF);
  putreg32(0, LPC17_GPIOINT2_INTENR);
  putreg32(0, LPC17_GPIOINT2_INTENF);

  /* Attach and enable the GPIO IRQ.  Note:  GPIO0 and GPIO2 interrupts share
   * the same position in the NVIC with External Interrupt 3
   */

  (void)irq_attach(LPC17_IRQ_EINT3, lpc17_gpiointerrupt);
  up_enable_irq(LPC17_IRQ_EINT3);
}

/****************************************************************************
 * Name: lpc17_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void lpc17_gpioirqenable(int irq)
{
  /* Map the IRQ number to a port number */

  int port = lpc17_irq2port(irq);
  if (port >= 0)
    {
      /* The IRQ number does correspond to an interrupt port.  Now get the base
       * address of the GPIOINT registers for the port.
       */

      uint32_t intbase = g_intbase[port];
      if (intbase != 0)
        {
          /* And get the pin number associated with the port */

          unsigned int pin   = lpc17_irq2pin(irq);
          unsigned int edges = lpc17_getintedge(port, pin);
          lpc17_setintedge(intbase, pin, edges);
        }
    }
}

/****************************************************************************
 * Name: lpc17_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void lpc17_gpioirqdisable(int irq)
{
  /* Map the IRQ number to a port number */

  int port = lpc17_irq2port(irq);
  if (port >= 0)
    {
      /* The IRQ number does correspond to an interrupt port.  Now get the base
       * address of the GPIOINT registers for the port.
       */

      uint32_t intbase = g_intbase[port];
      if (intbase != 0)
        {
          /* And get the pin number associated with the port */

          unsigned int pin   = lpc17_irq2pin(irq);
          lpc17_setintedge(intbase, pin, 0);
        }
    }
}

#endif /* CONFIG_GPIO_IRQ */

