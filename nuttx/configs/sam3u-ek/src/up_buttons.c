/****************************************************************************
 * configs/sam3u-ek/src/up_leds.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "sam3u_internal.h"
#include "sam3uek_internal.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_irqbutton1;
static xcpt_t g_irqbutton2;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqbuttonx
 *
 * Description:
 *   This function implements the core of the up_irqbutton() logic.
 *
 ****************************************************************************/

#if defined(CONFIG_GPIOA_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
static xcpt_t up_irqbuttonx(int irq, xcpt_t irqhandler, xcpt_t *store)
{
  xcpt_t oldhandler;
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the following
   * operations are atomic.
   */

  flags = irqsave();

  /* Get the old button interrupt handler and save the new one */

  oldhandler = *store;
  *store = irqhandler;

  /* Configure the interrupt */

  sam3u_gpioirq(irq);
  (void)irq_attach(irq, irqhandler);
  sam3u_gpioirqenable(irq);
  irqrestore(flags);

  /* Return the old button handler (so that it can be restored) */

  return oldhandler;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_buttoninit
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 ****************************************************************************/

void up_buttoninit(void)
{
  (void)sam3u_configgpio(GPIO_BUTTON1);
  (void)sam3u_configgpio(GPIO_BUTTON2);
}

/************************************************************************************
 * Name: up_buttons
 *
 * Description:
 *   After up_buttoninit() has been called, up_buttons() may be called to collect
 *   the state of all buttons.  up_buttons() returns an 8-bit bit set with each bit
 *   associated with a button.  See the BUTTON* definitions above for the meaning of
 *   each bit in the returned value.
 *
 ************************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t retval;

  retval  = sam3u_gpioread(GPIO_BUTTON1) ? 0 : GPIO_BUTTON1;
  retval |= sam3u_gpioread(GPIO_BUTTON2) ? 0 : GPIO_BUTTON2;

  return retval;
}

/****************************************************************************
 * Name: up_irqbutton
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above. The previous interrupt
 *   handler address isreturned (so that it may restored, if so desired).
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.  For button support, bits 2 and 3 must be set in
 *   CONFIG_AVR32_GPIOIRQSETB (PB2 and PB3).
 *
 ****************************************************************************/

#if defined(CONFIG_GPIOA_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
xcpt_t up_irqbutton(int id, xcpt_t irqhandler)
{
  if (id == BUTTON1)
    {
      return up_irqbuttonx(IRQ_BUTTON1, irqhandler, &g_irqbutton1);
    }
  else if (id == BUTTON2)
    {
      return up_irqbuttonx(IRQ_BUTTON2, irqhandler, &g_irqbutton2);
    }
  else
    {
      return NULL;
    }
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
