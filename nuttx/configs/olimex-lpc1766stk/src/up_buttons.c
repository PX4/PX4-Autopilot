/****************************************************************************
 * configs/olimex-lpc1766stk/src/up_buttons.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "lpc17_gpio.h"
#include "lpc1766stk_internal.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each STM3210E-EVAL button.  This array is indexed by
 * the BUTTON_* and JOYSTICK_* definitions in board.h
 */

static const uint16_t g_buttoncfg[BOARD_NUM_BUTTONS] =
{
  LPC1766STK_BUT1, LPC1766STK_BUT2, LPC1766STK_WAKEUP, LPC1766STK_CENTER,
  LPC1766STK_UP,   LPC1766STK_DOWN, LPC1766STK_LEFT,   LPC1766STK_RIGHT
};

/* This array defines all of the interupt handlers current attached to
 * button events.
 */

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
static xcpt_t g_buttonisr[BOARD_NUM_BUTTONS];

/* This array provides the mapping from button ID numbers to button IRQ
 * numbers.
 */

static uint8_t g_buttonirq[BOARD_NUM_BUTTONS] =
{
  LPC1766STK_BUT1_IRQ,   LPC1766STK_BUT2_IRQ, LPC1766STK_WAKEUP_IRQ,
  LPC1766STK_CENTER_IRQ, LPC1766STK_UP_IRQ,   LPC1766STK_DOWN_IRQ,
  LPC1766STK_LEFT_IRQ,   LPC1766STK_RIGHT_IRQ
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  int i;

  /* Configure the GPIO pins as interrupting inputs. */

  for (i = 0; i < BOARD_NUM_BUTTONS; i++)
    {
      lpc17_configgpio(g_buttoncfg[i]);
    }
}

/****************************************************************************
 * Name: up_buttons
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons.
 *
 *   up_buttons() may be called at any time to harvest the state of every
 *   button.  The state of the buttons is returned as a bitset with one
 *   bit corresponding to each button:  If the bit is set, then the button
 *   is pressed.  See the BOARD_BUTTON_*_BIT and BOARD_JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < BOARD_NUM_BUTTONS; i++)
    {
       /* A LOW value means that the key is pressed. */

       bool released = lpc17_gpioread(g_buttoncfg[i]);

       /* Accumulate the set of depressed (not released) keys */

       if (!released)
         {
            ret |= (1 << i);
         }
    }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_irqbutton() may be called to register button interrupt handlers.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BOARD_BUTTON_* and BOARD_JOYSTICK_* definitions in board.h for the meaning
 *   of enumeration values.  The previous interrupt handler address is returned
 *   (so that it may restored, if so desired).
 *
 *   Note that up_irqbutton() also enables button interrupts.  Button
 *   interrupts will remain enabled after the interrupt handler is attached.
 *   Interrupts may be disabled (and detached) by calling up_irqbutton with
 *   irqhandler equal to NULL.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
xcpt_t up_irqbutton(int id, xcpt_t irqhandler)
{
  xcpt_t oldhandler = NULL;
  irqstate_t flags;
  int irq;

  /* Verify that the button ID is within range */

  if ((unsigned)id < BOARD_NUM_BUTTONS)
    {
      /* Return the current button handler and set the new interrupt handler */

      oldhandler      = g_buttonisr[id];
      g_buttonisr[id] = irqhandler;

      /* Disable interrupts until we are done */

      flags = irqsave();

      /* Configure the interrupt.  Either attach and enable the new
       * interrupt or disable and detach the old interrupt handler.
       */

      irq = g_buttonirq[id];
      if (irqhandler)
        {
          /* Attach then enable the new interrupt handler */

          (void)irq_attach(irq, irqhandler);
          up_enable_irq(irq);
        }
      else
        {
          /* Disable then then detach the the old interrupt handler */

          up_disable_irq(irq);
          (void)irq_detach(irq);
        }
      irqrestore(flags);
    }
  return oldhandler;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
