/****************************************************************************
 * configs/hymini-stm32v/src/up_buttons.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>
#include "hymini_stm32v-internal.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each HY-mini button.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */

//static const uint16_t g_buttons[NUM_BUTTONS] =
//  {
//    GPIO_BTN_KEYA, GPIO_BTN_KEYB
//  };

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
    stm32_configgpio(GPIO_BTN_KEYA);
    stm32_configgpio(GPIO_BTN_KEYB);
  }

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
  {
    uint8_t ret = 0;
    bool pinValue;

    /* Check that state of each key */

    /* Pin is pulled up */
    pinValue = stm32_gpioread(GPIO_BTN_KEYA);
    if (!pinValue)
      {
        // Button pressed
        ret = 1 << BUTTON_KEYA;
      }

    /* Pin is pulled down */
    pinValue = stm32_gpioread(GPIO_BTN_KEYB);
    if (pinValue)
      {
        // Button pressed
        ret |= 1 << BUTTON_KEYB;
      }
    return ret;
  }

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 *   After up_buttoninit() has been called, up_buttons() may be called to
 *   collect the state of all buttons.  up_buttons() returns an 8-bit bit set
 *   with each bit associated with a button.  See the BUTTON_*_BIT and JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
xcpt_t up_irqbutton(int id, xcpt_t irqhandler)
  {
    xcpt_t oldhandler = NULL;
    uint32_t pinset = GPIO_BTN_KEYA;

    if (id == 1)
      {
        pinset = GPIO_BTN_KEYB;
      }
    if (id < 2)
      {
        oldhandler = stm32_gpiosetevent(pinset, true, true, true, irqhandler);
      }
    return oldhandler;
  }
#endif
#endif /* CONFIG_ARCH_BUTTONS */
