///****************************************************************************
// * boards/arm/rp23xx/raspberrypi-pico-2/src/rp23xx_buttons.c
// *
// * Licensed to the Apache Software Foundation (ASF) under one or more
// * contributor license agreements.  See the NOTICE file distributed with
// * this work for additional information regarding copyright ownership.  The
// * ASF licenses this file to you under the Apache License, Version 2.0 (the
// * "License"); you may not use this file except in compliance with the
// * License.  You may obtain a copy of the License at
// *
// *   http://www.apache.org/licenses/LICENSE-2.0
// *
// * Unless required by applicable law or agreed to in writing, software
// * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
// * License for the specific language governing permissions and limitations
// * under the License.
// *
// ****************************************************************************/
//
///****************************************************************************
// * Included Files
// ****************************************************************************/
//
//#include <nuttx/config.h>
//
//#include <stdint.h>
//#include <errno.h>
//
//#include <nuttx/arch.h>
//#include <nuttx/board.h>
//#include <arch/board/board.h>
//
//#include "rp23xx_gpio.h"
//#include "rp23xx_pico.h"
//
//#if defined(CONFIG_ARCH_BUTTONS)
//
///****************************************************************************
// * Pre-processor Definitions
// ****************************************************************************/
//
//#if defined(CONFIG_INPUT_BUTTONS) && !defined(CONFIG_ARCH_IRQBUTTONS)
//#  error "The NuttX Buttons Driver depends on IRQ support to work!\n"
//#endif
//
///****************************************************************************
// * Private Functions
// ****************************************************************************/
//
///* Pin configuration for external raspberrypi-pico-2 buttons. */
//
//static const uint32_t g_buttons[NUM_BUTTONS] =
//{
//  GPIO_BTN_USER1, GPIO_BTN_USER2
//};
//
///****************************************************************************
// * Public Functions
// ****************************************************************************/
//
///****************************************************************************
// * Name: board_button_initialize
// *
// * Description:
// *   board_button_initialize() must be called to initialize button resources.
// *   After that, board_buttons() may be called to collect the current state
// *   of all buttons or board_button_irq() may be called to register button
// *   interrupt handlers.
// *
// ****************************************************************************/
//
//uint32_t board_button_initialize(void)
//{
//  int i;
//
//  /* Configure the GPIO pins as inputs. And we will use interrupts */
//
//  for (i = 0; i < NUM_BUTTONS; i++)
//    {
//      /* Initialize input pin */
//
//      rp23xx_gpio_init(g_buttons[i]);
//
//      /* pull-up = false : pull-down = false */
//
//      rp23xx_gpio_set_pulls(g_buttons[i], false, false);
//    }
//
//  return NUM_BUTTONS;
//}
//
///****************************************************************************
// * Name: board_buttons
// ****************************************************************************/
//
//uint32_t board_buttons(void)
//{
//  uint32_t ret = 0;
//  int i;
//
//  /* Check that state of each key */
//
//  for (i = 0; i < NUM_BUTTONS; i++)
//    {
//      /* A LOW value means that the key is pressed. */
//
//      bool released = rp23xx_gpio_get(g_buttons[i]);
//
//      /* Accumulate the set of depressed (not released) keys */
//
//      if (!released)
//        {
//           ret |= (1 << i);
//        }
//    }
//
//  return ret;
//}
//
///****************************************************************************
// * Button support.
// *
// * Description:
// *   board_button_initialize() must be called to initialize button resources.
// *   After that, board_buttons() may be called to collect the current state
// *   of all buttons or board_button_irq() may be called to register button
// *   interrupt handlers.
// *
// *   After board_button_initialize() has been called, board_buttons() may be
// *   called to collect the state of all buttons.  board_buttons() returns
// *   an 32-bit bit set with each bit associated with a button.  See the
// *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
// *
// *   board_button_irq() may be called to register an interrupt handler that
// *   will be called when a button is depressed or released.  The ID value is
// *   a button enumeration value that uniquely identifies a button resource.
// *   See the BUTTON_* definitions in board.h for the meaning of enumeration
// *   value.
// *
// ****************************************************************************/
//
//#ifdef CONFIG_ARCH_IRQBUTTONS
//int board_button_irq(int id, xcpt_t irqhandler, void *arg)
//{
//  int ret = -EINVAL;
//
//  /* The following should be atomic */
//
//  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
//    {
//      /* Make sure the interrupt is disabled */
//
//      rp23xx_gpio_disable_irq(g_buttons[id]);
//
//      /* Attach the interrupt handler */
//
//      ret = rp23xx_gpio_irq_attach(g_buttons[id],
//                                   RP23XX_GPIO_INTR_EDGE_LOW,
//                                   irqhandler,
//                                   arg);
//      if (ret < 0)
//        {
//          syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
//          return ret;
//        }
//
//      /* Enable interruption for this pin */
//
//      rp23xx_gpio_enable_irq(g_buttons[id]);
//    }
//
//  return ret;
//}
//#endif
//
//#endif /* CONFIG_ARCH_BUTTONS */
