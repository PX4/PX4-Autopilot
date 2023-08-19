/****************************************************************************
 * boards/arm/stm32h7/stm32h747i-disco/src/stm32_autoleds.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32h747i-disco.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x) (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Indexed by BOARD_LED_<color> */

static const uint32_t g_ledmap[BOARD_NLEDS] =
{
  GPIO_LED_GREEN,
  GPIO_LED_ORANGE,
  GPIO_LED_RED,
  GPIO_LED_BLUE,
};

static bool g_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void phy_set_led(int led, bool state)
{
  /* Active Low */

  stm32_gpiowrite(g_ledmap[led], !state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  size_t i;

  /* Configure the LD1 GPIO for output. Initial state is OFF */

  for (i = 0; i < ARRAYSIZE(g_ledmap); i++)
    {
      stm32_configgpio(g_ledmap[i]);
    }
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
    default:
      break;

    case LED_HEAPALLOCATE:
      phy_set_led(BOARD_LED_BLUE, true);
      break;

    case LED_IRQSENABLED:
      phy_set_led(BOARD_LED_BLUE, false);
      phy_set_led(BOARD_LED_GREEN, true);
      break;

    case LED_STACKCREATED:
      phy_set_led(BOARD_LED_GREEN, true);
      phy_set_led(BOARD_LED_BLUE, true);
      g_initialized = true;
      break;

    case LED_INIRQ:
      phy_set_led(BOARD_LED_BLUE, true);
      break;

    case LED_SIGNAL:
      phy_set_led(BOARD_LED_GREEN, true);
      break;

    case LED_ASSERTION:
      phy_set_led(BOARD_LED_RED, true);
      phy_set_led(BOARD_LED_BLUE, true);
      break;

    case LED_PANIC:
      phy_set_led(BOARD_LED_RED, true);
      break;

    case LED_IDLE : /* IDLE */
      phy_set_led(BOARD_LED_RED, true);
    break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    default:
      break;

    case LED_SIGNAL:
      phy_set_led(BOARD_LED_GREEN, false);
      break;

    case LED_INIRQ:
      phy_set_led(BOARD_LED_BLUE, false);
      break;

    case LED_ASSERTION:
      phy_set_led(BOARD_LED_RED, false);
      phy_set_led(BOARD_LED_BLUE, false);
      break;

    case LED_PANIC:
      phy_set_led(BOARD_LED_RED, false);
      break;

    case LED_IDLE : /* IDLE */
      phy_set_led(BOARD_LED_RED, false);
    break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
