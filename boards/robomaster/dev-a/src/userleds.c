/****************************************************************************
 * boards/arm/stm32/stm3240g-eval/src/stm32_userleds.c
 *
 *   Copyright (C) 2011, 2013, 2015 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "board_config.h"



/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This array maps an LED number to GPIO pin configuration */

// User LEDs
#define GPIO_LED_A		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN1)
#define GPIO_LED_B		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN2)
#define GPIO_LED_C		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN3)
#define GPIO_LED_D		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_LED_E		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN5)
#define GPIO_LED_F		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN6)
#define GPIO_LED_G		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN7)
#define GPIO_LED_H		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN8)

#define BOARD_LED_A_BIT 1 << BOARD_LED_A
#define BOARD_LED_B_BIT 1 << BOARD_LED_B
#define BOARD_LED_C_BIT 1 << BOARD_LED_C
#define BOARD_LED_D_BIT 1 << BOARD_LED_D
#define BOARD_LED_E_BIT 1 << BOARD_LED_E
#define BOARD_LED_F_BIT 1 << BOARD_LED_F
#define BOARD_LED_G_BIT 1 << BOARD_LED_G
#define BOARD_LED_H_BIT 1 << BOARD_LED_H


static uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED_A, GPIO_LED_B, GPIO_LED_C, GPIO_LED_D, GPIO_LED_E, GPIO_LED_F, GPIO_LED_G, GPIO_LED_H
};




/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

void board_userled_initialize(void)
{
  /* Configure LED1-8 GPIOs for output */

  stm32_configgpio(GPIO_LED_A);
  stm32_configgpio(GPIO_LED_B);
  stm32_configgpio(GPIO_LED_C);
  stm32_configgpio(GPIO_LED_D);
  stm32_configgpio(GPIO_LED_E);
  stm32_configgpio(GPIO_LED_F);
  stm32_configgpio(GPIO_LED_G);
  stm32_configgpio(GPIO_LED_H);
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  syslog(LOG_INFO, "board_userled %d\n", led);
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint8_t ledset)
{
  syslog(LOG_INFO, "board_userled_all %d\n", ledset);
  stm32_gpiowrite(GPIO_LED_A, (ledset & BOARD_LED_A_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_B, (ledset & BOARD_LED_B_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_C, (ledset & BOARD_LED_C_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_D, (ledset & BOARD_LED_D_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_E, (ledset & BOARD_LED_E_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_F, (ledset & BOARD_LED_F_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_G, (ledset & BOARD_LED_G_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_H, (ledset & BOARD_LED_H_BIT) == 0);
}
