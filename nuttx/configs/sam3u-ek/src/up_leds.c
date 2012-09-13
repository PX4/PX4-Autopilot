/****************************************************************************
 * configs/sam3u-ek/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
#include "sam3u_internal.h"
#include "sam3uek_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef LED_DEBUG  /* Define to enable debug */

#ifdef LED_DEBUG
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

#define LED_OFF        0
#define LED_ON         1
#define LED_NOCHANGE   2
#define LED_MASK       3

#define LED0_SHIFT     0
#define LED0_OFF       (LED_OFF << LED0_SHIFT)
#define LED0_ON        (LED_ON << LED0_SHIFT)
#define LED0_NOCHANGE  (LED_NOCHANGE << LED0_SHIFT)
#define LED1_SHIFT     2
#define LED1_OFF       (LED_OFF << LED1_SHIFT)
#define LED1_ON        (LED_ON << LED1_SHIFT)
#define LED1_NOCHANGE  (LED_NOCHANGE << LED1_SHIFT)
#define LED2_SHIFT     4
#define LED2_OFF       (LED_OFF << LED2_SHIFT)
#define LED2_ON        (LED_ON << LED2_SHIFT)
#define LED2_NOCHANGE  (LED_NOCHANGE << LED2_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_ledon[8] =
{
	
  (LED0_OFF     |LED1_OFF     |LED2_OFF),      /* LED_STARTED  */
  (LED0_ON      |LED1_OFF     |LED2_ON),       /* LED_HEAPALLOCATE */
  (LED0_OFF     |LED1_ON      |LED2_OFF),      /* LED_IRQSENABLED  */
  (LED0_ON      |LED1_ON      |LED2_ON),       /* LED_STACKCREATED  */

  (LED0_NOCHANGE|LED1_OFF     |LED2_NOCHANGE), /* LED_INIRQ  */
  (LED0_NOCHANGE|LED1_NOCHANGE|LED2_OFF),      /* LED_SIGNAL  */
  (LED0_ON      |LED1_NOCHANGE|LED2_NOCHANGE), /* LED_ASSERTION  */
  (LED0_ON      |LED1_NOCHANGE|LED2_NOCHANGE)  /* LED_PANIC */
};

static const uint8_t g_ledoff[8] =
{
	
  (LED0_OFF     |LED1_OFF     |LED2_OFF),      /* LED_STARTED (does not happen) */
  (LED0_ON      |LED1_OFF     |LED2_ON),       /* LED_HEAPALLOCATE (does not happen) */
  (LED0_OFF     |LED1_ON      |LED2_OFF),      /* LED_IRQSENABLED (does not happen) */
  (LED0_ON      |LED1_ON      |LED2_ON),       /* LED_STACKCREATED (does not happen) */

  (LED0_NOCHANGE|LED1_ON      |LED2_NOCHANGE), /* LED_INIRQ  */
  (LED0_NOCHANGE|LED1_NOCHANGE|LED2_ON),       /* LED_SIGNAL */
  (LED0_OFF     |LED1_NOCHANGE|LED2_NOCHANGE), /* LED_ASSERTION */
  (LED0_OFF     |LED1_NOCHANGE|LED2_NOCHANGE)  /* LED_PANIC */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setled
 ****************************************************************************/

static void up_setled(uint16_t pinset, uint8_t state)
{
  /* Assume active high.  Initial state == 0 means active high */

  bool polarity = ((pinset & GPIO_OUTPUT_SET) == 0);
  switch (state)
    {
      case LED_OFF:
        polarity = !polarity;

      case LED_ON:
        break;

      case LED_NOCHANGE:
      default:
        return;
    }
  sam3u_gpiowrite(pinset, polarity);
}

/****************************************************************************
 * Name: up_setleds
 ****************************************************************************/

static void up_setleds(uint8_t state)
{
  up_setled(GPIO_LED0, (state >> LED0_SHIFT) & LED_MASK);
  up_setled(GPIO_LED1, (state >> LED1_SHIFT) & LED_MASK);
  up_setled(GPIO_LED2, (state >> LED2_SHIFT) & LED_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

void up_ledinit(void)
{
  (void)sam3u_configgpio(GPIO_LED0);
  (void)sam3u_configgpio(GPIO_LED1);
  (void)sam3u_configgpio(GPIO_LED2);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  up_setleds(g_ledon[led & 7]);
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  up_setleds(g_ledoff[led & 7]);
}

#endif /* CONFIG_ARCH_LEDS */
