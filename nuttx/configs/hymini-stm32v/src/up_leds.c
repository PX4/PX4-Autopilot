/****************************************************************************
 * configs/hymini-stm32v/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Laurent Latil <laurent@latil.nom.fr>
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
#include "stm32_internal.h"
#include "hymini_stm32v-internal.h"

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

/* The following definitions map the encoded LED setting to GPIO settings */

#define HYMINI_STM32_LED1     (1 << 0)
#define HYMINI_STM32_LED2     (1 << 1)

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (4)
#define OFF_SETBITS_SHIFT (8)
#define OFF_CLRBITS_SHIFT (12)

#define ON_BITS(v)        ((v) & 0xff)
#define OFF_BITS(v)       (((v) >> 8) & 0x0ff)
#define SETBITS(b)        ((b) & 0x0f)
#define CLRBITS(b)        (((b) >> 4) & 0x0f)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

/* On: !LED1 + !LED2   Off: - */
#define LED_STARTED_ON_SETBITS       ((0) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((HYMINI_STM32_LED1|HYMINI_STM32_LED2) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      (0 << OFF_CLRBITS_SHIFT)

/* On: LED1+!LED2   Off: N/A */
#define LED_HEAPALLOCATE_ON_SETBITS  ((HYMINI_STM32_LED1) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((HYMINI_STM32_LED2) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS (0)
#define LED_HEAPALLOCATE_OFF_CLRBITS (0)

/* On: LED2+!LED1   Off: N/A */
#define LED_IRQSENABLED_ON_SETBITS   ((HYMINI_STM32_LED2) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((HYMINI_STM32_LED1) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  (0)
#define LED_IRQSENABLED_OFF_CLRBITS  (0)

/* On: LED1+!LED2   Off: N/A */
#define LED_STACKCREATED_ON_SETBITS  ((HYMINI_STM32_LED1) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((HYMINI_STM32_LED2) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS (0)
#define LED_STACKCREATED_OFF_CLRBITS (0)

/* On: !LED1   Off: LED1 */
#define LED_INIRQ_ON_SETBITS         ((0) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((HYMINI_STM32_LED1) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((HYMINI_STM32_LED1) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((0) << OFF_CLRBITS_SHIFT)

/* On: LED2   Off: !LED2 */
#define LED_SIGNAL_ON_SETBITS        ((HYMINI_STM32_LED2) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((0) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((0) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((HYMINI_STM32_LED2) << OFF_CLRBITS_SHIFT)

/* On: LED1+LED2   Off: - */
#define LED_ASSERTION_ON_SETBITS     ((HYMINI_STM32_LED2|HYMINI_STM32_LED2) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((0) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((0) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((0) << OFF_CLRBITS_SHIFT)

/* On: LED1   Off: LED2 */
#define LED_PANIC_ON_SETBITS         ((HYMINI_STM32_LED1) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((HYMINI_STM32_LED2) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((HYMINI_STM32_LED2) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((HYMINI_STM32_LED1) << OFF_CLRBITS_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{ (LED_STARTED_ON_SETBITS | LED_STARTED_ON_CLRBITS | LED_STARTED_OFF_SETBITS
    | LED_STARTED_OFF_CLRBITS),

(LED_HEAPALLOCATE_ON_SETBITS | LED_HEAPALLOCATE_ON_CLRBITS
    | LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

(LED_IRQSENABLED_ON_SETBITS | LED_IRQSENABLED_ON_CLRBITS
    | LED_IRQSENABLED_OFF_SETBITS | LED_IRQSENABLED_OFF_CLRBITS),

(LED_STACKCREATED_ON_SETBITS | LED_STACKCREATED_ON_CLRBITS
    | LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

(LED_INIRQ_ON_SETBITS | LED_INIRQ_ON_CLRBITS | LED_INIRQ_OFF_SETBITS
    | LED_INIRQ_OFF_CLRBITS),

(LED_SIGNAL_ON_SETBITS | LED_SIGNAL_ON_CLRBITS | LED_SIGNAL_OFF_SETBITS
    | LED_SIGNAL_OFF_CLRBITS),

(LED_ASSERTION_ON_SETBITS | LED_ASSERTION_ON_CLRBITS | LED_ASSERTION_OFF_SETBITS
    | LED_ASSERTION_OFF_CLRBITS),

(LED_PANIC_ON_SETBITS | LED_PANIC_ON_CLRBITS | LED_PANIC_OFF_SETBITS
    | LED_PANIC_OFF_CLRBITS) };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & HYMINI_STM32_LED1) != 0)
  {
    stm32_gpiowrite(GPIO_LED1, false);
  }

  if ((clrbits & HYMINI_STM32_LED2) != 0)
  {
    stm32_gpiowrite(GPIO_LED2, false);
  }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & HYMINI_STM32_LED1) != 0)
  {
    stm32_gpiowrite(GPIO_LED1, true);
  }

  if ((setbits & HYMINI_STM32_LED2) != 0)
  {
    stm32_gpiowrite(GPIO_LED2, true);
  }
}

static void led_setonoff(unsigned int bits)
{
  led_clrbits(CLRBITS(bits));
  led_setbits(SETBITS(bits));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  /* Configure LED1 & LED2 GPIOs for output */
  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}

#endif /* CONFIG_ARCH_LEDS */
