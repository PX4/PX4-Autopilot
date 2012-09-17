/****************************************************************************
 * configs/olimex-strp711/src/up_leds.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* There are two LEDs are connected to  P1.8 & 9 */

#if defined(CONFIG_ARCH_LEDS) && !defined(CONFIG_STR71X_GPIO1)
#  error "LEDs require GPIO1"
#endif

#define STR71X_LED1GPIO1_BIT (0x0100)
#define STR71X_LED2GPIO1_BIT (0x0200)
#define STR71X_LEDGPIO1_BITS (STR71X_LED1GPIO1_BIT|STR71X_LED2GPIO1_BIT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_led2set;
static uint16_t g_led2clr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setleds
 ****************************************************************************/

static void up_setleds(uint16_t setbits, uint16_t clearbits)
{
  uint16_t reg16;

  /* Save the state of LED2 for later */

  g_led2set = setbits & STR71X_LED2GPIO1_BIT;
  g_led2clr = clearbits & STR71X_LED2GPIO1_BIT;

  /* Set and clear bits as directed */

  reg16  = getreg16(STR71X_GPIO1_PD);
  reg16 &= ~clearbits;
  reg16 |= setbits;
  putreg16(reg16, STR71X_GPIO1_PD);
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
  uint16_t reg16;

  /* Set normal function output */

  reg16  = getreg16(STR71X_GPIO1_PC0);
  reg16 |= STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PC0);

  reg16  = getreg16(STR71X_GPIO1_PC1);
  reg16 &= ~STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PC1);

  reg16  = getreg16(STR71X_GPIO1_PC2);
  reg16 |= STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PC2);

  /* Clear the LEDs (1 clears; 0 sets) */

  reg16  = getreg16(STR71X_GPIO1_PD);
  reg16 |= STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PD);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  /* The Olimex board has only two LEDs, so following states are faked as
   * follows
   *
   *                       SET         CLEAR
   *  LED_STARTED          (none)      (n/a)
   *  LED_HEAPALLOCATE     LED1        (n/a)
   *  LED_IRQSENABLED      LED1        (n/a)
   *  LED_STACKCREATED     LED1        (n/a)
   *  LED_INIRQ            LED1+LED2   LED1
   *  LED_SIGNAL           LED1+LED2   LED1
   *  LED_ASSERTION        LED1+LED2   LED1
   *  LED_PANIC            LED1+LED2*  LED1
   *
   *                      *The previous state of LED2 will be retained
   */

  switch (led)
    {
    default:
    case LED_STARTED:
      up_setleds(0, STR71X_LED1GPIO1_BIT|STR71X_LED2GPIO1_BIT); /* Clear LED1&2 */
      break;

    case LED_HEAPALLOCATE:
    case LED_IRQSENABLED:
    case LED_STACKCREATED:
      up_setleds(STR71X_LED1GPIO1_BIT, STR71X_LED2GPIO1_BIT); /* Set LED1, clear LED2 */
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
      up_setleds(STR71X_LED1GPIO1_BIT|STR71X_LED2GPIO1_BIT, 0); /* Set LED1&2 */
      break;

    case LED_PANIC:
      up_setleds(STR71X_LED2GPIO1_BIT|g_led2set, g_led2set); /* Set LED1, preserve LED2 */
      break;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  /* The Olimex board has only two LEDs, so following states are faked as
   * follows
   *
   *                       SET         CLEAR
   *  LED_STARTED          (none)      (n/a)
   *  LED_HEAPALLOCATE     LED1        (n/a)
   *  LED_IRQSENABLED      LED1        (n/a)
   *  LED_STACKCREATED     LED1        (n/a)
   *  LED_INIRQ            LED1+LED2   LED1
   *  LED_SIGNAL           LED1+LED2   LED1
   *  LED_ASSERTION        LED1+LED2   LED1
   *  LED_PANIC            LED1+LED2*  LED1
   *
   *                      *The previous state of LED2 will be retained
   */

  switch (led)
    {
    default:
    case LED_STARTED:
    case LED_HEAPALLOCATE:
    case LED_IRQSENABLED:
    case LED_STACKCREATED:
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
      up_setleds(STR71X_LED1GPIO1_BIT, STR71X_LED2GPIO1_BIT); /* Set LED1, clear LED2 */
      break;

    case LED_PANIC:
      up_setleds(g_led2set, STR71X_LED1GPIO1_BIT|g_led2clr); /* Clear LED1, preserve LED2 */
      break;
    }
}
#endif /* CONFIG_ARCH_LEDS */
