/************************************************************************************
 * configs/scp16c26/src/up_leds.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <arch/board/board.h>
#include "up_arch.h"
#include "up_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_LEDS

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/* The SKP62C26 has 3 LEDs control by bits 0 and 2 in port 7 and bit 0 in port 8. */

#define GREEN_LED            (1 << 2)   /* Bit 2, port 7 */
#define YELLOW_LED           (1 << 4)   /* Bit 4, port 7 */
#define RED_LED              (1 << 0)   /* Bit 0, port 8 */

#define GREEN_LED_ON         0
#define GREEN_LED_OFF        GREEN_LED
#define GREEN_LED_MASK       GREEN_LED
#define GREEN_LED_PORT       M16C_P7

#define YELLOW_LED_ON        0
#define YELLOW_LED_OFF       YELLOW_LED
#define YELLOW_LED_MASK      YELLOW_LED
#define YELLOW_LED_PORT      M16C_P7

#define GREENYELLOW_LED_MASK (GREEN_LED_MASK|YELLOW_LED_MASK)
#define GREENYELLOW_LED_PORT M16C_P7
#define GREENYELLOW_DIR_PORT M16C_PD7

#define RED_LED_ON           0
#define RED_LED_OFF          RED_LED
#define RED_LED_MASK         RED_LED
#define RED_LED_PORT         M16C_P8
#define RED_DIR_PORT         M16C_PD8

/************************************************************************************
 * Private Type Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Data Definitions
 ************************************************************************************/

static const uint8_t g_ledstate[7] =
{
  (GREEN_LED_OFF | YELLOW_LED_OFF | RED_LED_OFF), /* LED_STARTED */
  (GREEN_LED_ON  | YELLOW_LED_OFF | RED_LED_OFF), /* LED_HEAPALLOCATE */
  (GREEN_LED_OFF | YELLOW_LED_ON  | RED_LED_OFF), /* LED_IRQSENABLED */
  (GREEN_LED_ON  | YELLOW_LED_ON  | RED_LED_OFF), /* LED_STACKCREATED */

  (GREEN_LED_ON  | YELLOW_LED_OFF | RED_LED_ON ), /* LED_INIRQ */
  (GREEN_LED_OFF | YELLOW_LED_ON  | RED_LED_ON ), /* LED_SIGNAL */
  (GREEN_LED_ON  | YELLOW_LED_ON  | RED_LED_ON )  /* LED_ASSERTION */
};

static uint8_t g_prevled[3];
static uint8_t g_nestlevel;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_ledinit
 ************************************************************************************/

static void up_setleds(uint8_t gybits, uint8_t rbit)
{
  uint8_t regval;

  regval  = getreg8(GREENYELLOW_LED_PORT);
  regval &= ~GREENYELLOW_LED_MASK;
  regval |= gybits;
  putreg8(regval, GREENYELLOW_LED_PORT);
  
  regval  = getreg8(RED_LED_PORT);
  regval &= ~RED_LED_MASK;
  regval |= rbit;
  putreg8(regval, RED_LED_PORT);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_ledinit
 ************************************************************************************/

void up_ledinit(void)
{
  register uint8_t regval;

  /* Make sure that the LEDs are in the OFF state */

  regval  = getreg8(GREENYELLOW_LED_PORT);
  regval |= (GREEN_LED_OFF |YELLOW_LED_OFF);
  putreg8(regval, GREENYELLOW_LED_PORT);
  
  regval  = getreg8(RED_LED_PORT);
  regval |=  RED_LED_OFF;
  putreg8(regval, RED_LED_PORT);

  /* Set the direction to output */
  
  regval  = getreg8(GREENYELLOW_DIR_PORT);
  regval |= (GREEN_LED |YELLOW_LED);
  putreg8(regval, GREENYELLOW_DIR_PORT);
  
  regval  = getreg8(RED_DIR_PORT);
  regval |=  RED_LED;
  putreg8(regval, RED_DIR_PORT);
}

/************************************************************************************
 * Name: up_ledon
 ************************************************************************************/

void up_ledon(int led)
{
  uint8_t ledset;

  /* If this is the ASSERTION led, preserve the Y&G bits from the last setting and
   * set the RED LED on.
   */

  if (led == LED_ASSERTION)
    {
      ledset = g_ledstate[g_prevled[g_nestlevel]];
      up_setleds(ledset & GREENYELLOW_LED_MASK, RED_LED_ON);
    }
  else if (led < LED_ASSERTION)
    {
      /* Otherwise, just show the LEDs corresponding to this state */

      ledset = g_ledstate[led];
      up_setleds(ledset & GREENYELLOW_LED_MASK, ledset & RED_LED_MASK);

      /* If this was a nested states (INIRQ, SIGNAL, or ASSERTION) then
       * stack up the previous value.
       */

      if (led > LED_STACKCREATED)
        {
          g_nestlevel++;
        }
      g_prevled[g_nestlevel] = led;
    }
}

/************************************************************************************
 * Name: up_ledoff
 ************************************************************************************/

void up_ledoff(int led)
{
  uint8_t ledset;

  /* If this is the ASSERTION led then what we do depends on the previous state */

  if (led == LED_ASSERTION)
    {
      /* If the previous state was one of the nested states (INIRQ, SIGNAL, or ASSERTION),
       * then turn the green and yellow LEDs all off.  That way we can distinguish
       * that case from the simple cases.
       */

      if (g_nestlevel > 0)
        {
          ledset = 0;
        }
      else
        {
          ledset = g_ledstate[g_prevled[0]];
        }
      up_setleds(ledset & GREENYELLOW_LED_MASK, RED_LED_OFF);
    }
  else if (led > 0 && led < LED_ASSERTION)
    {
      /* If this was one of the nested states, then we want to back to the LED setting
       * before entering that nested statel.
       */

      if (g_nestlevel > 0)
        {
          g_nestlevel--;
          led = g_prevled[g_nestlevel];
        }
      else if (led > LED_STACKCREATED)
        {
	  /* This shouldn't happen */

          led--;
        }
      ledset = g_ledstate[led];
      up_setleds(ledset & GREENYELLOW_LED_MASK, ledset & RED_LED_MASK);
      g_prevled[g_nestlevel]= led;
    }
}

#endif /* CONFIG_ARCH_LEDS */
