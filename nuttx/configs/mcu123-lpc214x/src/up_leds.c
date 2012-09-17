/****************************************************************************
 * configs/mcu123-lpc214x/src/up_leds.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* P1.16-P1.23 control LEDS 1-8 */

#define LEDBIT(led)     (0x00010000 << (led))
#define ALLLEDS         (0x00ff0000)

#ifdef CONFIG_LPC214x_FIO
#  define putled(v,r)    putreg32((v),(LPC214X_FIO1_BASE+(r)))
#  define CLRLEDS        putled(ALLLEDS,LPC214X_FIO_SET_OFFSET)

#  define LED_SET_OFFSET LPC214X_FIO_SET_OFFSET
#  define LED_CLR_OFFSET LPC214X_FIO_CLR_OFFSET
#  define LED_DIR_OFFSET LPC214X_FIO_DIR_OFFSET

#else
#  define putled(v,r)    putreg32((v),(LPC214X_GPIO1_BASE+(r)))
#  define CLRLEDS        putled(ALLLEDS,LPC214X_GPIO_SET_OFFSET)

#  define LED_SET_OFFSET LPC214X_GPIO_SET_OFFSET
#  define LED_CLR_OFFSET LPC214X_GPIO_CLR_OFFSET
#  define LED_DIR_OFFSET LPC214X_GPIO_DIR_OFFSET
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  /* Initilize GIOs P1.16-P1.23 */

  putled(ALLLEDS,LED_DIR_OFFSET);
  putled(ALLLEDS,LED_SET_OFFSET);
  putled(LEDBIT(0),LED_CLR_OFFSET);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  putled(LEDBIT(led),LED_CLR_OFFSET);
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  putled(LEDBIT(led),LED_SET_OFFSET);
}
#endif /* CONFIG_ARCH_LEDS */
