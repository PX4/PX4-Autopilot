/**************************************************************************
 * arch/avr/src/at90usb/at90usb_lowinit.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <avr/wdt.h>
#include <avr/power.h>

#include "at90usb_config.h"
#include "up_internal.h"
#include "at90usb_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

#if defined(CONFIG_WDTO_15MS)
#  define WDTO_VALUE WDTO_15MS   
#elif defined(CONFIG_WDTO_30MS)
#  define WDTO_VALUE WDTO_30MS
#elif defined(CONFIG_WDTO_60MS)
#  define WDTO_VALUE WDTO_60MS
#elif defined(CONFIG_WDTO_120MS)
#  define WDTO_VALUE WDTO_120MS
#elif defined(CONFIG_WDTO_1250MS)
#  define WDTO_VALUE WDTO_250MS
#elif defined(CONFIG_WDTO_500MS)
#  define WDTO_VALUE WDTO_500MS
#elif defined(CONFIG_WDTO_1S)
#  define WDTO_VALUE WDTO_1S
#elif defined(CONFIG_WDTO_2S)
#  define WDTO_VALUE WDTO_2S
#elif defined(CONFIG_WDTO_4S)
#  define WDTO_VALUE WDTO_4S
#else /* if defined(CONFIG_WDTO_8S) */
#  define WDTO_VALUE WDTO_8S
#endif

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_wdtinit
 *
 * Description:
 *   Initialize the watchdog per the NuttX configuration.
 *
 **************************************************************************/

static inline void up_wdtinit(void)
{
#ifdef CONFIG_AVR_WDT
  wdt_enable(WDTO_VALUE);
#endif
}

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowinit
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void up_lowinit(void)
{
  /* Disable the watchdog timer */

  wdt_disable();

  /* Set the system clock divider to 1 */

  clock_prescale_set(clock_div_1);

  /* Initialize the watchdog timer */

  up_wdtinit();

  /* Initialize a console (probably a serial console) */

  up_consoleinit();

  /* Perform early serial initialization (so that we will have debug output
   * available as soon as possible).
   */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Perform board-level initialization */

  up_boardinitialize();
}


