/************************************************************************************
 * configs/cloudctrl/src/up_watchdog.c
 * arch/arm/src/board/up_watchdog.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Darcy Gong <darcy.gong@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/watchdog.h>
#include <arch/board/board.h>

#include "stm32_wdg.h"

#ifdef CONFIG_WATCHDOG

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
/* Wathdog hardware should be enabled */

#if !defined(CONFIG_STM32_WWDG) && !defined(CONFIG_STM32_IWDG)
#  warning "One of CONFIG_STM32_WWDG or CONFIG_STM32_IWDG must be defined"
#endif

/* Select the path to the registered watchdog timer device */

#ifndef CONFIG_STM32_WDG_DEVPATH
#  ifdef CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#    define CONFIG_STM32_WDG_DEVPATH CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#  else
#    define CONFIG_STM32_WDG_DEVPATH "/dev/watchdog0"
#  endif
#endif

/* Use the un-calibrated LSI frequency if we have nothing better */

#if defined(CONFIG_STM32_IWDG) && !defined(CONFIG_STM32_LSIFREQ)
#  define CONFIG_STM32_LSIFREQ STM32_LSI_FREQUENCY
#endif

/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog timer */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_WATCHDOG
#endif

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wdgdbg                 dbg
#  define wdglldbg               lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define wdgvdbg              vdbg
#    define wdgllvdbg            llvdbg
#  else
#    define wdgvdbg(x...)
#    define wdgllvdbg(x...)
#  endif
#else
#  define wdgdbg(x...)
#  define wdglldbg(x...)
#  define wdgvdbg(x...)
#  define wdgllvdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_wdginitialize()
 *
 * Description:
 *   Perform architecuture-specific initialization of the Watchdog hardware.
 *   This interface must be provided by all configurations using
 *   apps/examples/watchdog
 *
 ****************************************************************************/

int up_wdginitialize(void)
{
  /* Initialize tha register the watchdog timer device */

#if defined(CONFIG_STM32_WWDG)
  stm32_wwdginitialize(CONFIG_STM32_WDG_DEVPATH);
  return OK;
#elif defined(CONFIG_STM32_IWDG)
  stm32_iwdginitialize(CONFIG_STM32_WDG_DEVPATH, CONFIG_STM32_LSIFREQ);
  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_WATCHDOG */
