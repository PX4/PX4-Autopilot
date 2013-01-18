/************************************************************************************
 * configs/ea3152/src/up_boot.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "lpc31_internal.h"
#include "ea3152_internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc31_boardinitialize
 *
 * Description:
 *   All LPC31XX architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void lpc31_boardinitialize(void)
{
  /* Initialize configured, external memory resources */

#ifdef CONFIG_ARCH_EXTDRAM
  lpc31_meminitialize();
#endif

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
   * lpc31_spiinitialize() has been brought into the link.
   */

#if defined(CONFIG_LPC31_SPI)
  if (lpc31_spiinitialize)
    {
      lpc31_spiinitialize();
    }
#endif

   /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
    * disabled, and 3) the weak function lpc31_usbinitialize() has been brought
    * into the build.
    */

#if defined(CONFIG_USBDEV) && defined(CONFIG_LPC31_USB)
  if (lpc31_usbinitialize)
    {
      lpc31_usbinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  up_ledinit();
#endif

  /* Set up mass storage device to support on demand paging */

#if defined(CONFIG_PAGING)
  if (lpc31_pginitialize)
    {
      lpc31_pginitialize();
    }
#endif
}
