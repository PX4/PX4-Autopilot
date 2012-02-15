/************************************************************************************
 * configs/stm32f4discovery/src/up_qencoder.c
 * arch/arm/src/board/up_qencoder.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32_qencoder.h"
#include "stm32f4discovery-internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
/* The following checks assum that the quadrature encoder is on TIM2.  Make the
 * appropriate changes if your configuration differes.
 */

#define HAVE_QENCODER 1

#ifndef CONFIG_QENCODER
#  undef HAVE_QENCODER
#endif

#ifndef CONFIG_STM32_TIM2
#  undef HAVE_QENCODER
#endif

#ifndef CONFIG_STM32_TIM2_QE
#  undef HAVE_QENCODER
#endif

#ifdef HAVE_QENCODER

/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing the quadrature encoder */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_QENCODER
#endif

#ifdef CONFIG_DEBUG_QENCODER
#  define qedbg                 dbg
#  define qelldbg               lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define qevdbg              vdbg
#    define qellvdbg            llvdbg
#  else
#    define qelldbg(x...)
#    define qellvdbg(x...)
#  endif
#else
#  define qedbg(x...)
#  define qelldbg(x...)
#  define qevdbg(x...)
#  define qellvdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: qe_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/qencoder.
 *
 ************************************************************************************/

int qe_devinit(void)
{
  static initialized = false;
  int ret;

  /* Check if we are already initialized */

  if (!initialized)
    {
      /* Initialize a quadrature encoder interface. */

      qevdbg("Initializing the quadrature encoder\n");
      ret = stm32_qeinitialize("/dev/qe0", 2);
      if (ret < 0)
        {
          qedbg("stm32_qeinitialize failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_QENCODER */
