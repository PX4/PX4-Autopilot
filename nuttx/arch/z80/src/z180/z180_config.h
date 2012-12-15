/************************************************************************************
 * arch/z80/src/z180/z180_config.h
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

#ifndef __ARCH_Z80_SRC_Z180_Z180_CONFIG_H
#define __ARCH_Z80_SRC_Z180_Z180_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/z180/chip.h>

#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Verify that selected features match the capability of the selected CPU */

#ifndef HAVE_Z8X181
#  undef CONFIG_Z180_SCC
#  undef CONFIG_Z180_CTC
#endif

#ifndef HAVE_Z8X182
#  undef CONFIG_Z180_ESCCA
#  undef CONFIG_Z180_ESCCB
#  undef CONFIG_Z180_PORTC
#  undef CONFIG_Z180_MIMIC
#endif

#if !defined(HAVE_Z8X181) && !defined(HAVE_Z8X182)
#  undef CONFIG_Z180_PORTA
#  undef CONFIG_Z180_PORTB
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART
#undef HAVE_SCC
#undef HAVE_SERIAL

#if defined(CONFIG_Z180_UART0) || defined(CONFIG_Z180_UART1)
#  define HAVE_UART   1
#  define HAVE_SERIAL 1
#endif

#if defined(CONFIG_Z180_SCC) || defined(CONFIG_Z180_ESCCA) || \
    defined(CONFIG_Z180_ESCCB)
#  define HAVE_SCC    1
#  define HAVE_SERIAL 1
#endif

/* Make sure all features are disabled for disabled UARTs/[E]SCC channels.  This
 * simplifies checking later.
 */

#ifndef CONFIG_Z180_UART0
#  undef CONFIG_Z180_UART0_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_UART1
#  undef CONFIG_Z180_UART1_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_SCC
#  undef CONFIG_SCC_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_ESCCA
#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z180_ESCCB
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE
#endif

/* Is there a serial console? There should be at most one defined. */

#undef HAVE_UART_CONSOLE
#undef HAVE_SCC_CONSOLE
#undef HAVE_SERIAL_CONSOLE

#if defined(CONFIG_Z180_UART0_SERIAL_CONSOLE)
#  define HAVE_UART_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

  /* Disable other console selections */

#  undef CONFIG_Z180_UART1_SERIAL_CONSOLE
#  undef CONFIG_Z180_SCC_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

   /* If we are not using the serial driver, then the serial console is all
    * that we will support.
    */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCA
#    undef CONFIG_Z180_ESCCB
#  endif

#elif defined(CONFIG_Z180_UART1_SERIAL_CONSOLE)
#  define HAVE_UART_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

  /* Disable other console selections */

#  undef CONFIG_Z180_SCC_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

   /* If we are not using the serial driver, then the serial console is all
    * that we will support.
    */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCA
#    undef CONFIG_Z180_ESCCB
#  endif

#elif defined(CONFIG_Z180_SCC_SERIAL_CONSOLE)
#  define HAVE_SCC_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

  /* Disable other console selections */

#  undef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

   /* If we are not using the serial driver, then the serial console is all
    * that we will support.
    */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_ESCCA
#    undef CONFIG_Z180_ESCCB
#  endif

#elif defined(CONFIG_Z180_ESCCA_SERIAL_CONSOLE)
#  define HAVE_SCC_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

  /* Disable other console selections */

#  undef CONFIG_Z180_ESCCB_SERIAL_CONSOLE

   /* If we are not using the serial driver, then the serial console is all
    * that we will support.
    */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCB
#  endif

   /* If we are not using the serial driver, then the serial console is all
    * that we will support.
    */

#elif defined(CONFIG_Z180_ESCCB_SERIAL_CONSOLE)
#  define HAVE_SCC_CONSOLE 1
#  define HAVE_SERIAL_CONSOLE 1

   /* If we are not using the serial driver, then the serial console is all
    * that we will support.
    */

#  ifndef USE_SERIALDRIVER
#    undef CONFIG_Z180_UART0
#    undef CONFIG_Z180_UART1
#    undef CONFIG_Z180_SCC
#    undef CONFIG_Z180_ESCCA
#  endif

#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_Z80_SRC_Z180_Z180_CONFIG_H */
