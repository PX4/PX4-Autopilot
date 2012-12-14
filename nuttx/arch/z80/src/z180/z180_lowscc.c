/****************************************************************************
 * arch/z80/src/z180/z180_loweruart.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include <arch/io.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "chip/chip.h"
#include "common/up_internal.h"
#include "z80_config.h"

#ifdef USE_LOWUARTINIT

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_Z180_SCC_SERIAL_CONSOLE)
#  define CONSOLE_CR           Z181_SCC_CR
#  define CONSOLE_DR           Z181_SCC_DR
#  define CONSOLE_BAUD         CONFIG_Z180_SCC_BAUD
#  define CONSOLE_BITS         CONFIG_Z180_SCC_BITS
#  define CONSOLE_2STOP        CONFIG_Z180_SCC_2STOP
#  define CONSOLE_PARITY       CONFIG_Z180_SCC_PARITY

#elif defined(CONFIG_Z180_ESCCB_SERIAL_CONSOLE)
#  define CONSOLE_CR           Z182_ESCCA_CR
#  define CONSOLE_DR           Z182_ESCCA_DR
#  define CONSOLE_BAUD         CONFIG_Z180_ESCCA_BAUD
#  define CONSOLE_BITS         CONFIG_Z180_ESCCA_BITS
#  define CONSOLE_2STOP        CONFIG_Z180_ESCCA_2STOP
#  define CONSOLE_PARITY       CONFIG_Z180_ESCCA_PARITY

#elif defined(CONFIG_Z180_ESCCB_SERIAL_CONSOLE)
#  define CONSOLE_CR           Z182_ESCCB_CR
#  define CONSOLE_DR           Z182_ESCCB_DR
#  define CONSOLE_BAUD         CONFIG_Z180_ESCCB_BAUD
#  define CONSOLE_BITS         CONFIG_Z180_ESCCB_BITS
#  define CONSOLE_PARITY       CONFIG_Z180_ESCCB_PARITY
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
static void z180_setbaud(void)
{
#warning "Missing logic"
}
#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowuartinit
 ****************************************************************************/

void up_lowuartinit(void)
{
#ifdef HAVE_SERIAL
#warning "Missing logic"

  /* Configure for usage of {E]SCC channels (whether or not we have a console) */

#ifdef CONFIG_Z180_SCC
#warning "Missing logic"
#endif

#ifdef CONFIG_Z180_ESCCA
#warning "Missing logic"
#endif

#ifdef CONFIG_Z180_ESCCB
#warning "Missing logic"
#endif

  /* Configure the console for immediate usage */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
#warning "Missing logic"
#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_SERIAL */
}

#endif /* USE_LOWUARTINIT */
