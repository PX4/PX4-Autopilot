/************************************************************************************
 * arch/avr/src/at90usb/at90usb_config.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_AVR_SRC_ATMEGA_ATMEGA_CONFIG_H
#define __ARCH_AVR_SRC_ATMEGA_ATMEGA_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* USARTs ***************************************************************************/

#undef HAVE_USART_DEVICE
#if defined(CONFIG_AVR_USART1)
#  define HAVE_USART_DEVICE 1
#endif

/* Is there a serial console?  There should be at most one defined.  It
 * could be on any USARTn (but n=1 only for the AT90USB).
 */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_AVR_USART1)
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Determine which (if any) console driver to use */

#if CONFIG_NFILE_DESCRIPTORS == 0 || defined(CONFIG_DEV_LOWCONSOLE) || !defined(HAVE_USART_DEVICE)
#  undef CONFIG_USE_SERIALDRIVER
#  undef CONFIG_USE_EARLYSERIALINIT
#elif defined(CONFIG_DEV_CONSOLE) && CONFIG_NFILE_DESCRIPTORS > 0
#  define CONFIG_USE_SERIALDRIVER 1
#  define CONFIG_USE_EARLYSERIALINIT 1
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

#endif /* __ARCH_AVR_SRC_ATMEGA_ATMEGA_CONFIG_H */

