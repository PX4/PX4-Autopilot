/************************************************************************************
 * arch/hc/src/m9s12/serial.h
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

#ifndef __ARCH_HC_SRC_M9S12_CHIP_H
#define __ARCH_HC_SRC_M9S12_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* Is there a SCI enabled? */

#if defined(CONFIG_SCI0_DISABLE) && defined(CONFIG_SCI1_DISABLE)
#  undef HAVE_SERIAL_DEVICE
#  warning "No SCIs enabled"
#else
#  define HAVE_SERIAL_DEVICE 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && !defined(CONFIG_SCI0_DISABLE)
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && !defined(CONFIG_SCI1_DISABLE)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  warning "No valid CONFIG_SCIn_SERIAL_CONSOLE Setting"
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Sanity checking */

#ifndef CONFIG_SCI0_DISABLE
#  ifndef CONFIG_SCI0_PARITY
#    warning "CONFIG_SCI0_PARITY not defined -- Assuming none"
#    define CONFIG_SCI0_PARITY 0 
#  elif CONFIG_SCI0_PARITY != 0 && CONFIG_SCI0_PARITY != 2 && CONFIG_SCI0_PARITY != 2
#    error "CONFIG_SCI0_PARITY value not recognized"
#  endif
#  ifndef CONFIG_SCI0_BITS
#    warning "CONFIG_SCI0_BITS not defined -- Assuming 8"
#    define CONFIG_SCI0_BITS 8
#  elif CONFIG_SCI0_BITS != 8 && CONFIG_SCI0_BITS != 9
#    error "CONFIG_SCI0_BITS value not supported"
#  endif
#  if defined(CONFIG_SCI0_2STOP) && CONFIG_SCI0_2STOP != 0
#    error "Only a single stop bit is supported"
#  endif
#endif

#ifndef CONFIG_SCI1_DISABLE
#  ifndef CONFIG_SCI1_PARITY
#    warning "CONFIG_SCI1_PARITY not defined -- Assuming none"
#    define CONFIG_SCI1_PARITY 0 
#  elif CONFIG_SCI1_PARITY != 0 && CONFIG_SCI1_PARITY != 2 && CONFIG_SCI1_PARITY != 2
#    error "CONFIG_SCI1_PARITY value not recognized"
#  endif
#  ifndef CONFIG_SCI1_BITS
#    warning "CONFIG_SCI1_BITS not defined -- Assuming 8"
#    define CONFIG_SCI1_BITS 8
#  elif CONFIG_SCI1_BITS != 8 && CONFIG_SCI1_BITS != 9
#    error "CONFIG_SCI1_BITS value not supported"
#  endif
#  if defined(CONFIG_SCI1_2STOP) && CONFIG_SCI1_2STOP != 0
#    error "Only a single stop bit is supported"
#  endif
#endif

/* BAUD *****************************************************************************/
/* Baud calculations.  The SCI module is driven by the BUSCLK.  The SCIBR
 * register value divides down the BUSCLK to accomplish the required BAUD.
 *
 *   BAUD  = HCS12_BUSCLK / (16 * SCIBR)
 *   SCIBR = HCS12_BUSCLK / (16 * BAUD)
 */

#define SCIBR_VALUE(b) ((HCS12_BUSCLK * (b) + ((b) << 3))/((b) << 4))

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_CHIP_H */
