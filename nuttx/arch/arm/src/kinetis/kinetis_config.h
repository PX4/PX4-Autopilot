/************************************************************************************
 * arch/arm/src/kinetis/kinetis_config.h
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

#ifndef __ARCH_ARM_SRC_KINETISXX_KINETIS_CONFIG_H
#define __ARCH_ARM_SRC_KINETISXX_KINETIS_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration *********************************************************************/
/* Make that no unsupport UARTs are enabled */

#ifndef KINETIS_NISO7816
#  define KINETIS_NISO7816 0
#endif

#if (KINETIS_NISO7816 + KINETIS_NUART) < 6
#  undef CONFIG_KINETIS_UART5
#  if (KINETIS_NISO7816 + KINETIS_NUART) < 5
#    undef CONFIG_KINETIS_UART4
#    if (KINETIS_NISO7816 + KINETIS_NUART) < 4
#      undef CONFIG_KINETIS_UART3
#      if (KINETIS_NISO7816 + KINETIS_NUART) < 3
#        undef CONFIG_KINETIS_UART2
#        if (KINETIS_NISO7816 + KINETIS_NUART) < 2
#          undef CONFIG_KINETIS_UART1
#          if (KINETIS_NISO7816 + KINETIS_NUART) < 1
#            undef CONFIG_KINETIS_UART0
#          endif
#        endif
#      endif
#    endif
#  endif
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_KINETIS_UART0) || defined(CONFIG_KINETIS_UART1) || \
    defined(CONFIG_KINETIS_UART2) || defined(CONFIG_KINETIS_UART3) || \
    defined(CONFIG_KINETIS_UART5)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2,3,4,5
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART4)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART5)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Check UART flow control (Not yet supported) */

# undef CONFIG_UART0_FLOWCONTROL
# undef CONFIG_UART1_FLOWCONTROL
# undef CONFIG_UART2_FLOWCONTROL
# undef CONFIG_UART3_FLOWCONTROL
# undef CONFIG_UART4_FLOWCONTROL
# undef CONFIG_UART5_FLOWCONTROL

/* UART FIFO support is not fully implemented.
 *
 * NOTE:  UART0 has an 8-byte deep FIFO; the other UARTs have no FIFOs
 * (1-deep).  There appears to be no way to know when the FIFO is not
 * full (other than reading the FIFO length and comparing the FIFO count).
 * Hence, the FIFOs are not used in this implementation and, as a result
 * TDRE indeed mean that the single output buffer is available.
 *
 * Performance on UART0 could be improved by enabling the FIFO and by
 * redesigning all of the FIFO status logic.
 */

#undef CONFIG_KINETIS_UARTFIFOS

/* UART Default Interrupt Priorities */

#ifndef CONFIG_KINETIS_UART0PRIO
#  define CONFIG_KINETIS_UART0PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_UART1PRIO
#  define CONFIG_KINETIS_UART1PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_UART2PRIO
#  define CONFIG_KINETIS_UART2PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_UART3PRIO
#  define CONFIG_KINETIS_UART3PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_UART4PRIO
#  define CONFIG_KINETIS_UART4PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_UART5PRIO
#  define CONFIG_KINETIS_UART5PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif

/* Ethernet controller configuration */

#ifndef CONFIG_ENET_NRXBUFFERS
#  define CONFIG_ENET_NRXBUFFERS 6
#endif
#ifndef CONFIG_ENET_NTXBUFFERS
#  define CONFIG_ENET_NTXBUFFERS 2
#endif

#ifndef CONFIG_ENET_PHYADDR
#  define CONFIG_ENET_PHYADDR 1
#endif

#ifndef CONFIG_ENET_NETHIFS
# define CONFIG_ENET_NETHIFS 1
#endif

/* EMAC Default Interrupt Priorities */

#ifndef CONFIG_KINETIS_EMACTMR_PRIO
#  define CONFIG_KINETIS_EMACTMR_PRIO  NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_EMACTX_PRIO
#  define CONFIG_KINETIS_EMACTX_PRIO   NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_EMACRX_PRIO
#  define CONFIG_KINETIS_EMACRX_PRIO   NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_KINETIS_EMACMISC_PRIO
#  define CONFIG_KINETIS_EMACMISC_PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETISXX_KINETIS_CONFIG_H */
