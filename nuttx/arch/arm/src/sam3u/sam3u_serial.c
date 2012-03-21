/****************************************************************************
 * arch/arm/src/sam3u/sam3u_serial.c
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "sam3u_uart.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* If the USART is not being used as a UART, then it really isn't enabled
 * for our purposes.
 */

#ifndef CONFIG_USART0_ISUART
#  undef CONFIG_SAM3U_USART0
#endif
#ifndef CONFIG_USART1_ISUART
#  undef CONFIG_SAM3U_USART1
#endif
#ifndef CONFIG_USART2_ISUART
#  undef CONFIG_SAM3U_USART2
#endif
#ifndef CONFIG_USART3_ISUART
#  undef CONFIG_SAM3U_USART3
#endif

/* Is there a USART/USART enabled? */

#if !defined(CONFIG_SAM3U_UART)   && !defined(CONFIG_SAM3U_USART0) && \
    !defined(CONFIG_SAM3U_USART1) && !defined(CONFIG_SAM3U_USART2) && \
    !defined(CONFIG_SAM3U_USART3)
#  error "No USARTs enabled"
#endif

#if defined(CONFIG_SAM3U_USART0) || defined(CONFIG_SAM3U_USART1) ||\
    defined(CONFIG_SAM3U_USART2) || defined(CONFIG_SAM3U_USART3)
#  define HAVE_USART
#endif

/* Is there a serial console? */

#if defined(CONFIG_UART_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_UART)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART0)
#  undef CONFIG_UART_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART1)
#  undef CONFIG_UART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART2)
#  undef CONFIG_UART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_SAM3U_USART3)
#  undef CONFIG_UART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_UART_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART/USART with be tty0/console and which tty1? tty2? tty3? tty4? */

#if defined(CONFIG_UART_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uartport      /* UART=console */
#  define TTYS0_DEV       g_uartport      /* UART=ttyS0 */
#  ifdef CONFIG_SAM3U_USART0
#    define TTYS1_DEV     g_usart0port    /* UART=ttyS0;USART0=ttyS1 */
#    ifdef CONFIG_SAM3U_USART1
#      define TTYS2_DEV   g_usart1port    /* UART=ttyS0;USART0=ttyS1;USART1=ttyS2 */
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS3_DEV   g_usart2port  /* UART=ttyS0;USART0=ttyS1;USART1=ttyS2;USART2=ttyS3 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS4_DEV g_usart3port  /* UART=ttyS0;USART0=ttyS1;USART1=ttyS2;USART2=ttyS3;USART3=ttyS4 */
#        else
#          undef TTYS4_DEV                /* UART=ttyS0;USART0=ttyS1;USART1=ttyS2;USART2=ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* UART=ttyS0;USART0=ttyS1;USART1=ttyS;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* UART=ttyS0;USART0=ttyS1;USART1=ttyS;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS2_DEV g_usart2port    /* UART=ttyS0;USART0=ttyS1;USART2=ttys2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* UART=ttyS0;USART0=ttyS1;USART2=ttys2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* UART=ttyS0;USART0=ttyS1;USART2=ttys2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* UART=ttyS0;USART0=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* UART=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    endif
#  else
#    ifdef CONFIG_SAM3U_USART1
#      define TTYS1_DEV   g_usart1port    /* UART=ttyS0;USART1=ttyS1;No ttyS4 */
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS2_DEV   g_usart2port  /* UART=ttyS0;USART1=ttyS1;USART2=ttyS2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* UART=ttyS0;USART1=ttyS1;USART2=ttyS2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* UART=ttyS0;USART1=ttyS1;USART2=ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* UART=ttyS0;USART1=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* UART=ttyS0;USART1=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS1_DEV   g_usart2port  /* UART=ttyS0;USART2=ttyS1;No ttyS3;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* UART=ttyS0;USART2=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* UART=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS1_DEV g_usart3port  /* UART=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS1_DEV                /* UART=ttyS0;No ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#    undef TTYS4_DEV                      /* No ttyS4 */
#  endif
#elif defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart0port    /* USART0=console */
#  define TTYS0_DEV       g_usart0port    /* USART0=ttyS0 */
#  ifdef CONFIG_SAM3U_UART
#    define TTYS1_DEV     g_uartport      /* USART0=ttyS0;UART=ttyS1 */
#    ifdef CONFIG_SAM3U_USART1
#      define TTYS2_DEV   g_usart1port    /* USART0=ttyS0;UART=ttyS1;USART1=ttyS2 */
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS3_DEV   g_usart2port  /* USART0=ttyS0;UART=ttyS1;USART1=ttyS2;USART2=ttyS3 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS4_DEV g_usart3port  /* USART0=ttyS0;UART=ttyS1;USART1=ttyS2;USART2=ttyS3;USART3=ttyS4 */
#        else
#          undef TTYS4_DEV                /* USART0=ttyS0;UART=ttyS1;USART1=ttyS2;USART2=ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART0=ttyS0;UART=ttyS1;USART1=ttyS;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART0=ttyS0;UART=ttyS1;USART1=ttyS;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS2_DEV g_usart2port    /* USART0=ttyS0;UART=ttyS1;USART2=ttys2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART0=ttyS0;UART=ttyS1;USART2=ttys2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART0=ttyS0;UART=ttyS1;USART2=ttys2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART0=ttyS0;UART=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART0=ttyS0;UART=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    endif
#  else
#    ifdef CONFIG_SAM3U_USART1
#      define TTYS1_DEV   g_usart1port    /* USART0=ttyS0;USART1=ttyS1;No ttyS4 */
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS2_DEV   g_usart2port  /* USART0=ttyS0;USART1=ttyS1;USART2=ttyS2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART0=ttyS0;USART1=ttyS1;USART2=ttyS2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART0=ttyS0;USART1=ttyS1;USART2=ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART0=ttyS0;USART1=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART0=ttyS0;USART1=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS1_DEV   g_usart2port  /* USART0=ttyS0;USART2=ttyS1;No ttyS3;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART0=ttyS0;USART2=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART0=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS1_DEV g_usart3port  /* USART0=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS1_DEV                /* USART0=ttyS0;No ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#    undef TTYS4_DEV                      /* No ttyS4 */
#  endif
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart1port    /* USART1=console */
#  define TTYS0_DEV       g_usart1port    /* USART1=ttyS0 */
#  ifdef CONFIG_SAM3U_UART
#    define TTYS1_DEV     g_uartport      /* USART1=ttyS0;UART=ttyS1 */
#    ifdef CONFIG_SAM3U_USART0
#      define TTYS2_DEV   g_usart0port    /* USART1=ttyS0;UART=ttyS1;USART0=ttyS2 */
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS3_DEV   g_usart2port  /* USART1=ttyS0;UART=ttyS1;USART0=ttyS2;USART2=ttyS3 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS4_DEV g_usart3port  /* USART1=ttyS0;UART=ttyS1;USART0=ttyS2;USART2=ttyS3;USART3=ttyS4 */
#        else
#          undef TTYS4_DEV                /* USART1=ttyS0;UART=ttyS1;USART0=ttyS2;USART2=ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART1=ttyS0;UART=ttyS1;USART0=ttyS;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART1=ttyS0;UART=ttyS1;USART0=ttyS;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS2_DEV g_usart2port    /* USART1=ttyS0;UART=ttyS1;USART2=ttys2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART1=ttyS0;UART=ttyS1;USART2=ttys2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART1=ttyS0;UART=ttyS1;USART2=ttys2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART1=ttyS0;UART=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART1=ttyS0;UART=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    endif
#  else
#    ifdef CONFIG_SAM3U_USART0
#      define TTYS1_DEV   g_usart0port    /* USART1=ttyS0;USART0=ttyS1;No ttyS4 */
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS2_DEV   g_usart2port  /* USART1=ttyS0;USART0=ttyS1;USART2=ttyS2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART1=ttyS0;USART0=ttyS1;USART2=ttyS2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART1=ttyS0;USART0=ttyS1;USART2=ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART1=ttyS0;USART0=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART1=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART2
#        define TTYS1_DEV   g_usart2port  /* USART1=ttyS0;USART2=ttyS1;No ttyS3;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART1=ttyS0;USART2=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART1=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS1_DEV g_usart3port  /* USART1=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS1_DEV                /* USART1=ttyS0;No ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#    undef TTYS4_DEV                      /* No ttyS4 */
#  endif
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart2port    /* USART2=console */
#  define TTYS0_DEV       g_usart2port    /* USART2=ttyS0 */
#  ifdef CONFIG_SAM3U_UART
#    define TTYS1_DEV     g_uartport      /* USART2=ttyS0;UART=ttyS1 */
#    ifdef CONFIG_SAM3U_USART0
#      define TTYS2_DEV   g_usart0port    /* USART2=ttyS0;UART=ttyS1;USART0=ttyS2 */
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS3_DEV   g_usart1port  /* USART2=ttyS0;UART=ttyS1;USART0=ttyS2;USART1=ttyS3 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS4_DEV g_usart3port  /* USART2=ttyS0;UART=ttyS1;USART0=ttyS2;USART1=ttyS3;USART3=ttyS4 */
#        else
#          undef TTYS4_DEV                /* USART2=ttyS0;UART=ttyS1;USART0=ttyS2;USART1=ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART2=ttyS0;UART=ttyS1;USART0=ttyS;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART2=ttyS0;UART=ttyS1;USART0=ttyS;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS2_DEV g_usart1port    /* USART2=ttyS0;UART=ttyS1;USART1=ttys2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART2=ttyS0;UART=ttyS1;USART1=ttys2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART2=ttyS0;UART=ttyS1;USART1=ttys2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART2=ttyS0;UART=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART2=ttyS0;UART=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    endif
#  else
#    ifdef CONFIG_SAM3U_USART0
#      define TTYS1_DEV   g_usart0port    /* USART2=ttyS0;USART0=ttyS1;No ttyS4 */
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS2_DEV   g_usart1port  /* USART2=ttyS0;USART0=ttyS1;USART1=ttyS2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS3_DEV g_usart3port  /* USART2=ttyS0;USART0=ttyS1;USART1=ttyS2;USART3=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART2=ttyS0;USART0=ttyS1;USART1=ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART2=ttyS0;USART0=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART2=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS1_DEV   g_usart1port  /* USART2=ttyS0;USART1=ttyS1;No ttyS3;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS2_DEV g_usart3port  /* USART2=ttyS0;USART1=ttyS1;USART3=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART2=ttyS0;USART1=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART3
#          define TTYS1_DEV g_usart3port  /* USART2=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS1_DEV                /* USART2=ttyS0;No ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#    undef TTYS4_DEV                      /* No ttyS4 */
#  endif
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart3port    /* USART3=console */
#  define TTYS0_DEV       g_usart3port    /* USART3=ttyS0 */
#  ifdef CONFIG_SAM3U_UART
#    define TTYS1_DEV     g_uartport      /* USART3=ttyS0;UART=ttyS1 */
#    ifdef CONFIG_SAM3U_USART0
#      define TTYS2_DEV   g_usart0port    /* USART3=ttyS0;UART=ttyS1;USART0=ttyS2 */
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS3_DEV   g_usart1port  /* USART3=ttyS0;UART=ttyS1;USART0=ttyS2;USART1=ttyS3 */
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS4_DEV g_usart2port  /* USART3=ttyS0;UART=ttyS1;USART0=ttyS2;USART1=ttyS3;USART2=ttyS4 */
#        else
#          undef TTYS4_DEV                /* USART3=ttyS0;UART=ttyS1;USART0=ttyS2;USART1=ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS3_DEV g_usart2port  /* USART3=ttyS0;UART=ttyS1;USART0=ttyS;USART2=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART3=ttyS0;UART=ttyS1;USART0=ttyS;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS2_DEV g_usart1port    /* USART3=ttyS0;UART=ttyS1;USART1=ttys2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS3_DEV g_usart2port  /* USART3=ttyS0;UART=ttyS1;USART1=ttys2;USART2=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART3=ttyS0;UART=ttyS1;USART1=ttys2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS4_DEV                  /* No ttyS4 */
#      else
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS2_DEV g_usart2port  /* USART3=ttyS0;UART=ttyS1;USART2=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART3=ttyS0;UART=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#        undef TTYS4_DEV                  /* No ttyS4 */
#      endif
#    endif
#  else
#    ifdef CONFIG_SAM3U_USART0
#      define TTYS1_DEV   g_usart0port    /* USART3=ttyS0;USART0=ttyS1;No ttyS4 */
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS2_DEV   g_usart1port  /* USART3=ttyS0;USART0=ttyS1;USART1=ttyS2;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS3_DEV g_usart2port  /* USART3=ttyS0;USART0=ttyS1;USART1=ttyS2;USART2=ttyS3;No ttyS4 */
#        else
#          undef TTYS3_DEV                /* USART3=ttyS0;USART0=ttyS1;USART1=ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS2_DEV g_usart2port  /* USART3=ttyS0;USART0=ttyS1;USART2=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART3=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS3_DEV                  /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_SAM3U_USART1
#        define TTYS1_DEV   g_usart1port  /* USART3=ttyS0;USART1=ttyS1;No ttyS3;No ttyS4 */
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS2_DEV g_EEEEport    /* USART3=ttyS0;USART1=ttyS1;USART2=ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS2_DEV                /* USART3=ttyS0;USART1=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#      else
#        ifdef CONFIG_SAM3U_USART2
#          define TTYS1_DEV g_usart2port  /* USART3=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        else
#          undef TTYS1_DEV                /* USART3=ttyS0;No ttyS1;No ttyS2;No ttyS3;No ttyS4 */
#        endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#    undef TTYS4_DEV                      /* No ttyS4 */
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t usartbase; /* Base address of USART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t imr;       /* Saved interrupt mask bits value */
  uint32_t sr;        /* Saved status bits */
  uint8_t  irq;       /* IRQ associated with this USART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_SAM3U_UART
static char g_uartrxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_uarttxbuffer[CONFIG_UART_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM3U_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM3U_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM3U_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_SAM3U_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

/* This describes the state of the SAM3U UART port. */

#ifdef CONFIG_SAM3U_UART
static struct up_dev_s g_uartpriv =
{
  .usartbase      = SAM3U_UART_BASE,
  .baud           = CONFIG_UART_BAUD,
  .irq            = SAM3U_IRQ_UART,
  .parity         = CONFIG_UART_PARITY,
  .bits           = CONFIG_UART_BITS,
  .stopbits2      = CONFIG_UART_2STOP,
};

static uart_dev_t g_uartport =
{
  .recv     =
  {
    .size   = CONFIG_UART_RXBUFSIZE,
    .buffer = g_uartrxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_TXBUFSIZE,
    .buffer = g_uarttxbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uartpriv,
};
#endif

/* This describes the state of the SAM3U USART1 ports. */

#ifdef CONFIG_SAM3U_USART0
static struct up_dev_s g_usart0priv =
{
  .usartbase      = SAM3U_USART0_BASE,
  .baud           = CONFIG_USART0_BAUD,
  .irq            = SAM3U_IRQ_USART0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
};

static uart_dev_t g_usart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_usart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_usart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart0priv,
};
#endif

/* This describes the state of the SAM3U USART1 ports. */

#ifdef CONFIG_SAM3U_USART1
static struct up_dev_s g_usart1priv =
{
  .usartbase      = SAM3U_USART1_BASE,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = SAM3U_IRQ_USART1,
  .parity         = CONFIG_USART1_PARITY,
  .bits           = CONFIG_USART1_BITS,
  .stopbits2      = CONFIG_USART1_2STOP,
};

static uart_dev_t g_usart1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_usart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_usart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart1priv,
};
#endif

/* This describes the state of the SAM3U USART2 port. */

#ifdef CONFIG_SAM3U_USART2
static struct up_dev_s g_usart2priv =
{
  .usartbase      = SAM3U_USART2_BASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = SAM3U_IRQ_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
};

static uart_dev_t g_usart2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_usart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_usart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart2priv,
};
#endif

/* This describes the state of the SAM3U USART3 port. */

#ifdef CONFIG_SAM3U_USART3
static struct up_dev_s g_usart3priv =
{
  .usartbase      = SAM3U_USART3_BASE,
  .baud           = CONFIG_USART3_BAUD,
  .irq            = SAM3U_IRQ_USART3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
};

static uart_dev_t g_usart3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_usart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_usart3txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart3priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_enableint
 ****************************************************************************/

static inline void up_enableint(struct up_dev_s *priv)
{
  up_serialout(priv, SAM3U_UART_IER_OFFSET, priv->imr);
}

/****************************************************************************
 * Name: up_disableint
 ****************************************************************************/

static inline void up_disableint(struct up_dev_s *priv)
{
  up_serialout(priv, SAM3U_UART_IDR_OFFSET, ~priv->imr);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static void up_restoreusartint(struct up_dev_s *priv, uint32_t imr)
{
  /* Save the interrupt mask */

  priv->imr = imr;

  /* And re-enable interrrupts previoulsy disabled by up_disableallints */

  up_enableint(priv);
}

/****************************************************************************
 * Name: up_disableallints
 ****************************************************************************/

static void up_disableallints(struct up_dev_s *priv, uint32_t *imr)
{
  if (imr)
    {
      /* Return the current interrupt mask */
 
      *imr = priv->imr;
    }

  /* Disable all interrupts */

  priv->imr = 0;
  up_disableint(priv);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled and the pins were configured in sam3u_lowsetup().
   */

  /* The shutdown method will put the UART in a known, disabled state */

  up_shutdown(dev);

  /* Set up the mode register.  Start with normal UART mode and the MCK
   * as the timing source
   */

  regval = (USART_MR_MODE_NORMAL|USART_MR_USCLKS_MCK);

  /* OR in settings for the selected number of bits */

  if (priv->bits == 5)
    {
      regval |= USART_MR_CHRL_5BITS; /* 5 bits */
    }
  else if (priv->bits == 6)
    {
      regval |= USART_MR_CHRL_6BITS;  /* 6 bits */
    }
  else if (priv->bits == 7)
    {
      regval |= USART_MR_CHRL_7BITS; /* 7 bits */
    }
#ifdef HAVE_USART
#ifdef CONFIG_SAM3U_UART
  /* UART does not support 9bit mode */

  else if (priv->bits == 9 && priv->usartbase != SAM3U_UART_BASE)
#else
  else if (priv->bits == 9) /* Only USARTS */
#endif
    {
      regval |= USART_MR_MODE9; /* 9 bits */
    }
#endif
  else /* if (priv->bits == 8) */
    {
      regval |= USART_MR_CHRL_8BITS; /* 8 bits (default) */
    }

  /* OR in settings for the selected parity */

  if (priv->parity == 1)
    {
      regval |= UART_MR_PAR_ODD;
    }
  else if (priv->parity == 2)
    {
      regval |= UART_MR_PAR_EVEN;
    }
  else
    {
      regval |= UART_MR_PAR_NONE;
    }

  /* OR in settings for the number of stop bits */

  if (priv->stopbits2)
    {
      regval |= USART_MR_NBSTOP_2;
    }
  else
    {
      regval |= USART_MR_NBSTOP_1;
    }

  /* And save the new mode register value */

  up_serialout(priv, SAM3U_UART_MR_OFFSET, regval);

  /* Configure the console baud */

  regval  = (SAM3U_MCK_FREQUENCY + (priv->baud << 3))/(priv->baud << 4);
  up_serialout(priv, SAM3U_UART_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  up_serialout(priv, SAM3U_UART_CR_OFFSET, (UART_CR_RXEN|UART_CR_TXEN));
#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Reset and disable receiver and transmitter */

  up_serialout(priv, SAM3U_UART_CR_OFFSET,
               (UART_CR_RSTRX|UART_CR_RSTTX|UART_CR_RXDIS|UART_CR_TXDIS));

  /* Disable all interrupts */

  up_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the USART
        */

       up_enable_irq(priv->irq);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint32_t           pending;
  int                passes;
  bool               handled;

#ifdef CONFIG_SAM3U_UART
  if (g_uartpriv.irq == irq)
    {
      dev = &g_uartport;
    }
  else
#endif
#ifdef CONFIG_SAM3U_USART0
  if (g_usart0priv.irq == irq)
    {
      dev = &g_usart0port;
    }
  else
#endif
#ifdef CONFIG_SAM3U_USART1
  if (g_usart1priv.irq == irq)
    {
      dev = &g_usart1port;
    }
  else
#endif
#ifdef CONFIG_SAM3U_USART2
  if (g_usart2priv.irq == irq)
    {
      dev = &g_usart2port;
    }
  else
#endif
#ifdef CONFIG_SAM3U_USART3
  if (g_usart3priv.irq == irq)
    {
      dev = &g_usart3port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the UART/USART status (we are only interested in the unmasked interrupts). */

      priv->sr = up_serialin(priv, SAM3U_UART_SR_OFFSET); /* Save for error reporting */
      pending  = priv->sr & priv->imr;                      /* Mask out disabled interrupt sources */

      /* Handle an incoming, receive byte.  RXRDY: At least one complete character
       * has been received and US_RHR has not yet been read.
       */

      if ((pending & UART_INT_RXRDY) != 0)
        {
           /* Received data ready... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes. XRDY: There is no character in the
       * US_THR.
       */

      if ((pending & UART_INT_TXRDY) != 0)
        {
           /* Transmit data regiser empty ... process outgoing bytes */

           uart_xmitchars(dev);
           handled = true;
        }
    }
    return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  int                ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Return the error information in the saved status */

  *status  = priv->sr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return (int)(up_serialin(priv, SAM3U_UART_RHR_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->imr |= UART_INT_RXRDY;
      up_enableint(priv);
#endif
    }
  else
    {
      priv->imr &= ~UART_INT_RXRDY;
      up_disableint(priv);
    }
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, SAM3U_UART_SR_OFFSET) & UART_INT_RXRDY) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART/USART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, SAM3U_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->imr |= UART_INT_TXRDY;
      up_enableint(priv);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->imr &= ~UART_INT_TXRDY;
      up_disableint(priv);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, SAM3U_UART_SR_OFFSET) & UART_INT_TXRDY) != 0);
 }

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, SAM3U_UART_SR_OFFSET) & UART_INT_TXEMPTY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the USARTs was performed in
   * sam3u_lowsetup
   */

  /* Disable all USARTS */

  up_disableallints(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableallints(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableallints(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableallints(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disableallints(TTYS4_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  (void)uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  (void)uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint16_t imr;

  up_disableallints(priv, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(priv, imr);
#endif
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
