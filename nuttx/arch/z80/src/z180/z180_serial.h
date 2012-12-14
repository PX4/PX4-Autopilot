/****************************************************************************
 * arch/z80/src/z180/z180_serial.h
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

#ifndef __ARCH_Z80_SRC_Z180_Z180_SERIAL_H
#define __ARCH_Z80_SRC_Z180_Z180_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "up_internal.h"
#include "z180_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 /****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_uart_lowinit
 *
 * Description:
 *   Called early in the boot sequence to initialize the UART(s)
 *
 ****************************************************************************/

#ifdef HAVE_UART
void z180_uart_lowinit(void);
#else
#  define z180_uart_lowinit()
#endif

/****************************************************************************
 * Name: z180_scc_lowinit
 *
 * Description:
 *   Called early in the boot sequence to initialize the [E]SCC channel(s)
 *
 ****************************************************************************/

#ifdef HAVE_SCC
void z180_scc_lowinit(void);
#else
#  define z180_scc_lowinit()
#endif

/****************************************************************************
 * Name: z180_putc
 *
 * Description:
 *   Low-level character output
 *
 ****************************************************************************/

void z180_putc(uint8_t ch) __naked;

/****************************************************************************
 * Name: up_putc/up_lowputc
 *
 * Description:
 *   Low-level console output
 *
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
int up_lowputc(int ch);
#else
int up_putc(int ch);
#  define up_lowputc(ch) up_putc(ch)
#endif

#endif /* __ARCH_Z80_SRC_Z180_Z180_SERIAL_H */
