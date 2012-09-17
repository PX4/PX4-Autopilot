/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_uart.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_LOWSETUP_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_LOWSETUP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "chip/lpc43_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization of the serial console.
 *
 ****************************************************************************/

EXTERN void lpc43_lowsetup(void);

/****************************************************************************
 * Name: lpc43_usart0_reset, lpc43_uart1_reset, lpc43_usart2_reset, and
 *       lpc43_usart3_reset
 *
 * Description:
 *   Reset a U[S]ART.  These functions are used by the serial driver when a
 *   U[S]ART is closed.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_USART0
EXTERN void lpc43_usart0_reset(void);
#endif
#ifdef CONFIG_LPC43_UART1
EXTERN void lpc43_uart1_reset(void);
#endif
#ifdef CONFIG_LPC43_USART2
EXTERN void lpc43_usart2_reset(void);
#endif
#ifdef CONFIG_LPC43_USART3
EXTERN void lpc43_usart3_reset(void);
#endif

/****************************************************************************
 * Name: lpc43_usart0_setup, lpc43_uart1_setup, lpc43_usart2_setup, and
 *       lpc43_usart3_setup
 *
 * Description:
 *   Configure the U[S]ART.  This involves:
 *
 *   1. Connecting the input clock to the U[S]ART as specified in the
 *      board.h file,
 *   2. Configuring the U[S]ART pins
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_USART0
EXTERN void lpc43_usart0_setup(void);
#endif

#ifdef CONFIG_LPC43_UART1
EXTERN void lpc43_uart1_setup(void);
#endif

#ifdef CONFIG_LPC43_USART2
EXTERN void lpc43_usart2_setup(void);
#endif

#ifdef CONFIG_LPC43_USART3
EXTERN void lpc43_usart3_setup(void);
#endif

/****************************************************************************
 * Name: lpc43_setbaud
 *
 * Description:
 *   Configure the U[S]ART divisors to accomplish the desired BAUD given the
 *   U[S]ART base frequency.
 *
 *   This computationally intensive algorithm is based on the same logic
 *   used in the NXP sample code.
 *
 ****************************************************************************/

EXTERN void lpc43_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_LOWSETUP_H */
