/************************************************************************************
 * arch/arm/src/str71x/str71x_uart.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_UART_H
#define __ARCH_ARM_SRC_STR71X_STR71X_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Registers offsets ****************************************************************/

#define STR71X_UART_BR_OFFSET       (0x0000) /* 16-bits wide */
#define STR71X_UART_TXBUFR_OFFSET   (0x0004) /* 16-bits wide */
#define STR71X_UART_RXBUFR_OFFSET   (0x0008) /* 16-bits wide */
#define STR71X_UART_CR_OFFSET       (0x000c) /* 16-bits wide */
#define STR71X_UART_IER_OFFSET      (0x0010) /* 16-bits wide */
#define STR71X_UART_SR_OFFSET       (0x0014) /* 16-bits wide */
#define STR71X_UART_GTR_OFFSET      (0x0018) /* 16-bits wide */
#define STR71X_UART_TOR_OFFSET      (0x001c) /* 16-bits wide */
#define STR71X_UART_TXRSTR_OFFSET   (0x0020) /* 16-bits wide */
#define STR71X_UART_RXRSTR_OFFSET   (0x0024) /* 16-bits wide */

/* Registers addresses **************************************************************/

#define STR71X_UART_BR(b)           ((b) + STR71X_UART_BR_OFFSET)
#define STR71X_UART_TXBUFR(b)       ((b) + STR71X_UART_TXBUFR_OFFSET)
#define STR71X_UART_RXBUFR(b)       ((b) + STR71X_UART_RXBUFR_OFFSET)
#define STR71X_UART_CR(b)           ((b) + STR71X_UART_CR_OFFSET)
#define STR71X_UART_IER(b)          ((b) + STR71X_UART_IER_OFFSET)
#define STR71X_UART_SR(b)           ((b) + STR71X_UART_SR_OFFSET)
#define STR71X_UART_GTR(b)          ((b) + STR71X_UART_GTR_OFFSET)
#define STR71X_UART_TOR(b)          ((b) + STR71X_UART_TOR_OFFSET)
#define STR71X_UART_TXRSTR(b)       ((b) + STR71X_UART_TXRSTR_OFFSET)
#define STR71X_UART_RXRSTR(b)       ((b) + STR71X_UART_RXRSTR_OFFSET)

#define STR71X_UART0_BR             (STR71X_UART0_BASE + STR71X_UART_BR_OFFSET)
#define STR71X_UART0_TXBUFR         (STR71X_UART0_BASE + STR71X_UART_TXBUFR_OFFSET)
#define STR71X_UART0_RXBUFR         (STR71X_UART0_BASE + STR71X_UART_RXBUFR_OFFSET)
#define STR71X_UART0_CR             (STR71X_UART0_BASE + STR71X_UART_CR_OFFSET)
#define STR71X_UART0_IER            (STR71X_UART0_BASE + STR71X_UART_IER_OFFSET)
#define STR71X_UART0_SR             (STR71X_UART0_BASE + STR71X_UART_SR_OFFSET)
#define STR71X_UART0_GTR            (STR71X_UART0_BASE + STR71X_UART_GTR_OFFSET)
#define STR71X_UART0_TOR            (STR71X_UART0_BASE + STR71X_UART_TOR_OFFSET)
#define STR71X_UART0_TXRSTR         (STR71X_UART0_BASE + STR71X_UART_TXRSTR_OFFSET)
#define STR71X_UART0_RXRSTR         (STR71X_UART0_BASE + STR71X_UART_RXRSTR_OFFSET)

#define STR71X_UART1_BR             (STR71X_UART1_BASE + STR71X_UART_BR_OFFSET)
#define STR71X_UART1_TXBUFR         (STR71X_UART1_BASE + STR71X_UART_TXBUFR_OFFSET)
#define STR71X_UART1_RXBUFR         (STR71X_UART1_BASE + STR71X_UART_RXBUFR_OFFSET)
#define STR71X_UART1_CR             (STR71X_UART1_BASE + STR71X_UART_CR_OFFSET)
#define STR71X_UART1_IER            (STR71X_UART1_BASE + STR71X_UART_IER_OFFSET)
#define STR71X_UART1_SR             (STR71X_UART1_BASE + STR71X_UART_SR_OFFSET)
#define STR71X_UART1_GTR            (STR71X_UART1_BASE + STR71X_UART_GTR_OFFSET)
#define STR71X_UART1_TOR            (STR71X_UART1_BASE + STR71X_UART_TOR_OFFSET)
#define STR71X_UART1_TXRSTR         (STR71X_UART1_BASE + STR71X_UART_TXRSTR_OFFSET)
#define STR71X_UART1_RXRSTR         (STR71X_UART1_BASE + STR71X_UART_RXRSTR_OFFSET)

#define STR71X_UART2_BR             (STR71X_UART2_BASE + STR71X_UART_BR_OFFSET)
#define STR71X_UART2_TXBUFR         (STR71X_UART2_BASE + STR71X_UART_TXBUFR_OFFSET)
#define STR71X_UART2_RXBUFR         (STR71X_UART2_BASE + STR71X_UART_RXBUFR_OFFSET)
#define STR71X_UART2_CR             (STR71X_UART2_BASE + STR71X_UART_CR_OFFSET)
#define STR71X_UART2_IER            (STR71X_UART2_BASE + STR71X_UART_IER_OFFSET)
#define STR71X_UART2_SR             (STR71X_UART2_BASE + STR71X_UART_SR_OFFSET)
#define STR71X_UART2_GTR            (STR71X_UART2_BASE + STR71X_UART_GTR_OFFSET)
#define STR71X_UART2_TOR            (STR71X_UART2_BASE + STR71X_UART_TOR_OFFSET)
#define STR71X_UART2_TXRSTR         (STR71X_UART2_BASE + STR71X_UART_TXRSTR_OFFSET)
#define STR71X_UART2_RXRSTR         (STR71X_UART2_BASE + STR71X_UART_RXRSTR_OFFSET)

#define STR71X_UART3_BR             (STR71X_UART3_BASE + STR71X_UART_BR_OFFSET)
#define STR71X_UART3_TXBUFR         (STR71X_UART3_BASE + STR71X_UART_TXBUFR_OFFSET)
#define STR71X_UART3_RXBUFR         (STR71X_UART3_BASE + STR71X_UART_RXBUFR_OFFSET)
#define STR71X_UART3_CR             (STR71X_UART3_BASE + STR71X_UART_CR_OFFSET)
#define STR71X_UART3_IER            (STR71X_UART3_BASE + STR71X_UART_IER_OFFSET)
#define STR71X_UART3_SR             (STR71X_UART3_BASE + STR71X_UART_SR_OFFSET)
#define STR71X_UART3_GTR            (STR71X_UART3_BASE + STR71X_UART_GTR_OFFSET)
#define STR71X_UART3_TOR            (STR71X_UART3_BASE + STR71X_UART_TOR_OFFSET)
#define STR71X_UART3_TXRSTR         (STR71X_UART3_BASE + STR71X_UART_TXRSTR_OFFSET)
#define STR71X_UART3_RXRSTR         (STR71X_UART3_BASE + STR71X_UART_RXRSTR_OFFSET)

/* Register bit settings ***********************************************************/

/* UART control register (CR) */

#define STR71X_UARTCR_MODEMASK      (0x0007) /* Bits 0-2: Mode */
#define STR71X_UARTCR_MODE8BIT      (0x0001) /*   8-bit */
#define STR71X_UARTCR_MODE7BITP     (0x0003) /*   7-bit with parity bit */
#define STR71X_UARTCR_MODE9BIT      (0x0004) /*   9-bit */
#define STR71X_UARTCR_MODE8BITWU    (0x0005) /*   8-bit with wakeup bit */
#define STR71X_UARTCR_MODE8BITP     (0x0007) /*   8-bit with parity bit */
#define STR71X_UARTCR_STOPBITSMASK  (0x0018) /* Bits 3-4: Stop bits */
#define STR71X_UARTCR_STOPBIT05     (0x0000) /*   0.5 stop bits */
#define STR71X_UARTCR_STOPBIT10     (0x0008) /*   1.0 stop bit */
#define STR71X_UARTCR_STOPBIT15     (0x0010) /*   1.5 stop bits */
#define STR71X_UARTCR_STOPBIT20     (0x0018) /*   2.0 stop bits */
#define STR71X_UARTCR_PARITYODD     (0x0020) /* Bit 5: Parity selection */
#define STR71X_UARTCR_LOOPBACK      (0x0040) /* Bit 6: Loopback mode enable */
#define STR71X_UARTCR_RUN           (0x0080) /* Bit 7: Baudrate generator run bit */
#define STR71X_UARTCR_RXENABLE      (0x0100) /* Bit 8: Receiver enable */
#define STR71X_UARTCR_SCENABLE      (0x0200) /* Bit 9: SmartCard mode enable */
#define STR71X_UARTCR_FIFOENABLE    (0x0400) /* Bit 10: FIFO enable */

/* UART interrupt enable (IER) register */

#define STR71X_UARTIER_RNE          (0x0001) /* Bit 0: Rx buffer not empty */
#define STR71X_UARTIER_TE           (0x0002) /* Bit 1: Tx empty */
#define STR71X_UARTIER_THE          (0x0004) /* Bit 2: Tx half empty */
#define STR71X_UARTIER_PERROR       (0x0008) /* Bit 3: Parity error */
#define STR71X_UARTIER_FRERROR      (0x0010) /* Bit 4: Frame error */
#define STR71X_UARTIER_OVERRUN      (0x0020) /* Bit 5: Overrun error */
#define STR71X_UARTIER_TIMEOUTNE    (0x0040) /* Bit 6: Time out not empty*/
#define STR71X_UARTIER_TIMEOUTIDLE  (0x0080) /* Bit 7: Timeout out idle */
#define STR71X_UARTIER_RHF          (0x0100) /* Bit 8: Rx half full */
#define STR71X_UARTIER_ALL          (0x01ff) /* All interrupt bits */

/* UART status register (SR) */

#define STR71X_UARTSR_RNE           (0x0001) /* Bit 0: Rx buffer not empty */
#define STR71X_UARTSR_TE            (0x0002) /* Bit 1: Tx empty */
#define STR71X_UARTSR_THE           (0x0004) /* Bit 2: Tx half empty */
#define STR71X_UARTSR_PERR          (0x0008) /* Bit 3: Parity error */
#define STR71X_UARTSR_FRERROR       (0x0010) /* Bit 4: Frame error */
#define STR71X_UARTSR_OVERRUN       (0x0020) /* Bit 5: Overrun error */
#define STR71X_UARTSR_TIMEOUTNE     (0x0040) /* Bit 6: Time out not empty */
#define STR71X_UARTSR_TIMEOUTIDLE   (0x0080) /* Bit 7: Timeout out idle */
#define STR71X_UARTSR_RHF           (0x0100) /* Bit 8: Rx half full */
#define STR71X_UARTSR_TF            (0x0200) /* Bit 9: Tx full */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_UART_H */
