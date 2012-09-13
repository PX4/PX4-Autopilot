/************************************************************************************
 * arch/arm/src/lpc214x/uart.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

#ifndef __LPC214X_UART_H
#define __LPC214X_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/board/board.h> /* For clock settings */

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* PINSEL0 bit definitions for UART0/1 */

#define LPC214X_UART0_PINSEL       0x00000005  /* PINSEL0 value for UART0 */
#define LPC214X_UART0_PINMASK      0x0000000f  /* PINSEL0 mask for UART0 */

#define LPC214X_UART1_PINSEL       0x00050000  /* PINSEL0 value for UART1 */
#define LPC214X_UART1_PINMASK      0x000f0000  /* PINSEL0 mask for UART1 */

/* Derive baud divisor setting from clock settings (see board.h) */

#define UART_BAUD(baud)            ((LPC214X_FOSC * LPC214X_PLL_M) / (baud * 16))

/* Interrupt Enable Register (IER) bit definitions */

#define LPC214X_IER_ERBFI          (1 << 0)    /* Enable receive data available int */
#define LPC214X_IER_ETBEI          (1 << 1)    /* Enable THR empty Interrupt */
#define LPC214X_IER_ELSI           (1 << 2)    /* Enable receive line status int */
#define LPC214X_IER_EDSSI          (1 << 3)    /* Enable MODEM atatus interrupt (2146/6/8 UART1 Only) */
#define LPC214X_IER_ALLIE          0x0f        /* All interrupts */

/* Interrupt ID Register(IIR) bit definitions */

#define LPC214X_IIR_NO_INT         (1 << 0)    /* No interrupts pending  */
#define LPC214X_IIR_MS_INT         (0 << 1)    /* MODEM Status (UART1 only) */
#define LPC214X_IIR_THRE_INT       (1 << 1)    /* Transmit Holding Register Empty */
#define LPC214X_IIR_RDA_INT        (2 << 1)    /* Receive Data Available */
#define LPC214X_IIR_RLS_INT        (3 << 1)    /* Receive Line Status */
#define LPC214X_IIR_CTI_INT        (6 << 1)    /* Character Timeout Indicator */
#define LPC214X_IIR_MASK           0x0e

/* FIFO Control Register (FCR) bit definitions */

#define LPC214X_FCR_FIFO_ENABLE    (1 << 0)    /* FIFO enable */
#define LPC214X_FCR_RX_FIFO_RESET  (1 << 1)    /* Reset receive FIFO */
#define LPC214X_FCR_TX_FIFO_RESET  (1 << 2)    /* Reset transmit FIFO */
#define LPC214X_FCR_FIFO_TRIG1     (0 << 6)    /* Trigger @1 character in FIFO */
#define LPC214X_FCR_FIFO_TRIG4     (1 << 6)    /* Trigger @4 characters in FIFO */
#define LPC214X_FCR_FIFO_TRIG8     (2 << 6)    /* Trigger @8 characters in FIFO */
#define LPC214X_FCR_FIFO_TRIG14    (3 << 6)    /* Trigger @14 characters in FIFO */

/* Line Control Register (LCR) bit definitions */

#define LPC214X_LCR_CHAR_5         (0 << 0)    /* 5-bit character length */
#define LPC214X_LCR_CHAR_6         (1 << 0)    /* 6-bit character length */
#define LPC214X_LCR_CHAR_7         (2 << 0)    /* 7-bit character length */
#define LPC214X_LCR_CHAR_8         (3 << 0)    /* 8-bit character length */
#define LPC214X_LCR_STOP_1         (0 << 2)    /* 1 stop bit */
#define LPC214X_LCR_STOP_2         (1 << 2)    /* 2 stop bits */
#define LPC214X_LCR_PAR_NONE       (0 << 3)    /* No parity */
#define LPC214X_LCR_PAR_ODD        (1 << 3)    /* Odd parity */
#define LPC214X_LCR_PAR_EVEN       (3 << 3)    /* Even parity */
#define LPC214X_LCR_PAR_MARK       (5 << 3)    /* Mark "1" parity */
#define LPC214X_LCR_PAR_SPACE      (7 << 3)    /* Space "0" parity */
#define LPC214X_LCR_BREAK_ENABLE   (1 << 6)    /* Output BREAK */
#define LPC214X_LCR_DLAB_ENABLE    (1 << 7)    /* Enable divisor latch access */

/* Modem Control Register (MCR) bit definitions */

#define LPC214X_MCR_DTR            (1 << 0)    /* Data terminal ready */
#define LPC214X_MCR_RTS            (1 << 1)    /* Request to send */
#define LPC214X_MCR_LB             (1 << 4)    /* Loopback */

/* Line Status Register (LSR) bit definitions */

#define LPC214X_LSR_RDR            (1 << 0)    /* Receive data ready */
#define LPC214X_LSR_OE             (1 << 1)    /* Overrun error */
#define LPC214X_LSR_PE             (1 << 2)    /* Parity error */
#define LPC214X_LSR_FE             (1 << 3)    /* Framing error */
#define LPC214X_LSR_BI             (1 << 4)    /* Break interrupt */
#define LPC214X_LSR_THRE           (1 << 5)    /* THR empty */
#define LPC214X_LSR_TEMT           (1 << 6)    /* Transmitter empty */
#define LPC214X_LSR_RXFE           (1 << 7)    /* Error in receive FIFO */
#define LPC214X_LSR_ERR_MASK       0x1e

/* Modem Status Register (MSR) bit definitions */

#define LPC214X_MSR_DCTS           (1 << 0)    /* Delta clear to send */
#define LPC214X_MSR_DDSR           (1 << 1)    /* Delta data set ready */
#define LPC214X_MSR_TERI           (1 << 2)    /* Trailing edge ring indicator */
#define LPC214X_MSR_DDCD           (1 << 3)    /* Delta data carrier detect */
#define LPC214X_MSR_CTS            (1 << 4)    /* Clear to send */
#define LPC214X_MSR_DSR            (1 << 5)    /* Data set ready */
#define LPC214X_MSR_RI             (1 << 6)    /* Ring indicator */
#define LPC214X_MSR_DCD            (1 << 7)    /* Data carrier detect */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Global Function Prototypes
 ************************************************************************************/

#endif  /* __LPC214X_UART_H */
