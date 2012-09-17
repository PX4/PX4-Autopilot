/************************************************************************************
 * arch/arm/src/lpc2378/lpc2378/uart.h
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC2378_LPC23XX_UART_H
#define __ARCH_ARM_SRC_LPC2378_LPC23XX_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/board/board.h> /* For clock settings */

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Derive baud divisor setting from clock settings (see board.h) */
//--F_in = 57 600 000 Hz U0DLM=0, U0DLL=25, DIVADDVAL=3, MULVAL=12, baudrate=115200, err = 0.000000 %
//--F_in = 57 600 000 Hz U0DLM=1, U0DLL=119, DIVADDVAL=0, MULVAL=1, baudrate=9600, err = 0.000000 %
/* Used only if CONFIG_UART_MULVAL is not defined */
#define DIVADDVAL	0
#define MULVAL 		1
#define DLMVAL		1
#define DLLVAL		119

/* UARTx PCLK divider valid values are 1,2,4 */
#define U0_PCLKDIV		1
//~ #define U1_PCLKDIV		1
#define U2_PCLKDIV		1
//~ #define U3_PCLKDIV		1


#define U0_PCLK	(CCLK / U0_PCLKDIV)
//~ #define U1_PCLK	(CCLK / U1_PCLKDIV)
#define U2_PCLK	(CCLK / U2_PCLKDIV)
//~ #define U3_PCLK	(CCLK / U3_PCLKDIV)

#define U0_PCLKSEL_MASK	(0x000000C0)
#define U2_PCLKSEL_MASK	(0x00030000)

/* PCKLSEL0 bits 7:6, 00=CCLK/4, 01=CCLK/1 , 10=CCLK/2  */
#ifdef U0_PCLKDIV
# if U0_PCLKDIV == 1
#	define U0_PCLKSEL	(0x00000040)
# elif U0_PCLKDIV == 2
#	 define U0_PCLKSEL	(0x00000080)
# elif U0_PCLKDIV == 4
#	 define U0_PCLKSEL	(0x00000000)
# endif
#else
#	error "UART0 PCLK divider not set"
#endif

/* PCKLSEL1 bits 17:16, 00=CCLK/4, 01=CCLK/1 , 10=CCLK/2  */
#ifdef U2_PCLKDIV
# if U2_PCLKDIV == 1
#	define U2_PCLKSEL	(0x00010000)
# elif U2_PCLKDIV == 2
#	 define U2_PCLKSEL	(0x00020000)
# elif U2_PCLKDIV == 4
#	 define U2_PCLKSEL	(0x00000000)
# endif
#else
#	error "UART2 PCLK divider not set"
#endif




/* Universal Asynchronous Receiver Transmitter Base Addresses */
#define UART0_BASE_ADDR		0xE000C000
#define UART1_BASE_ADDR		0xE0010000
#define UART2_BASE_ADDR		0xE0078000
#define UART3_BASE_ADDR		0xE007C000

/* UART 0/1/2/3 Register Offsets */
#define UART_RBR_OFFSET		 0x00   /* R: Receive Buffer Register (DLAB=0) */
#define UART_THR_OFFSET		 0x00   /* W: Transmit Holding Register (DLAB=0) */
#define UART_DLL_OFFSET		 0x00   /* W: Divisor Latch Register (LSB, DLAB=1) */
#define UART_IER_OFFSET		 0x04   /* W: Interrupt Enable Register (DLAB=0) */
#define UART_DLM_OFFSET		 0x04   /* RW: Divisor Latch Register (MSB, DLAB=1) */
#define UART_IIR_OFFSET		 0x08   /* R: Interrupt ID Register */
#define UART_FCR_OFFSET		 0x08   /* W: FIFO Control Register */
#define UART_LCR_OFFSET		 0x0c   /* RW: Line Control Register */
#define UART_MCR_OFFSET		 0x10   /* RW: Modem Control REgister (2146/6/8 UART1 Only) */
#define UART_LSR_OFFSET		 0x14   /* R: Scratch Pad Register */
#define UART_MSR_OFFSET		 0x18   /* RW: MODEM Status Register (2146/6/8 UART1 Only) */
#define UART_SCR_OFFSET		 0x1c   /* RW: Line Status Register */
#define UART_ACR_OFFSET		 0x20   /* RW: Autobaud Control Register */
#define UART_FDR_OFFSET		 0x28   /* RW: Fractional Divider Register */
#define UART_TER_OFFSET		 0x30   /* RW: Transmit Enable Register */

/* PINSEL0 bit definitions for UART0/2 */

#define UART0_PINSEL       0x00000050  /* PINSEL0 value for UART0 */
#define UART0_PINMASK      0x000000F0  /* PINSEL0 mask for UART0 */

#define UART1_TX_PINSEL    0x40000000  /* PINSEL0 value for UART1 Tx */
#define UART1_TXPINMASK    0xC0000000  /* PINSEL0 mask for UART1 Tx */
#define UART1_RX_PINSEL    0x00000001  /* PINSEL1 value for UART1 Rx */
#define UART1_RX_PINMASK   0x00000003  /* PINSEL1 mask for UART1 Rx */
#define UART1_MODEM_PINSEL 0x00001555  /* PINSEL1 mask for UART1 Modem Interface */
#define UART1_CTS_PINMASK  0x00003FFF  /* PINSEL1 mask for UART1 Modem Interface */
//~ #define UART1_CTS_PINSEL   0x00000004  /* PINSEL1 mask for UART1 CTS */
//~ #define UART1_CTS_PINMASK  0x0000000C  /* PINSEL1 mask for UART1 CTS */
//~ #define UART1_CTS_PINSEL   0x00000010  /* PINSEL1 mask for UART1 Rx */
//~ #define UART1_CTS_PINMASK  0x00000030  /* PINSEL1 mask for UART1 Rx */
#define UART2_PINSEL       0x00500000  /* PINSEL0 value for UART2 */
#define UART2_PINMASK      0x00F00000  /* PINSEL0 mask for UART2 */

#define UART3_PINSEL       0x0F000000  /* PINSEL9 value for UART3 */
#define UART3_PINMASK      0x0F000000  /* PINSEL9 mask for UART3 */

/* Interrupt Enable Register (IER) bit definitions */

#define IER_ERBFI          (1 << 0)    /* Enable receive data available int */
#define IER_ETBEI          (1 << 1)    /* Enable THR empty Interrupt */
#define IER_ELSI           (1 << 2)    /* Enable receive line status int */
#define IER_EDSSI          (1 << 3)    /* Enable MODEM atatus interrupt (2146/6/8 UART1 Only) */
#define IER_ALLIE          0x0f        /* All interrupts */

/* Interrupt ID Register(IIR) bit definitions */

#define IIR_NO_INT         (1 << 0)    /* No interrupts pending  */
#define IIR_MS_INT         (0 << 1)    /* MODEM Status (UART1 only) */
#define IIR_THRE_INT       (1 << 1)    /* Transmit Holding Register Empty */
#define IIR_RDA_INT        (2 << 1)    /* Receive Data Available */
#define IIR_RLS_INT        (3 << 1)    /* Receive Line Status */
#define IIR_CTI_INT        (6 << 1)    /* Character Timeout Indicator */
#define IIR_MASK           0x0e

/* FIFO Control Register (FCR) bit definitions */

#define FCR_FIFO_ENABLE    (1 << 0)    /* FIFO enable */
#define FCR_RX_FIFO_RESET  (1 << 1)    /* Reset receive FIFO */
#define FCR_TX_FIFO_RESET  (1 << 2)    /* Reset transmit FIFO */
#define FCR_FIFO_TRIG1     (0 << 6)    /* Trigger @1 character in FIFO */
#define FCR_FIFO_TRIG4     (1 << 6)    /* Trigger @4 characters in FIFO */
#define FCR_FIFO_TRIG8     (2 << 6)    /* Trigger @8 characters in FIFO */
#define FCR_FIFO_TRIG14    (3 << 6)    /* Trigger @14 characters in FIFO */

/* Line Control Register (LCR) bit definitions */

#define LCR_CHAR_5         (0 << 0)    /* 5-bit character length */
#define LCR_CHAR_6         (1 << 0)    /* 6-bit character length */
#define LCR_CHAR_7         (2 << 0)    /* 7-bit character length */
#define LCR_CHAR_8         (3 << 0)    /* 8-bit character length */
#define LCR_STOP_1         (0 << 2)    /* 1 stop bit */
#define LCR_STOP_2         (1 << 2)    /* 2 stop bits */
#define LCR_PAR_NONE       (0 << 3)    /* No parity */
#define LCR_PAR_ODD        (1 << 3)    /* Odd parity */
#define LCR_PAR_EVEN       (3 << 3)    /* Even parity */
#define LCR_PAR_MARK       (5 << 3)    /* Mark "1" parity */
#define LCR_PAR_SPACE      (7 << 3)    /* Space "0" parity */
#define LCR_BREAK_ENABLE   (1 << 6)    /* Output BREAK */
#define LCR_DLAB_ENABLE    (1 << 7)    /* Enable divisor latch access */

/* Modem Control Register (MCR) bit definitions */

#define MCR_DTR            (1 << 0)    /* Data terminal ready */
#define MCR_RTS            (1 << 1)    /* Request to send */
#define MCR_LB             (1 << 4)    /* Loopback */

/* Line Status Register (LSR) bit definitions */

#define LSR_RDR            (1 << 0)    /* Receive data ready */
#define LSR_OE             (1 << 1)    /* Overrun error */
#define LSR_PE             (1 << 2)    /* Parity error */
#define LSR_FE             (1 << 3)    /* Framing error */
#define LSR_BI             (1 << 4)    /* Break interrupt */
#define LSR_THRE           (1 << 5)    /* THR empty */
#define LSR_TEMT           (1 << 6)    /* Transmitter empty */
#define LSR_RXFE           (1 << 7)    /* Error in receive FIFO */
#define LSR_ERR_MASK       0x1e

/* Modem Status Register (MSR) bit definitions */

#define MSR_DCTS           (1 << 0)    /* Delta clear to send */
#define MSR_DDSR           (1 << 1)    /* Delta data set ready */
#define MSR_TERI           (1 << 2)    /* Trailing edge ring indicator */
#define MSR_DDCD           (1 << 3)    /* Delta data carrier detect */
#define MSR_CTS            (1 << 4)    /* Clear to send */
#define MSR_DSR            (1 << 5)    /* Data set ready */
#define MSR_RI             (1 << 6)    /* Ring indicator */
#define MSR_DCD            (1 << 7)    /* Data carrier detect */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Global Function Prototypes
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_LPC2378_LPC23XX_UART_H */
