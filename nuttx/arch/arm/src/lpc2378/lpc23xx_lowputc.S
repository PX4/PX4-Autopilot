/**************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_lowputc.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include "up_internal.h"
#include "up_arch.h"
#include "lpc23xx_pinsel.h"
#include "lpc23xx_scb.h"
#include "lpc23xx_uart.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/
@    //-- Pins
@    PINSEL0 |= (0x01<<4) |  //-- P0.2  TXD0
@                (0x01<<6);  //-- P0.3  RXD0
@PCLKSEL0 |= (0x01 << 6); //-- bit 7:6 =01-> Clock div = 1 for UART0

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define UARTxBASE    UART0_BASE_ADDR
#  define PINSELECT	   LPC23XX_PINSEL0		
#  define UARTxPCLKSEL 0xE01FC1A8
#  define PCLKSEL_MASK U0_PCLKSEL_MASK
#  define UARTxPINSEL  UART0_PINSEL
#  define UARTxPINMASK UART0_PINMASK
#  define UARTxBAUD    CONFIG_UART0_BAUD
#  define UARTxBITS    CONFIG_UART0_BITS
#  define UARTxPARITY  CONFIG_UART0_PARITY
#  define UARTx2STOP   CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define UARTxBASE    UART1_BASE_ADDR
/* #  define PINSELECT	   LPC23XX_PINSEL1	only Uart0/Uart2 share same Pinsel */
#  define UARTxPCLKSEL 0xE01FC1A8
#  define PCLKSEL_MASK U1_PCLKSEL_MASK
#  define UARTxPINSEL  UART1_PINSEL
#  define UARTxPINMASK UART1_PINMASK
#  define UARTxBAUD    CONFIG_UART1_BAUD
#  define UARTxBITS    CONFIG_UART1_BITS
#  define UARTxPARITY  CONFIG_UART1_PARITY
#  define UARTx2STOP   CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define UARTxBASE    UART2_BASE_ADDR
#  define PINSELECT	   LPC23XX_PINSEL0 
#  define UARTxPCLKSEL 0xE01FC1AC
#  define PCLKSEL_MASK U2_PCLKSEL_MASK
#  define UARTxPINSEL  UART2_PINSEL
#  define UARTxPINMASK UART2_PINMASK
#  define UARTxBAUD    CONFIG_UART2_BAUD
#  define UARTxBITS    CONFIG_UART2_BITS
#  define UARTxPARITY  CONFIG_UART2_PARITY
#  define UARTx2STOP   CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define PINSELECT	   LPC23XX_PINSEL0	
#  define UARTxBASE    UART3_BASE_ADDR
#  define UARTxPCLKSEL 0xE01FC1AC
#  define PCLKSEL_MASK U2_PCLKSEL_MASK
#  define UARTxPINSEL  UART3_PINSEL
#  define UARTxPINMASK UART3_PINMASK
#  define UARTxBAUD    CONFIG_UART3_BAUD
#  define UARTxBITS    CONFIG_UART3_BITS
#  define UARTxPARITY  CONFIG_UART3_PARITY
#  define UARTx2STOP   CONFIG_UART3_2STOP
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

#if UARTxBITS == 5
#  define LCR_CHAR LCR_CHAR_5
#elif UARTxBITS == 6
#  define LCR_CHAR LCR_CHAR_6
#elif UARTxBITS == 7
#  define LCR_CHAR LCR_CHAR_7
#elif UARTxBITS == 8
#  define LCR_CHAR LCR_CHAR_8
#else
#  error "No CONFIG_UARTn_BITS Setting"
#endif

#if UARTxPARITY == 0
#  define LCR_PAR LCR_PAR_NONE
#elif UARTxPARITY == 1
#  define LCR_PAR LCR_PAR_ODD
#elif UARTxPARITY == 2
#  define LCR_PAR LCR_PAR_EVEN
#elif UARTxPARITY == 3
#  define LCR_PAR LCR_PAR_MARK
#elif UARTxPARITY == 4
#  define LCR_PAR LCR_PAR_SPACE
#else
#  error "No CONFIG_UARTn_PARITY Setting"
#endif

#if UARTx2STOP != 0
#  define LCR_STOP LCR_STOP_2
#else
#  define LCR_STOP LCR_STOP_1
#endif

#define LCR_VALUE (LCR_CHAR | LCR_PAR | LCR_STOP)
#define FCR_VALUE (FCR_FIFO_TRIG8 | FCR_TX_FIFO_RESET | \
                           FCR_RX_FIFO_RESET | FCR_FIFO_ENABLE)
@#define MULVAL (12 << 4)
@#define DIVADDVAL 3
/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowputc
 **************************************************************************/

/* This assembly language version has the advantage that it does not
 * require a C stack and uses only r0-r1.  Hence it can be used during
 * early boot phases.
 */

	.text
	.global	up_lowputc
	.type	up_lowputc, function
up_lowputc:
	/* On entry, r0 holds the character to be printed */

	ldr	r1, =UARTxBASE
	strb	r0, [r1, #UART_THR_OFFSET]

	/* Wait for the byte to be transferred */

1:	ldr	r0, [r1, #UART_LSR_OFFSET]
	ands	r0, #LSR_TEMT  /* Transmitter empty */
	beq	1b

	/* And return */

	mov     pc, lr
	.size	up_lowputc, . - up_lowputc

/* This performs basic initialization of the UART.  This can be called very
 * early in initialization because it does not depend on having a stack.  It
 * modifies r0-r2 and r14.
 */

	.text
	.globl	up_lowsetup
	.type	up_lowsetup, function
up_lowsetup:
	/* Configure PINSEL0 */

	ldr	r0, =PINSELECT  /* TODO: generalize this for different uart pins */
	ldr	r1, [r0]
	ldr	r2, =(~UARTxPINMASK)
	and	r1, r2
	
	ldr	r2, =(UARTxPINSEL)
	orr	r1, r2
	str	r1, [r0]
	
	/* Power Up Uart0 */
	ldr r0, =UARTxPCLKSEL /* PCLKSEL0 address */
	ldr r1, [r0]
	ldr r2, =(~PCLKSEL_MASK)
	and r1, r2
	
	ldr r2, =(U0_PCLKSEL)
	orr r1, r2
	str r1, [r0]	
	
/* Configure parity, data bits, stop bits and set DLAB=1 */

	ldr	r0, =UARTxBASE
	mov	r1, #(LCR_VALUE | LCR_DLAB_ENABLE)
	strb	r1, [r0, #UART_LCR_OFFSET]

/* Set the BAUD divisor */

	mov r1, #((MULVAL << 4) | DIVADDVAL)
	strb r1, [r0, #UART_FDR_OFFSET]
	
	mov	 r1, #DLMVAL
	strb r1, [r0, #UART_DLM_OFFSET]

	mov  r1, #DLLVAL
	strb r1, [r0, #UART_DLL_OFFSET]
	
/* Clear DLAB and Set format 8N1 */

	mov r1, #LCR_VALUE
	strb r1, [r0, #UART_LCR_OFFSET]
	
/* Configure the FIFOs */

	mov	r1, #FCR_VALUE 
	strb	r1, [r0, #UART_FCR_OFFSET]
	
	mov	r1, #LCR_VALUE
	strb	r1, [r0, #UART_LCR_OFFSET]

/* And return */

	mov	pc, lr
	.size	up_lowsetup, . - up_lowsetup
	.end
