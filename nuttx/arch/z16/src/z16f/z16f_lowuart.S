/*************************************************************************
 * arch/z16/src/z16f/z16f_lowuart.asm
 * Z16F UART management
 *
 *   Copyright (C) 2008, 2012 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSeqUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************/

/*************************************************************************
 * Included Files
 *************************************************************************/

#include <nuttx/config.h>
#include "chip/chip.h"
#include "common/up_internal.h"

#ifdef USE_LOWUARTINIT

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	_z16f_lowuartinit
	xref	_SYS_CLK_FREQ:EROM
#ifdef CONFIG_Z16_LOWPUTC
	xdef	_up_lowputc
#endif
#ifdef CONFIG_Z16_LOWGETC
	xdef	_up_lowgetc
#endif

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: z16f_lowuartinit
 *
 * Description:
 *   Initialize UART0 or UART1
 *
 * Parameters:
 *   None
 *
 *************************************************************************/

_z16f_lowuartinit:
	/* Calculate and set the baud rate generation register */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ld	r3, #CONFIG_UART1_BAUD	/* r3 = Selected UART1 baud */
#else
	ld	r3, #CONFIG_UART0_BAUD	/* r3 = Selected UART0 (default) baud */
#endif
	ld	r0, r3			/* r0 = baud */
	sll	r0, #3			/* r0 = baud * 8 */
	add	r0, #_SYS_CLK_FREQ	/* r3 = freq + baud * 8*/
	sll	r3, #4			/* r3 = baud * 16 */
	udiv	r0, r3			/* BRG = (freq + baud * 8)/(baud * 16) */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ld.w	Z16F_UART1_BR, r0	/* Z16F_UART1_BR = BRG */

	/* Set the GPIO Alternate Function Register Lo (AFL) register */

	ld	r0, #%30
	or.b	Z16F_GPIOD_AFL, r0	/* Z16F_GPIOD_AFL |= %30 */

	/* Enable UART receive (REN) and transmit (TEN) */

	clr.b	Z16F_UART1_CTL1 	/* Z16F_UART1_CTL1 = 0 */
	ld	r0, #(Z16F_UARTCTL0_TEN|Z16F_UARTCTL0_REN)
	ld.b	Z16F_UART1_CTL0, r0	/* Z16F_UART1_CTL0 = %c0 */
#else
	ld.w	Z16F_UART0_BR, r0	/* Z16F_UART0_BR = BRG */

	/* Set the GPIO Alternate Function Register Lo (AFL) register */

	ld	r0, #%30
	or.b	Z16F_GPIOA_AFL, r0	/* Z16F_GPIOA_AFL |= %30 */

	/* Enable UART receive (REN) and transmit (TEN) */

	clr.b	Z16F_UART0_CTL1 	/* Z16F_UART0_CTL1 = 0 */
	ld	r0, #(Z16F_UARTCTL0_TEN|Z16F_UARTCTL0_REN)
	ld.b	Z16F_UART0_CTL0, r0	/* Z16F_UART0_CTL0 = %c0 */
#endif
	ret				/* Return */
#endif /* USE_LOWUARTINIT */

/*************************************************************************
 * Name: _up_lowputc
 *
 * Description:
 *   Send one character to the selected serial console
 *
 * Parmeters:
 *   r1 = character
 *
 * Return:
 *   None
 *
 * Modifies r0 (and maybe r1)
 *
 *************************************************************************/

#ifdef CONFIG_Z16_LOWPUTC
_up_lowputc:
	/* Check if the character to output is a linefeed */

	ext.ub	r0, r1			/* r0=Character masked to 8-bits */ 
	cp	r0, #10			/* Is it a linefeed ('\n') */
	jp	ne, _z16f_xmitc		/* No? Jump to _z16f_xmitc with character in r1 */

	/* Output a carriage return before the linefeed */

	ld	r1, #13			/* Output carriage reuturn ('\r') */
	call	_z16f_xmitc		/* Call _z16f_xmitc with r1='\r' */
	ld	r1, #10			/* Restore r1=linefeed to output */
					/* Fall through to _z16f_xmitc with linefeed in r1 */

/*************************************************************************
 * Name: _z16f_xmitc
 *
 * Description:
 *   Send one character on the selected port (really a part of up_lowputc)
 *
 * Parameters:
 *   r1 = character
 *
 * Return:
 *   None
 *
 * Modifies r0
 *
 *************************************************************************/

_z16f_xmitc:
_z16f_xmitc1:
	ld	r0, Z16F_UARTSTAT0_TDRE		/* TDRE=Transmitter Data Register Empty */
#ifdef CONFIG_UART1_SERIAL_CONSOLE
	tm.b	Z16F_UART1_STAT0, r0		/* r0 = Z16F_UART1_STAT0 */
	jp	eq, _z16f_xmitc1 		/* While (!(Z16F_UART1_STAT0 & TDRE)) */
	ld.b	Z16F_UART1_TXD, r1		/* Z16F_UART1_TXD = r1 (character) */
#else
	tm.b	Z16F_UART0_STAT0, r0		/* r0 = Z16F_UART0_STAT1 */
	jp	eq, _z16f_xmitc1		/* While (!(Z16F_UART0_STAT0 & TDRE)) */
	ld.b	Z16F_UART0_TXD, r1		/* Z16F_UART0_TXD = r1 (character) */
#endif
	ret					/* Return */

#endif /* CONFIG_Z16_LOWPUTC */

/*************************************************************************
 * Name: _up_lowgetc
 *
 * Description:
 *   Get a character from the serial port
 *
 * Parmeters:
 *   None
 *
 * Return
 *   R0 = Character read
 *
 *************************************************************************/

#ifdef CONFIG_Z16_LOWGETC
_up_lowgetc:
_up_lowgetc1:
	ld	r0, #Z16F_UARTSTAT0_RDA		/* RDA=Receive data available */
#ifdef CONFIG_UART1_SERIAL_CONSOLE
	tm.b	Z16F_UART1_STAT0, r0
	jp	eq, _up_lowgetc1		/* While (!Z16F_UART1_STAT0 & RDA)) */
	ld.ub	r0, Z16F_UART1_RXD		/* r0 = Z16F_UART1_RXD */
#else
	tm.b	Z16F_UART0_STAT0,r0		/* While (!Z16F_UART0_STAT0 & RDA) */
	jp	eq, _up_lowgetc1
	ld.ub	r0, Z16F_UART0_RXD		/* r0 = Z16F_UART0_RXD */
#endif
	cp	r0, #%0d			/* Test for '\r' */
	jp	eq, _up_lowgetc2

	cp	r0, #%0d			/* Test \r + high bit */
	jp	ne, _up_lowgetc3
_up_lowgetc2:
	ld	r0, #%0a			/* Convert '\r' to '\n' */
_up_lowgetc3:					/* Return value in r0 */
	ret					/* Return */
#endif

	end

