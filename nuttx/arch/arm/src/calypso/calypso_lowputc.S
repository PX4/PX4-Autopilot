/**************************************************************************
 * calypso/calypso_lowputc.S
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
 *   Author: Stefan Richter <ichgeh@l--putt.de>
 *
 * based on: c5471/c5471_lowputc.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

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

/* This assembly language version has the advantage that it can does not
 * require a C stack and uses only r0-r1.  Hence it can be used during
 * early boot phases.
 */

	.text
	.global	up_lowputc
	.type	up_lowputc, function
up_lowputc:
	/* On entry, r0 holds the character to be printed */

#ifdef CONFIG_SERIAL_IRDA_CONSOLE
	ldr	r2, =UART_IRDA_BASE	/* r2=IRDA UART base */
#else
	ldr	r2, =UART_MODEM_BASE	/* r2=Modem UART base */
#endif

	/* Poll bit 0 of the UART_SSR register.  When the bit
	 * is clear, the TX FIFO is no longer full
	 */

1:	ldrb	r1, [r2, #UART_SSR_OFFS]
	tst	r1, #UART_SSR_TXFULL
	bne	1b

	/* Send the character by writing it into the UART_THR
	 * register.
	 */

	strb     r0, [r2, #UART_THR_OFFS]

	/* Wait for the tranmsit holding regiser (THR) to be
	 * emptied. This is detemined when bit 6 of the LSR
	 * is set.
	 */

2:	ldrb     r1, [r2, #UART_LSR_OFFS]
	tst     r1, #0x00000020
	beq     2b

	/* If the character that we just sent was a linefeed,
	 * then send a carriage return as well.
	 */

	teq     r0, #'\n'
	moveq   r0, #'\r'
	beq     1b

	/* And return */

	mov     pc, lr

