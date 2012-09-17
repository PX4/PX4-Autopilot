/**************************************************************************
 * arch/arm/src/dm320/dm320_lowputc.S
 * arch/arm/src/chip/dm320_lowputc.S
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
#include "up_internal.h"
#include "up_arch.h"

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

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ldr	r2, =DM320_UART1_REGISTER_BASE	/* r2=UART1 base */
#else
	ldr	r2, =DM320_UART0_REGISTER_BASE	/* r2=UART0 base */
#endif

	/* Poll the TX fifo trigger level bit of the UART_SSR
	 * register.  When the bit is non-zero, the TX FIFO is no
	 * longer full
	 */

1:	ldrh	r1, [r2, #UART_SR]
	tst	r1, #UART_SR_TFTI
	beq	1b

	/* Send the character by writing it into the UART_DTRR
	 * register.
	 */

	strh	r0, [r2, #UART_DTRR]

	/* Wait for the tranmsit regiser to be emptied. This is
	 * detemined when TX register empty bit of the SR is zero.
	 */

2:	ldrh	r1, [r2, #UART_SR]
	tst	r1, #UART_SR_TREF
	bne	2b

	/* If the character that we just sent was a linefeed,
	 * then send a carriage return as well.
	 */

	teq     r0, #'\n'
	moveq   r0, #'\r'
	beq     1b

	/* And return */

	mov     pc, lr

