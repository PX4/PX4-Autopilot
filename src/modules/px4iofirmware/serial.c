/****************************************************************************
 *
 *   Copyright (C) 2012,2013 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file serial.c
 *
 * Serial communication for the PX4IO module.
 */

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

/* XXX might be able to prune these */
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>
#include <stm32_internal.h>
#include <systemlib/hx_stream.h>

//#define DEBUG
#include "px4io.h"

static hx_stream_t	if_stream;
static volatile bool	sending = false;

static int		serial_interrupt(int vector, void *context);
static void		serial_callback(void *arg, const void *data, unsigned length);

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(SERIAL_BASE + _x))
#define rSR		REG(STM32_USART_SR_OFFSET)
#define rDR		REG(STM32_USART_DR_OFFSET)
#define rBRR		REG(STM32_USART_BRR_OFFSET)
#define rCR1		REG(STM32_USART_CR1_OFFSET)
#define rCR2		REG(STM32_USART_CR2_OFFSET)
#define rCR3		REG(STM32_USART_CR3_OFFSET)
#define rGTPR		REG(STM32_USART_GTPR_OFFSET)

void
interface_init(void)
{

	/* XXX do serial port init here */

	irq_attach(SERIAL_VECTOR, serial_interrupt);
	if_stream = hx_stream_init(-1, serial_callback, NULL);

	/* XXX add stream stats counters? */

	debug("serial init");
}

void
interface_tick()
{
	/* XXX nothing interesting to do here */
}

static int
serial_interrupt(int vector, void *context)
{
	uint32_t sr = rSR;

	if (sr & USART_SR_TXE) {
		int c = hx_stream_send_next(if_stream);
		if (c == -1) {
			/* no more bytes to send, not interested in interrupts now */
			rCR1 &= ~USART_CR1_TXEIE;
			sending = false;
		} else {
			rDR = c;
		}
	}

	if (sr & USART_SR_RXNE) {
		uint8_t c = rDR;

		hx_stream_rx(if_stream, c);
	}
	return 0;
}

static void
serial_callback(void *arg, const void *data, unsigned length)
{
	const uint8_t *message = (const uint8_t *)data;

	/* malformed frame, ignore it */
	if (length < 2)
		return;

	/* we got a new request while we were still sending the last reply - not supported */
	if (sending)
		return;

	/* reads are page / offset / length */
	if (length == 3) {
		uint16_t *registers;
		unsigned count;

		/* get registers for response, send an empty reply on error */
		if (registers_get(message[0], message[1], &registers, &count) < 0)
			count = 0;

		/* constrain count to requested size or message limit */
		if (count > message[2])
			count = message[2];
		if (count > HX_STREAM_MAX_FRAME)
			count = HX_STREAM_MAX_FRAME;

		/* start sending the reply */
		sending = true;
		hx_stream_reset(if_stream);
		hx_stream_start(if_stream, registers, count * 2 + 2);

		/* enable the TX-ready interrupt */
		rCR1 |= USART_CR1_TXEIE;
		return;

	} else {

		/* it's a write operation, pass it to the register API */
		registers_set(message[0], 
				message[1],
				(const uint16_t *)&message[2],
				(length - 2) / 2);
	}
}