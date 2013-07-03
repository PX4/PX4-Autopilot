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

static volatile bool	sending = false;

static void		dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static DMA_HANDLE	tx_dma;
static DMA_HANDLE	rx_dma;

#define MAX_RW_REGS	32 // by agreement w/FMU

#pragma pack(push, 1)
struct IOPacket {
	uint8_t 	count;
#define PKT_CTRL_WRITE			(1<<7)
	uint8_t 	spare;
	uint8_t 	page;
	uint8_t 	offset;
	uint16_t	regs[MAX_RW_REGS];
};
#pragma pack(pop)

static struct IOPacket	dma_packet;

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(PX4FMU_SERIAL_BASE + _x))
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
	/* allocate DMA */
	tx_dma = stm32_dmachannel(PX4FMU_SERIAL_TX_DMA);
	rx_dma = stm32_dmachannel(PX4FMU_SERIAL_RX_DMA);

	/* configure pins for serial use */
	stm32_configgpio(PX4FMU_SERIAL_TX_GPIO);
	stm32_configgpio(PX4FMU_SERIAL_RX_GPIO);

	/* reset and configure the UART */
	rCR1 = 0;
	rCR2 = 0;
	rCR3 = 0;

	/* configure line speed */
	uint32_t usartdiv32 = PX4FMU_SERIAL_CLOCK / (PX4FMU_SERIAL_BITRATE / 2);
	uint32_t mantissa = usartdiv32 >> 5;
	uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
	rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);

	/* enable UART and DMA */
	rCR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	/* configure DMA */
	stm32_dmasetup(
		tx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		sizeof(dma_packet),
		DMA_CCR_DIR		|
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS);

	stm32_dmasetup(
		rx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		sizeof(dma_packet),
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS);

	/* start receive DMA ready for the first packet */
	stm32_dmastart(rx_dma, dma_callback, NULL, false);

	debug("serial init");
}

void
interface_tick()
{
	/* XXX look for stuck/damaged DMA and reset? */
}

static void
dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	if (!(status & DMA_STATUS_TCIF)) {
		/* XXX what do we do here? it's fatal! */
		return;
	}

	/* if this is transmit-complete, make a note */
	if (handle == tx_dma) {
		sending = false;
		return;
	}

	/* we just received a request; sort out what to do */
	/* XXX implement check byte */
	/* XXX if we care about overruns, check the UART received-data-ready bit */
	bool send_reply = false;
	if (dma_packet.count & PKT_CTRL_WRITE) {

		/* it's a blind write - pass it on */
		registers_set(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], dma_packet.count);
	} else {

		/* it's a read - get register pointer for reply */
		int result;
		unsigned count;
		uint16_t *registers;

		result = registers_get(dma_packet.page, dma_packet.offset, &registers, &count);
		if (result < 0)
			count = 0;

		/* constrain reply to packet size */
		if (count > MAX_RW_REGS)
			count = MAX_RW_REGS;

		/* copy reply registers into DMA buffer */
		send_reply = true;
		memcpy((void *)&dma_packet.regs[0], registers, count);
		dma_packet.count = count;
	}

	/* re-set DMA for reception first, so we are ready to receive before we start sending */
	stm32_dmastart(rx_dma, dma_callback, NULL, false);

	/* if we have a reply to send, start that now */
	if (send_reply)
		stm32_dmastart(tx_dma, dma_callback, NULL, false);
}
