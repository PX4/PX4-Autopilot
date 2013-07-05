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
#include <systemlib/perf_counter.h>

//#define DEBUG
#include "px4io.h"

static volatile bool	sending = false;

static perf_counter_t	pc_rx;
static perf_counter_t	pc_errors;
static perf_counter_t	pc_ore;
static perf_counter_t	pc_fe;
static perf_counter_t	pc_ne;
static perf_counter_t	pc_regerr;

static void		rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static void		tx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static DMA_HANDLE	tx_dma;
static DMA_HANDLE	rx_dma;

static int		serial_interrupt(int irq, void *context);
static void		dma_reset(void);

/* if we spend this many ticks idle, reset the DMA */
static unsigned		idle_ticks;

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
	pc_rx = perf_alloc(PC_COUNT, "rx count");
	pc_errors = perf_alloc(PC_COUNT, "errors");
	pc_ore = perf_alloc(PC_COUNT, "overrun");
	pc_fe = perf_alloc(PC_COUNT, "framing");
	pc_ne = perf_alloc(PC_COUNT, "noise");
	pc_regerr = perf_alloc(PC_COUNT, "regerr");

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

	/* clear status/errors */
	(void)rSR;
	(void)rDR;

	/* configure line speed */
	uint32_t usartdiv32 = PX4FMU_SERIAL_CLOCK / (PX4FMU_SERIAL_BITRATE / 2);
	uint32_t mantissa = usartdiv32 >> 5;
	uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
	rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);

	/* connect our interrupt */
	irq_attach(PX4FMU_SERIAL_VECTOR, serial_interrupt);
	up_enable_irq(PX4FMU_SERIAL_VECTOR);

	/* enable UART and DMA */
	/*rCR3 = USART_CR3_EIE; */
	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE /*| USART_CR1_IDLEIE*/;

#if 0	/* keep this for signal integrity testing */
	for (;;) {
		while (!(rSR & USART_SR_TXE))
			;
		rDR = 0xfa;
		while (!(rSR & USART_SR_TXE))
			;
		rDR = 0xa0;
	}
#endif

	/* configure RX DMA and return to listening state */
	dma_reset();

	debug("serial init");
}

void
interface_tick()
{
	/* XXX look for stuck/damaged DMA and reset? */
	if (idle_ticks++ > 100) {
		dma_reset();
		idle_ticks = 0;
	}
}

static void
tx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	sending = false;
	rCR3 &= ~USART_CR3_DMAT;
}

static void
rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	/* we just received a request; sort out what to do */

	rCR3 &= ~USART_CR3_DMAR;
	idle_ticks = 0;

	/* work out how big the packet actually is */
	//unsigned rx_length = sizeof(IOPacket) - stm32_dmaresidual(rx_dma);

	/* XXX implement check byte */

	perf_count(pc_rx);

	/* default to not sending a reply */
	bool send_reply = false;
	if (dma_packet.count & PKT_CTRL_WRITE) {

		dma_packet.count &= ~PKT_CTRL_WRITE;

		/* it's a blind write - pass it on */
		if (registers_set(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], dma_packet.count))
			perf_count(pc_regerr);

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
	/* XXX latency here could mean loss of back-to-back writes; do we want to always send an ack? */
	/* XXX always sending an ack would simplify the FMU side (always wait for reply) too */
	dma_reset();

	/* if we have a reply to send, start that now */
	if (send_reply) {
		stm32_dmasetup(
			tx_dma,
			(uint32_t)&rDR,
			(uint32_t)&dma_packet,
			sizeof(dma_packet),		/* XXX cut back to actual transmit size */
			DMA_CCR_DIR		|
			DMA_CCR_MINC		|
			DMA_CCR_PSIZE_8BITS	|
			DMA_CCR_MSIZE_8BITS);
		sending = true;
		stm32_dmastart(tx_dma, tx_dma_callback, NULL, false);
		rCR3 |= USART_CR3_DMAT;
	}
}

static int
serial_interrupt(int irq, void *context)
{
	uint32_t sr = rSR;	/* get UART status register */

	/* handle error/exception conditions */
	if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_IDLE)) {
		/* read DR to clear status */
		(void)rDR;
	} else {
		ASSERT(0);
	}

	if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		USART_SR_NE |		/* noise error - we have lost a byte due to noise */
		USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */

		perf_count(pc_errors);
		if (sr & USART_SR_ORE)
			perf_count(pc_ore);
		if (sr & USART_SR_NE)
			perf_count(pc_ne);
		if (sr & USART_SR_FE)
			perf_count(pc_fe);

		/* reset DMA state machine back to listening-for-packet */		
		dma_reset();

		/* don't attempt to handle IDLE if it's set - things went bad */
		return 0;
	}

	if (sr & USART_SR_IDLE) {

		/* XXX if there was DMA transmission still going, this is an error */

		/* stop the receive DMA */
		stm32_dmastop(rx_dma);

		/* and treat this like DMA completion */
		rx_dma_callback(rx_dma, DMA_STATUS_TCIF, NULL);
	}

	return 0;
}

static void
dma_reset(void)
{
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	(void)rSR;
	(void)rDR;
	(void)rDR;

	/* kill any pending DMA */
	stm32_dmastop(tx_dma);
	stm32_dmastop(rx_dma);

	/* reset the RX side */
	stm32_dmasetup(
		rx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		sizeof(dma_packet),
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS);

	/* start receive DMA ready for the next packet */
	stm32_dmastart(rx_dma, rx_dma_callback, NULL, false);
	rCR3 |= USART_CR3_DMAR;
}