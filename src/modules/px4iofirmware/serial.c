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

static perf_counter_t	pc_txns;
static perf_counter_t	pc_errors;
static perf_counter_t	pc_ore;
static perf_counter_t	pc_fe;
static perf_counter_t	pc_ne;
static perf_counter_t	pc_idle;
static perf_counter_t	pc_badidle;
static perf_counter_t	pc_regerr;
static perf_counter_t	pc_crcerr;

static void		rx_handle_packet(void);
static void		rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static DMA_HANDLE	tx_dma;
static DMA_HANDLE	rx_dma;

static int		serial_interrupt(int irq, void *context);
static void		dma_reset(void);

/* if we spend this many ticks idle, reset the DMA */
static unsigned		idle_ticks;

#define PKT_MAX_REGS	32 // by agreement w/FMU

#pragma pack(push, 1)
struct IOPacket {
	uint8_t 	count_code;
	uint8_t 	crc;
	uint8_t 	page;
	uint8_t 	offset;
	uint16_t	regs[PKT_MAX_REGS];
};
#pragma pack(pop)

#define PKT_CODE_READ		0x00	/* FMU->IO read transaction */
#define PKT_CODE_WRITE		0x40	/* FMU->IO write transaction */
#define PKT_CODE_SUCCESS	0x00	/* IO->FMU success reply */
#define PKT_CODE_CORRUPT	0x40	/* IO->FMU bad packet reply */
#define PKT_CODE_ERROR		0x80	/* IO->FMU register op error reply */

#define PKT_CODE_MASK		0xc0
#define PKT_COUNT_MASK		0x3f

#define PKT_COUNT(_p)	((_p).count_code & PKT_COUNT_MASK)
#define PKT_CODE(_p)	((_p).count_code & PKT_CODE_MASK)
#define PKT_SIZE(_p)	((uint8_t *)&((_p).regs[PKT_COUNT(_p)]) - ((uint8_t *)&(_p)))

static uint8_t		crc_packet(void);

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
	pc_txns = perf_alloc(PC_ELAPSED, "txns");
	pc_errors = perf_alloc(PC_COUNT, "errors");
	pc_ore = perf_alloc(PC_COUNT, "overrun");
	pc_fe = perf_alloc(PC_COUNT, "framing");
	pc_ne = perf_alloc(PC_COUNT, "noise");
	pc_idle = perf_alloc(PC_COUNT, "idle");
	pc_badidle = perf_alloc(PC_COUNT, "badidle");
	pc_regerr = perf_alloc(PC_COUNT, "regerr");
	pc_crcerr = perf_alloc(PC_COUNT, "crcerr");

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
	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;

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
rx_handle_packet(void)
{
	/* check packet CRC */
	uint8_t crc = dma_packet.crc;
	dma_packet.crc = 0;
	if (crc != crc_packet()) {
		perf_count(pc_crcerr);

		/* send a CRC error reply */
		dma_packet.count_code = PKT_CODE_CORRUPT;
		dma_packet.page = 0xff;
		dma_packet.offset = 0xff;

		return;
	}

	if (PKT_CODE(dma_packet) == PKT_CODE_WRITE) {

		/* it's a blind write - pass it on */
		if (registers_set(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], PKT_COUNT(dma_packet))) {
			perf_count(pc_regerr);
			dma_packet.count_code = PKT_CODE_ERROR;
		} else {
			dma_packet.count_code = PKT_CODE_SUCCESS;
		}
		return;
	} 

	if (PKT_CODE(dma_packet) == PKT_CODE_READ) {

		/* it's a read - get register pointer for reply */
		unsigned count;
		uint16_t *registers;

		if (registers_get(dma_packet.page, dma_packet.offset, &registers, &count) < 0) {
			perf_count(pc_regerr);
			dma_packet.count_code = PKT_CODE_ERROR;
		} else {
			/* constrain reply to requested size */
			if (count > PKT_MAX_REGS)
				count = PKT_MAX_REGS;
			if (count > PKT_COUNT(dma_packet))
				count = PKT_COUNT(dma_packet);

			/* copy reply registers into DMA buffer */
			memcpy((void *)&dma_packet.regs[0], registers, count);
			dma_packet.count_code = count | PKT_CODE_SUCCESS;
		}
		return;
	}

	/* send a bad-packet error reply */
	dma_packet.count_code = PKT_CODE_CORRUPT;
	dma_packet.page = 0xff;
	dma_packet.offset = 0xfe;
}

static void
rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	perf_begin(pc_txns);

	/* disable UART DMA */
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

	/* reset the idle counter */
	idle_ticks = 0;

	/* handle the received packet */
	rx_handle_packet();

	/* re-set DMA for reception first, so we are ready to receive before we start sending */
	dma_reset();

	/* send the reply to the previous request */
	dma_packet.crc = 0;
	dma_packet.crc = crc_packet();
	stm32_dmasetup(
		tx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		PKT_SIZE(dma_packet),
		DMA_CCR_DIR		|
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS);
	stm32_dmastart(tx_dma, NULL, NULL, false);
	rCR3 |= USART_CR3_DMAT;

	perf_end(pc_txns);
}

static int
serial_interrupt(int irq, void *context)
{
	uint32_t sr = rSR;	/* get UART status register */
	(void)rDR;		/* required to clear any of the interrupt status that brought us here */

#if 0
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
#endif
	if (sr & USART_SR_IDLE) {

		/* the packet might have been short - go check */
		unsigned length = sizeof(dma_packet) - stm32_dmaresidual(rx_dma);
		if ((length < 1) || (length < PKT_SIZE(dma_packet))) {
			perf_count(pc_badidle);
			return 0;
		}

		perf_count(pc_idle);

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

static const uint8_t crc8_tab[256] =
{
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

static uint8_t
crc_packet()
{
	uint8_t *end = (uint8_t *)(&dma_packet.regs[PKT_COUNT(dma_packet)]);
	uint8_t *p = (uint8_t *)&dma_packet;
	uint8_t c = 0;

	while (p < end)
		c = crc8_tab[c ^ *(p++)];

	return c;
}
