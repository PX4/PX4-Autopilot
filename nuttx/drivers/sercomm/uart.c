/****************************************************************************
 * drivers/sercomm/uart.c
 * Calypso DBB internal UART Driver
 *
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2010 by Ingo Albrecht <prom@berlin.ccc.de>
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

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/calypso/memory.h>
#include <arch/calypso/debug.h>

#include <arch/calypso/defines.h>
//#include <arch/calypso/console.h>
#include <nuttx/sercomm/sercomm.h>

#include "uart.h"

#define BASE_ADDR_UART_MODEM	0xffff5000
#define OFFSET_IRDA		0x800

#define UART_REG(n,m)	(BASE_ADDR_UART_MODEM + ((n)*OFFSET_IRDA)+(m))

#define LCR7BIT		0x80
#define LCRBFBIT	0x40
#define MCR6BIT		0x20
#define REG_OFFS(m)	((m) & ~(LCR7BIT|LCRBFBIT|MCR6BIT))
/* read access LCR[7] = 0 */
enum uart_reg {
	RHR	= 0,
	IER	= 1,
	IIR	= 2,
	LCR	= 3,
	MCR	= 4,
	LSR	= 5,
	MSR	= 6,
	SPR	= 7,
	MDR1	= 8,
	DMR2	= 9,
	SFLSR	= 0x0a,
	RESUME	= 0x0b,
	SFREGL	= 0x0c,
	SFREGH	= 0x0d,
	BLR	= 0x0e,
	ACREG	= 0x0f,
	SCR	= 0x10,
	SSR	= 0x11,
	EBLR	= 0x12,
/* read access LCR[7] = 1 */
	DLL	= RHR | LCR7BIT,
	DLH	= IER | LCR7BIT,
	DIV1_6	= ACREG | LCR7BIT,
/* read/write access LCR[7:0] = 0xbf */
	EFR	= IIR | LCRBFBIT,
	XON1	= MCR | LCRBFBIT,
	XON2	= LSR | LCRBFBIT,
	XOFF1	= MSR | LCRBFBIT,
	XOFF2 	= SPR | LCRBFBIT,
/* read/write access if EFR[4] = 1 and MCR[6] = 1 */
	TCR	= MSR | MCR6BIT,
	TLR	= SPR | MCR6BIT,
};
/* write access LCR[7] = 0 */
#define THR	RHR
#define FCR	IIR		/* only if EFR[4] = 1 */
#define TXFLL	SFLSR
#define TXFLH	RESUME
#define RXFLL	SFREGL
#define RXFLH	SFREGH

enum fcr_bits {
	FIFO_EN		= (1 << 0),
	RX_FIFO_CLEAR	= (1 << 1),
	TX_FIFO_CLEAR	= (1 << 2),
	DMA_MODE	= (1 << 3),
};
#define TX_FIFO_TRIG_SHIFT	4
#define RX_FIFO_TRIG_SHIFT	6

enum iir_bits {
	IIR_INT_PENDING			= 0x01,
	IIR_INT_TYPE			= 0x3E,
	IIR_INT_TYPE_RX_STATUS_ERROR 	= 0x06,
	IIR_INT_TYPE_RX_TIMEOUT		= 0x0C,
	IIR_INT_TYPE_RHR		= 0x04,
	IIR_INT_TYPE_THR		= 0x02,
	IIR_INT_TYPE_MSR		= 0x00,
	IIR_INT_TYPE_XOFF		= 0x10,
	IIR_INT_TYPE_FLOW		= 0x20,
	IIR_FCR0_MIRROR			= 0xC0,
};

#define UART_REG_UIR	0xffff6000

/* enable or disable the divisor latch for access to DLL, DLH */
static void uart_set_lcr7bit(int uart, int on)
{
	uint8_t reg;

	reg = readb(UART_REG(uart, LCR));
	if (on)
		reg |= (1 << 7);
	else
		reg &= ~(1 << 7);
	writeb(reg, UART_REG(uart, LCR));
}

static uint8_t old_lcr;
static void uart_set_lcr_bf(int uart, int on)
{
	if (on) {
		old_lcr = readb(UART_REG(uart, LCR));
		writeb(0xBF, UART_REG(uart, LCR));
	} else {
		writeb(old_lcr, UART_REG(uart, LCR));
	}
}

/* Enable or disable the TCR_TLR latch bit in MCR[6] */
static void uart_set_mcr6bit(int uart, int on)
{
	uint8_t mcr;
	/* we assume EFR[4] is always set to 1 */
	mcr = readb(UART_REG(uart, MCR));
	if (on)
		mcr |= (1 << 6);
	else
		mcr &= ~(1 << 6);
	writeb(mcr, UART_REG(uart, MCR));
}

static void uart_reg_write(int uart, enum uart_reg reg, uint8_t val)
{
	if (reg & LCRBFBIT)
		uart_set_lcr_bf(uart, 1);
	else if (reg & LCR7BIT)
		uart_set_lcr7bit(uart, 1);
	else if (reg & MCR6BIT)
		uart_set_mcr6bit(uart, 1);

	writeb(val, UART_REG(uart, REG_OFFS(reg)));

	if (reg & LCRBFBIT)
		uart_set_lcr_bf(uart, 0);
	else if (reg & LCR7BIT)
		uart_set_lcr7bit(uart, 0);
	else if (reg & MCR6BIT)
		uart_set_mcr6bit(uart, 0);
}

/* read from a UART register, applying any required latch bits */
static uint8_t uart_reg_read(int uart, enum uart_reg reg)
{
	uint8_t ret;

	if (reg & LCRBFBIT)
		uart_set_lcr_bf(uart, 1);
	else if (reg & LCR7BIT)
		uart_set_lcr7bit(uart, 1);
	else if (reg & MCR6BIT)
		uart_set_mcr6bit(uart, 1);

	ret = readb(UART_REG(uart, REG_OFFS(reg)));

	if (reg & LCRBFBIT)
		uart_set_lcr_bf(uart, 0);
	else if (reg & LCR7BIT)
		uart_set_lcr7bit(uart, 0);
	else if (reg & MCR6BIT)
		uart_set_mcr6bit(uart, 0);

	return ret;
}

#if 0
static void uart_irq_handler_cons(__unused enum irq_nr irqnr)
{
	const uint8_t uart = CONS_UART_NR;
	uint8_t iir;

	//uart_putchar_nb(uart, 'U');

	iir = uart_reg_read(uart, IIR);
	if (iir & IIR_INT_PENDING)
		return;

	switch (iir & IIR_INT_TYPE) {
	case IIR_INT_TYPE_RHR:
		break;
	case IIR_INT_TYPE_THR:
		if (cons_rb_flush() == 1) {
			/* everything was flushed, disable THR IRQ */
			uint8_t ier = uart_reg_read(uart, IER);
			ier &= ~(1 << 1);
			uart_reg_write(uart, IER, ier);
		}
		break;
	case IIR_INT_TYPE_MSR:
		break;
	case IIR_INT_TYPE_RX_STATUS_ERROR:
		break;
	case IIR_INT_TYPE_RX_TIMEOUT:
		break;
	case IIR_INT_TYPE_XOFF:
		break;
	}
}
#endif

static void uart_irq_handler_sercomm(__unused enum irq_nr irqnr, __unused void *context)
{
	const uint8_t uart = SERCOMM_UART_NR;
	uint8_t iir, ch;

	//uart_putchar_nb(uart, 'U');

	iir = uart_reg_read(uart, IIR);
	if (iir & IIR_INT_PENDING)
		return;

	switch (iir & IIR_INT_TYPE) {
	case IIR_INT_TYPE_RX_TIMEOUT:
	case IIR_INT_TYPE_RHR:
		/* as long as we have rx data available */
		while (uart_getchar_nb(uart, &ch)) {
			if (sercomm_drv_rx_char(ch) < 0) {
				/* sercomm cannot receive more data right now */
				uart_irq_enable(uart, UART_IRQ_RX_CHAR, 0);
			}
		}
		break;
	case IIR_INT_TYPE_THR:
		/* as long as we have space in the FIFO */
		while (!uart_tx_busy(uart)) {
			/* get a byte from sercomm */
			if (!sercomm_drv_pull(&ch)) {
				/* no more bytes in sercomm, stop TX interrupts */
				uart_irq_enable(uart, UART_IRQ_TX_EMPTY, 0);
				break;
			}
			/* write the byte into the TX FIFO */
			uart_putchar_nb(uart, ch);
		}
		break;
	case IIR_INT_TYPE_MSR:
		printf("UART IRQ MSR\n");
		break;
	case IIR_INT_TYPE_RX_STATUS_ERROR:
		printf("UART IRQ RX_SE\n");
		break;
	case IIR_INT_TYPE_XOFF:
		printf("UART IRQXOFF\n");
		break;
	}
}

static const uint8_t uart2irq[] = {
	[0]	= IRQ_UART_IRDA,
	[1]	= IRQ_UART_MODEM,
};

void uart_init(uint8_t uart, uint8_t interrupts)
{
	uint8_t irq = uart2irq[uart];

	uart_reg_write(uart, IER, 0x00);

	if (uart == SERCOMM_UART_NR) {
		sercomm_init();
		irq_attach(IRQ_UART_MODEM, (xcpt_t)uart_irq_handler_sercomm);
		up_enable_irq(IRQ_UART_MODEM);
		uart_irq_enable(uart, UART_IRQ_RX_CHAR, 1);
	}

#if 0
	if (uart == CONS_UART_NR) {
		cons_init();
		if(interrupts) {
			irq_register_handler(irq, &uart_irq_handler_cons);
			irq_config(irq, 0, 0, 0xff);
			irq_enable(irq);
		}
	} else {
		sercomm_init();
		if(interrupts) {
			irq_register_handler(irq, &uart_irq_handler_sercomm);
			irq_config(irq, 0, 0, 0xff);
			irq_enable(irq);
		}
		uart_irq_enable(uart, UART_IRQ_RX_CHAR, 1);
	}
#endif
#if 0
	if (uart == 1) {
		/* assign UART to MCU and unmask interrupts*/
		writeb(UART_REG_UIR, 0x00);
	}
#endif

	/* if we don't initialize these, we get strange corruptions in the
	   received data... :-( */
	uart_reg_write(uart,  MDR1, 0x07); /* turn off UART */
	uart_reg_write(uart,  XON1, 0x00); /* Xon1/Addr Register */
	uart_reg_write(uart,  XON2, 0x00); /* Xon2/Addr Register */
	uart_reg_write(uart, XOFF1, 0x00); /* Xoff1 Register */
	uart_reg_write(uart, XOFF2, 0x00); /* Xoff2 Register */
	uart_reg_write(uart,   EFR, 0x00); /* Enhanced Features Register */

	/* select  UART mode */
	uart_reg_write(uart, MDR1, 0);
	/* no XON/XOFF flow control, ENHANCED_EN, no auto-RTS/CTS */
	uart_reg_write(uart, EFR, (1 << 4));
	/* enable Tx/Rx FIFO, Tx trigger at 56 spaces, Rx trigger at 60 chars */
	uart_reg_write(uart, FCR, FIFO_EN | RX_FIFO_CLEAR | TX_FIFO_CLEAR |
			(3 << TX_FIFO_TRIG_SHIFT) | (3 << RX_FIFO_TRIG_SHIFT));

	/* THR interrupt only when TX FIFO and TX shift register are empty */
	uart_reg_write(uart, SCR, (1 << 0));// | (1 << 3));

	/* 8 bit, 1 stop bit, no parity, no break */
	uart_reg_write(uart, LCR, 0x03);

	uart_set_lcr7bit(uart, 0);
}

void uart_poll(uint8_t uart) {
/*	if(uart == CONS_UART_NR) {
		uart_irq_handler_cons(0);
	} else
*/	{
		uart_irq_handler_sercomm(0, NULL);
	}
}

void uart_irq_enable(uint8_t uart, enum uart_irq irq, int on)
{
	uint8_t ier = uart_reg_read(uart, IER);
	uint8_t mask = 0;

	switch (irq) {
	case UART_IRQ_TX_EMPTY:
		mask = (1 << 1);
		break;
	case UART_IRQ_RX_CHAR:
		mask = (1 << 0);
		break;
	}

	if (on)
		ier |= mask;
	else
		ier &= ~mask;

	uart_reg_write(uart, IER, ier);
}


void uart_putchar_wait(uint8_t uart, int c)
{
	/* wait while TX FIFO indicates full */
	while (readb(UART_REG(uart, SSR)) & 0x01) { }

	/* put character in TX FIFO */
	writeb(c, UART_REG(uart, THR));
}

int uart_putchar_nb(uint8_t uart, int c)
{
	/* if TX FIFO indicates full, abort */
	if (readb(UART_REG(uart, SSR)) & 0x01)
		return 0;

	writeb(c, UART_REG(uart, THR));
	return 1;
}

int uart_getchar_nb(uint8_t uart, uint8_t *ch)
{
	uint8_t lsr;

	lsr = readb(UART_REG(uart, LSR));

	/* something strange happened */
	if (lsr & 0x02)
		printf("LSR RX_OE\n");
	if (lsr & 0x04)
		printf("LSR RX_PE\n");
	if (lsr & 0x08)
		printf("LSR RX_FE\n");
	if (lsr & 0x10)
		printf("LSR RX_BI\n");
	if (lsr & 0x80)
		printf("LSR RX_FIFO_STS\n");

	/* is the Rx FIFO empty? */
	if (!(lsr & 0x01))
		return 0;

	*ch = readb(UART_REG(uart, RHR));
	//printf("getchar_nb(%u) = %02x\n", uart, *ch);
	return 1;
}

int uart_tx_busy(uint8_t uart)
{
	if (readb(UART_REG(uart, SSR)) & 0x01)
		return 1;
	return 0;
}

static const uint16_t divider[] = {
	[UART_38400]	= 21,	/*   38,690 */
	[UART_57600]	= 14,	/*   58,035 */
	[UART_115200]	= 7,	/*  116,071 */
	[UART_230400]	= 4,	/*  203,125! (-3% would be 223,488) */
	[UART_460800]	= 2,	/*  406,250! (-3% would be 446,976) */
	[UART_921600]	= 1,	/*  812,500! (-3% would be 893,952) */
};

int uart_baudrate(uint8_t uart, enum uart_baudrate bdrt)
{
	uint16_t div;

	if (bdrt > ARRAY_SIZE(divider))
		return -1;

	div = divider[bdrt];
	uart_set_lcr7bit(uart, 1);
	writeb(div & 0xff, UART_REG(uart, DLL));
	writeb(div >> 8, UART_REG(uart, DLH));
	uart_set_lcr7bit(uart, 0);

	return 0;
}
