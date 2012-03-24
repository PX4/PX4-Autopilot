/****************************************************************************
 * arch/arm/include/calypso/irq.h
 * Driver for Calypso IRQ controller
 *
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Stefan Richter <ichgeh@l--putt.de>
 *
 * This source code is derivated from Osmocom-BB project and was
 * relicensed as BSD with permission from original authors.
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

#ifndef __INCLUDE_NUTTX_IRQ_H
#error "This file should never be included directly! Use <nuttx/irq.h>"
#endif

#ifndef _CALYPSO_IRQ_H
#define _CALYPSO_IRQ_H

#ifndef __ASSEMBLY__

enum irq_nr {
	IRQ_WATCHDOG		= 0,
	IRQ_TIMER1		= 1,
	IRQ_TIMER2		= 2,
	IRQ_TSP_RX		= 3,
	IRQ_TPU_FRAME		= 4,
	IRQ_TPU_PAGE		= 5,
	IRQ_SIMCARD		= 6,
	IRQ_UART_MODEM		= 7,
	IRQ_KEYPAD_GPIO		= 8,
	IRQ_RTC_TIMER		= 9,
	IRQ_RTC_ALARM_I2C	= 10,
	IRQ_ULPD_GAUGING	= 11,
	IRQ_EXTERNAL		= 12,
	IRQ_SPI			= 13,
	IRQ_DMA			= 14,
	IRQ_API			= 15,
	IRQ_SIM_DETECT		= 16,
	IRQ_EXTERNAL_FIQ	= 17,
	IRQ_UART_IRDA		= 18,
	IRQ_ULPD_GSM_TIMER	= 19,
	IRQ_GEA			= 20,
	_NR_IRQS
};

#endif /* __ASSEMBLY__ */

/* Don't use _NR_IRQS!!! Won't work in preprocessor... */
#define NR_IRQS			21

#define IRQ_SYSTIMER		IRQ_TIMER2

#endif /* _CALYPSO_IRQ_H */
