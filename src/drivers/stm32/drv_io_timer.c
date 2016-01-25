/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include "drv_io_timer.h"

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

#define MAX_CHANNELS_PER_TIMER 4

#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)

#define rCR1(_tmr)    	REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)    	REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)   	REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)   	REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)     	REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)    	REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)  	REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)  	REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)   	REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)    	REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)    	REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)    	REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)   	REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)   	REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)   	REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)   	REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)    	REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)   	REG(_tmr, STM32_GTIM_DMAR_OFFSET)
#define rBDTR(_tmr)		REG(_tmr, STM32_ATIM_BDTR_OFFSET)

#define GTIM_SR_CCIF (GTIM_SR_CC4IF|GTIM_SR_CC3IF|GTIM_SR_CC2IF|GTIM_SR_CC1IF)
#define GTIM_SR_CCOF (GTIM_SR_CC4OF|GTIM_SR_CC3OF|GTIM_SR_CC2OF|GTIM_SR_CC1OF)


typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];


static int io_timer_handler(uint16_t timer_index)
{
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending and enabled? */

	uint16_t statusr = rSR(timer_index);
	uint16_t enabled =  rDIER(timer_index) & GTIM_SR_CCIF;

	/* Iterate over the timer_io_channels table */

	for (int chan_index = tmr->first_channel_index; chan_index <= tmr->last_channel_index; chan_index++) {

		uint16_t masks = timer_io_channels[chan_index].masks;

		/* Do we have an enabled channel */

		if (enabled & masks) {


			if (statusr & masks & GTIM_SR_CCIF) {

				io_timer_channel_stats[chan_index].isr_cout++;

				/* Call the client to read the CCxR etc and clear the CCxIF */

				if (channel_handlers[chan_index].callback) {
					channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
									      chan_index, &timer_io_channels[chan_index],
									      now , count);
				}
			}

			if (statusr & masks & GTIM_SR_CCOF) {

				/* Error we has a second edge before we cleared CCxR */

				io_timer_channel_stats[chan_index].overflows++;
			}
		}
	}

	/* Clear all the SR bits for interrupt enabled channels only */

	rSR(timer_index) = ~(statusr & (enabled | enabled << 8));
	return 0;
}

int io_timer_handler0(int irq, void *context)
{

	return io_timer_handler(0);
}

int io_timer_handler1(int irq, void *context)
{
	return io_timer_handler(1);

}

int io_timer_handler2(int irq, void *context)
{
	return io_timer_handler(2);

}

int io_timer_handler3(int irq, void *context)
{
	return io_timer_handler(3);

}
