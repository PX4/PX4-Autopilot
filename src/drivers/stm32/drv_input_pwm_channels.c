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
 * @file drv_input_pwm.c
 *
 * PWM Input driver supporting PWM RC inputs connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have input pins, works with interrupts.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

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

#include "drv_input_pwm_channels.h"

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#define REG(_tmr, _reg)	(*(volatile uint32_t *)(input_pwm_timers[_tmr].base + _reg))

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
#define rBDTR(_tmr)	REG(_tmr, STM32_ATIM_BDTR_OFFSET)

/** PWM decoder state machine */
struct {
        uint16_t        last_edge;      /**< last capture time */
        enum {
                UNSYNCH = 0,
                ARM,
                ACTIVE,
                INACTIVE
        } phase;
} pwm;

//static int		input_pwm_timer_isr(int irq, void *context);
static int		input_pwm_timer_isr(uint8_t timer, uint8_t *channel, uint16_t *value);
static uint16_t		input_pwm_decode(uint32_t status, uint8_t timer, uint8_t timer_channel);
static void		input_pwm_timer_init(unsigned timer);
static void		input_pwm_channel_init(unsigned channel);

/**
 * Handle the timer interrupt by reading and parsing the status register
 * for the given timer.
 *
 * Return the channel number(s) in binary flags (0+2+3=>channel=12)
 * and a pointer to an array 4 long uint16_t value[4]
 */
int
up_input_pwm_timer_isr(uint8_t timer, uint8_t *channel, uint16_t value[4])
{
        uint32_t status;
        /* copy interrupt status */
        status = rSR(timer);

        /* ack the interrupts we just read */
        rSR(timer) = ~status;

	*channel = 0;
	/* which channel was this */
	/* channel 1 */
	if (status & (GTIM_SR_CC1IF | GTIM_SR_CC1OF)) {
		value[0] = input_pwm_decode(status & (GTIM_SR_CC1IF | GTIM_SR_CC1OF), timer, 1);
		*channel |= (1 << 0);
	}
	/* channel 2 */
	if (status & (GTIM_SR_CC2IF | GTIM_SR_CC2OF)){
		value[1] = input_pwm_decode(status & (GTIM_SR_CC2IF | GTIM_SR_CC2OF), timer, 2);
		*channel |= (1 << 1);
	}
	/* channel 3 */
	if (status & (GTIM_SR_CC3IF | GTIM_SR_CC3OF)){
		value[2] = input_pwm_decode(status & (GTIM_SR_CC3IF | GTIM_SR_CC3OF), timer, 3);
		*channel |= (1 << 2);
	}
	/* channel 4 */
	if (status & (GTIM_SR_CC4IF | GTIM_SR_CC4OF)){
		value[3] = input_pwm_decode(status & (GTIM_SR_CC4IF | GTIM_SR_CC4OF), timer, 4);
		*channel |= (1 << 3);
	}
        return OK;
}

uint16_t
input_pwm_decode(uint32_t status, uint8_t timer, uint8_t timer_channel)
{
	uint16_t count;

	// for now we don't care about CCxOF, state machine will take care of it
	switch (timer_channel) {

	case 1:
		count = rCCR1(timer);
		break;
		/* did we miss an edge? */
		if (status & GTIM_SR_CC1OF)
			goto error;

	case 2:
		count = rCCR2(timer);
		break;
		/* did we miss an edge? */
		if (status & GTIM_SR_CC2OF)
			goto error;

	case 3:
		count = rCCR3(timer);
		break;
		/* did we miss an edge? */
		if (status & GTIM_SR_CC3OF)
			goto error;

	case 4:
		count = rCCR4(timer);
		break;
		/* did we miss an edge? */
		if (status & GTIM_SR_CC4OF)
			goto error;

	}
	return count;
        /* the state machine is corrupted; reset it */
error:
        /* we don't like the state of the decoder, reset it and try again */
	printf("overwrite error\n");
	return 0;
}

static void
input_pwm_timer_init(unsigned timer)
{
	/* enable the timer clock before we try to talk to it */
	modifyreg32(input_pwm_timers[timer].clock_register, 0, input_pwm_timers[timer].clock_bit);

	/* disable and configure the timer */
	rCR1(timer) = 0;
	rCR2(timer) = 0;
	rSMCR(timer) = 0;
	rDIER(timer) = 0;
	rCCER(timer) = 0;
	rCCMR1(timer) = 0;
	rCCMR2(timer) = 0;
	rCCER(timer) = 0;
	rDCR(timer) = 0;

	/* configure the timer to free-run at 1MHz */
	rPSC(timer) = (input_pwm_timers[timer].clock_freq / 1000000) - 1;

        /* run the full span of the counter */
        rARR(timer) = 0xffff;

        /* generate an update event; reloads the counter, all registers */
        rEGR(timer) = GTIM_EGR_UG;
        /* enable the timer and disable the update flag event */
        rCR1(timer) = GTIM_CR1_UDIS | GTIM_CR1_CEN;
}

static void
input_pwm_channel_init(unsigned channel)
{
	unsigned timer = input_pwm_channels[channel].timer_index;

	/* configure the GPIO first */
	stm32_configgpio(input_pwm_channels[channel].gpio);

	/* configure the channel */
	switch (input_pwm_channels[channel].timer_channel) {
	case 1:
		rDIER(timer) |= GTIM_DIER_CC1IE;
		rCCMR1(timer) |= (GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC1S_SHIFT);
		rCCR1(timer) = 1000;
		rCCER(timer) |= GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP;
		break;

	case 2:
		rDIER(timer) |= GTIM_DIER_CC2IE;
		rCCMR1(timer) |= (GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC2S_SHIFT);
		rCCR2(timer) = 1000;
		rCCER(timer) |= GTIM_CCER_CC2E | GTIM_CCER_CC2P | GTIM_CCER_CC2NP;
		break;

	case 3:
		rDIER(timer) |= GTIM_DIER_CC3IE;
		rCCMR2(timer) |= (GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR2_CC3S_SHIFT);
		rCCR3(timer) = 1000;
		rCCER(timer) |= GTIM_CCER_CC3E | GTIM_CCER_CC3P | GTIM_CCER_CC3NP;
		break;

	case 4:
		rDIER(timer) |= GTIM_DIER_CC4IE;
		rCCMR2(timer) |= (GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR2_CC4S_SHIFT);
		rCCR4(timer) = 1000;
		rCCER(timer) |= GTIM_CCER_CC4E | GTIM_CCER_CC4P | GTIM_CCER_CC4NP;
		break;
	}
}

int
up_input_pwm_init()
{
	/* do basic timer initialisation first */
	for (unsigned i = 0; i < INPUT_PWM_MAX_TIMERS; i++) {
		if (input_pwm_timers[i].base != 0)
			input_pwm_timer_init(i);
	}

	/* now init channels */
	for (unsigned i = 0; i < INPUT_PWM_MAX_CHANNELS; i++) {
		/* don't do init for disabled channels; this leaves the pin configs alone */
		if (input_pwm_channels[i].timer_channel != 0)
			input_pwm_channel_init(i);
	}

	return OK;
}

/*
 Initialize a timer referenced by timer_index, initialize its channels and return (ref) its irq vector
*/

int
up_input_pwm_timer_init(uint8_t timer_index)
{
	/* make sure timer_index is in bounds */
	if (timer_index < 0 || timer_index > INPUT_PWM_MAX_TIMERS)
		return -1;

	/* do basic timer initialization first */
	if (input_pwm_timers[timer_index].base != 0)
			input_pwm_timer_init(timer_index);

	/* now init channels associated with this timer */
	for (unsigned i = 0; i < INPUT_PWM_MAX_CHANNELS; i++) {
		/* don't do init for disabled channels; this leaves the pin configs alone */
		if (input_pwm_channels[i].timer_index == timer_index)
			input_pwm_channel_init(i);
	}

	return OK;
}
