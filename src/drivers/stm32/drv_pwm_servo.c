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
#include "drv_pwm_servo.h"

#include <stm32_tim.h>

/* pwm_oneshot_mode true implies all servo outputs are oneshot */
static bool pwm_oneshot_mode = false;

/* this is a bitfield: if bit N is set, that timer is used for oneshot only */
static uint8_t oneshot_timers;

void up_pwm_set_oneshot_mode(bool value)
{
	pwm_oneshot_mode = value;
}

bool up_pwm_get_oneshot_mode()
{
	return pwm_oneshot_mode;
}

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	return io_timer_set_ccr(channel, value);
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	return io_channel_get_ccr(channel);
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	/* Init channels */
	io_timer_channel_mode_t chmode = IOTimerChanMode_PWMOut;

	if (pwm_oneshot_mode) {
		chmode = IOTimerChanMode_OneShot;
	}

	uint32_t current = io_timer_get_mode_channels(chmode);

	// First free the current set of PWMs

	for (unsigned channel = 0; current != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (current & (1 << channel)) {
			io_timer_free_channel(channel);
			current &= ~(1 << channel);
		}
	}

	oneshot_timers = 0;

	// Now allocate the new set

	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {

			// First free any that were not PWM mode before

			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			io_timer_channel_init(channel, chmode, NULL, NULL);
			channel_mask &= ~(1 << channel);

			if (pwm_oneshot_mode) {
				// update the oneshot_timers bitfield
				oneshot_timers |= (1 << timer_io_channels[channel].timer_index);
			}
		}
	}

	return OK;
}

void up_pwm_servo_deinit(void)
{
	/* disable the timers */
	up_pwm_servo_arm(false);
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	/* limit update rate to 1..10000Hz; somewhat arbitrary but safe */
	if (rate < 1) {
		return -ERANGE;
	}

	if (rate > 10000) {
		return -ERANGE;
	}

	if ((group >= MAX_IO_TIMERS) || (io_timers[group].base == 0)) {
		return ERROR;
	}

	io_timer_set_rate(group, rate);

	return OK;
}

void up_pwm_force_update(void)
{
	for (unsigned i = 0; i < 8; i++) {
		if (oneshot_timers & (1 << i)) {
			io_timer_force_update(i);
		}
	}
}

int up_pwm_servo_set_rate(unsigned rate)
{
	for (unsigned i = 0; i < MAX_IO_TIMERS; i++) {
		up_pwm_servo_set_rate_group_update(i, rate);
	}

	return 0;
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	return io_timer_get_group(group);
}

void
up_pwm_servo_arm(bool armed)
{
	if (pwm_oneshot_mode) {
		io_timer_set_enable(armed, IOTimerChanMode_OneShot, IO_TIMER_ALL_MODES_CHANNELS);

	} else {
		io_timer_set_enable(armed, IOTimerChanMode_PWMOut, IO_TIMER_ALL_MODES_CHANNELS);
	}
}
