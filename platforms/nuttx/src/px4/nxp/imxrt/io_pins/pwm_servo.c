/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * Servo driver supporting PWM servos connected to FLexPWM timer blocks.
 * N.B. Groups:channels have a 1:1 correspondence on FlexPWM
 *
 */

#include <px4_platform_common/px4_config.h>
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

#include <px4_arch/io_timer.h>

//#include <chip.h>

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
	uint32_t current = io_timer_get_mode_channels(IOTimerChanMode_PWMOut) |
			   io_timer_get_mode_channels(IOTimerChanMode_OneShot);

	// First free the current set of PWMs

	for (unsigned channel = 0; current != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (current & (1 << channel)) {
			io_timer_set_enable(false, IOTimerChanMode_PWMOut, 1 << channel);
			io_timer_unallocate_channel(channel);
			current &= ~(1 << channel);
		}
	}


	/* Now allocate the new set */

	int ret_val = OK;
	int channels_init_mask = 0;

	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {

			// First free any that were not PWM mode before

			ret_val = io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);
			channel_mask &= ~(1 << channel);

			if (OK == ret_val) {
				channels_init_mask |= 1 << channel;

			} else if (ret_val == -EBUSY) {
				/* either timer or channel already used - this is not fatal */
				ret_val = 0;
			}
		}
	}

	return ret_val == OK ? channels_init_mask : ret_val;
}

void up_pwm_servo_deinit(uint32_t channel_mask)
{
	/* disable the timers */
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set_rate_group_update(unsigned channel, unsigned rate)
{
	if (io_timer_validate_channel_index(channel) < 0) {
		return ERROR;
	}

	/* Allow a rate of 0 to enter oneshot mode */

	if (rate != 0) {

		/* limit update rate to 1..10000Hz; somewhat arbitrary but safe */

		if (rate < 1) {
			return -ERANGE;
		}

		if (rate > 10000) {
			return -ERANGE;
		}
	}

	return io_timer_set_pwm_rate(channel, rate);
}

void up_pwm_update(unsigned channel_mask)
{
	io_timer_trigger(channel_mask);
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	/* only return the set of channels in the group which we own */
	return (io_timer_get_mode_channels(IOTimerChanMode_PWMOut) |
		io_timer_get_mode_channels(IOTimerChanMode_OneShot)) &
	       io_timer_get_group(group);
}

void
up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	io_timer_set_enable(armed, IOTimerChanMode_OneShot, channel_mask);
	io_timer_set_enable(armed, IOTimerChanMode_PWMOut, channel_mask);
}
